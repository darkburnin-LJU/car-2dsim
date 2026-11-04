// sim-core.js - 2D 자동차 시뮬레이션의 계산 부분 (화면(DOM, canvas)을 건드리지 않는 코드만)
// index.html(화면)과 tests/run-tests.js(자동 테스트)가 함께 씀
// - 브라우저: <script src="sim-core.js"></script>로 불러오면 전역 createSim()이 생김
// - Node.js: const { createSim } = require("./sim-core.js")
// 계산식·판단 기준은 index_v14.html에 있던 것을 글자 그대로 옮긴 것

(function (root) {
  // 시뮬레이션 하나를 새로 만들기. 부를 때마다 새 차·새 장애물·새 설정값(기본값)
  // (테스트는 테스트마다 새로 만들어서 서로 영향을 주지 않게 함)
  function createSim() {
    // ===== 쉽게 바꿀 수 있는 값 (처음 열었을 때의 기본값) =====
    // 가속도·감속도 단위: m/s² (1초마다 속도가 몇 m/s 바뀌는지)
    // 기본값: 설정 화면의 [기본값으로 되돌리기]를 누르면 이 값으로 돌아감
    const DEFAULT_ACCELERATION = 2.5;  // 가속도: 목표 속도보다 느릴 때 (0→100km/h 약 11초)
    const DEFAULT_DECELERATION = 3.0;  // 감속도: 목표 속도보다 빠를 때 (가볍게 브레이크 밟는 정도)
    const DEFAULT_BRAKE_DECELERATION = 7.0; // 브레이크 감속도: 브레이크 버튼을 누르는 동안 (급제동 정도)
    // 지금 적용 중인 값. 설정 화면의 입력칸으로 바꿀 수 있어서 const 대신 let을 씀
    let ACCELERATION = DEFAULT_ACCELERATION;
    let DECELERATION = DEFAULT_DECELERATION;
    let BRAKE_DECELERATION = DEFAULT_BRAKE_DECELERATION;
    const START_TARGET_KMH = 60;  // 처음 목표 속도(km/h)

    // 차량·노면 기본값 (설정 화면에서 바꿀 수 있음)
    const DEFAULT_MASS_KG = 1500;         // 차 질량(kg), 중형 승용차 정도
    const DEFAULT_DRAG_CD = 0.30;         // 공기저항계수(단위 없음), 보통 승용차 0.25~0.35
    const DEFAULT_FRONTAL_AREA_M2 = 2.2;  // 정면 면적(m²): 앞에서 본 차의 넓이
    const DEFAULT_ROLLING_CRR = 0.010;    // 구름저항계수(단위 없음): 타이어가 구르면서 생기는 저항, 아스팔트 0.01 정도
    const DEFAULT_ROAD_MU = 0.8;          // 노면 마찰계수(단위 없음): 마른 아스팔트 0.7~0.9, 젖으면 0.4~0.6
    let MASS_KG = DEFAULT_MASS_KG;
    let DRAG_CD = DEFAULT_DRAG_CD;
    let FRONTAL_AREA_M2 = DEFAULT_FRONTAL_AREA_M2;
    let ROLLING_CRR = DEFAULT_ROLLING_CRR;
    let ROAD_MU = DEFAULT_ROAD_MU;

    // 날씨별 노면 마찰계수. 모두 일반(여름) 타이어 기준
    const WEATHER_PRESETS = {
      clear: { name: "맑음", mu: 0.8 },   // 마른 아스팔트 (8단계 기본값과 같음)
      rain:  { name: "비",   mu: 0.5 },   // 젖은 아스팔트: 학술지 표 0.4~0.5, 최대값 자료 0.50~0.70
      snow:  { name: "눈",   mu: 0.25 },  // 룬드대(VTI 측정) 자료 여름 타이어 0.26 (겨울 타이어 0.40)
      ice:   { name: "결빙", mu: 0.08 }   // 같은 자료 여름 타이어 0.077, SAE 960959 빙판 대표값 0.10
    };
    let customMu = null;  // 사용자가 직접 입력한 μ(날씨 표에 없는 값)를 기억. 없으면 null

    // 날씨별 센서 감지 거리 비율 기본값 (1.0 = 설정한 감지 거리 그대로)
    const DEFAULT_SENSOR_FACTORS = {
      clear: { radar: 1.0, lidar: 1.0, camera: 1.0 },
      // 레이더: 심한 비 최대 45% 감소 시뮬레이션(Zang 외) → 보통 비로 줄여 잡음. 라이다·카메라: 예시값
      rain:  { radar: 0.9, lidar: 0.6, camera: 0.6 },
      // 라이다: "영향 없음"(ASME)과 "감소"(TechWorks) 자료가 엇갈려 중간값. 레이더·카메라: 예시값
      snow:  { radar: 0.9, lidar: 0.8, camera: 0.5 },
      // 레이더는 안개에 강함. 라이다 30m는 한국건설기술연구원 실험(짙은 안개에서 금속 20~30m 감지 안 됨)과
      // 비슷한 수준. 카메라: 예시값
      fog:   { radar: 1.0, lidar: 0.3, camera: 0.2 }
    };
    // 지금 값 (맑음은 항상 1.0이라 바꾸는 통로가 없음)
    const sensorFactors = {
      clear: Object.assign({}, DEFAULT_SENSOR_FACTORS.clear),
      rain: Object.assign({}, DEFAULT_SENSOR_FACTORS.rain),
      snow: Object.assign({}, DEFAULT_SENSOR_FACTORS.snow),
      fog: Object.assign({}, DEFAULT_SENSOR_FACTORS.fog)
    };
    let skyWeather = "clear";  // 지금 날씨: "clear" / "rain" / "snow" / "fog" (노면 μ와는 따로)

    // 레이더 기본값 (설정 화면에서 바꿀 수 있음)
    // Continental(Aumovio) ARS 408-21 데이터시트(원거리 250m ±9°, 근거리 70m ±45°)를 단순화한 값
    const DEFAULT_RADAR_FAR_RANGE_M = 250;   // 원거리 감지 거리(m)
    const DEFAULT_RADAR_FAR_HALF_DEG = 9;    // 원거리 감지 각도(±°, 가운데에서 한쪽으로)
    const DEFAULT_RADAR_NEAR_RANGE_M = 70;   // 근거리 감지 거리(m)
    const DEFAULT_RADAR_NEAR_HALF_DEG = 45;  // 근거리 감지 각도(±°)
    const DEFAULT_SAFETY_MARGIN_M = 2;       // 자동 정지 안전 여유(m): 이만큼 더 일찍 브레이크
    let RADAR_FAR_RANGE_M = DEFAULT_RADAR_FAR_RANGE_M;
    let RADAR_FAR_HALF_DEG = DEFAULT_RADAR_FAR_HALF_DEG;
    let RADAR_NEAR_RANGE_M = DEFAULT_RADAR_NEAR_RANGE_M;
    let RADAR_NEAR_HALF_DEG = DEFAULT_RADAR_NEAR_HALF_DEG;
    let SAFETY_MARGIN_M = DEFAULT_SAFETY_MARGIN_M;

    // 라이다 기본값: Velodyne VLP-16 데이터시트(측정 거리 100m, 수평 360°, 각도 해상도 0.1~0.4°)
    const DEFAULT_LIDAR_RANGE_M = 100;  // 감지 거리(m)
    const DEFAULT_LIDAR_RES_DEG = 0.4;  // 광선 간격(°) → 360 ÷ 0.4 = 광선 900개
    let LIDAR_RANGE_M = DEFAULT_LIDAR_RANGE_M;
    let LIDAR_RES_DEG = DEFAULT_LIDAR_RES_DEG;

    // 카메라 기본값: Bosch 신형 다목적 카메라 발표 자료(수평 시야 120°, 감지 거리 최대 300m)
    const DEFAULT_CAMERA_RANGE_M = 300; // 감지 거리(m)
    const DEFAULT_CAMERA_HALF_DEG = 60; // 감지 각도(±°) → 시야 120°
    let CAMERA_RANGE_M = DEFAULT_CAMERA_RANGE_M;
    let CAMERA_HALF_DEG = DEFAULT_CAMERA_HALF_DEG;

    // 센서 반응 기본값 = 이상적 센서 (매 계산마다 측정, 바로 확인, 기억 없음)
    const DEFAULT_RADAR_PERIOD_MS = 0;   // 레이더 측정 주기(ms). 0 = 매 계산마다
    const DEFAULT_LIDAR_PERIOD_MS = 0;   // 라이다 측정 주기(ms)
    const DEFAULT_CAMERA_PERIOD_MS = 0;  // 카메라 측정 주기(ms)
    const DEFAULT_CONFIRM_COUNT = 1;     // 연속 몇 번 감지해야 "확인됨"인지
    const DEFAULT_TRACK_MEMORY_S = 0;    // 놓친 뒤 몇 초 동안 기억할지(초)
    let RADAR_PERIOD_MS = DEFAULT_RADAR_PERIOD_MS;
    let LIDAR_PERIOD_MS = DEFAULT_LIDAR_PERIOD_MS;
    let CAMERA_PERIOD_MS = DEFAULT_CAMERA_PERIOD_MS;
    let CONFIRM_COUNT = DEFAULT_CONFIRM_COUNT;
    let TRACK_MEMORY_S = DEFAULT_TRACK_MEMORY_S;

    // 현실 센서 값 (설정 화면의 [현실 센서] 버튼)
    const REAL_SENSOR_VALUES = {
      RADAR_PERIOD_MS: 60,   // ARS 408-21 데이터시트 측정 주기 약 60ms
      LIDAR_PERIOD_MS: 100,  // VLP-16 회전 5~20Hz 중 10Hz(예시)
      CAMERA_PERIOD_MS: 33,  // Bosch 카메라 헤드 30 FPS(차량용 수치는 확인 못 함, 예시)
      CONFIRM_COUNT: 3,      // 예시값(공개 자료 없음)
      TRACK_MEMORY_S: 0.5    // 예시값(공개 자료 없음)
    };

    // 바꿀 수 없는 상수
    const AIR_DENSITY = 1.225; // 공기 밀도 kg/m³ (국제표준대기, 해수면 15°C)
    const GRAVITY = 9.81;      // 중력가속도 m/s²

    // 도로·자동차·화면
    const ROAD_LENGTH_M = 1000;  // 도로 전체 길이(m)
    const LANE_WIDTH_M = 3.5;    // 차로 폭(m), 예시값
    const LANE_COUNT = 2;        // 차로 수
    const ROAD_WIDTH_M = LANE_WIDTH_M * LANE_COUNT;  // 도로 폭 7m
    const MY_LANE_Y_M = LANE_WIDTH_M / 2;  // 내 차가 달리는 차로의 가운데 (+1.75m, 아래쪽 차로)
    const CAR_LENGTH_M = 4.5;    // 자동차 길이(m)
    const CAR_WIDTH_M = 1.8;     // 자동차 폭(m)
    const CAR_HEIGHT_M = 1.5;    // 차 높이(m), 예시값. 지금은 계산에 안 쓰고 10단계(멀티뷰)용
    const PED_SIZE_M = 0.5;      // 보행자 가로·세로(m), 예시값
    const PED_HEIGHT_M = 1.7;    // 보행자 키(m), 제8차 한국인 인체치수조사 남성 평균 172.5cm 반올림
    const PED_WALK_SPEED_MS = 1.0; // 보행 속도(m/s), 경찰청 교통신호기 업무편람(2022) 횡단보행속도
    const MAX_OBSTACLES = 10;    // 장애물 최대 개수
    const ROADSIDE_LIMIT_M = 3;  // 도로 가장자리에서 이만큼 밖까지만 장애물을 놓을 수 있음(m)

    // km/h ↔ m/s 바꾸기 (1 m/s = 3.6 km/h)
    function kmhToMs(kmh) {
      return kmh / 3.6;
    }
    function msToKmh(ms) {
      return ms * 3.6;
    }

    // 2. 옆 방향 위치 y(m): 0 = 도로 가운데 선, + = 화면 아래쪽, − = 화면 위쪽
    //    도로는 y = −3.5m ~ +3.5m (도로 폭 7m)

    // 3. 자동차 설정 (위치는 미터 단위)
    const car = {
      xM: 0,             // 차 뒤쪽 끝의 위치(m), 도로 왼쪽 끝 = 0
      speedMs: 0,        // 현재 속도(m/s), 처음에는 멈춰 있음
      targetSpeedKmh: START_TARGET_KMH, // 목표 속도(km/h), 슬라이더나 입력칸으로 바뀜
      accelMs2: 0        // 지금 이 순간 실제로 받는 가속도(m/s²), +가속 / −감속
    };

    // 4. 브레이크를 누르고 있는지 / 작동시켜 두었는지 기억하는 변수
    let braking = false;       // "브레이크 (누르고 있기)"를 누르는 동안 true
    let brakeLatched = false;  // "브레이크 작동"을 켜 두면 true (다시 누를 때까지 유지)
    let autoBraking = false;   // 자동 정지가 브레이크를 거는 중이면 true
    let aebEnabled = false;    // 자동 정지 켜짐/꺼짐 (기본 꺼짐)
    // 자동 정지에 쓸 센서 (기본 셋 다)
    let aebUseRadar = true, aebUseLidar = true, aebUseCamera = true;

    // 브레이크가 켜져 있는지 (누르고 있기 / 작동 / 자동 정지 중 하나라도)
    function isBrakeActive() {
      return braking || brakeLatched || autoBraking;
    }

    // 제동거리 기록
    const brakeStats = {
      wasActive: false,   // 직전에 브레이크가 켜져 있었는지 (켜짐/꺼짐 바뀌는 순간을 찾으려고)
      startKmh: null,     // 브레이크 시작 속도(km/h), 아직 안 밟았으면 null
      segmentM: 0,        // 브레이크 구간 거리(m): 켜져 있는 동안 달린 거리
      stopM: 0,           // 정지까지 제동거리(m)
      status: ""          // "측정 중" / "정지 완료" / "중간에 뗌" / "이미 정지"
    };

    // 장애물 목록. 하나마다:
    // kind("car" 앞차 / "ped" 보행자), 가운데 위치 xM·yM(m), 길이 lengthM·폭 widthM·높이 heightM(m),
    // 옆 방향 속도 vyMs(m/s, + = 아래쪽으로), 부딪혔는지 hit
    const obstacles = [];
    let collided = false;   // 충돌해서 멈춘 상태인지 (지우기 전에는 다시 출발 못 함)
    let crashKmh = 0;       // 충돌 직전 속도(km/h)

    // ===== 센서 =====
    // 센서 하나 = 이름, 설치 위치(xM·yM, 내 차 기준), 설치 높이, 감지 방식("부채꼴"/"광선"), 색
    // 부채꼴 센서는 감지 영역 목록(이름, 거리, 반각, 색)도 가짐 → detectObjects를 같이 씀
    // 설치 위치·높이는 모두 예시값. 높이는 10단계(멀티뷰)용이라 계산에는 안 씀
    // (get = 읽을 때마다 지금 값을 다시 계산하는 칸)
    const radar = {
      name: "레이더",
      get xM() { return car.xM + CAR_LENGTH_M; },  // 내 차 앞범퍼
      get yM() { return MY_LANE_Y_M; },            // 앞범퍼 가운데
      mountHeightM: 0.5,
      method: "부채꼴",
      color: "rgb(80,160,255)",
      // 감지 영역: 설정 화면의 지금 값을 읽어서 씀
      get zones() {
        return [
          { name: "원거리", rangeM: RADAR_FAR_RANGE_M * sensorFactor("radar"), halfAngleDeg: RADAR_FAR_HALF_DEG,
            fill: "rgba(80,160,255,0.22)", line: "rgba(80,160,255,0.9)" },
          { name: "근거리", rangeM: RADAR_NEAR_RANGE_M * sensorFactor("radar"), halfAngleDeg: RADAR_NEAR_HALF_DEG,
            fill: "rgba(80,200,120,0.22)", line: "rgba(80,200,120,0.9)" }
        ];
      }
    };
    const lidar = {
      name: "라이다",
      get xM() { return car.xM + CAR_LENGTH_M / 2; }, // 지붕 가운데
      get yM() { return MY_LANE_Y_M; },
      mountHeightM: 1.8,
      method: "광선",
      color: "rgb(150,80,220)"
    };
    const camera = {
      name: "카메라",
      get xM() { return car.xM + CAR_LENGTH_M - 1.5; }, // 앞유리 위
      get yM() { return MY_LANE_Y_M; },
      mountHeightM: 1.3,
      method: "부채꼴",
      color: "rgb(255,150,50)",
      get zones() {
        return [
          { name: "카메라", rangeM: CAMERA_RANGE_M * sensorFactor("camera"), halfAngleDeg: CAMERA_HALF_DEG,
            fill: "rgba(255,150,50,0.18)", line: "rgba(255,150,50,0.9)" }
        ];
      }
    };
    const sensors = [radar, lidar, camera];

    // 남은 거리와 경로 안 여부 (어느 센서든 같은 기준)
    function pathInfo(ob) {
      // 남은 거리: 내 차 앞범퍼 → 장애물 가까운 쪽 끝
      const gapM = (ob.xM - ob.lengthM / 2) - (car.xM + CAR_LENGTH_M);
      // 경로 안: 내 차 폭과 옆으로 겹치고, 아직 앞에 있음
      const inPath = Math.abs(ob.yM - MY_LANE_Y_M) < CAR_WIDTH_M / 2 + ob.widthM / 2 && gapM > 0;
      return { gapM: gapM, inPath: inPath };
    }

    // 부채꼴 센서(레이더·카메라)가 감지한 물체 목록 (가까운 순서)
    function detectObjects(sensor) {
      const result = [];
      for (const ob of obstacles) {
        // 센서에서 장애물 가운데까지
        const dx = ob.xM - sensor.xM;
        const dy = ob.yM - sensor.yM;
        if (dx <= 0) continue; // 센서 뒤쪽은 감지 안 함

        const distM = Math.sqrt(dx * dx + dy * dy);
        const angleDeg = Math.atan2(dy, dx) * 180 / Math.PI; // + = 아래쪽, − = 위쪽

        // 영역 중 하나라도 들어가면 감지
        const zoneNames = [];
        for (const zone of sensor.zones) {
          if (distM <= zone.rangeM && Math.abs(angleDeg) <= zone.halfAngleDeg) zoneNames.push(zone.name);
        }
        if (zoneNames.length === 0) continue;

        // 접근 속도: 장애물의 상대 속도(0 − 내 차 속도, 옆 방향 속도)를 센서→장애물 방향으로 투영하고 −를 붙임
        // (가까워지면 +)
        const relVx = 0 - car.speedMs;
        const relVy = ob.vyMs;
        const closingMs = -(relVx * dx + relVy * dy) / distM;

        const info = pathInfo(ob);
        result.push({
          obstacle: ob, distM: distM, angleDeg: angleDeg, zone: zoneNames.join("+"),
          closingMs: closingMs, inPath: info.inPath, gapM: info.gapM,
          kind: ob.kind === "car" ? "차" : "보행자"  // 카메라만 이 값을 씀 (레이더는 종류를 모름)
        });
      }
      result.sort(function (a, b) { return a.distM - b.distM; });
      return result;
    }

    // 광선 하나가 축에 나란한 직사각형에 처음 닿는 거리(m). 안 닿으면 null (slab 방식)
    // 가로 칸(slab)과 세로 칸 각각에 들어가고 나오는 거리를 구해, 두 칸에 모두 들어 있는 구간을 찾음
    function rayHitRect(ox, oy, dirX, dirY, left, right, top, bottom) {
      let tEnter = -Infinity, tExit = Infinity;
      const axes = [[ox, dirX, left, right], [oy, dirY, top, bottom]];
      for (const [o, d, lo, hi] of axes) {
        if (Math.abs(d) < 1e-12) {
          if (o < lo || o > hi) return null; // 이 방향으로 나란히 가면서 칸 밖 → 안 닿음
        } else {
          let t1 = (lo - o) / d, t2 = (hi - o) / d;
          if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
          tEnter = Math.max(tEnter, t1);
          tExit = Math.min(tExit, t2);
        }
      }
      if (tEnter > tExit || tEnter < 0) return null; // 안 겹치거나 뒤쪽(또는 안에서 출발)
      return tEnter;
    }

    // 지금 날씨 키, 날씨 바꾸기 (노면 μ는 건드리지 않음)
    function skyMode() {
      return skyWeather;
    }
    function applySky(key) {
      skyWeather = key;
    }

    // 지금 날씨의 센서 감지 거리 비율 ("radar" / "lidar" / "camera")
    function sensorFactor(name) {
      return sensorFactors[skyWeather][name];
    }

    // 날씨를 반영한 라이다 감지 거리(m)
    function effectiveLidarRangeM() {
      return LIDAR_RANGE_M * sensorFactor("lidar");
    }

    // 라이다: 0°~360°를 LIDAR_RES_DEG 간격 광선으로 훑음
    // 광선은 처음 닿은 장애물에서 끝남 → 뒤 물체는 못 봄(가려짐). 내 차는 장애물 목록에 없어서 자동으로 제외
    // 돌려주는 값: 감지 목록(장애물별 닿은 광선 수·가장 가까운 닿은 점 거리·각도), 그림용 닿은 점 목록
    function scanLidar(sensor) {
      const ox = sensor.xM, oy = sensor.yM;
      const rayCount = Math.floor(360 / LIDAR_RES_DEG + 1e-9); // 기본 0.4° → 900개
      const byObstacle = new Map();
      const hitPoints = [];
      const rangeM = effectiveLidarRangeM(); // 날씨를 반영한 감지 거리

      // 속도 개선: 라이다 범위 안에 들어올 수 있는 장애물만 광선 계산에 넣음
      // 판단 기준은 장애물 가운데가 아니라 "직사각형에서 설치 위치와 가장 가까운 점"까지의 거리
      // (가운데는 범위 밖이어도 가장자리가 범위 안이면 광선이 닿을 수 있음)
      // 가장 가까운 점이 범위 밖이면 어떤 광선도 범위 안에서 닿을 수 없으므로 결과는 그대로
      const inRange = obstacles.filter(function (ob) {
        const gapX = Math.max(ob.xM - ob.lengthM / 2 - ox, 0, ox - (ob.xM + ob.lengthM / 2)); // 가로로 떨어진 거리
        const gapY = Math.max(ob.yM - ob.widthM / 2 - oy, 0, oy - (ob.yM + ob.widthM / 2)); // 세로로 떨어진 거리
        return Math.sqrt(gapX * gapX + gapY * gapY) <= rangeM;
      });
      if (inRange.length === 0) return { list: [], hitPoints: hitPoints }; // 범위 안에 아무것도 없음

      for (let i = 0; i < rayCount; i++) {
        const angleDeg = i * LIDAR_RES_DEG;
        const rad = angleDeg * Math.PI / 180;
        const dirX = Math.cos(rad), dirY = Math.sin(rad);

        // 이 광선이 처음 닿는 장애물 찾기
        let bestT = rangeM, bestOb = null;
        for (const ob of inRange) {
          const t = rayHitRect(ox, oy, dirX, dirY,
            ob.xM - ob.lengthM / 2, ob.xM + ob.lengthM / 2, ob.yM - ob.widthM / 2, ob.yM + ob.widthM / 2);
          if (t !== null && t <= bestT) { bestT = t; bestOb = ob; }
        }
        if (bestOb === null) continue;

        hitPoints.push({ xM: ox + dirX * bestT, yM: oy + dirY * bestT });
        // 각도는 −180°~180°로 (레이더와 같은 모양: + = 아래쪽)
        const shownDeg = angleDeg > 180 ? angleDeg - 360 : angleDeg;
        const rec = byObstacle.get(bestOb);
        if (!rec) {
          byObstacle.set(bestOb, { rays: 1, distM: bestT, angleDeg: shownDeg });
        } else {
          rec.rays++;
          if (bestT < rec.distM) { rec.distM = bestT; rec.angleDeg = shownDeg; }
        }
      }

      const list = [];
      byObstacle.forEach(function (rec, ob) {
        const info = pathInfo(ob);
        list.push({ obstacle: ob, rays: rec.rays, distM: rec.distM, angleDeg: rec.angleDeg,
                    inPath: info.inPath, gapM: info.gapM });
      });
      list.sort(function (a, b) { return a.distM - b.distM; });
      return { list: list, hitPoints: hitPoints };
    }

    // 센서 3개 감지를 한 번에 (그릴 때마다 + 자동 정지 판단 때 부름)
    function detectAll() {
      const lidarScan = scanLidar(lidar);
      return {
        radar: detectObjects(radar),
        lidar: lidarScan.list,
        lidarHits: lidarScan.hitPoints,
        camera: detectObjects(camera)
      };
    }

    // 어떤 장애물을 센서가 감지했는지 찾기 (없으면 null)
    function findDetection(list, ob) {
      return list.find(function (d) { return d.obstacle === ob; }) || null;
    }

    // 자동 정지에 체크된 센서 중 하나라도 감지했는지
    function detectedByChecked(det, ob) {
      return (aebUseRadar && findDetection(det.radar, ob) !== null) ||
             (aebUseLidar && findDetection(det.lidar, ob) !== null) ||
             (aebUseCamera && findDetection(det.camera, ob) !== null);
    }

    // 자동 정지 후보(체크된 센서가 감지 + 경로 안) 중 남은 거리가 가장 작은 것 (없으면 null)
    function nearestCandidate(det) {
      let target = null;
      for (const ob of obstacles) {
        if (!detectedByChecked(det, ob)) continue;
        const info = pathInfo(ob);
        if (info.inPath && (target === null || info.gapM < target.gapM)) {
          target = { obstacle: ob, gapM: info.gapM };
        }
      }
      return target;
    }

    // 지금 속도에서 멈추는 데 필요한 거리(m)
    // 공기저항·구름저항은 빼고 계산 → 실제보다 조금 길게 나와서 조금 일찍 멈추는 안전한 쪽
    function requiredStopM() {
      const availDecel = Math.min(BRAKE_DECELERATION, ROAD_MU * GRAVITY); // 쓸 수 있는 감속도
      return availDecel > 0 ? car.speedMs * car.speedMs / (2 * availDecel) : Infinity;
    }

    // ===== 추적 =====
    // 센서별·장애물별 추적 기록 (Map: 장애물 → 기록). 처음 만든 것을 계속 쓰고, 지울 때는 비우기만 함
    // 기록 = state("확인 중"/"확인됨"/"기억 중"), hits(연속 감지 횟수),
    //        last(마지막 측정값: 거리·각도·접근 속도·종류·광선 수), gapM(마지막 남은 거리),
    //        inPath(마지막 경로 안 여부), seenAtS·seenOdoM(마지막으로 본 시각과 그때의 누적 주행 거리)
    const tracks = { radar: new Map(), lidar: new Map(), camera: new Map() };
    const measureTimers = { radar: 0, lidar: 0, camera: 0 }; // 센서별 측정 타이머(초)
    let simTimeS = 0;   // 시뮬레이션 시간(초)
    let odometerM = 0;  // 누적 주행 거리(m). 도로 한 바퀴 돌아도 계속 늘어남

    // 지금 ROAD_MU와 같은 mu를 가진 날씨 키. 어느 것과도 다르면 "custom"
    function weatherMode() {
      for (const key in WEATHER_PRESETS) {
        if (WEATHER_PRESETS[key].mu === ROAD_MU) return key;
      }
      return "custom";
    }

    // 날씨 적용: 노면 마찰계수만 바꿈 (구름저항 등 다른 값은 그대로)
    function applyWeather(key) {
      ROAD_MU = WEATHER_PRESETS[key].mu;
    }

    // 기억해 둔 사용자 μ로 돌아가기. 기억한 값이 없으면 아무것도 안 하고 false
    function applyCustomWeather() {
      if (customMu === null) return false;
      ROAD_MU = customMu;
      return true;
    }

    // 지금 센서 반응 설정: "ideal"(이상적 센서) / "real"(현실 센서) / "custom"(사용자 설정)
    function sensorMode() {
      const now = [RADAR_PERIOD_MS, LIDAR_PERIOD_MS, CAMERA_PERIOD_MS, CONFIRM_COUNT, TRACK_MEMORY_S];
      const ideal = [DEFAULT_RADAR_PERIOD_MS, DEFAULT_LIDAR_PERIOD_MS, DEFAULT_CAMERA_PERIOD_MS, DEFAULT_CONFIRM_COUNT, DEFAULT_TRACK_MEMORY_S];
      const r = REAL_SENSOR_VALUES;
      const real = [r.RADAR_PERIOD_MS, r.LIDAR_PERIOD_MS, r.CAMERA_PERIOD_MS, r.CONFIRM_COUNT, r.TRACK_MEMORY_S];
      const same = function (list) { return list.every(function (v, i) { return v === now[i]; }); };
      return same(ideal) ? "ideal" : same(real) ? "real" : "custom";
    }
    function isIdealSensors() {
      return sensorMode() === "ideal";
    }

    // 이번 스텝에 이 센서가 측정할 차례인지. 주기 0 = 매 계산마다
    // (소수 오차로 측정이 한 스텝 밀리지 않게 비교에 아주 작은 여유를 둠)
    function timeToMeasure(name, periodMs, dt) {
      if (periodMs === 0) return true;
      const periodS = periodMs / 1000;
      measureTimers[name] += dt;
      if (measureTimers[name] < periodS - 1e-9) return false;
      measureTimers[name] -= periodS;
      return true;
    }

    // 측정 결과(감지 목록)를 추적 기록에 반영
    function applyMeasurement(map, list) {
      const seen = new Set();
      for (const d of list) {
        seen.add(d.obstacle);
        let rec = map.get(d.obstacle);
        if (!rec) {
          rec = { state: "확인 중", hits: 0 };
          map.set(d.obstacle, rec);
        }
        rec.hits++;
        // 이미 확인됨/기억 중이었으면 바로 확인됨, 아니면 연속 횟수가 찼을 때 확인됨
        if (rec.state !== "확인 중" || rec.hits >= CONFIRM_COUNT) rec.state = "확인됨";
        rec.last = { distM: d.distM, angleDeg: d.angleDeg, closingMs: d.closingMs, kind: d.kind, rays: d.rays };
        rec.gapM = d.gapM;
        rec.inPath = d.inPath;
        rec.seenAtS = simTimeS;
        rec.seenOdoM = odometerM;
      }
      // 이번 측정에서 감지 안 된 기록
      map.forEach(function (rec, ob) {
        if (seen.has(ob)) return;
        rec.hits = 0;
        if (rec.state === "확인 중" || TRACK_MEMORY_S === 0) map.delete(ob); // 기억 0초면 바로 삭제
        else rec.state = "기억 중";
      });
    }

    // 센서마다 측정 차례면 측정하고, 오래된 기록을 지우기
    function updateTracks(dt) {
      if (timeToMeasure("radar", RADAR_PERIOD_MS, dt)) applyMeasurement(tracks.radar, detectObjects(radar));
      if (timeToMeasure("lidar", LIDAR_PERIOD_MS, dt)) applyMeasurement(tracks.lidar, scanLidar(lidar).list);
      if (timeToMeasure("camera", CAMERA_PERIOD_MS, dt)) applyMeasurement(tracks.camera, detectObjects(camera));

      for (const name of ["radar", "lidar", "camera"]) {
        tracks[name].forEach(function (rec, ob, map) {
          if (!obstacles.includes(ob)) map.delete(ob); // 장애물이 없어짐
          else if (rec.state === "기억 중" && simTimeS - rec.seenAtS > TRACK_MEMORY_S) map.delete(ob);
        });
      }
    }

    // 추적 기록과 측정 타이머 모두 비우기 (새로 만들지 않음)
    function clearTracks() {
      for (const name of ["radar", "lidar", "camera"]) {
        tracks[name].clear();
        measureTimers[name] = 0;
      }
    }

    // 추적 기록의 지금 남은 거리 (물체가 멈춰 있다고 가정한 예측)
    // 이상적 센서는 매 스텝 측정하므로 실제 남은 거리와 같음
    function trackGapM(rec) {
      return rec.gapM - (odometerM - rec.seenOdoM);
    }

    // 자동 정지에 체크된 센서의 추적 기록 목록 (확인 중은 뺌)
    function checkedTracks() {
      const maps = [];
      if (aebUseRadar) maps.push(tracks.radar);
      if (aebUseLidar) maps.push(tracks.lidar);
      if (aebUseCamera) maps.push(tracks.camera);
      const list = [];
      for (const map of maps) {
        map.forEach(function (rec, ob) {
          if (rec.state !== "확인 중") list.push({ obstacle: ob, rec: rec });
        });
      }
      return list;
    }

    // 자동 정지 후보(체크된 센서의 확인됨·기억 중 기록 + 경로 안) 중 지금 남은 거리가 가장 작은 것 (없으면 null)
    function nearestTrackCandidate() {
      let target = null;
      for (const item of checkedTracks()) {
        if (!item.rec.inPath) continue;
        const gapM = trackGapM(item.rec);
        if (target === null || gapM < target.gapM) target = { obstacle: item.obstacle, gapM: gapM };
      }
      return target;
    }

    // 이 장애물의 추적 상태 (테두리 색용): null = 후보 아님, "안" = 경로 안, "밖" = 경로 밖
    function trackPathState(ob) {
      let state = null;
      for (const item of checkedTracks()) {
        if (item.obstacle !== ob) continue;
        if (item.rec.inPath) return "안";
        state = "밖";
      }
      return state;
    }

    // 자동 정지: 체크된 센서의 추적 기록으로만 판단 (확인 못 한 물체는 경로 안이어도 무시)
    function updateAEB() {
      const before = autoBraking;
      if (!aebEnabled) {
        autoBraking = false;
      } else {
        const target = nearestTrackCandidate();
        if (!autoBraking && target !== null && target.gapM <= requiredStopM() + SAFETY_MARGIN_M) {
          autoBraking = true;   // 남은 거리가 필요 제동거리 + 안전 여유 이하 → 제동 시작
        } else if (autoBraking && car.speedMs === 0 && target === null) {
          // 한 번 걸면 멈출 때까지 유지 (도중에 센서가 놓쳐도 풀지 않음)
          // 멈춘 뒤에는 후보가 하나도 없을 때 해제
          autoBraking = false;
        }
      }
      if (autoBraking !== before) checkBrakeChange(); // 제동거리 측정을 수동 브레이크와 똑같이
    }

    // 지금 속도와 상태에서 차에 걸리는 힘을 항목별로 계산 (단위 N, 뉴턴)
    // 나중에 경사·바람 같은 힘은 여기에 항목 하나로 더하면 됨
    function computeForces(speedMs) {
      // 타이어가 낼 수 있는 최대 힘 = 마찰계수 × 차 무게(질량 × 중력)
      const frictionLimitN = ROAD_MU * MASS_KG * GRAVITY;
      // 공기저항 = ½ × 공기 밀도 × 공기저항계수 × 정면 면적 × 속도² (빠를수록 제곱으로 커짐)
      const dragN = 0.5 * AIR_DENSITY * DRAG_CD * FRONTAL_AREA_M2 * speedMs * speedMs;
      // 구름저항 = 구름저항계수 × 차 무게. 멈춰 있으면 0 (멈춘 차가 뒤로 밀리지 않게)
      const rollingN = speedMs > 0 ? ROLLING_CRR * MASS_KG * GRAVITY : 0;

      // 타이어 힘: 앞으로 미는 힘은 +, 제동은 −
      // 요청한 힘(가속도 × 질량)이 마찰 한계보다 크면 마찰 한계까지만
      const targetMs = kmhToMs(car.targetSpeedKmh);
      let mode, requestN, tireN;
      if (isBrakeActive()) {
        mode = "브레이크";
        requestN = BRAKE_DECELERATION * MASS_KG;
        tireN = -Math.min(requestN, frictionLimitN);
      } else if (speedMs < targetMs) {
        mode = "가속";
        requestN = ACCELERATION * MASS_KG;
        tireN = Math.min(requestN, frictionLimitN);
      } else if (speedMs > targetMs) {
        mode = "감속";
        requestN = DECELERATION * MASS_KG;
        tireN = -Math.min(requestN, frictionLimitN);
      } else {
        // 정속 주행: 저항만큼만 밀어서 속도 유지
        mode = "정속";
        requestN = dragN + rollingN;
        tireN = Math.min(requestN, frictionLimitN);
      }
      const limited = requestN > frictionLimitN;

      // 합력 = 타이어 힘 − 공기저항 − 구름저항 (저항은 항상 달리는 반대 방향)
      const totalN = tireN - dragN - rollingN;
      // 가속도 = 합력 ÷ 질량 (뉴턴의 운동 법칙 F = m × a)
      const accelMs2 = totalN / MASS_KG;

      return { frictionLimitN, dragN, rollingN, tireN, limited, mode, totalN, accelMs2 };
    }

    // 현재 속도를 힘 계산 결과(가속도)만큼 바꾸기 (dt = 지난 시간, 초)
    function updateSpeed(dt) {
      const forces = computeForces(car.speedMs);
      const targetMs = kmhToMs(car.targetSpeedKmh);
      const newSpeed = car.speedMs + forces.accelMs2 * dt;

      if (forces.mode === "브레이크") {
        // 브레이크(둘 중 하나라도 켜짐): 0 아래로는 안 내려감
        // 0이 되어도 켜져 있으면 계속 0에 머묾
        car.speedMs = Math.max(0, newSpeed);
      } else if (forces.mode === "가속") {
        // 가속: 목표를 넘지 않게 (저항이 더 크면 속도가 줄 수도 있음)
        car.speedMs = Math.max(0, Math.min(targetMs, newSpeed));
      } else if (forces.mode === "감속") {
        // 감속: 목표 아래로 내려가지 않게
        car.speedMs = Math.max(0, Math.max(targetMs, newSpeed));
      } else {
        // 정속: 목표 속도 그대로
        car.speedMs = targetMs;
      }
    }

    // 브레이크가 켜졌는지/꺼졌는지 확인해서, 바뀌는 순간에 측정을 시작하거나 취소
    // (브레이크 버튼 두 개 중 무엇이 바뀌든 이 함수를 부름)
    function checkBrakeChange() {
      const active = isBrakeActive();
      if (active && !brakeStats.wasActive) {
        // 꺼짐 → 켜짐: 새로 측정 시작
        brakeStats.startKmh = msToKmh(car.speedMs);
        brakeStats.segmentM = 0;
        brakeStats.stopM = 0;
        brakeStats.status = car.speedMs === 0 ? "이미 정지" : "측정 중";
      } else if (!active && brakeStats.wasActive) {
        // 켜짐 → 꺼짐: 구간 거리는 그대로 고정, 정지 전에 뗐으면 취소
        if (brakeStats.status === "측정 중") brakeStats.status = "중간에 뗌";
      }
      brakeStats.wasActive = active;
    }

    // 프레임마다 달린 거리를 더하기 (달리는 중에만 불림 → 일시정지 중엔 안 늘어남)
    function addBrakeDistance(stepM) {
      if (!isBrakeActive()) return;
      brakeStats.segmentM += stepM;
      if (brakeStats.status === "측정 중") {
        brakeStats.stopM += stepM;
        if (car.speedMs === 0) brakeStats.status = "정지 완료";
      }
    }

    // 보행자를 옆 방향으로 움직이기. 도로 아래쪽 밖에 닿으면 그 자리에서 멈춤
    function moveObstacles(dt) {
      const endYM = ROAD_WIDTH_M / 2 + PED_SIZE_M;
      for (const ob of obstacles) {
        if (ob.vyMs === 0) continue;
        // 위로 걷는 보행자: 도로 위쪽 밖에 닿으면 그 자리에서 멈춤
        if (ob.vyMs < 0) {
          ob.yM += ob.vyMs * dt;
          if (ob.yM <= -endYM) {
            ob.yM = -endYM;
            ob.vyMs = 0;
          }
          continue;
        }
        ob.yM += ob.vyMs * dt;
        if (ob.yM >= endYM) {
          ob.yM = endYM;
          ob.vyMs = 0;
        }
      }
    }

    // 내 차와 장애물이 겹치는지 검사 (경계에 딱 닿는 것은 겹침 아님)
    // 겹치면 충돌 처리하고 true
    function checkCollision() {
      const carLeft = car.xM;
      const carRight = car.xM + CAR_LENGTH_M;
      const carTop = MY_LANE_Y_M - CAR_WIDTH_M / 2;
      const carBottom = MY_LANE_Y_M + CAR_WIDTH_M / 2;

      for (const ob of obstacles) {
        const overlapX = carLeft < ob.xM + ob.lengthM / 2 && carRight > ob.xM - ob.lengthM / 2;
        const overlapY = carTop < ob.yM + ob.widthM / 2 && carBottom > ob.yM - ob.widthM / 2;
        if (!(overlapX && overlapY)) continue;

        crashKmh = msToKmh(car.speedMs); // 충돌 직전 속도
        car.speedMs = 0;
        car.accelMs2 = 0;
        ob.hit = true;
        if (brakeStats.status === "측정 중") brakeStats.status = "충돌로 중단";
        collided = true;
        return true;
      }
      return false;
    }

    // 장애물 놓기. 앞쪽 거리 distM = 내 차 앞범퍼에서 장애물의 가까운 쪽 끝까지
    // 놓았으면 null, 못 놓으면 이유("max" = 개수 초과, "roadEnd" = 도로 끝을 넘음)
    function placeObstacle(kind, lengthM, widthM, heightM, yM, vyMs, distM) {
      if (obstacles.length >= MAX_OBSTACLES) {
        return "max";
      }
      const nearM = car.xM + CAR_LENGTH_M + distM;  // 가까운 쪽 끝
      if (nearM + lengthM > ROAD_LENGTH_M) {
        return "roadEnd";
      }
      obstacles.push({
        kind: kind,
        xM: nearM + lengthM / 2,  // 가운데 위치
        yM: yM,
        lengthM: lengthM,
        widthM: widthM,
        heightM: heightM,         // 10단계(멀티뷰)용
        vyMs: vyMs,
        hit: false
      });
      return null;
    }

    // 클릭으로 놓는 건너오는 보행자의 옆 속도: 도로 가운데 선보다 아래면 위로, 아니면 아래로
    function pedCrossVy(yM) {
      return yM > 0 ? -PED_WALK_SPEED_MS : PED_WALK_SPEED_MS;
    }

    // 장애물 가운데를 (xM, yM)에 놓기 (그림판 클릭용)
    // 놓았으면 null, 못 놓으면 이유("max" = 개수 초과, "roadEnd" = 도로 끝을 넘음,
    // "offRoad" = 도로에서 너무 멂, "overlapCar" = 내 차와 겹침)
    function placeObstacleAt(kind, lengthM, widthM, heightM, xM, yM, vyMs) {
      if (obstacles.length >= MAX_OBSTACLES) return "max";
      if (xM - lengthM / 2 < 0 || xM + lengthM / 2 > ROAD_LENGTH_M) return "roadEnd";
      if (Math.abs(yM) > ROAD_WIDTH_M / 2 + ROADSIDE_LIMIT_M) return "offRoad";

      // 내 차와 겹치는지 (checkCollision과 같은 기준: 경계에 딱 닿는 것은 겹침 아님)
      const overlapX = car.xM < xM + lengthM / 2 && car.xM + CAR_LENGTH_M > xM - lengthM / 2;
      const overlapY = MY_LANE_Y_M - CAR_WIDTH_M / 2 < yM + widthM / 2 && MY_LANE_Y_M + CAR_WIDTH_M / 2 > yM - widthM / 2;
      if (overlapX && overlapY) return "overlapCar";

      // 건너오는 보행자가 이미 멈추는 위치보다 아래에 놓이면 그 자리에 서 있게 함
      // (그대로 두면 moveObstacles가 멈추는 위치로 끌어올림)
      if (vyMs > 0 && yM >= ROAD_WIDTH_M / 2 + PED_SIZE_M) vyMs = 0;
      // 위로 걷는 보행자도 마찬가지 (이미 위쪽 멈추는 위치보다 위에 놓이면 서 있게)
      if (vyMs < 0 && yM <= -(ROAD_WIDTH_M / 2 + PED_SIZE_M)) vyMs = 0;

      obstacles.push({
        kind: kind,
        xM: xM,
        yM: yM,
        lengthM: lengthM,
        widthM: widthM,
        heightM: heightM,         // 10단계(멀티뷰)용
        vyMs: vyMs,
        hit: false
      });
      return null;
    }

    // 한 프레임 계산 (그리기 없음). 화면은 실제 경과 시간(최대 MAX_DT), 테스트는 고정 0.01초를 dt로 넣음
    // 순서: 속도 → 이동 → 제동거리 누적 → 보행자 이동 → 충돌 → 자동 정지
    // 충돌하면 true를 돌려주고 그 프레임에서 끝 (자동 정지 판단은 안 함)
    function stepSimulation(dt) {
      simTimeS += dt;

      // 속도를 먼저 바꾸고, 그 속도로 차를 오른쪽으로 이동
      const speedBefore = car.speedMs;
      updateSpeed(dt);
      if (dt > 0) car.accelMs2 = (car.speedMs - speedBefore) / dt; // 실제 속도 변화로 현재 가속도 계산

      // 이번 프레임에 달린 거리(m) = 이전 속도와 새 속도의 평균 × 시간
      const stepM = (speedBefore + car.speedMs) / 2 * dt;
      car.xM += stepM;
      odometerM += stepM;
      addBrakeDistance(stepM);

      // 도로 오른쪽 끝을 완전히 지나가면 다시 왼쪽 밖에서 시작
      if (car.xM > ROAD_LENGTH_M) {
        car.xM = -CAR_LENGTH_M;
      }

      // 건너오는 보행자 걷기 (달리는 중에만 → 일시정지 중엔 멈춤)
      moveObstacles(dt);

      // 충돌 검사: 부딪혔으면 일시정지하고 이 프레임에서 끝 (다음 프레임 요청 안 함)
      if (checkCollision()) return true;

      // 추적 갱신. 이상적 센서 + 자동 정지 꺼짐이면 추적 기록이 필요 없어서 건너뛰고 비워 둠
      // (화면은 이상적 센서일 때 날 감지 결과를 직접 씀. 자동 정지를 켜면 그 스텝에서 바로 측정·확인됨)
      if (isIdealSensors() && !aebEnabled) clearTracks();
      else updateTracks(dt);

      // 자동 정지 판단 (다음 프레임부터 브레이크에 반영)
      updateAEB();
      return false;
    }

    // ===== 바깥에 돌려줄 sim 상자 =====
    // 바뀌지 않는 상수, 객체·배열(처음 만든 것 그대로), 함수
    const sim = {
      // 상수
      DEFAULT_ACCELERATION, DEFAULT_DECELERATION, DEFAULT_BRAKE_DECELERATION, START_TARGET_KMH,
      DEFAULT_MASS_KG, DEFAULT_DRAG_CD, DEFAULT_FRONTAL_AREA_M2, DEFAULT_ROLLING_CRR, DEFAULT_ROAD_MU,
      DEFAULT_RADAR_FAR_RANGE_M, DEFAULT_RADAR_FAR_HALF_DEG, DEFAULT_RADAR_NEAR_RANGE_M,
      DEFAULT_RADAR_NEAR_HALF_DEG, DEFAULT_SAFETY_MARGIN_M, DEFAULT_LIDAR_RANGE_M, DEFAULT_LIDAR_RES_DEG,
      DEFAULT_CAMERA_RANGE_M, DEFAULT_CAMERA_HALF_DEG, AIR_DENSITY, GRAVITY, ROAD_LENGTH_M, LANE_WIDTH_M,
      LANE_COUNT, ROAD_WIDTH_M, MY_LANE_Y_M, CAR_LENGTH_M, CAR_WIDTH_M, CAR_HEIGHT_M, PED_SIZE_M,
      PED_HEIGHT_M, PED_WALK_SPEED_MS, MAX_OBSTACLES, ROADSIDE_LIMIT_M,
      DEFAULT_RADAR_PERIOD_MS, DEFAULT_LIDAR_PERIOD_MS, DEFAULT_CAMERA_PERIOD_MS, DEFAULT_CONFIRM_COUNT,
      DEFAULT_TRACK_MEMORY_S, REAL_SENSOR_VALUES, WEATHER_PRESETS, DEFAULT_SENSOR_FACTORS,
      // 객체·배열: 처음 만든 것을 계속 씀 (비울 때도 obstacles.length = 0처럼 새로 만들지 않음)
      car, obstacles, brakeStats, radar, lidar, camera, sensors, tracks, measureTimers,
      // 함수
      kmhToMs, msToKmh, isBrakeActive, computeForces, updateSpeed, checkBrakeChange, addBrakeDistance,
      moveObstacles, checkCollision, placeObstacle, placeObstacleAt, pathInfo, detectObjects, rayHitRect, scanLidar, detectAll,
      findDetection, detectedByChecked, nearestCandidate, requiredStopM, updateAEB, stepSimulation,
      sensorMode, isIdealSensors, updateTracks, clearTracks, trackGapM, nearestTrackCandidate, trackPathState, pedCrossVy,
      weatherMode, applyWeather, applyCustomWeather, skyMode, applySky, sensorFactor, effectiveLidarRangeM
    };

    // 바뀌는 값은 읽기·쓰기 통로(getter/setter)로 연결
    // → sim.MASS_KG = 2000처럼 바깥에서 바꾸면 안쪽 계산이 바로 그 값을 씀 (값은 안쪽 한 곳에만 있음)
    function link(name, get, set) {
      Object.defineProperty(sim, name, { get: get, set: set, enumerable: true });
    }
    link("ACCELERATION", function () { return ACCELERATION; }, function (v) { ACCELERATION = v; });
    link("DECELERATION", function () { return DECELERATION; }, function (v) { DECELERATION = v; });
    link("BRAKE_DECELERATION", function () { return BRAKE_DECELERATION; }, function (v) { BRAKE_DECELERATION = v; });
    link("MASS_KG", function () { return MASS_KG; }, function (v) { MASS_KG = v; });
    link("DRAG_CD", function () { return DRAG_CD; }, function (v) { DRAG_CD = v; });
    link("FRONTAL_AREA_M2", function () { return FRONTAL_AREA_M2; }, function (v) { FRONTAL_AREA_M2 = v; });
    link("ROLLING_CRR", function () { return ROLLING_CRR; }, function (v) { ROLLING_CRR = v; });
    link("ROAD_MU", function () { return ROAD_MU; }, function (v) { ROAD_MU = v; });
    link("customMu", function () { return customMu; }, function (v) { customMu = v; });
    // 날씨별 센서 비율 9개 (FACTOR_RAIN_RADAR, FACTOR_FOG_LIDAR 처럼). 맑음은 통로가 없음
    for (const sky of ["rain", "snow", "fog"]) {
      for (const name of ["radar", "lidar", "camera"]) {
        link("FACTOR_" + sky.toUpperCase() + "_" + name.toUpperCase(),
          function () { return sensorFactors[sky][name]; },
          function (v) { sensorFactors[sky][name] = v; });
      }
    }
    link("RADAR_FAR_RANGE_M", function () { return RADAR_FAR_RANGE_M; }, function (v) { RADAR_FAR_RANGE_M = v; });
    link("RADAR_FAR_HALF_DEG", function () { return RADAR_FAR_HALF_DEG; }, function (v) { RADAR_FAR_HALF_DEG = v; });
    link("RADAR_NEAR_RANGE_M", function () { return RADAR_NEAR_RANGE_M; }, function (v) { RADAR_NEAR_RANGE_M = v; });
    link("RADAR_NEAR_HALF_DEG", function () { return RADAR_NEAR_HALF_DEG; }, function (v) { RADAR_NEAR_HALF_DEG = v; });
    link("SAFETY_MARGIN_M", function () { return SAFETY_MARGIN_M; }, function (v) { SAFETY_MARGIN_M = v; });
    link("LIDAR_RANGE_M", function () { return LIDAR_RANGE_M; }, function (v) { LIDAR_RANGE_M = v; });
    link("LIDAR_RES_DEG", function () { return LIDAR_RES_DEG; }, function (v) { LIDAR_RES_DEG = v; });
    link("CAMERA_RANGE_M", function () { return CAMERA_RANGE_M; }, function (v) { CAMERA_RANGE_M = v; });
    link("CAMERA_HALF_DEG", function () { return CAMERA_HALF_DEG; }, function (v) { CAMERA_HALF_DEG = v; });
    link("RADAR_PERIOD_MS", function () { return RADAR_PERIOD_MS; }, function (v) { RADAR_PERIOD_MS = v; });
    link("LIDAR_PERIOD_MS", function () { return LIDAR_PERIOD_MS; }, function (v) { LIDAR_PERIOD_MS = v; });
    link("CAMERA_PERIOD_MS", function () { return CAMERA_PERIOD_MS; }, function (v) { CAMERA_PERIOD_MS = v; });
    link("CONFIRM_COUNT", function () { return CONFIRM_COUNT; }, function (v) { CONFIRM_COUNT = v; });
    link("TRACK_MEMORY_S", function () { return TRACK_MEMORY_S; }, function (v) { TRACK_MEMORY_S = v; });
    link("simTimeS", function () { return simTimeS; }, function (v) { simTimeS = v; });
    link("odometerM", function () { return odometerM; }, function (v) { odometerM = v; });
    link("braking", function () { return braking; }, function (v) { braking = v; });
    link("brakeLatched", function () { return brakeLatched; }, function (v) { brakeLatched = v; });
    link("autoBraking", function () { return autoBraking; }, function (v) { autoBraking = v; });
    link("aebEnabled", function () { return aebEnabled; }, function (v) { aebEnabled = v; });
    link("aebUseRadar", function () { return aebUseRadar; }, function (v) { aebUseRadar = v; });
    link("aebUseLidar", function () { return aebUseLidar; }, function (v) { aebUseLidar = v; });
    link("aebUseCamera", function () { return aebUseCamera; }, function (v) { aebUseCamera = v; });
    link("collided", function () { return collided; }, function (v) { collided = v; });
    link("crashKmh", function () { return crashKmh; }, function (v) { crashKmh = v; });
    // 목표 속도(km/h)는 car 안에 있음
    link("targetSpeedKmh", function () { return car.targetSpeedKmh; }, function (v) { car.targetSpeedKmh = v; });

    return sim;
  }

  // 내보내기: Node.js면 module.exports, 브라우저면 전역(window.createSim)
  if (typeof module !== "undefined" && module.exports) {
    module.exports = { createSim: createSim };
  } else {
    root.createSim = createSim;
  }
})(typeof window !== "undefined" ? window : globalThis);
