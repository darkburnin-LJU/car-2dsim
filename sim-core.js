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
          { name: "원거리", rangeM: RADAR_FAR_RANGE_M, halfAngleDeg: RADAR_FAR_HALF_DEG,
            fill: "rgba(80,160,255,0.22)", line: "rgba(80,160,255,0.9)" },
          { name: "근거리", rangeM: RADAR_NEAR_RANGE_M, halfAngleDeg: RADAR_NEAR_HALF_DEG,
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
          { name: "카메라", rangeM: CAMERA_RANGE_M, halfAngleDeg: CAMERA_HALF_DEG,
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

    // 라이다: 0°~360°를 LIDAR_RES_DEG 간격 광선으로 훑음
    // 광선은 처음 닿은 장애물에서 끝남 → 뒤 물체는 못 봄(가려짐). 내 차는 장애물 목록에 없어서 자동으로 제외
    // 돌려주는 값: 감지 목록(장애물별 닿은 광선 수·가장 가까운 닿은 점 거리·각도), 그림용 닿은 점 목록
    function scanLidar(sensor) {
      const ox = sensor.xM, oy = sensor.yM;
      const rayCount = Math.floor(360 / LIDAR_RES_DEG + 1e-9); // 기본 0.4° → 900개
      const byObstacle = new Map();
      const hitPoints = [];

      // 속도 개선: 라이다 범위 안에 들어올 수 있는 장애물만 광선 계산에 넣음
      // 판단 기준은 장애물 가운데가 아니라 "직사각형에서 설치 위치와 가장 가까운 점"까지의 거리
      // (가운데는 범위 밖이어도 가장자리가 범위 안이면 광선이 닿을 수 있음)
      // 가장 가까운 점이 범위 밖이면 어떤 광선도 범위 안에서 닿을 수 없으므로 결과는 그대로
      const inRange = obstacles.filter(function (ob) {
        const gapX = Math.max(ob.xM - ob.lengthM / 2 - ox, 0, ox - (ob.xM + ob.lengthM / 2)); // 가로로 떨어진 거리
        const gapY = Math.max(ob.yM - ob.widthM / 2 - oy, 0, oy - (ob.yM + ob.widthM / 2)); // 세로로 떨어진 거리
        return Math.sqrt(gapX * gapX + gapY * gapY) <= LIDAR_RANGE_M;
      });
      if (inRange.length === 0) return { list: [], hitPoints: hitPoints }; // 범위 안에 아무것도 없음

      for (let i = 0; i < rayCount; i++) {
        const angleDeg = i * LIDAR_RES_DEG;
        const rad = angleDeg * Math.PI / 180;
        const dirX = Math.cos(rad), dirY = Math.sin(rad);

        // 이 광선이 처음 닿는 장애물 찾기
        let bestT = LIDAR_RANGE_M, bestOb = null;
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

    // 자동 정지: 체크된 센서가 "감지한" 물체로만 판단 (감지 못 한 물체는 경로 안이어도 무시)
    function updateAEB() {
      const before = autoBraking;
      if (!aebEnabled) {
        autoBraking = false;
      } else {
        const target = nearestCandidate(detectAll());
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

    // 한 프레임 계산 (그리기 없음). 화면은 실제 경과 시간(최대 MAX_DT), 테스트는 고정 0.01초를 dt로 넣음
    // 순서: 속도 → 이동 → 제동거리 누적 → 보행자 이동 → 충돌 → 자동 정지
    // 충돌하면 true를 돌려주고 그 프레임에서 끝 (자동 정지 판단은 안 함)
    function stepSimulation(dt) {
      // 속도를 먼저 바꾸고, 그 속도로 차를 오른쪽으로 이동
      const speedBefore = car.speedMs;
      updateSpeed(dt);
      if (dt > 0) car.accelMs2 = (car.speedMs - speedBefore) / dt; // 실제 속도 변화로 현재 가속도 계산

      // 이번 프레임에 달린 거리(m) = 이전 속도와 새 속도의 평균 × 시간
      const stepM = (speedBefore + car.speedMs) / 2 * dt;
      car.xM += stepM;
      addBrakeDistance(stepM);

      // 도로 오른쪽 끝을 완전히 지나가면 다시 왼쪽 밖에서 시작
      if (car.xM > ROAD_LENGTH_M) {
        car.xM = -CAR_LENGTH_M;
      }

      // 건너오는 보행자 걷기 (달리는 중에만 → 일시정지 중엔 멈춤)
      moveObstacles(dt);

      // 충돌 검사: 부딪혔으면 일시정지하고 이 프레임에서 끝 (다음 프레임 요청 안 함)
      if (checkCollision()) return true;

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
      PED_HEIGHT_M, PED_WALK_SPEED_MS, MAX_OBSTACLES,
      // 객체·배열: 처음 만든 것을 계속 씀 (비울 때도 obstacles.length = 0처럼 새로 만들지 않음)
      car, obstacles, brakeStats, radar, lidar, camera, sensors,
      // 함수
      kmhToMs, msToKmh, isBrakeActive, computeForces, updateSpeed, checkBrakeChange, addBrakeDistance,
      moveObstacles, checkCollision, placeObstacle, pathInfo, detectObjects, rayHitRect, scanLidar, detectAll,
      findDetection, detectedByChecked, nearestCandidate, requiredStopM, updateAEB, stepSimulation
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
    link("RADAR_FAR_RANGE_M", function () { return RADAR_FAR_RANGE_M; }, function (v) { RADAR_FAR_RANGE_M = v; });
    link("RADAR_FAR_HALF_DEG", function () { return RADAR_FAR_HALF_DEG; }, function (v) { RADAR_FAR_HALF_DEG = v; });
    link("RADAR_NEAR_RANGE_M", function () { return RADAR_NEAR_RANGE_M; }, function (v) { RADAR_NEAR_RANGE_M = v; });
    link("RADAR_NEAR_HALF_DEG", function () { return RADAR_NEAR_HALF_DEG; }, function (v) { RADAR_NEAR_HALF_DEG = v; });
    link("SAFETY_MARGIN_M", function () { return SAFETY_MARGIN_M; }, function (v) { SAFETY_MARGIN_M = v; });
    link("LIDAR_RANGE_M", function () { return LIDAR_RANGE_M; }, function (v) { LIDAR_RANGE_M = v; });
    link("LIDAR_RES_DEG", function () { return LIDAR_RES_DEG; }, function (v) { LIDAR_RES_DEG = v; });
    link("CAMERA_RANGE_M", function () { return CAMERA_RANGE_M; }, function (v) { CAMERA_RANGE_M = v; });
    link("CAMERA_HALF_DEG", function () { return CAMERA_HALF_DEG; }, function (v) { CAMERA_HALF_DEG = v; });
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
