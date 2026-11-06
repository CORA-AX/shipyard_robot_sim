# 파일·알고리즘 안내

[개요](../README.md) · [실행 방법](usage.md)

아래 경로는 모두 `hunter/` 기준이다.

## 실행 파일 — `scripts/`

| 파일 | 역할 | 사용 기술 |
| --- | --- | --- |
| `simulate.py` | 시뮬레이터 실행 진입점 | `sim.runtime` 호출 |
| `teleop.py` | 수동 운전 진입점 | `ros.teleop` 호출 |
| `waypoint.py` | 자체 단일 목표 주행 진입점 | `ros.custom_waypoint` 호출 |
| `nav2.py` | RViz 클릭 주행 진입점 | `ros.nav2.runner` 호출 |
| `patrol.py` | 저장 경로 순찰 진입점 | 같은 runner에 shipyard·route 기본값 전달 |
| `export_map.py` | 지도 생성 환경 선택·실행 | OpenUSD CPU 라이브러리, subprocess |
| `setup_ros.sh` | 시스템 Python용 Ackermann 메시지 준비 | ROS apt 패키지 다운로드·로컬 추출 |
| `setup_nav2.sh` | ROS 준비 후 Nav2 설치 도구 실행 | Bash, `tools/setup_nav2.py` |
| `ros_env.bash` | Humble·로컬 메시지 경로 설정 | Bash source, AMENT/Python/라이브러리 경로 |
| `nav2_env.bash` | 로컬 Nav2 경로 추가 | `ros_env.bash`, Nav2/OMPL 라이브러리 경로 |

## 차량 제어 — `control/`

| 파일 | 역할 | 알고리즘·라이브러리 |
| --- | --- | --- |
| `controller.py` | 차량 설정 검사, 좌우 조향각·후륜 속도 목표 계산 | Ackermann 기구학, 내·외륜 차등, 가속도·조향 변화율 제한; math, PyYAML |
| `commands.py` | 속도·조향 명령과 수신 시각 관리 | 유효값 검사, 수신 만료 watchdog; dataclasses |
| `keyboard.py` | 키 입력을 속도·가상 조향각으로 변환 | 터미널 반복 입력·만료; termios/select |

입력은 후륜 중앙 속도와 가상 앞바퀴 조향각이다. 실제 좌우 앞바퀴 각도는 다르게 계산한다.
후륜은 속도 목표, 조향 관절은 위치 목표를 사용한다. 앞바퀴 회전 관절은 수동 구동하지 않는다.

## 경로 계산 — `navigation/`

| 파일 | 역할 | 알고리즘·라이브러리 |
| --- | --- | --- |
| `state.py` | 공통 Pose·Observation·Segment, 각도 정규화·곡률 이동 | 평면 기구학, 일정 곡률 적분; math/dataclasses |
| `custom/path.py` | 자체 전진 경로 생성·샘플링·진행 위치 투영 | Dubins CSC/CCC 후보 비교, 목표 앞 직선 접근; 직접 구현 |
| `custom/follower.py` | 자체 추종과 단일 목표 상태 관리 | Pure Pursuit, 곡률·접근 감속, 도착·정지·시간 cost; 직접 구현 |
| `nav2/config.py` | 차량 한계로 Nav2/RViz 설정 생성, Twist 변환 | 회전반경·footprint, δ=atan(Lω/v); PyYAML |
| `nav2/position_goal.py` | 위치 전용 목표의 도착 방향 후보·경로 거리 계산 | 12개 균등 방향 + 현재/목표 방향·회전 원 접선, 유한 후보 검색 |
| `routes.py` | 왕복 목록·시작점 검증, 순찰용 계획 여유 폭 | waypoint 목록, 전체 차량 영역 검사; PyYAML |
| `maps.py` | 지도 읽기·원본 변경·지지면·차량 영역 검사 | PGM, SHA-256, 사각형/격자 SAT 겹침 검사 |

`custom/`은 math 기반 자체 구현이다. `nav2/`는 Nav2 설정과 보조 계산이며 계획기를 다시 구현하지 않는다.
공통 상태는 `state.py`에서 공유하므로 Nav2가 자체 Dubins·Pure Pursuit 구현을 import하지 않는다.

## ROS 연결 — `ros/`

| 파일 | 역할 | 사용 기술 |
| --- | --- | --- |
| `bridge.py` | 명령 수신, odom·관절·TF·환경·시계 발행 | rclpy, Ackermann/nav/sensor/std/rosgraph/tf2 메시지 |
| `odometry.py` | 물리 차체 원점 → 후륜 중앙 변환, ROS odom 해석 | 강체 변환, v_point=v_link+ω×offset, quaternion; ROS import 없이 계산 |
| `teleop.py` | 키보드/고정 명령을 ROS로 발행 | rclpy, AckermannDriveStamped |
| `custom_waypoint.py` | 자체 추종기의 ROS 루프·결과 기록 | Mission + odom + Ackermann 명령, JSON |
| `nav2/goal.py` | 클릭 목표 검사·후보 비교·실행·취소·정지 확인 | rclpy action/service, ComputePathToPose·FollowPath·NavigateToPose |
| `nav2/patrol.py` | 저장 목록 순차 실행·복귀·전체 취소·누적 비용 | `HunterNav2Goal` 재사용, 도착·정지 후 다음 목표 전달 |
| `nav2/runner.py` | Nav2 서버·lifecycle·RViz 시작/종료 | subprocess, 프로세스 그룹·종료 신호, 세션 로그 |
| `nav2/rviz.py` | RViz 자식 프로세스의 환경 구성 | Qt/Conda 경로 정리, 선택적 Mesa 소프트웨어 렌더링 |

Nav2의 **Smac Hybrid-A***는 전진 Dubins 운동 모델로 경로를 만들고,
**Regulated Pure Pursuit(RPP)**는 그 경로를 추종한다. 중간 `Twist`는 속도·조향으로 변환한다.
최종 `ackermann_cmd`는 Hunter 목표 관리 노드 하나가 발행한다.

## 시뮬레이션·환경

| 파일 | 역할 | 사용 기술 |
| --- | --- | --- |
| `sim/runtime.py` | CLI·Isaac 시작·단일 물리 루프·관절 목표 적용 | IsaacLab AppLauncher/Articulation, Torch; 200Hz 물리·50Hz 제어 |
| `sim/scene.py` | 로봇 USD 배치·관절·차축 치수·collider 검사 | OpenUSD, IsaacLab |
| `sim/world.py` | 평지 또는 조선소 배치·환경 합성 검사 | IsaacLab GroundPlane/UsdFile, OpenUSD |
| `sim/environment.py` | Isaac와 시스템 ROS Python 경로 분리 | 환경 변수·ABI 경로 관리 |
| `sim/launcher.py` | Kit 설정·로그·프로세스 종료 관리 | subprocess, signal |
| `environments/config.py` | 시뮬레이터·지도·RViz 공통 환경 프로필 | dataclasses, PyYAML |
| `environments/raster.py` | 충돌체에서 지지면·장애물 지도 생성 | OpenUSD, NumPy, 삼각형 rasterization·SAT·연결 영역 탐색 |
| `tools/setup_nav2.py` | Nav2 다운로드·로컬 설치·라이브러리 검사 | apt-get, dpkg-deb, ldd; sudo 없이 `.runtime/`에 설치 |

시각 바퀴는 유지하고 물리 충돌은 숨겨진 원통 collider를 사용한다.
runtime은 원통의 convex mesh 근사를 끄고 저장된 drive target을 실행 시 초기화한다. USD 자체는 저장하지 않는다.

## 설정·데이터

| 파일 | 내용 |
| --- | --- |
| `config/robot.yaml` | USD 경로, 관절 이름·부호, 바퀴·차축 치수, 속도·조향·가속도 한계 |
| `config/navigation.yaml` | 자체 Pure Pursuit lookahead·접근 거리·도착 오차·속도 |
| `config/nav2.yaml` | Smac DUBIN·RPP·goal checker·정적 costmap |
| `config/nav2_tree.xml` | ComputePathToPose → FollowPath 행동 트리; spin/backup 복구 없음 |
| `config/environments/flat.yaml` | 평지 spawn·카메라·빈 시험 지도 범위 |
| `config/environments/shipyard.yaml` | 조선소 spawn·카메라·USD·지도 범위 |
| `config/routes/shipyard.yaml` | 후륜 중앙 기준 출발점, `smoke`·`flat_only` 방문 목록 |
| `assets/hunter2_sim/hunter2_sim.usd` | 현재 운용 로봇의 USD 진입 레이어 |
| `assets/hunter2_sim/configuration/hunter2_sim_base.usd` | 분리된 base 구성 레이어 |
| `assets/hunter2_sim/configuration/hunter2_sim_robot.usd` | 분리된 robot 구성 레이어 |
| `assets/hunter2_sim/configuration/hunter2_sim_physics.usd` | 분리된 physics 구성 레이어 |
| `assets/hunter2_sim/configuration/hunter2_sim_sensor.usd` | 분리된 sensor 구성 레이어; 센서 인지 기능 구현 여부와 별개 |
| `assets/environments/shipyard.usda` | 공용 로컬 layout/physics/routes 참조; 원격 warehouse 제외 |
| `maps/shipyard.yaml` | 지도 이미지 경로·원점·해상도 |
| `maps/shipyard.pgm` | free/occupied/unknown 지도 이미지 |
| `maps/shipyard.json` | 입력 USD·생성 코드 해시, 지도 생성 근거 |

## 검사 — `tests/`

| 파일 | 검사 대상 |
| --- | --- |
| `run_cpu.py` | OpenUSD로 단위 검사 실행; Kit/물리 시작 없음 |
| `test_control.py` | Ackermann 내·외륜 계산·변화율·제한 |
| `test_keyboard.py` | 입력 전환·만료·터미널 복구 |
| `test_ros.py` | 명령 유효성·watchdog·ROS 옵션 |
| `test_odometry.py` | 기준점 변환·회전 중 속도 |
| `test_scene.py` | 실제 USD 관절·크기·collider·drive |
| `test_navigation.py` | 자체 Dubins 기하와 Pure Pursuit 폐루프 |
| `test_nav2.py` | Nav2 설정·Twist 변환·방향 후보·RViz |
| `test_routes.py` | 왕복 목록·시작점·차량 영역·여유 폭 |
| `test_world.py` | 환경 합성·지도 재현성·지지면·해시 |
| `test_isolation.py` | 다른 로봇과의 독립성·폴더 이동·실행 진입점 |
| `check_ros_transport.py` | 실제 DDS·명령·Isaac/System Python 연결 |
| `check_waypoint_ros.py` | 자체 추종기 + ROS + CPU bicycle 폐루프 |
| `check_nav2_ros.py` | 실제 Smac/RPP + CPU bicycle; 클릭·순찰·취소·통신 오류 |

모든 `__init__.py`는 Python 패키지 경계를 나타낸다. `.gitignore`는 캐시·런타임·결과를 제외한다.
`README.md`는 개요, `docs/usage.md`는 사용법, 이 문서는 파일 사전이다.
`outputs/`의 JSON은 주행 결과, `.runtime/`은 로컬 의존성·세션 로그다. 두 디렉터리는 소스가 아니다.
