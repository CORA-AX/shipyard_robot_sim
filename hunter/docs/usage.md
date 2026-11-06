# 실행·조작 안내

[개요](../README.md) · [파일과 알고리즘](files.md)

아래 명령은 `/home/rodix-gpu/shipyard_robot_sim`에서 실행한다.
시뮬레이터는 Isaac Conda 터미널, ROS 제어기는 **Conda를 활성화하지 않은 새 터미널**을 사용한다.
각 터미널의 `ROS_DOMAIN_ID`는 같게 설정한다. 기존 teleop/Nav2 등 다른 명령 발행기는 종료한다.

## 1. 환경 준비 — 최초 한 번

Isaac Sim 5.0 Conda, IsaacLab, 시스템 ROS2 Humble이 설치되어 있어야 한다. 일반 터미널에서:

```bash
cd /home/rodix-gpu/shipyard_robot_sim
bash hunter/scripts/setup_ros.sh
bash hunter/scripts/setup_nav2.sh
```

메시지·Nav2 의존성은 `hunter/.runtime/`에 준비한다. 시스템 패키지를 설치하거나 colcon build하지 않는다.
일반 ROS 터미널은 `source hunter/scripts/ros_env.bash`,
Nav2 터미널은 `source hunter/scripts/nav2_env.bash`로 환경을 읽는다. source는 설치 명령이 아니다.
별도 Isaac Conda 위치는 `HUNTER_ISAAC_PREFIX`에 지정하고 해당 환경을 활성화한다.

## 2. 시뮬레이터 — 터미널 1

```bash
cd /home/rodix-gpu/shipyard_robot_sim
conda activate /home/rodix-gpu/shipyard_robot_sim/.conda/isaacsim-5.0
export ROS_DOMAIN_ID=0
python hunter/scripts/simulate.py --ros --environment shipyard
```

- `HUNTER2_ROS_READY`가 나오면 ROS 주행 준비가 된 상태다. 첫 그래픽 초기화는 시간이 걸릴 수 있다.
- 빈 평지는 `--environment flat`을 사용한다. 생략하면 `flat`이다.
- 기본 실행 시간제한은 없다. 창을 닫거나 `Ctrl+C`로 종료한다.
- ROS 없이 직진만 확인하려면 `python hunter/scripts/simulate.py --speed 0.5`를 실행한다.
- 운용 모델은 `hunter/assets/hunter2_sim/`이다. 실행기가 USD를 덮어쓰지 않는다.
  YAML의 상대 경로는 `hunter/`, CLI에 넘긴 상대 경로는 명령을 실행한 위치를 기준으로 한다.

## 3. 운전 방식 선택 — 터미널 2

아래 모드 중 하나만 실행한다. 모든 예시는 프로젝트 루트에서 실행한다.

### 키보드 수동 운전

```bash
source hunter/scripts/ros_env.bash
export ROS_DOMAIN_ID=0
/usr/bin/python3 hunter/scripts/teleop.py --keyboard --speed 0.5
```

| 키 | 동작 |
| --- | --- |
| W / S | 전진 / 후진, 조향 중앙 복귀 |
| A / D | 현재 이동 방향을 유지하며 좌 / 우 조향 |
| C | 이동 속도를 유지하며 조향 중앙 복귀 |
| Space / X | 정지 요청, 현재 조향 유지 |
| Q | 정지 요청 후 teleop 종료 |

기본 조향각은 0.3rad이며 `--keyboard-steering-angle`로 바꾼다. 정지 중에도 A/D 조향은 가능하다.
터미널은 반복 문자 입력 방식이다. W를 누른 채 D를 눌렀다가 D만 놓으면 W 반복 입력이 재개되지 않아
멈출 수 있으므로 W도 놓았다 다시 누른다. 입력 만료는 기본 0.6초, 시뮬레이터의 명령 수신 만료는 0.5초다.
고정 명령은 `teleop.py --drive-speed 0.5 --steering-angle 0.3`처럼 실행한다.

### RViz 클릭 목표 주행 — Nav2

```bash
source hunter/scripts/nav2_env.bash
export ROS_DOMAIN_ID=0
/usr/bin/python3 hunter/scripts/nav2.py --environment shipyard --speed 0.4
```

시뮬레이터와 `--environment`를 일치시킨다. `HUNTER_NAV2_READY` 이후 RViz에서:

1. 상단 **Publish Point**를 선택하고 흰색 지지면을 클릭하면 **위치만** 목표로 준다.
2. **2D Goal Pose**를 선택하고 클릭·드래그하면 **위치와 도착 방향**을 준다.
3. 한 목표가 끝나고 정지한 다음 새 목표를 지정한다. 여러 클릭을 대기열에 쌓지는 않는다.

처음에는 `(-22,18)`, 도착 후 `(-25,12)` 부근으로 시험할 수 있다.
흰색은 주행 가능 지지면, 검정은 정적 장애물, 회색은 미확인/지원되지 않는 바닥이다.
격자는 거리 참고선이며 주황색 사각형은 지도 범위다. 실제 계획에는 차량 전체 크기도 반영한다.
초록색은 선택된 경로이며 차량 외형과 접촉 물리는 Isaac 창에서 본다.

위치 전용 목표는 최대 16개 도착 방향 후보에 대해 Smac 경로를 계산하고 유효한 후보 중 XY 거리가 가장 짧은
경로를 그대로 RPP에 전달한다. 방향을 요구하는 목표는 `NavigateToPose`를 사용한다.
Nav2 성공 후 위치 오차≤0.15m, 실제 속도≤0.03m/s, yaw rate≤0.05rad/s를 0.3초 확인한다.
방향 지정 목표는 방향 오차≤0.15rad도 확인한다.

기본 RViz는 지도 GLSL 오류를 피하기 위해 소프트웨어 렌더링을 사용한다. Isaac 렌더러에는 적용하지 않는다.
필요하면 `--rviz-rendering native`를 선택한다. `--no-rviz`는 Nav2만 실행하며 Isaac을 실행하는 옵션이 아니다.

### 저장 waypoint 순차 왕복 — Nav2

```bash
source hunter/scripts/nav2_env.bash
export ROS_DOMAIN_ID=0
/usr/bin/python3 hunter/scripts/patrol.py --route smoke --speed 0.4
```

초기 출발 위치 0.4m 이내에서 정지해 있어야 한다. 준비가 끝나면 자동 출발한다.
각 지점에서 도착·정지를 확인한 후 다음 지점으로 이동하며 마지막에는 출발 **위치**로 복귀한다.
마지막 방향은 지정하지 않는다. 완료 후에는 정지 상태를 유지한다.

| 경로 | 방문 순서 |
| --- | --- |
| `smoke` (기본값) | 출발점 약 `(-15.72,18)` → 북쪽 도로 `(-22,18)` → 출발점 |
| `flat_only` | 출발점 → 북쪽 교차로 `(-25,18)` → 우회로 `(-25,-1)` → 남쪽 교차로 `(-25,-20)` → 역순 복귀 |

전체 순찰 명령은 다음과 같다. waypoint 직선 거리의 합은 약 95m이며 실제 곡선 주행은 더 길 수 있다.

```bash
/usr/bin/python3 hunter/scripts/patrol.py --route flat_only --speed 0.4
```

방문 지점은 `config/routes/shipyard.yaml`에서 수정한다. Go2 평지 경로의 주요 지점을 사용하며,
각 지점 사이를 Hunter의 Nav2가 연결한다. 기본 2m 간격으로 나눈 모든 점에서 정지하지 않는다.
순찰용 footprint padding은 0.20m, 클릭 모드는 0.02m다. 순찰은 지지면이 없는 영역 주변의 비용과
경로 후보 전체의 차량 영역을 추가 검사한다. RViz의 보라색 연결선은 방문 순서, 초록색은 실제 선택 경로다.
순찰 모드에서는 RViz 클릭 목표를 받지 않는다.

`--route-idle`을 추가하면 자동 출발하지 않는다. 별도 ROS 터미널에서 시작/취소할 수 있다.

```bash
source hunter/scripts/nav2_env.bash
export ROS_DOMAIN_ID=0
ros2 service call /hunter2_0/patrol/start std_srvs/srv/Trigger '{}'
ros2 service call /hunter2_0/patrol/cancel std_srvs/srv/Trigger '{}'
```

취소·실패 후 이전 대기열은 자동 재개하지 않는다. 중간 취소 후 재시작하려면 초기 출발 위치로 복귀/리셋한다.

### 자체 Dubins + Pure Pursuit 단일 목표

학습·비교용 구현이다. 장애물 없는 `flat` 시뮬레이터를 사용한다.

```bash
source hunter/scripts/ros_env.bash
export ROS_DOMAIN_ID=0
/usr/bin/python3 hunter/scripts/waypoint.py --x 4.0 --y 3.0 --yaw-deg 90 --speed 0.4
```

좌표는 odom 절대 좌표, 기준은 후륜 중앙이다. 자체 구현에는 장애물 회피가 없다.
목표 도착 후 결과를 기록하고 제어기는 종료한다. Isaac은 별도로 계속 실행된다.

## 4. 상태·취소·시간 cost

별도 ROS 터미널에서 `source hunter/scripts/nav2_env.bash`, `export ROS_DOMAIN_ID=0` 이후:

```bash
ros2 topic echo /hunter2_0/odom --field twist.twist
ros2 topic echo /hunter2_0/joint_states --once
ros2 topic echo /hunter2_0/nav2/status
ros2 topic echo /hunter2_0/patrol/status
ros2 service call /hunter2_0/nav2/cancel std_srvs/srv/Trigger '{}'
```

`nav2/cancel`은 클릭 목표 또는 실행 중인 전체 순찰을 취소한다. 제어기 터미널의 `Ctrl+C`도 정지 명령을 보낸다.
실행 중 새 목표, 다른 명령 발행기, 오래된 odom, 환경 불일치, reset 등을 검사한다.
Nav2 종료는 자신의 Nav2/RViz 프로세스를 정리하며 별도로 실행한 Isaac은 종료하지 않는다.

| 결과 위치 (`hunter/` 기준) | 내용 |
| --- | --- |
| `outputs/waypoint/<timestamp>.json` | 자체 추종 결과 |
| `outputs/nav2/<timestamp>.json` | Nav2 목표별 시간·오차·주행 거리·방향 후보 |
| `outputs/patrol/<timestamp>.json` | 전체 왕복 시간·지점별 결과·완료 개수 |
| `.runtime/nav2/sessions/<timestamp>/` | Nav2/RViz 로그·실행 파라미터 |

`time_cost_s`는 목표 수락/순찰 시작부터 실제 도착·정지 확인까지의 **시뮬레이션 경과 시간**이다.
경로 계산 대기·가감속·정지 확인을 포함한다. 실패·취소의 성공 비용은 `null`이다.
후보 선택은 경로 거리 기준이며, 시간 비용을 최소화하는 최적화는 아직 구현하지 않았다.

## 5. 좌표·ROS 인터페이스

`base_link`는 후륜 축 중앙, `chassis_link`는 물리 USD의 차체 원점이다. x는 전방, y는 좌측, z는 위쪽이다.
대표 TF는 `map → hunter2_0/odom → hunter2_0/base_link → hunter2_0/chassis_link`다.
위치는 Isaac ground truth이며 `map → odom`은 identity다. 속도는 차체 회전으로 생기는 기준점 차이도 반영한다.

| 이름 (기본 namespace `hunter2_0`) | 형식·의미 |
| --- | --- |
| `/hunter2_0/ackermann_cmd` | `AckermannDriveStamped`: 후륜 중앙 속도 m/s, 가상 조향각 rad(좌측 양수) |
| `/hunter2_0/odom` | `Odometry`: 후륜 중앙 위치·방향·속도 |
| `/hunter2_0/joint_states` | `JointState`: 실제 관절 상태, 회전 위치 rad·속도 rad/s |
| `/hunter2_0/environment`, `/hunter2_0/reset_count` | 환경 이름·리셋 번호 |
| `/clock`, `/tf`, `/tf_static` | 시뮬레이션 시간·좌표 변환 |
| `/hunter2_0/nav2/goal_point` | `PointStamped`: 위치 전용 목표 |
| `/hunter2_0/nav2/goal_pose` | `PoseStamped`: 위치·방향 목표 |
| `/hunter2_0/nav2/cmd_vel` | Nav2 내부 `Twist`: 어댑터가 속도·조향으로 변환 |
| `/hunter2_0/nav2/selected_path` | `Path`: 최종 선택 경로 |
| `/hunter2_0/nav2/status`, `/hunter2_0/patrol/status` | JSON을 담은 `String` 상태 |

차량 한계는 `config/robot.yaml`에서 관리한다. 현재 속도 한계 1m/s, 가상 조향 한계 0.46rad,
가속도 한계 0.5m/s²다. 자동 주행은 전진만 사용하고 제자리 회전을 요청하지 않는다.

## 6. 지도 갱신·검사

조선소는 공용 `assets/environments/shipyard/`의 로컬 layout/physics/routes 레이어를 참조한다.
원격 warehouse 참조와 무한 바닥은 추가하지 않는다. 지도는 지지면·정적 충돌 형상으로 만들며
수면·도크 구멍·경사로·계단, 평지와 연결되지 않은 바닥을 주행 가능 평지로 간주하지 않는다.
범위 x=[-45,25], y=[-32,28]m, 해상도 0.05m다. 환경을 수정하면 일반 터미널에서:

```bash
python3 hunter/scripts/export_map.py --environment shipyard
```

원본 해시로 오래된 지도를 검사한다. 갱신 후 시뮬레이터와 Nav2를 재시작한다.
지도 생성은 OpenUSD CPU 작업이며 Isaac 앱·물리를 시작하지 않는다.

```bash
python3 hunter/tests/run_cpu.py
python3 hunter/tests/check_ros_transport.py
python3 hunter/tests/check_waypoint_ros.py
python3 hunter/tests/check_nav2_ros.py
python3 hunter/tests/check_nav2_ros.py --environment shipyard
python3 hunter/tests/check_nav2_ros.py --patrol
python3 hunter/tests/check_nav2_ros.py --patrol --patrol-route flat_only
```

CPU 검사는 제어·USD·지도·독립 실행을 확인한다. ROS 검사는 실제 DDS/Nav2와 CPU bicycle 모델을 사용한다.
검사 domain은 transport=93, 자체 waypoint=94, Nav2 평지=95, 조선소=96, 짧은 순찰=97, 전체 순찰=98이다.
같은 검사를 동시에 중복 실행하지 않는다. 이 검사들은 Isaac 접촉 물리·RViz GUI를 실행하지 않는다.
