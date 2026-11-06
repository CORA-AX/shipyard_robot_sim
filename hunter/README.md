# Hunter 2.0 독립 개발

Hunter의 제어·ROS 통신·시뮬레이션 실행·설정·운용 USD·검사를 이 폴더에서 관리한다.
기존 `ros2_ws` 패키지나 다른 로봇의 Python 구현을 import하지 않는다.

## 구성

| 위치 | 역할 |
|---|---|
| `simulate.py` | GUI 시뮬레이터 실행 진입점 |
| `teleop.py` | 시스템 ROS Humble에서 실행할 명령 발행 진입점 |
| `config/robot.yaml` | 모델 경로, 차축 치수, 바퀴 반지름, Joint 이름과 주행 제한 |
| `control/controller.py` | Ackermann 계산과 속도·조향 변화율 제한 |
| `control/commands.py` | 수신 명령 유효성 검사·만료 처리 |
| `control/keyboard.py` | Hunter 전용 터미널 키보드 입력 |
| `ros/bridge.py` | cmd_vel 수신 및 odometry·joint_states·TF·clock 발행 |
| `ros/teleop.py` | 키보드 또는 고정 명령의 ROS 발행 |
| `sim/runtime.py` | 단일 물리 루프와 Joint 목표 적용 |
| `sim/scene.py` | USD 배치와 Joint 검증 |
| `sim/environment.py`, `sim/launcher.py` | Python·ROS 환경 분리와 프로세스·Kit 경로 관리 |
| `assets/hunter2_sim/` | 현재 운용 USD와 configuration 참조 레이어 |
| `tests/` | 제어·입력·ROS 명령·USD 구성·독립 실행 검사 |

## 외부 실행 환경

Isaac Sim 5.0, IsaacLab, PyTorch, PyYAML 및 ROS2 Humble은 설치된 환경을 사용한다.
IsaacLab은 현재 프로젝트의 `third_party/IsaacLab`에 editable 설치되어 있으므로 유지한다.
Python 환경이나 IsaacLab 소스 자체를 이 폴더에 복제하지 않는다.

기본 Isaac 환경은 상위 프로젝트의 `.conda/isaacsim-5.0`이다.
다른 위치의 환경을 사용하거나 이 폴더를 별도로 옮긴 경우
`HUNTER_ISAAC_PREFIX`에 해당 Conda 환경의 절대 경로를 지정한다.

## ROS 키보드 주행

프로젝트 루트에서 두 터미널을 사용한다. 이 명령은 별도의 GUI를 실행한다.

터미널 1 — Isaac Conda 환경:

```bash
cd /home/rodix-gpu/shipyard_robot_sim
conda activate /home/rodix-gpu/shipyard_robot_sim/.conda/isaacsim-5.0
export ROS_DOMAIN_ID=0
python hunter/simulate.py --ros
```

`HUNTER2_ROS_READY`가 나온 다음 터미널 2에서 조작한다.
시간 제한 없이 창을 닫거나 Ctrl+C로 종료할 때까지 실행한다.

`HUNTER2_BOOT`는 `starting_app` → `app_started` → `creating_scene` →
`loading_model` → `initializing_physics` 순서로 시작 단계를 알린다.
Kit의 `app ready`는 Hunter 주행 준비 완료를 의미하지 않는다.
첫 실행에서 그래픽·셰이더 캐시 초기화에 수 분이 걸릴 수 있으며,
상세 Kit 로그에는 `Waiting for RtPso async group async compilation`이 표시될 수 있다.
이때 모델·ROS 단계까지 도달했는지는 위 시작 단계로 구분한다.

터미널 2 — Isaac Conda를 활성화하지 않은 새 터미널:

```bash
cd /home/rodix-gpu/shipyard_robot_sim
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 hunter/teleop.py --keyboard --speed 0.5 --turn-speed 0.3
```

`ros2_ws/install/setup.bash`나 colcon build는 이 실행에 필요하지 않다.
기존 teleop이나 `ros2 topic pub` 발행기는 종료한 뒤 실행한다.

- W/S: 전진·후진 직진, 조향 중앙 복귀.
- W/S로 출발한 다음 A/D를 길게 누르기: 현재 이동 방향을 유지하며 좌우 조향.
- Space/X: 감속 정지. 정지 요청은 현재 조향을 유지한다.
- Q: teleop 종료와 정지 명령 발행.
- 입력이 0.6초 동안 없으면 정지 명령을 발행한다. 통신 수신 만료는 시뮬레이터에서 별도로 0.5초다.

현재 키보드는 터미널 반복 문자 방식이다. W를 누른 채 D를 눌렀다가 D만 놓으면
W 반복 입력이 재개되지 않아 멈출 수 있다. 이 경우 W도 놓았다가 다시 누른다.
분리 작업에서는 기존 입력 동작을 유지했다.

`--speed`는 차체 m/s, `--turn-speed`는 차체 yaw rate(rad/s)다.
조향각 직접 입력은 아니다. 기존 이름인 `--keyboard-speed`, `--keyboard-turn-speed`도 지원한다.
기본 namespace는 `hunter2_0`, 입력 토픽은 `/hunter2_0/cmd_vel`이다.

## 고정 명령과 상태 확인

Isaac 터미널에서 시간 제한 없이 0.5 m/s 직진:

```bash
python hunter/simulate.py --speed 0.5
```

ROS 모드에서 ROS 터미널로 일정한 좌회전 명령을 발행할 수도 있다:

```bash
ros2 topic pub -r 20 /hunter2_0/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.3}}'
```

별도 ROS 터미널에서 확인:

```bash
ros2 topic echo /hunter2_0/odom --field twist.twist
ros2 topic echo /hunter2_0/joint_states --once
```

시뮬레이터의 `HUNTER2_READY`에는 실제 USD 경로, 게인, 바퀴 반지름과 원통 근사 옵션이 나온다.
`HUNTER2_STATE`에는 목표/실제 조향각, 후륜 속도, 차체 속도와 yaw rate가 나온다.
`--config`, `--usd-path`, `--namespace`, `--cmd-vel-topic`으로 입력을 변경할 수 있다.
설정의 상대 USD 경로는 `hunter/` 기준이며, CLI의 상대 경로는 명령을 실행한 위치 기준이다.

## 모델과 기존 경로

`assets/hunter2_sim/`은 분리 시점의
`../assets/hunter2/usd/hunter2_sim.urdf/hunter2_sim/`을 참조 레이어와 함께 복사한 사본이다.
앞으로 이 실행기의 모델은 **이 폴더 안의 USD**를 수정한다.
기존 경로의 USD를 수정해도 새 사본에 자동 반영되지 않는다.

네 바퀴의 Cylinder 충돌과 숨김 상태, 기존 Mesh 충돌 비활성화 상태를 유지한다.
뒤쪽 Drive는 damping 100, stiffness 0, Max Force 100이다.
YAML의 후륜 속도 제한 8 rad/s는 실행 장면에만 적용된다.
저장된 Target Velocity는 실행 장면에서 0으로 초기화한다.
`Approximate Cylinders With Convex Meshes`는 새 Stage에서 명시적으로 끈다.
실행기는 원본 USD를 저장하지 않는다.

기존 `ros2_ws`의 Hunter 전용 제어기·설정·시뮬레이션 코드와
`scripts/hunter2/simulate.py`, 중복 제어 테스트는 제거했다.
Hunter 제어·실행·검사의 기준은 이 폴더다.
Go2·Lekiwi의 공통 코드는 유지하며, 공용 키보드의 Hunter 전용 선택 분기는 제거했다.
기존 모델은 `../assets/hunter2/`에, URDF 재생성·과거 모델 비교 도구는
`../scripts/hunter2/`에 그대로 보존한다.

## 검사

다음 명령은 Isaac 환경의 OpenUSD 라이브러리를 이용하는 **CPU 검사**다.
시뮬레이터 앱이나 GPU 물리 루프를 실행하지 않는다.

```bash
python3 hunter/tests/run_cpu.py
```

일반 Python에서도 제어·입력·명령·독립 실행 검사를 실행할 수 있다.
OpenUSD가 없으면 USD 검사만 생략된다.

```bash
python3 -m unittest discover -s hunter/tests -p 'test_*.py' -v
```

시뮬레이터를 시작하지 않는 실제 ROS 송수신 검사는 별도다:

```bash
python3 hunter/tests/check_ros_transport.py
```

이 검사는 전용 ROS domain 93을 사용하고, 시스템 Humble과 Isaac 번들 rclpy 사이를 확인한다.
로봇 상태는 검사 데이터이므로 물리 주행 검증을 대신하지 않는다.

## 다음 개발

분리 후 GUI에서 전후진·좌우 조향·정지를 재확인한다.
현재 odometry 선속도는 기존과 같이 루트 링크 질량중심의 속도다.
위치·속도 기준점 정리, waypoint 추종, 지도·센서 기반 장애물 회피는 후속 작업이다.
