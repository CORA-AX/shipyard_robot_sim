# ROS 2 Humble workspace

현재 `robot_control`, `robot_sim`, `robot_bringup` 세 `ament_python` 패키지를 구현했다.
시스템 Humble의 Python 3.10 launch/명령 노드와 Isaac Sim 5.0의 Python 3.11을
별도 프로세스로 실행하고 DDS로 연결한다. 현재 지원 범위는 **warehouse 또는 flat에서 Go2/Lekiwi 단독 실행 및 Go2+Lekiwi 동시 실행**다.
navigation·상위 플래너 확장 설계는 [DESIGN.md](DESIGN.md)를 참고한다.

## 빌드

프로젝트 루트에서 실행한다. 이 서버에는 Humble이 설치되어 있다.
시스템 colcon이 없으면 `build.sh`는 프로젝트의 `.runtime/ros2_build_tools`를 사용한다.

```bash
bash ros2_ws/build.sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

다른 서버에서는 Humble과 `python3-colcon-common-extensions`를 준비하거나,
프로젝트 내부 빌드 도구를 다음과 같이 설치한다.

```bash
/usr/bin/python3 -m pip install --target .runtime/ros2_build_tools \
  colcon-common-extensions 'setuptools<81'
bash ros2_ws/build.sh
```

`build/`, `install/`, `log/`는 생성물이므로 Git에서 제외한다.
소스 수정은 `--symlink-install`로 실행 코드에 반영된다.
새 파일·console entrypoint·패키지 의존성을 추가하면 다시 빌드한다.

## 실행

두 터미널에서 각각 시스템 Humble와 이 workspace를 source한다.
ROS 명령을 실행하는 터미널에서는 Isaac Conda 활성화가 필요하지 않다.

```bash
# 터미널 1: warehouse + Go2 + 로컬 창
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch robot_bringup simulation.launch.py

# 화면 없이 실행
ros2 launch robot_bringup simulation.launch.py display:=headless

# WebRTC
ros2 launch robot_bringup simulation.launch.py display:=webrtc server_ip:=192.168.1.149
```

`GO2_ROS_READY` 또는 `LEKIWI_ROS_READY` 이후 명령을 보낸다.
첫 로컬 실행은 warehouse 재질·shader 준비 때문에 시간이 걸릴 수 있다.
준비 중에는 `GO2_ROS_PREPARING`을 출력하며, 준비 완료 후 ROS 명령 수신을 시작한다.

```bash
# 터미널 2: 키보드
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run robot_control teleop --robot-id go2_0 --keyboard

# 전진 5초, 이후 정지
ros2 run robot_control teleop --robot-id go2_0 --vx 0.4 --duration 5

# 다른 namespace를 사용한 경우
ros2 run robot_control teleop --namespace validation/go2_0 --keyboard

# 상태 확인
ros2 topic echo /go2_0/odom
ros2 topic echo /go2_0/joint_states
ros2 topic echo /clock --qos-reliability best_effort
```

키보드는 W/S 전후, A/D 횡이동, Z/C 회전, Space/X 정지, Q/Ctrl+C 종료다.
포커스는 실행 터미널에 둔다. 키 입력이 0.6초 없으면 목표 속도가 0이 된다.
기본 이동/회전 속도는 각각 1.5 m/s, 1.5 rad/s이며 속도 상한은 적용하지 않는다.
`--keyboard-speed`, `--keyboard-turn-speed`, `--keyboard-timeout`으로 변경한다.
공통 기본값은 `src/robot_control/robot_control/keyboard.py`에서 수정한다.
teleop 종료 시 명시적인 정지 명령을 보내며, abrupt 종료는 simulator의 명령 만료가 처리한다.

## RViz: 로컬 창과 원격 브라우저

Isaac Sim의 `display`와 RViz의 `rviz_display`를 별도로 선택한다.
`rviz_display:=auto`가 기본이며 `display:=local`에서는 로컬 RViz 창,
`display:=webrtc`에서는 원격 브라우저 RViz, `display:=headless`에서는 RViz를 끈다.
RViz가 필요 없으면 어느 모드에서든 `rviz_display:=off`를 지정한다.

원격 화면 도구는 Ubuntu 22.04 시스템 패키지를 프로젝트 내부에 추출한다.
sudo나 시스템 패키지 변경 없이 최초 한 번 준비한다. Humble desktop과 시스템 그래픽 라이브러리가 필요하다.

```bash
bash ros2_ws/setup_rviz_web.sh
bash ros2_ws/build.sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

현재 서버에는 이 도구를 `.runtime/rviz_web/sysroot`에 준비했다.
설치 버전은 `.runtime/rviz_web/packages.json`에 기록한다.

**이미 Isaac Sim을 WebRTC로 보고 있다면** 새 시뮬레이터를 실행하지 않고 RViz만 붙인다.
`robots_file`과 `ros_domain_id`는 실행 중인 시뮬레이터와 맞춘다.

```bash
# 서버 SSH 터미널: Go2 단독 시뮬레이터에 RViz만 연결
ros2 launch robot_bringup rviz.launch.py rviz_display:=web

# Go2 + Lekiwi 시뮬레이터라면 위 명령 대신 실행
ros2 launch robot_bringup rviz.launch.py rviz_display:=web \
  robots_file:="$PWD/ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml"
```

접속 PC의 별도 터미널에서 SSH 터널을 열고 유지한다. VS Code SSH 포트 전달로
서버의 6080 포트를 Local Port 6080에 연결해도 된다.

```bash
ssh -N -L 6080:127.0.0.1:6080 rodix-gpu@192.168.1.149
```

접속 PC 브라우저에서 <http://localhost:6080/vnc.html?autoconnect=true&resize=scale>을 연다.
Isaac 영상은 기존 Streaming Client에서 계속 보고, RViz는 브라우저에서 조작한다.
RViz·ROS 노드는 서버에서 실행되므로 접속 PC에 ROS나 RViz를 설치할 필요가 없다.
VNC와 HTTP/WebSocket은 서버의 loopback에만 바인딩하며 SSH 터널로 접근한다.

처음부터 함께 띄울 때는 다음을 사용한다. 이미 실행 중인 시뮬레이터와 중복 실행하지 않는다.

```bash
# 로컬: Isaac Sim 창 + RViz 창
ros2 launch robot_bringup simulation.launch.py display:=local

# 원격: Isaac WebRTC + 브라우저 RViz
ros2 launch robot_bringup simulation.launch.py \
  display:=webrtc server_ip:=192.168.1.149

# Isaac 화면 없이 브라우저 RViz만 표시
ros2 launch robot_bringup simulation.launch.py display:=headless rviz_display:=web

# 기존처럼 Isaac만 실행
ros2 launch robot_bringup simulation.launch.py \
  display:=webrtc server_ip:=192.168.1.149 rviz_display:=off
```

RViz는 `map` 기준 Orbit 뷰와 0.5 m 간격 Grid, 전체 로봇의 odom/TF,
선택한 로봇의 목표 화살표를 표시하며 `use_sim_time=true`를 사용한다.
초기 카메라 거리는 6 m, TF Marker Scale은 1.5로 설정해 로봇 주변을 크게 보여 준다.
화면에서 `Views → Current View → Distance`를 줄이면 더 가까이 볼 수 있고
평행 이동·줌의 이동 폭도 작아진다. `Robot frames → Marker Scale`로 TF 크기를 조정한다.
기본값은 `src/robot_bringup/config/robots.rviz`에서 변경한다.
설정 파일을 바꾼 뒤에는 RViz launch만 다시 실행하면 된다.
툴바의 **2D Goal Pose** 또는 G 키를 누른 뒤 렌더링 영역에서 클릭·드래그하면
공통 `/waypoint/goal_pose`에 `geometry_msgs/PoseStamped`를 발행한다.
실제 추종 로봇은 아래 waypoint 노드의 터미널에서 Space로 선택한다.
`rviz_robot`은 초기 카메라 중심을 둘 로봇 id이며 PD 제어 대상은 변경하지 않는다.
`rviz_goal_topic`으로 공통 입력 토픽을 바꿀 수 있다. 이 경우 waypoint의 `--goal-topic`도 맞춘다.
TF/시계가 들어오기 전에는 상태 경고가 표시될 수 있다.

RViz의 `Waypoint selection and goal` 표시는 선택된 로봇·추종 상태·수락한 목표를 보여 준다.
waypoint 노드를 실행하지 않으면 좌표 입력만 발행하며 자동으로 이동하지 않는다.
warehouse 메시/점유 지도와 전체 로봇 메시도 RViz에 전송하지 않는다.
실제 warehouse 장면은 Isaac WebRTC에서 확인한다.

원격 모드는 기본적으로 Mesa 소프트웨어 렌더링을 사용하고 이 설정은 RViz에만 적용한다.
Isaac의 GPU 렌더링은 기존과 동일하다. 세션 로그·생성한 RViz 설정·PID는
`.runtime/rviz/view_*/`에 남는다. Ctrl+C 또는 RViz 창 종료 시 해당 세션의 화면 도구도 정리한다.
RViz 창을 정상 종료해도 함께 실행한 Isaac은 계속 실행된다.
원격 화면 도구가 비정상 종료하면 RViz를 정리하고 launch도 오류로 종료한다.

원격 화면 입력·ROS 수신·로컬 모드·정상/오류 종료 검증:

```bash
# 테스트 전용: Chrome이 설치된 서버에서 실행
/usr/bin/python3 -m pip install --target .runtime/rviz_test_tools playwright
/usr/bin/python3 tests/check_rviz_web.py
```

이 검사는 실제 ROS bridge에 합성 상태를 공급하며 Isaac/GPU 물리를 시작하지 않는다.
Chrome에서 noVNC에 접속해 실제 마우스로 목표를 찍고 `map` 좌표와 시뮬레이션 타임스탬프를 확인한다.
결과와 화면은 `.runtime/rviz_check/result.json`, `.runtime/rviz_check/browser.png`에 저장한다.

## PD waypoint 추종: Space로 로봇 선택

시뮬레이터·RViz와 별도 터미널에서 실행한다. Isaac Conda 활성화는 필요 없다.
**이번 변경 후에는 시뮬레이터와 RViz를 다시 실행한다.** 새 RViz 설정은 공통 목표 토픽을 사용하고,
새 simulator bridge는 자동/수동 리셋을 `reset_count`로 알린다.
waypoint 노드가 로봇별 최종 `cmd_vel`을 소유하므로 기존 teleop/다른 제어 송신기는 종료한다.
다른 `cmd_vel` 발행자가 감지되면 waypoint는 정지 명령을 보내고 오류로 종료한다.

```bash
bash ros2_ws/build.sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# 터미널 1: 공통 warehouse + 두 로봇 + 원격 RViz
ros2 launch robot_bringup simulation.launch.py \
  robots_file:="$PWD/ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml" \
  display:=webrtc server_ip:=192.168.1.149

# 터미널 2: 위 두 setup 파일을 source하고 실행
ros2 run robot_control waypoint
```

로컬에서는 터미널 1의 `display:=local`을 사용한다. 원격 브라우저/SSH 터널은 위 RViz 절차와 같다.
터미널 2에 포커스를 두고 다음 순서로 조작한다.

1. 기본 선택은 첫 Go2(`go2_0`)다. RViz에서 2D Goal Pose로 목표를 찍으면 Go2만 추종한다.
2. 터미널 2에서 **Space**를 누르면 다음 로봇(`lekiwi_0`)으로 선택이 바뀐다.
3. 이전 목표를 취소하고 모든 로봇에 0 속도를 보낸다. odom으로 이전 로봇의 정지를 확인한다.
4. RViz에서 새 목표를 찍으면 Lekiwi만 추종한다. 정지 확인 중 받은 새 목표는 그 후 실행한다.
5. Space를 다시 누르면 Go2로 돌아오며 **새 목표를 기다린다. 이전 목표는 재개하지 않는다.**

Space는 터미널에서 누른다. 브라우저/RViz에서 누르는 Space는 로봇 선택 입력이 아니다.
키를 누른 뒤 Enter는 필요 없다. 길게 누르거나 연타할 때는 0.3초 간격으로 전환한다.
**X/Esc**는 현재 목표 취소, **Q/Ctrl+C**는 모든 로봇에 정지 명령을 보내고 종료한다.
기존 teleop에서 Space가 정지인 동작과 별도 구현이다.

```bash
# 배치/namespace/로봇 수가 다르면 시뮬레이터와 같은 파일 지정
ros2 run robot_control waypoint \
  --robots-file "$PWD/ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml"

# Go2 단독
ros2 run robot_control waypoint \
  --robots-file "$PWD/ros2_ws/src/robot_bringup/config/go2_single.yaml"

# 초기 선택 변경 / 목표 방향까지 추종
ros2 run robot_control waypoint --robot lekiwi_0 --track-yaw

# 튜닝 파일과 실행 기록
ros2 run robot_control waypoint \
  --config "$PWD/ros2_ws/src/robot_control/config/waypoint.yaml" \
  --report "$PWD/.runtime/waypoint/run.json"
```

기본은 XY 위치 추종이며 목표를 수락한 시점의 yaw를 유지한다.
`--track-yaw`를 지정하면 RViz에서 드래그한 목표 yaw도 추종한다.
PD는 `Kp × 위치 오차 − Kd × 측정 속도`로 map 기준 목표 속도를 계산한 뒤 몸체 기준으로 변환한다.
속도 피드백 필터와 로봇별 속도/가속도 제한을 사용한다. D항은 측정 속도를 사용해 목표 교체 시
오차 미분이 튀는 현상을 피한다. 기존 Go2 보행 정책과 Lekiwi 바퀴 제어를 그대로 사용한다.

게인·속도 제한·도착 허용 오차는 `robot_control/config/waypoint.yaml`의 `profiles`에서 수정한다.
기본 최고 이동 속도는 Go2 0.45 m/s, Lekiwi 0.25 m/s이고 위치 허용 오차는 각각 0.10 m, 0.06 m다.
이 값은 자동 추종 전용이다. 위치·속도가 허용 범위에서 0.3/0.35초 유지되면 `REACHED`로 정지한다.
리셋·위치 순간 이동·시간 역행·0.75초 동안 새 odom 미수신·목표 시간 초과 시 목표를 해제한다.
연결이 복구되거나 선택이 돌아와도 새 목표 없이 주행을 재개하지 않는다.
로봇 전환 중 정지 확인 시간이 초과되면 대기하며, 예약한 목표도 취소한다.

| 인터페이스 | 타입 | 역할 |
|---|---|---|
| `/waypoint/goal_pose` | `geometry_msgs/PoseStamped` | RViz 공통 목표 입력, 현재 선택 로봇에만 적용 |
| `/waypoint/selected_robot` | `std_msgs/String` | 선택한 robot id, transient local |
| `/waypoint/status` | `std_msgs/String` | 선택·상태·목표·시간 JSON, transient local |
| `/waypoint/markers` | `visualization_msgs/MarkerArray` | 선택 표시·목표 화살표·목표 연결선 |
| `/waypoint/next_robot` | `std_srvs/Trigger` | Space와 동일한 전환 |
| `/waypoint/cancel` | `std_srvs/Trigger` | 현재 목표 취소 |
| `/<namespace>/reset_count` | `std_msgs/UInt64` | simulator에서 실제 리셋 후 증가, transient local |

TTY 없이 서비스로 조작하려면 `--no-keyboard`를 지정한다.

```bash
ros2 service call /waypoint/next_robot std_srvs/srv/Trigger '{}'
ros2 service call /waypoint/cancel std_srvs/srv/Trigger '{}'
ros2 topic echo /waypoint/status
```

장애물·다른 로봇을 피하는 경로 계획은 포함하지 않는다. 첫 시험은 빈 공간의 가까운 목표로 진행한다.
선택 로봇에만 명령을 보내는 것은 충돌 회피 기능이 아니다.

```bash
# 시스템 Humble/workspace를 source한 뒤 실행
/usr/bin/python3 -m unittest tests.test_waypoint tests.test_rviz_view
/usr/bin/python3 tests/check_waypoint_ros.py
/usr/bin/python3 tests/check_waypoint_ros.py --backend isaac
/usr/bin/python3 tests/check_waypoint_rviz.py
```

검사는 실제 PTY로 Space/X/Q를 보내고 도착·비선택 로봇 0 명령·목표 미재개·리셋·터미널 복원을 확인한다.
기본 synthetic 검사는 DDS/동역학 지연 모델 및 odom 수신 중단도 확인한다.
`--backend isaac`은 실제 warehouse와 기존 Go2 flat 정책, Lekiwi articulation으로 실행한다.
결과는 `.runtime/waypoint/{synthetic,isaac}/integration.json`에 기록한다.
2026-10-15 실제 warehouse 검증에서는 Go2 0.7 m 이동 목표의 도착 오차가 약 0.094 m,
Lekiwi 목표의 도착 오차가 약 0.055 m였다. 2,400 제어 스텝에서 Go2 수동 리셋 1회,
Lekiwi 리셋 0회였고 기존 Go2 정책 SHA-256을 유지했다.
`check_waypoint_rviz.py`는 Chrome/noVNC에서 목표를 두 번 찍고 PTY Space로 대상을 전환한다.
RViz를 재시작하지 않고 선택 표시와 명령 대상이 바뀌는지 검사하며, 합성 동역학을 사용한다.
결과와 화면은 `.runtime/waypoint/browser/`에 저장한다.

## Lekiwi 이동 베이스

프로젝트 루트에서 workspace를 빌드한 뒤 각 터미널에서 Humble/workspace를 source한다.

```bash
# 터미널 1: warehouse + Lekiwi, 로컬 창
ros2 launch robot_bringup simulation.launch.py \
  robots_file:="$PWD/ros2_ws/src/robot_bringup/config/lekiwi_single.yaml"

# 원격 화면: 위 실행 대신 사용, IP는 서버 주소로 변경
ros2 launch robot_bringup simulation.launch.py \
  robots_file:="$PWD/ros2_ws/src/robot_bringup/config/lekiwi_single.yaml" \
  display:=webrtc server_ip:=192.168.1.149

# 터미널 2: 검증한 낮은 속도로 키보드 이동
ros2 run robot_control teleop --robot-id lekiwi_0 --keyboard \
  --keyboard-speed 0.2 --keyboard-turn-speed 0.5

# 몸체 기준 전진 / 횡이동 / 회전, 각 명령은 별도로 실행
ros2 run robot_control teleop --robot-id lekiwi_0 --vx 0.2 --duration 2
ros2 run robot_control teleop --robot-id lekiwi_0 --vy 0.2 --duration 2
ros2 run robot_control teleop --robot-id lekiwi_0 --wz 0.5 --duration 2

ros2 topic echo /lekiwi_0/odom
ros2 topic echo /lekiwi_0/joint_states
```

단위는 m/s, rad/s이며 +X 전진·+Y 좌측·+Z 반시계 회전이다.
명령 만료는 Go2와 동일하게 기본 0.5초다. 키보드 공통 기본값과 속도 상한 정책은 유지한다.
`joint_states`에는 구동 바퀴 3개와 수동 롤러 36개, 총 39개 회전 관절을 발행한다.
팔의 6개 관절은 이번 이동 베이스 단계에서 고정 관절로 잠그므로 회전 관절 목록에서 제외된다.
base TF만 발행하며 전체 관절 TF/RViz robot description은 아직 제공하지 않는다.

자산은 `assets/Lekiwi/lekiwi_isaacsim/urdf/lekiwi/lekiwi.usd`를 참조한다.
실행 stage에서 월드 고정 루트를 해제하고, 비어 있던 바퀴 관절 축을 명시적인 축·프레임으로
보정한다. 단일 바퀴 convex hull 접촉은 비활성화하고 바퀴당 12개의 자유 회전 capsule 롤러로
교체한다. 로봇의 몸체 속도/위치를 매 스텝 직접 덮어쓰지 않고 바퀴의 velocity drive로 움직인다.
기하·구동·롤러 설정은 `robot_control/config/lekiwi.yaml`에서 수정한다.
이 접촉 모델은 횡이동 검증용 근사 모델이며 실제 롤러 형상·마찰·모터/질량의 실측 보정은 하지 않았다.
자산 상태와 수정 근거는 [Lekiwi 자산 안내](../assets/Lekiwi/README.md)를 참고한다.

## Go2 + Lekiwi 동시 제어

프로젝트 루트에서 빌드한 후 각 터미널에서 시스템 Humble와 workspace를 source한다.
한 Isaac Sim 프로세스에 warehouse 하나, SimulationContext 하나, 두 로봇의 articulation을 생성한다.

```bash
# 터미널 1: 로컬 화면
ros2 launch robot_bringup simulation.launch.py \
  robots_file:="$PWD/ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml"

# WebRTC: 위 실행 대신 사용, IP는 서버 주소로 변경
ros2 launch robot_bringup simulation.launch.py \
  robots_file:="$PWD/ros2_ws/src/robot_bringup/config/go2_lekiwi.yaml" \
  display:=webrtc server_ip:=192.168.1.149
```

`MULTI_ROS_READY` 이후 명령을 보낸다. 표시 없이 실행하려면 `display:=headless`를 사용한다.

```bash
# 터미널 2: Go2
ros2 run robot_control teleop --robot-id go2_0 --keyboard \
  --keyboard-speed 0.4 --keyboard-turn-speed 0.5

# 터미널 3: Lekiwi
ros2 run robot_control teleop --robot-id lekiwi_0 --keyboard \
  --keyboard-speed 0.2 --keyboard-turn-speed 0.5

# 정해진 서로 다른 명령도 로봇별 별도 터미널에서 실행 가능
ros2 run robot_control teleop --robot-id go2_0 --vx 0.3 --duration 5
ros2 run robot_control teleop --robot-id lekiwi_0 --vy 0.2 --duration 5
```

토픽·TF는 `/go2_0/*`, `/lekiwi_0/*`로 분리한다. 두 로봇의 상태 timestamp는 같은 물리 시간이다.
전역 `/clock`은 `simulation_clock` 노드 하나만 발행한다. 초기 자세 안정화 후 실험 clock을 0부터
시작하며, 이후 로봇 리셋으로 시간을 되돌리지 않는다. 물리는 200Hz, 정책/바퀴 제어·상태 발행은 50Hz다.
렌더링 성능에 따라 벽시계 기준 진행 속도는 달라질 수 있다.

로봇을 선택해서 초기 위치·방향으로 되돌릴 수 있다.

```bash
ros2 service call /go2_0/reset std_srvs/srv/Trigger '{}'
ros2 service call /lekiwi_0/reset std_srvs/srv/Trigger '{}'
```

서비스의 success는 다음 제어 경계에서 처리할 리셋 요청의 접수다. 리셋은 선택한 articulation의
위치·속도·관절 및 제어기 이력만 초기화하고 이전 속도 명령을 지운다. 다른 로봇과 clock은 계속 진행한다.
계속 발행 중인 teleop/navigation의 새 명령은 다음 tick부터 다시 수신한다.
`reset` 서비스는 현재 공통 다중 로봇 실행부에서 제공하며, 기존 단독 실행부에는 추가하지 않았다.
Go2는 base 높이 0.12m 미만 또는 몸체 기울기의 중력 z성분 > -0.5일 때 해당 로봇만 자동 리셋한다.
Lekiwi의 높이 기준은 0.015m이며 기울기 기준은 같다. 이 높이 기준은 현재 바닥 z=0인 시나리오용이다.

`go2_lekiwi.yaml`에서 로봇 목록·고유 id/namespace·position·yaw를 수정한다.
현재 기본 배치는 Go2 `[0, -0.8, 0.4]`, Lekiwi `[0, 0.8, 0.04]`다.
각 articulation은 `/World/Robots/<id>`에 생성한다. 서로 다른 로봇 간 충돌을 끄는 필터는 추가하지 않는다.
로봇 간 양의 접촉 impulse 기록은 실행 보고서의 `robot_contacts`에 저장한다.

Go2 공통 장면용 제어기는 기존 48개 관측 순서·지난 액션·관절 순서·0.25 action scale과
DC motor 설정을 사용한다. `ManagerBasedRLEnv`의 환경 전체 step/reset 대신 공통 루프가 물리를
진행한다. 기존 단독 실행과 viewer/TCP 경로도 유지한다.
Hunter2, 경로 추종·충돌 회피·상위 작업 배정은 아직 구현하지 않았다.

## 실제 폴더와 코드 책임

```text
ros2_ws/src/
  robot_control/
    package.xml, setup.py, setup.cfg, resource/robot_control
    robot_control/
      commands.py          # 공통 속도 명령·벽시계 만료
      keyboard.py          # 공통 터미널 입력
      go2/policy.py        # 기존 flat 정책 로더, 관측 48·액션 12
      go2/controller.py    # Go2 관측에 속도 반영·관절 목표 계산
      lekiwi/controller.py # 몸체 속도 → 3개 바퀴 rad/s
      nodes/teleop.py      # 시스템 Humble 명령 송신기
      waypoint.py          # 로봇별 waypoint PD 계산·도착 판정
      waypoint_keyboard.py # Space 선택·X 취소·Q 종료
      nodes/waypoint.py    # 공통 목표·단일 선택·odom/TF·정지 확인·RViz 표시
    config/go2.yaml        # 현재 모델 계약의 참조값
    config/lekiwi.yaml     # 바퀴 기하·모터·수동 롤러 파라미터
    config/waypoint.yaml   # PD 게인·자동 추종 제한·도착 오차
  robot_sim/
    package.xml, setup.py, setup.cfg, resource/robot_sim
    robot_sim/
      bootstrap.py         # 로봇 설정 검증·공통 launcher 연결
      launcher.py          # Go2/Lekiwi worker·로컬/WebRTC·종료 상태 검사
      config.py            # 로봇 목록·중복 이름·시작 위치 검증
      environment.py       # Python 3.10/3.11 ROS 라이브러리 분리
      ros_bridge.py        # cmd_vel 수신·odom/joints/TF/clock 발행
      runtime.py           # 기존 단일 Go2 물리·정책 루프
      multi_runtime.py     # 공통 물리·로봇별 제어·독립 리셋
      scene.py             # articulation 생성·선택 리셋·로봇 간 접촉 기록
      lekiwi_runtime.py    # 단일 Lekiwi 물리·ROS 루프
      lekiwi_scene.py      # USD 참조·이동 루트·팔 고정·수동 롤러
  robot_bringup/
    package.xml, setup.py, setup.cfg, resource/robot_bringup
    robot_bringup/launch_support.py
    launch/simulation.launch.py
    config/go2_single.yaml
    config/lekiwi_single.yaml
    config/go2_lekiwi.yaml
```

`robot_sim`은 robots 목록이 하나면 기존 Go2/Lekiwi 단독 실행부를 사용하고,
둘 이상이면 `multi_runtime.py`의 공통 장면을 사용한다. 지원 type은 `go2`, `lekiwi`이며
중복 id/namespace와 유효하지 않은 설정값을 Isaac 시작 전에 거부한다. 현재 동시 물리 검증 대상은
Go2 한 대 + Lekiwi 한 대다. 단일 선택 waypoint PD를 제공하며 Hunter2, 경로 계획/fleet는 후속 단계다.

기존 `scripts/go2/ros.py`, `ros_command.py`, `control.py`도 실행할 수 있다.
`control/go2/`의 정책·키보드·ROS 파일은 workspace 구현을 가져오는 호환 모듈이다.
viewer ↔ control의 TCP 제어 경로는 유지한다.
루트 `assets/`의 로봇·정책·출처와 `scenarios/`의 환경 설정을 재사용하며 자산을 복제하지 않는다.

## 로봇 배치와 launch 옵션

`robot_bringup/config/go2_single.yaml`:

```yaml
world:
  scenario: warehouse
  seed: 42
robots:
  - id: go2_0
    type: go2
    namespace: go2_0
    position: [0.0, 0.0, 0.4]
    yaw: 0.0
```

시작 위치·yaw는 reset에도 적용된다. 사용자 파일은 `robots_file:=/absolute/path/robots.yaml`로 전달한다.

| launch 인자 | 기본값 | 역할 |
|---|---|---|
| `project_root` | source/현재 경로에서 자산을 찾아 결정 | 프로젝트 자산·시나리오·Isaac 경로 |
| `isaac_python` | `<project_root>/.conda/isaacsim-5.0/bin/python` | 시뮬레이터 전용 Python |
| `robots_file` | 설치된 `go2_single.yaml` | 로봇·namespace·시작 위치·환경·seed |
| `display` | `local` | `local`, `headless`, `webrtc` |
| `rviz_display` | `auto` | `auto`, `local`, `web`, `off`; auto는 local→local, webrtc→web, headless→off |
| `rviz_robot` | 첫 로봇 id | 초기 RViz 카메라 중심에 둘 로봇; PD 선택은 터미널 Space |
| `rviz_goal_topic` | `/waypoint/goal_pose` | RViz 공통 목표 입력 토픽 |
| `rviz_port` | `6080` | 브라우저 접속 포트, loopback에만 바인딩 |
| `rviz_resolution` | `1440x900` | 원격 화면 크기; 로컬에서는 초기 창 크기 |
| `rviz_rendering` | `auto` | auto는 web에서 software, local에서 native; `software`, `native`로 지정 가능 |
| `device` | `cuda:0` | 시뮬레이션 장치 |
| `policy_device` | `cpu` | Go2 정책 추론 장치 |
| `command_timeout` | `0.5` | 벽시계 기준 명령 만료 초 |
| `ros_domain_id` | `ROS_DOMAIN_ID` 또는 `0` | 외부 노드와 동일하게 사용 |
| `steps` | `0` | 50Hz 제어 스텝 수, 0은 무제한 |
| `report` | 미설정 | 결과 JSON 절대/상대 경로 |
| `checkpoint` | manifest의 기존 flat 모델 | Go2 정책 override; Lekiwi에서는 사용 불가 |
| `server_ip` | `PUBLIC_IP` 또는 미설정 | WebRTC 주소 |

launch는 시스템 ROS의 Python/라이브러리 overlay를 Isaac 자식 환경에서 제거하고
Isaac 내장 Humble와 프로젝트 Python 소스 경로를 넣는다. 새 Conda 환경이나 별도의
Go2 정책 모델을 만들지 않는다. ROS 환경을 source한 터미널에서도 Isaac 작업은 분리된다.

## 토픽과 프레임

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/go2_0/cmd_vel` | `geometry_msgs/Twist` | 몸체 좌표 vx/vy/wz, 수신 queue depth 1 |
| `/go2_0/odom` | `nav_msgs/Odometry` | simulator 실제 위치·속도, 50Hz |
| `/go2_0/joint_states` | `sensor_msgs/JointState` | Go2 12개 관절 위치·속도, 50Hz |
| `/tf` | `tf2_msgs/TFMessage` | `go2_0/odom` → `go2_0/base_link` |
| `/tf_static` | `tf2_msgs/TFMessage` | `map` → `go2_0/odom`, identity, transient local |
| `/clock` | `rosgraph_msgs/Clock` | 물리 시간, 50Hz, best-effort/volatile |

Lekiwi는 위 토픽·프레임의 `go2_0` 대신 `lekiwi_0`를 사용한다.

odom은 ground truth이며 estimator/SLAM의 출력이 아니다. 이 단계는 base 프레임의 TF만 발행한다.
전체 관절·카메라 frame의 TF와 RViz robot description은 후속 기능이다.
상태와 clock의 timestamp는 같은 시뮬레이션 시간을 사용한다. 외부 navigation에서는
`use_sim_time`을 켠다. command 만료는 monotonic 벽시계로 검사한다.

기존 standalone 실행의 기본 namespace는 `unitree_go2`, workspace launch의 기본 namespace는
`go2_0`이다. 수신과 송신에서 같은 topic/namespace와 ROS domain을 사용한다.
같은 domain에는 clock 소유 시뮬레이터 하나만 실행한다. teleop와 navigation은 한 로봇에
대해 동시에 명령을 보내지 않게 운용한다.

## 검증

```bash
# 시뮬레이터 없이 코드/환경 격리 검사
.conda/isaacsim-5.0/bin/python -m unittest \
  tests.test_ros_workspace tests.test_go2_policy tests.test_go2_ros tests.test_go2_keyboard

# 시스템 Humble -> workspace launch -> Isaac GPU 실제 통신/보행 검사
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
/usr/bin/python3 tests/check_ros_workspace.py
```

통합 검사는 900스텝 동안 설치된 teleop 실행, 전진·회전·횡이동·후진·명령 만료,
namespace, 단일 clock, odom·관절 상태·TF, 기본 모델 SHA-256과 리셋 수를 검사한다.
출력은 `.runtime/ros2_workspace/integration_check.json`, 상세 물리 보고서는
`.runtime/ros2_workspace/simulation.json`에 저장한다. 로그는 같은 폴더의 `simulation.log`다.

2026-10-14 교차 환경 검사: 시스템 Python 3.10.12 → Isaac Python 3.11,
warehouse 900스텝, 리셋 0회, clock·odom·joint_states·TF 각각 901개 수신,
static TF 1개 수신, 명령 만료·설치된 teleop 실행을 확인했다.
기본 로컬 화면 launch도 100스텝·리셋 0회로 정상 종료했다.
시스템 Humble의 설치된 키보드 송신기는 W/Z/정지·입력 만료·Q 종료·TTY 복원을 확인했다.

Lekiwi 검사는 시스템 Humble에서 실행한다. 매번 하나씩 실행하며 같은 ROS domain의 다른
시뮬레이터를 종료한 상태로 검사한다.

```bash
/usr/bin/python3 tests/check_lekiwi_ros.py --scenario flat
/usr/bin/python3 tests/check_lekiwi_ros.py --scenario warehouse --display webrtc --server-ip 서버_IP
```

전후·좌우·양방향 회전·정지 및 명령 만료 후 실제 몸체 속도, namespace/시작 yaw,
단일 clock, TF, 39개 관절 상태와 물리 시간 일치를 검사한다. WebRTC 모드는 로컬 signaling
포트 개방까지 확인하며 원격 클라이언트 영상 수신은 별도로 확인해야 한다.

2026-10-15 Lekiwi 검사 결과: flat/headless와 warehouse/WebRTC에서 각각 900스텝,
리셋·전복 0회, 전후·좌우·양방향 회전·명시적 정지·명령 만료 후 실제 정지를 확인했다.
warehouse/WebRTC에서는 clock·odom·joint_states·TF 각각 901개와 static TF 1개를 수신했다.
0.2m/s 명령의 평균 이동 속도는 약 0.195~0.197m/s, 0.5rad/s 명령의 회전 속도는 약 0.484rad/s였다.
WebRTC signaling TCP 포트 개방을 검사했으며 원격 클라이언트의 영상 수신까지 검사한 것은 아니다.
명령 유지 구간은 `/clock` 기준으로 측정하므로 렌더링이 실시간보다 느려져도 같은 물리 시간을 검사한다.
공통 launcher로 이동한 Go2도 기존 모델 SHA를 유지한 warehouse 900스텝·리셋 0회 회귀 검사를 통과했다.

공통 실행부 검사는 다음과 같다. 각 검사를 하나씩 실행한다.

```bash
# 기존 Go2 관리자의 관측/액션과 공통 장면 제어기의 수치 일치 확인
.conda/isaacsim-5.0/bin/python tests/check_go2_shared_adapter.py

# 외부 Humble -> 공유 warehouse의 두 로봇, 명령/정지/리셋/실제 충돌
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
/usr/bin/python3 tests/check_multi_robot_ros.py
/usr/bin/python3 tests/check_multi_robot_ros.py --display webrtc --server-ip 서버_IP
```

공통 실행 결과는 `.runtime/multi_ros/<display>/simulation.json`, 검사 판정은
같은 폴더의 `integration_check.json`에 저장한다. Go2 계약 비교는 `.runtime/multi_ros/go2_contract.json`이다.
검사는 로봇별 세 번의 선택 리셋을 요청하고 의도적인 충돌 후 자동 리셋/복구를 확인한다.
`robot_contacts.events`는 두 로봇 사이 양의 접촉 힘이 측정된 물리 샘플 수이며,
`impulse_sum`은 몸체 쌍별 접촉 힘 크기를 물리 dt로 적분한 값(N·s)이다.

2026-10-15 공통 warehouse 검사 결과: headless와 WebRTC 각각 1,400 제어 스텝(28초 물리),
clock 1,401개와 로봇별 odom·joint_states 각각 1,401개를 수신했다. 단일 clock, 서로 다른 명령,
Lekiwi 명령 만료 중 Go2 계속 주행, 다른 로봇 주행 중 선택 리셋을 검사했다.
로봇마다 수동 리셋 3회를 요청했으며, 의도적 접촉 후 Go2의 전복 자동 리셋 1회가 추가됐다.
Lekiwi와 clock은 계속 진행했고, 두 로봇은 초기 배치로 복구했다. 바닥을 제외한 로봇 간 접촉
힘의 양의 impulse를 GPU tensor로 확인했다. Go2 관측/관절 목표는 기존 관리자와 80스텝 대조 시
최대 오차가 모두 0이었고 모델 SHA도 유지됐다. 기존 Go2 단독 ROS 검사도 900스텝·리셋 0회 통과했다.
WebRTC는 서버 실행·signaling 포트까지 확인했으며 원격 클라이언트 영상 수신은 별도 확인해야 한다.
공통 장면 WebRTC의 첫 재질/셰이더 준비는 수분이 걸릴 수 있으며 `MULTI_ROS_READY`를 기다린다.
