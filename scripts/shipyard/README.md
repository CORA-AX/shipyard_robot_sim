# 조선소 전용 waypoint 실행

기존 소스·설정·USD를 수정하지 않고 추가한 실행 도구다. 기존 `WaypointNode` / `WaypointPD`,
Go2 정책, Lekiwi 휠 제어기, 로봇 생성 함수와 ROS bridge를 import해서 재사용한다.
기존 모듈의 전역 함수 교체(monkey patch)는 하지 않는다. 별도 패키지 설치나 ROS 재빌드는 필요 없다.

## 실행

프로젝트 루트에서 실행한다. 기존 simulator, waypoint, teleop, Nav2는 종료하고
아래 두 터미널의 `ROS_DOMAIN_ID`를 일치시킨다. 두 로봇은 하나의 물리 환경과 `/clock`을 공유한다.

터미널 1 — Go2와 Lekiwi를 함께 배치한다.

```bash
export ROS_DOMAIN_ID=0
.conda/isaacsim-5.0/bin/python scripts/shipyard/simulate.py --display local

# 원격 WebRTC 사용 시 위 명령 대신 실행 (주소를 실제 서버 IP로 변경)
.conda/isaacsim-5.0/bin/python scripts/shipyard/simulate.py --display webrtc --server-ip <서버_IP>
```

`SHIPYARD_READY`가 나오면 터미널 2에서 경로 관리를 실행한다.
ROS 터미널에서는 Conda Python 대신 시스템 Python을 사용한다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# 먼저 짧은 왕복: Go2는 1m 전진 후 복귀, Lekiwi는 교차로 접근 + 1m 이동 후 복귀
/usr/bin/python3 scripts/shipyard/waypoints.py --route smoke

# 전체 평지 왕복 순찰은 위 프로세스를 종료한 뒤 실행
/usr/bin/python3 scripts/shipyard/waypoints.py --route flat_only
```

- 기본 선택은 `go2_0`. `--robot lekiwi_0`로 Lekiwi부터 시작할 수 있다.
- **Space**: 현재 경로를 취소하고 다음 로봇을 선택한다. 이전 로봇의 정지를 확인하고
  새 로봇의 경로를 시작한다. 경로를 끝내고 복귀한 뒤 전환하는 것이 기본 사용 방식이다.
- **X / Esc**: 경로 전체 취소. 이후 waypoint를 자동으로 이어 보내지 않는다.
- **Q / Ctrl+C**: 정지 명령을 보낸 뒤 경로 관리 종료. simulator는 별도로 종료한다.
- 로봇은 한 번에 한 대만 이동한다. 나머지 로봇도 기존 제어기로 정지 상태를 유지한다.

터미널 키 입력 없이 실행할 때는 `--no-keyboard`를 지정한다.

```bash
# 정지 상태로 시작하여 서비스 요청 대기
/usr/bin/python3 scripts/shipyard/waypoints.py --route smoke --no-keyboard --idle

# 다른 ROS 터미널에서
ros2 service call /shipyard/start_route std_srvs/srv/Trigger '{}'
ros2 service call /waypoint/cancel std_srvs/srv/Trigger '{}'
ros2 service call /waypoint/next_robot std_srvs/srv/Trigger '{}'
```

중도 취소 후에는 선택한 로봇을 출발 위치로 리셋한 뒤 `/shipyard/start_route`를 호출한다.
현재 위치가 설정된 주차 위치에서 0.4m보다 멀면 임의의 지형을 가로질러 복귀시키지 않고 거부한다.

```bash
ros2 service call /go2_0/reset std_srvs/srv/Trigger '{}'
ros2 service call /lekiwi_0/reset std_srvs/srv/Trigger '{}'
```

## 경로와 설정

`fleet.yaml`이 이 도구의 로봇 목록과 시작 위치를 정한다. 기존 bringup YAML과 독립적이다.

| 로봇 | 시작 위치 | 기본 평지 경로 |
| --- | --- | --- |
| Go2 | `(-16,18,0.4)` | 기존 apron → 북쪽 교차로 → 평지 우회로 → 남쪽 교차로 → 역순 복귀 |
| Lekiwi | `(-23.5,-20,0.04)` | 남쪽 주차 위치 → 남쪽 교차로 → 평지 우회로 → 북쪽 교차로 → 역순 복귀 |

Lekiwi 주차 위치는 도로 중심에서 1.5m 떨어져 있어 Go2 경로를 막지 않는다.
모든 경로는 [route_hints.json](../../assets/environments/shipyard/route_hints.json)의 연결을 사용한다.
주차 위치와 경로 입구 사이의 최대 2m 연결은 이 배치의 평지 접근로를 전제로 한다.
사용자가 주차 위치나 환경을 바꾸면 해당 연결의 충돌 여유를 다시 검사해야 한다.

긴 간선은 기본 2m 간격으로 나눠 기존 단일 목표 제어기에 하나씩 전달한다.
목표에 도착하고 정지한 것이 확인된 후 다음 목표로 진행하므로 각 waypoint에서 잠시 멈춘다.
끝까지 완료하면 출발 위치로 돌아온다. 경로를 저장·수정하는 파일은 별도로 덮어쓰지 않는다.

```bash
# ROS 연결 없이 전체 경로를 먼저 확인
python3 scripts/shipyard/waypoints.py --robot lekiwi_0 --route flat_only --preview

# Go2 경사로는 명시적으로 선택한 경우에만 허용하는 실험 경로
/usr/bin/python3 scripts/shipyard/waypoints.py --route service_loop --allow-ramps
```

계단은 항상 거부한다. Lekiwi의 경사로도 현재 거부한다. Go2는 평지용 정책을 그대로 사용하므로
`--allow-ramps`는 등판 성공을 보장하지 않는다. 새 학습 정책이나 계단 제어기는 포함하지 않는다.
`--spacing`은 0.25~3m, `--ready-timeout`은 simulator/odom 준비 대기 시간(기본 300초)이다.
`--exit-on-complete`를 주면 완료 또는 실패 후 종료한다. 실패는 종료 코드 1이다.

## 상태와 중단 조건

- `/shipyard/route_status`: 선택 로봇, 경로, 현재 waypoint 번호, 완료 횟수, 중단 이유.
- `/shipyard/route_markers`: RViz용 전체 경로 선. Fixed Frame은 `map`이다.
- `/waypoint/status`, `/waypoint/markers`: 기존 목표 추종기의 상태·현재 목표 표시.
- odom 단절, 로봇 리셋, 제어권 충돌, 목표 시간 초과 시 경로를 중단한다.
- 다른 로봇이 현재 이동 구간의 1m 이내에 있으면 정지하고 실패 상태로 남긴다.
  자동 재출발·교차로 예약·회피 경로 탐색은 하지 않는다.

이 도구는 저장된 경로의 순차 추종기다. Nav2, 임의 장애물 회피, 주행 가능 지형 인식,
두 로봇의 동시 자율 주행은 포함하지 않는다. RViz의 임의 2D 목표 입력도 이 도구에서는 받지 않는다.
현재 `map`과 환경의 `/World` 좌표가 일치하는 배치를 전제로 한다.

보고서는 `outputs/shipyard/waypoints/`의 `simulation.json`, `controller.json`, `route.json`에 저장된다.

## 검사

```bash
python3 tests/test_shipyard_waypoints.py

source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="$PWD/outputs/shipyard/waypoints/ros_logs"
/usr/bin/python3 tests/check_shipyard_waypoints_ros.py --backend synthetic --domain 86
/usr/bin/python3 tests/check_shipyard_waypoints_ros.py --backend isaac --domain 87
```

합성 동역학 검사는 실제 ROS 토픽으로 두 로봇의 왕복·선택 전환·취소·리셋·odom 단절을 검증한다.
Isaac 검사는 저장된 조선소에서 두 로봇을 실제 물리로 생성해 짧은 왕복과 정지를 검증한다.
각 결과는 `outputs/shipyard/waypoints/{synthetic,isaac}/integration.json`에 저장된다.
짧은 왕복 검사는 전체 순찰 경로나 경사로의 주행 검증을 대신하지 않는다.

## 추가 파일의 역할

| 파일 | 역할 |
| --- | --- |
| `simulate.py` | 조선소 전용 local/WebRTC/headless 실행 진입점 |
| `runtime.py` | 기존 로봇·제어기·ROS bridge를 조합하는 공통 물리 루프 |
| `common.py` / `fleet.yaml` | 전용 설정과 기존 로봇 검증 함수 재사용 |
| `route_plan.py` | 방향·지형 능력 검사, 왕복 경로 구성, waypoint 분할 |
| `waypoints.py` | 기존 waypoint 노드 확장, 순차 목표 전달·정지·상태 표시 |

기존 `simulation.launch.py`, `navigation.launch.py`의 시나리오 제한은 바꾸지 않았다.
이 기능은 위의 새 `simulate.py`로 실행해야 한다.
