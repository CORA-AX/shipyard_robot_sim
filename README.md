# Shipyard robot simulation

전체 명령 옵션·기본값·실행 예시는 [커맨드 옵션 안내](command_options.md)를 참고한다.

Isaac Sim 5.0 + Isaac Lab v2.2.1 기반 로봇 시뮬레이션 프로젝트다.
기본 실험 환경은 원본 isaac-go2-ros2의 NVIDIA Simple Warehouse를 이식한 `warehouse`다.
[warehouse 자산 출처·시작 위치·실행 안내](assets/environments/warehouse/README.md)를 참고한다.
현재 Go2 한 대의 환경 표시·정책 제어를 지원한다. Hunter2는 자산 준비·정적 검증 단계이며
여러 로봇의 동시 실행은 아직 구현하지 않았다.

기본 분리 실행에서는 **viewer가 환경·로봇·물리를 관리하고, 별도 control 프로세스가 정책을 실행한다.**
ROS 직접 제어는 `scripts/go2/ros.py`가 정책과 물리를 함께 실행한다.
명령은 모두 프로젝트 루트에서 실행한다. 공통 Conda 환경은 `.conda/isaacsim-5.0`이다.
Isaac Lab은 `third_party/IsaacLab`을 사용한다.

## 기존 서버에서 바로 실행

사용할 터미널마다 환경을 활성화한다. 다른 컴퓨터에서는 프로젝트·Conda 경로를 바꾼다.

```bash
cd /home/rodix-gpu/shipyard_robot_sim
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate "$PWD/.conda/isaacsim-5.0"
```

기본 제어 모델은 `assets/go2/policies/upstream_flat_6800.pt`다.
Zhefan-Xu/isaac-go2-ros2의 `flat_model_6800.pt`를 사용한다. 다운로드·체크섬 검증은 다음 명령으로 실행한다.

```bash
python scripts/go2/prepare_go2_policy.py --control-policy
```
기존 공식 정책을 사용할 때는 먼저 다운로드·체크섬 확인을 실행한다.

```bash
python scripts/go2/prepare_go2_policy.py
```

## 환경과 로봇 표시

| 목적 | viewer 옵션 |
|---|---|
| 창고 환경만 표시 | `--scenario warehouse` |
| 평지 환경만 표시 | `--scenario flat` |
| 환경에 Go2 추가 | `--robot go2` |
| 서버 로컬 화면 | `--no-webrtc` (기본값) |
| 원격 WebRTC 화면 | `--webrtc --server-ip 서버_IP` |

```bash
# 환경만 표시
python scripts/viewer.py --scenario warehouse --no-webrtc

# 환경 + Go2 표시: 로컬 창
python scripts/viewer.py --scenario warehouse --robot go2 --no-webrtc

# 환경 + Go2 표시: WebRTC + headless
python scripts/viewer.py --scenario warehouse --robot go2 \
  --webrtc --server-ip 192.168.1.149
```

위 viewer 명령 중 하나를 실행한다. `192.168.1.149`는 실제 서버 IP로 바꾼다.
WebRTC 클라이언트에서 같은 IP로 연결하며 TCP 49100·UDP 47998을 사용한다.
`PUBLIC_IP`를 설정했다면 `--server-ip`를 생략할 수 있다.
로컬 화면은 서버 데스크톱의 DISPLAY와 X11 접근 권한이 필요하다.

환경만 보는 viewer는 정적 배치를 표시한다. Go2 viewer도 제어기 연결 전에는 물리를 멈추고 기다린다.
환경 전용 준비 로그는 `SCENARIO_VIEW_READY`, Go2 준비 로그는 `ROBOT_VIEW_READY`다.

## Go2 정책 제어

Go2 viewer를 실행한 상태에서 **같은 서버의 두 번째 터미널**에 공통 Conda 환경을 활성화한다.

```bash
python scripts/go2/control.py --vx 0.4
```

제어기는 기본 CPU에서 정책을 추론하고 `127.0.0.1:8765`로 viewer에 연결한다.
정책 추론 장치는 `--device cuda:0`으로 변경할 수 있다. 정책은 장애물 회피를 수행하지 않는다.
기본 모델은 Zhefan-Xu/isaac-go2-ros2의 `flat_model_6800.pt`이며,
프로젝트에서는 `upstream_flat_6800.pt`라는 이름으로 보관한다.
`--checkpoint 경로`로 다른 모델을 지정한다. 실제 모델 경로·iteration은 `GO2_POLICY_LOADED`에 출력된다.
[모델 선택·제어 실행 안내](docs/viewer_control.md)를 참고한다.
제어기에서 Ctrl+C를 누르면 물리를 멈추고 viewer 화면은 유지한다.
같은 명령을 다시 실행하면 현재 상태에서 재개한다. 전체 종료는 viewer도 종료한다.
한 Go2에 제어기 하나만 연결할 수 있으며, WebRTC viewer도 한 번에 하나씩 실행한다.

## 제어 테스트와 결과 저장

```bash
# 터미널 1: 총 500개 액션 적용 후 종료
python scripts/viewer.py --scenario warehouse --robot go2 --no-webrtc \
  --steps 500 --report .runtime/go2/viewer_control.json

# 터미널 2: 500스텝 정책 실행 및 적용 확인
python scripts/go2/control.py --vx 0.4 --steps 500 --report .runtime/go2/control.json
```

WebRTC 테스트는 터미널 1의 `--no-webrtc`를 `--webrtc --server-ip 서버_IP`로 바꾼다.
평지 테스트는 `--scenario flat`을 사용한다.
viewer를 무제한 실행하고 제어기에만 `--steps 500`을 주면 500스텝 후 물리를 멈추고 재연결을 기다린다.

| 옵션·결과 | 의미 |
|---|---|
| viewer `--steps` | 로봇 없으면 렌더 프레임 수, Go2 있으면 적용한 제어 스텝 수 |
| control `--steps` | 이번 연결에서 적용이 확인된 액션 수 |
| `--steps 0` | 무제한 (기본값) |
| control `--vx` | Go2 전진 속도 명령, 0.2~0.8m/s |
| 양쪽 `--control-port`, `--robot-id` | 포트·로봇 ID 변경 시 같은 값 지정 |
| viewer 보고서 | 전진 거리·최저 높이·리셋·연결 횟수·정지 상태 보존 |
| control 보고서 | 적용 확인 스텝 수·마지막 로봇 상태 |

`passed`는 실행·통신 완료 결과다. 보행 성능은 보고서의 거리·높이·리셋 횟수로 확인한다.
현재 자동 보행 합격 판정과 정책 export 기능은 제공하지 않는다.
상세 통신·재접속 동작은 [viewer와 control 사용 안내](docs/viewer_control.md)를 참고한다.

## ROS로 Go2 직접 제어

`scripts/go2/ros.py`는 시뮬레이션과 선택된 정책을 한 프로세스에서 실행한다.
ROS 명령으로 직접 제어하는 독립 실행 경로이며 기본 모델은 `upstream_flat_6800.pt`다.
공통 Conda 환경을 활성화한 뒤 실행한다.

```bash
# 터미널 1: 창고 + Go2 + ROS, 로컬 화면
python scripts/go2/ros.py --scenario warehouse

# 화면 없이 실행하려면
python scripts/go2/ros.py --scenario flat --headless

# WebRTC 화면으로 실행하려면
python scripts/go2/ros.py --scenario warehouse --webrtc --server-ip 192.168.1.149
```

위 명령 중 하나를 실행하고 `GO2_ROS_READY`를 기다린다. 두 번째 터미널에서도
같은 Conda 환경을 활성화한 뒤 명령을 보낸다. 아래 도구는 Isaac Sim의 내장
Humble 라이브러리를 사용하므로 별도 시스템 ROS 설치가 필요하지 않다.

```bash
# 전진 5초; 완료 시 정지 명령 전송
python scripts/go2/ros_command.py --vx 0.4 --duration 5

# 제자리 회전 / 횡이동 / 정지
python scripts/go2/ros_command.py --wz 0.5
python scripts/go2/ros_command.py --vy 0.3
python scripts/go2/ros_command.py
```

외부 ROS 2 Humble 노드에서도 `/unitree_go2/cmd_vel`에 `geometry_msgs/msg/Twist`를
발행할 수 있다. `linear.x`, `linear.y`는 로봇 몸체 좌표의 m/s,
`angular.z`는 rad/s다. 세 축의 명령 속도에는 상한을 적용하지 않으며, NaN/무한대 입력은 거부한다.
명령은 20Hz 정도로 반복 발행한다. 0.5초 동안 유효한 명령이 없으면 목표 속도를
0으로 바꾸고 정책·물리는 계속 실행한다. `/clock`은 물리가 진행한 시간을 50Hz로
발행한다. ROS 도구의 `use_sim_time`을 켜면 이 시간을 사용한다.

```bash
# 별도로 설치된 ROS Humble 터미널에서 실행하는 예
ros2 topic pub -r 20 /unitree_go2/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.4}, angular: {z: 0.0}}'
ros2 topic echo /clock
```

`--ros-domain-id`의 기본값은 `ROS_DOMAIN_ID`, 미설정 시 0이다. 외부 노드도 같은
domain을 사용해야 한다. 원격 노드는 DDS 통신이 가능한 네트워크에 있어야 한다.
같은 domain에는 `/clock` 발행 시뮬레이터를 하나만 실행한다.
전체 옵션·검사 명령은 [커맨드 옵션 안내](command_options.md#ros-go2-독립-실행)를 참고한다.

## 키보드 이동

세 실행기에서 `--keyboard`로 실행한 터미널에 포커스를 두고 조작한다.

```bash
# viewer를 실행한 뒤 별도 control 터미널에서 조작
python scripts/go2/control.py --keyboard

# 시뮬레이션·정책을 한 프로세스에서 실행하고 같은 터미널에서 조작
python scripts/go2/ros.py --scenario warehouse --keyboard

# ros.py를 일반 ROS 모드로 실행한 뒤 별도 터미널에서 cmd_vel 발행
python scripts/go2/ros_command.py --keyboard
```

| 키 | 동작 |
|---|---|
| W / S 또는 ↑ / ↓ | 전진 / 후진 |
| A / D 또는 ← / → | 왼쪽 / 오른쪽 횡이동 |
| Z / C | 왼쪽 / 오른쪽 회전 |
| Space / X | 즉시 목표 속도 0 |
| Q / Ctrl+C | 실행기 종료 |

키를 누르고 있으면 OS의 반복 입력으로 이동을 유지한다. 마지막 유효한 키 입력 후
0.6초가 지나면 목표 속도가 0이 된다. 기본 이동 속도는 1.5m/s, 회전은 1.5rad/s다.
`--keyboard-speed`, `--keyboard-turn-speed`, `--keyboard-timeout`으로 변경한다.
이동·회전 속도 옵션은 유한한 0 이상의 값을 받으며 상한은 없다. 공통 기본값은
`control/go2/keyboard.py`의 `DEFAULT_KEYBOARD_SPEED`, `DEFAULT_KEYBOARD_TURN_SPEED`에서 수정한다.
`ros_command.py --keyboard`는 Q 또는 Ctrl+C까지 실행하고 종료 시 정지 명령을 발행한다.
`ros.py --keyboard`는 터미널 입력으로 직접 제어하며 `/clock`을 발행한다.
별도 ROS 송신기에서 조작하려면 `ros.py`를 일반 ROS 모드로 실행한다.
SSH에서는 TTY를 할당한다 (`ssh -t`). `control.py --keyboard` 사용 전에는 viewer도 재시작한다.

## Go2 PPO 학습 결과

프로젝트 루트에서 공통 Conda 환경을 활성화한 뒤 Isaac Lab의 평지 Go2 학습을 실행한다.

```bash
CUDA_VISIBLE_DEVICES=0 python third_party/IsaacLab/scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
  --num_envs 32 --max_iterations 10 --headless
```

학습 로그·체크포인트·설정은 `rl/` 아래의 `rl/rsl_rl/<experiment_name>/<실행시각>/`에
저장한다. 평지 Go2는 `rl/rsl_rl/unitree_go2_flat/`을 사용한다. `train.py --resume`과
`play.py`의 기본 체크포인트 검색도 이 경로를 사용한다. 학습 결과는 Git에서 제외한다.
기존 `logs/rsl_rl/`의 학습 결과는 `rl/rsl_rl/`로 이동했다.

```bash
tensorboard --logdir rl/rsl_rl/unitree_go2_flat --host 127.0.0.1 --port 6006
```

몸체 높이 유지와 제자리 회전을 포함한 cmd_vel 추종 학습은 프로젝트 task를 사용한다.

```bash
python rl/go2/train.py --task Shipyard-Go2-Flat-Tracking-v0 \
  --num_envs 1024 --max_iterations 5000 --seed 42 --run_name trot_v4_fresh --headless
```

이 task는 높이 목표 0.28m, 강화된 속도·yaw 추종 보상과 오차 페널티,
30% 제자리 회전 및 저속 전진·전진 회전 명령을 사용한다. 현재 v4에는 대각선 trot 타이밍 보상,
발 미끄러짐과 0.4초를 넘는 발 공중 시간 페널티가 포함된다. 처음부터 학습하려면
`--resume`과 체크포인트 로드 옵션을 생략한다. 결과는 `rl/rsl_rl/go2_flat_tracking/`에 저장한다.
[보상 변경 내용·4 GPU 학습·평가 방법](rl/README.md)을 참고한다.

## 코드 수정 위치

| 폴더 | 역할 |
|---|---|
| `assets/<robot>/` | 모델·관절·모터 설정·정책 파일·출처 |
| `rl/` | Go2 학습 task·실행기·평가·분석과 `rsl_rl/` 실행 결과 |
| `scenarios/` | 공통 배경·장애물·로봇 배치·관측·액션·리셋 설정 |
| `control/<robot>/` | 로봇별 정책 추론과 viewer 내부 물리·상태 처리 |
| `control/control_channel.py` | viewer·제어기 사이의 로컬 통신 |
| `scripts/viewer.py` | 환경·로봇 선택 및 화면 실행 명령 |
| `scripts/<robot>/` | 정책 준비·제어 연결·자산 관리 명령 |
| `scripts/visualization/` | 공통 로컬/WebRTC 초기화·캐시·프로세스 관리 |
| `tests/` | 단위 테스트·설치 및 기본 물리 검사 |
| `third_party/` | submodule로 연결한 외부 소스 |
| `.runtime/`, `.cache/`, `.conda/` | 로컬 실행 결과·캐시·가상환경 |

warehouse 자산·시작 위치·카메라는 `scenarios/warehouse/scene_cfg.py`, Go2 모터 설정은
`assets/go2/config/robot_cfg.py`, 정책 추론은 `control/go2/policy.py`를 수정한다.
로봇 추가 절차는 [담당자용 개발 안내](docs/developer_guide.md)를 참고한다.

## 저장소와 참조 소스 받기

IsaacLab과 Hunter 원본 소스는 CORA-AX의 별도 저장소를 submodule로 연결한다.
연결 설정이 원격에 반영된 뒤 다른 컴퓨터에서는 다음 명령으로 함께 받는다.

```bash
git clone --recurse-submodules https://github.com/CORA-AX/shipyard_robot_sim.git
cd shipyard_robot_sim
```

[Organization 저장소 구성](docs/organization_sources.md)에 각 저장소의 역할과 현재 연결 상태가 있다.

## 필요한 환경과 의존성

| 작업 | 필요한 환경 |
|---|---|
| HUNTER URDF 생성·정적 검사·단위 테스트 | Python 3.10 이상 표준 라이브러리, Git, `third_party/`의 원본 저장소와 파일 |
| Isaac Sim 설치·실행 | Conda, Python 3.11, NVIDIA GPU 및 호스트 GPU 드라이버·OS 라이브러리 |
| Go2 평지 정책 실행 | 동일한 `.conda/isaacsim-5.0` 환경에 Isaac Lab v2.2.1 + RSL-RL 2.3.3 설치 |
| 원격 화면 연결 | 서버의 Isaac Sim과 노트북의 NVIDIA Isaac Sim WebRTC Streaming Client |

현재 프로젝트는 Linux 서버에서 **Conda로 Python 환경을 만들고 pip로 Isaac Sim과 PyTorch를 설치**하는 구성을 사용한다.
가상환경은 프로젝트 내부 `.conda/isaacsim-5.0`에 있다.
GPU 드라이버와 OS 라이브러리는 호스트에 준비되어 있어야 한다.

2026-10-08 통합한 환경의 주요 설치 버전은 다음과 같다. 전체 목록은
[Python 패키지 버전 기록](requirements.lock.txt)에 있다.

| 패키지 | 기록된 버전 / 용도 |
|---|---|
| Python | 3.11.16 |
| Isaac Sim | 제품 버전 5.0.0, pip 배포 버전 5.0.0.0 |
| PyTorch / torchvision | `2.7.0+cu128` / `0.22.0+cu128`, CUDA 12.8용 wheel |
| NumPy / SciPy | 1.26.0 / 1.15.3 |
| trimesh / OpenCV headless | 4.5.1 / 4.11.0.86 |
| packaging / wheel | 23.0 / 0.45.1 |

`isaacsim[all,extscache]`는 GUI, 로봇·센서, URDF importer, ROS 연동, Replicator, RL 등의 전체 기능과
Kit/SDK/Physics 확장 캐시를 포함한다. `extscache`는 실행용 확장 파일이며 pip 다운로드 캐시와 별개다.
Isaac Lab은 `third_party/IsaacLab`의 고정 소스를 동일한 Conda 환경에 editable 설치한다.
`isaacsim-ros2` 패키지만으로 별도 ROS 2 배포판 전체가 설치되는 것은 아니다.
URDF 정적 검사에는 ROS나 Isaac Sim 설치가 필요하지 않다.

## 환경 정의 파일의 역할

| 파일 | 용도 |
|---|---|
| [environment.isaacsim.yml](environment.isaacsim.yml) | Conda 기반 환경 생성: Python 3.11과 pip |
| [requirements.txt](requirements.txt) | Isaac Sim·Isaac Lab·RSL-RL 통합 설치 요구사항, NVIDIA 인덱스·editable 소스 |
| [requirements.lock.txt](requirements.lock.txt) | 검증한 Python 패키지 230개 버전 제약. `requirements.txt`가 `-c`로 참조 |
| [conda-isaacsim-linux-64.lock.txt](conda-isaacsim-linux-64.lock.txt) | Linux용 Conda 기반 패키지 설치 기록 |

requirements는 **설치 요구사항**과 **검증된 버전 제약** 두 파일로 관리한다.
CUDA PyTorch를 먼저 설치하고 submodule·빌드 도구를 준비한 뒤 `requirements.txt`로 통합 설치한다.
`--no-build-isolation`을 사용하므로 setuptools·wheel·toml을 먼저 설치해야 한다.
`requirements.lock.txt`는 해시·다운로드 출처까지 고정한 완전한 lock 파일은 아니다.
버전 제약은 의존성 설치에 적용되지만 기록된 모든 패키지를 별도로 설치하는 목록은 아니다.

Isaac Sim 5.0의 `packaging==23.0` 요구사항과 호환되도록 `wheel==0.45.1`을 지정했다.
pip가 설치 과정에서 packaging/wheel 버전을 바꿨으므로 Conda 기록만으로 전체 환경을 복원할 수 없다.

## 새 환경 설치

Conda를 사용할 수 있는 Bash 터미널에서 프로젝트 루트로 이동한다.
아래 경로는 현재 서버 기준이며, 다른 위치에 프로젝트를 둔 경우 해당 경로로 바꾼다.
이미 `.conda/isaacsim-5.0` 환경이 있다면 다음 절의 활성화부터 진행한다.

```bash
cd /home/rodix-gpu/shipyard_robot_sim

# 현재 서버의 Conda 초기화. 이미 초기화된 셸이면 생략한다.
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh

# 1. 프로젝트 내부에 Python 3.11 환경 생성
conda env create --prefix ./.conda/isaacsim-5.0 -f environment.isaacsim.yml
conda activate "$PWD/.conda/isaacsim-5.0"

# 2. CUDA 12.8용 PyTorch를 먼저 설치
python -m pip install 'torch==2.7.0' 'torchvision==0.22.0' --index-url https://download.pytorch.org/whl/cu128

# 3. 원본 소스와 editable 설치에 필요한 빌드 도구 준비
git submodule update --init --recursive
python -m pip install -c requirements.lock.txt setuptools wheel toml

# 4. Isaac Sim + Isaac Lab 통합 설치
python -m pip install --no-build-isolation -r requirements.txt
python -m pip check
```

다른 서버에서는 `source` 경로를 해당 Conda 설치 경로로 바꾼다.
최초 패키지 설치에는 패키지 인덱스에 접근할 수 있는 네트워크가 필요하다.

## 기존 환경 활성화와 설치 확인

새 터미널을 열 때마다 프로젝트 루트에서 활성화한다.

```bash
cd /home/rodix-gpu/shipyard_robot_sim
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate "$PWD/.conda/isaacsim-5.0"
which python
python --version
python -m pip check
```

GPU에 접근 가능한 호스트에서 패키지·CUDA 연산과 Isaac Sim 실행을 확인한다.

```bash
python tests/check_isaacsim_install.py
python tests/smoke_test_isaacsim.py
```

첫 번째 스크립트는 Python 3.11, 프로젝트 환경 경로, 주요 패키지, CUDA tensor 연산을 검사한다.
두 번째 스크립트는 headless Kit 실행, URDF importer 명령, 큐브 낙하를 통한 PhysX 동작을 확인한다.
기본 제어 정책은 `python scripts/go2/prepare_go2_policy.py --control-policy`로 준비한다.
공식 정책을 사용할 때는 `python scripts/go2/prepare_go2_policy.py`로 준비하고
제어기에 `--checkpoint assets/go2/policies/flat.pt`를 지정한다.
Isaac Sim 첫 실행 시에는 NVIDIA 라이선스 동의 안내에 직접 응답해야 한다.
검사 결과는 `docs/isaacsim_install_evidence/`에 기록된다. HUNTER의 USD 변환·주행 검증은 별도 작업이다.

서버 WebRTC 실행 명령, 연결 포트, 프로젝트 내부 cache/config 설정은
[Isaac Sim 설정 문서](docs/isaacsim_5_setup.md)를 참고한다.
Go2 설치와 정책 준비는 [Go2 실행 문서](docs/go2_setup.md)를 참고한다.
환경 사용을 마치면 `conda deactivate`로 활성화를 해제한다.

## Isaac Sim 없이 HUNTER asset 검사

```bash
cd /home/rodix-gpu/shipyard_robot_sim
python3 scripts/hunter2/prepare_hunter2_asset.py
python3 scripts/hunter2/validate_hunter2_asset.py
python3 -m unittest discover -s tests -v
```

위 명령은 Python 3.10 이상 환경에서 실행할 수 있다. 생성 스크립트는 로컬 `third_party/` 저장소의
commit과 원본 파일의 SHA-256을 확인하므로 해당 저장소와 Git metadata가 있어야 한다.
원본이 준비되어 있으면 네트워크 접속 없이 동작한다.

## 로봇 자료와 현재 상태

- [최신 조향축·중복 형상 교정 결과](docs/hunter2_source_correction.md)
- [HUNTER 분석 및 변경 내역](assets/hunter2/README.md)
- [향후 USD import 계획](docs/hunter2_usd_import_plan.md)
- [Isaac Sim 입력 적합성 추가 검토](docs/hunter2_isaac_source_review.md): 조향축 및 중복 wheel mesh 정량 검증
- [공식 source commit 및 SHA-256](assets/hunter2/config/sources.lock.json)
- [구조 검사와 좌표·mesh 분석 결과](assets/hunter2/config/validation_report.json)

`scenarios/`는 공통 장면과 로봇별 환경 연결, `assets/environments/`는 공통 배경 자산을 관리한다.
`control/go2/`는 정책 제어·환경 실행·상태 검증 코드를 관리한다.
`control/{hunter2,omni}`는 향후 제어 adapter용 관리 문서를 두었다.
`navigation`, `ros2_ws`는 향후 작업용 디렉터리다.
[시나리오 관리 기준](scenarios/README.md)과 [로봇 제어 관리 기준](control/README.md)을 참고한다.
상위 navigation은 Goal/Path와 공통 상태 인터페이스를 사용하고 로봇별 adapter를 둔다.
프로젝트 공통 controller/adapter와 waypoint follower는 아직 구현하지 않았다.
Go2는 `scripts/viewer.py --robot go2`로 생성하고 `scripts/go2/control.py`로 제어한다.

**HUNTER 상태: 조향축·중복 형상 교정 및 정적 검증 완료, Isaac import·물리/주행 검증 미완료.**
독립 조향 운동 검증을 정상 Ackermann 주행 controller 완성으로 해석하지 않는다.
