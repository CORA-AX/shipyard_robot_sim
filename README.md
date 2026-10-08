# Shipyard robot simulation

전체 명령 옵션·기본값·실행 예시는 [커맨드 옵션 안내](command_options.md)를 참고한다.

Isaac Sim 5.0 + Isaac Lab v2.2.1 기반 조선소 로봇 시뮬레이션 프로젝트다.
현재 Go2 한 대의 환경 표시·정책 제어를 지원한다. Hunter2는 자산 준비·정적 검증 단계이며
여러 로봇의 동시 실행은 아직 구현하지 않았다.

**viewer가 환경·로봇·물리를 관리하고, 별도 control 프로세스가 정책을 실행한다.**
명령은 모두 프로젝트 루트에서 실행한다. 공통 Conda 환경은 `.conda/isaacsim-5.0`이다.
Isaac Lab은 `third_party/IsaacLab`을 사용한다.

## 기존 서버에서 바로 실행

사용할 터미널마다 환경을 활성화한다. 다른 컴퓨터에서는 프로젝트·Conda 경로를 바꾼다.

```bash
cd /home/rodix-gpu/shipyard_robot_sim
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate "$PWD/.conda/isaacsim-5.0"
```

정책 실행 전 체크포인트를 준비한다. 이미 있으면 체크섬만 확인한다.

```bash
python scripts/go2/prepare_go2_policy.py
```

## 환경과 로봇 표시

| 목적 | viewer 옵션 |
|---|---|
| 조선소 환경만 표시 | `--scenario shipyard` |
| 평지 환경만 표시 | `--scenario flat` |
| 환경에 Go2 추가 | `--robot go2` |
| 서버 로컬 화면 | `--no-webrtc` (기본값) |
| 원격 WebRTC 화면 | `--webrtc --server-ip 서버_IP` |

```bash
# 환경만 표시
python scripts/viewer.py --scenario shipyard --no-webrtc

# 환경 + Go2 표시: 로컬 창
python scripts/viewer.py --scenario shipyard --robot go2 --no-webrtc

# 환경 + Go2 표시: WebRTC + headless
python scripts/viewer.py --scenario shipyard --robot go2 \
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
현재 사전 학습 정책은 낮은 보행 자세(몸체 기준 높이 약 0.16m)를 보인다. 정상 자세의 보행은 아직 검증되지 않았다.
[몸체 높이 비교 검사](docs/go2_setup.md#몸체-높이-검사)를 참고한다.
제어기에서 Ctrl+C를 누르면 물리를 멈추고 viewer 화면은 유지한다.
같은 명령을 다시 실행하면 현재 상태에서 재개한다. 전체 종료는 viewer도 종료한다.
한 Go2에 제어기 하나만 연결할 수 있으며, WebRTC viewer도 한 번에 하나씩 실행한다.

## 제어 테스트와 결과 저장

```bash
# 터미널 1: 총 500개 액션 적용 후 종료
python scripts/viewer.py --scenario shipyard --robot go2 --no-webrtc \
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

## 코드 수정 위치

| 폴더 | 역할 |
|---|---|
| `assets/<robot>/` | 모델·관절·모터 설정·정책 파일·출처 |
| `scenarios/` | 공통 배경·장애물·로봇 배치·관측·액션·리셋 설정 |
| `control/<robot>/` | 로봇별 정책 추론과 viewer 내부 물리·상태 처리 |
| `control/control_channel.py` | viewer·제어기 사이의 로컬 통신 |
| `scripts/viewer.py` | 환경·로봇 선택 및 화면 실행 명령 |
| `scripts/<robot>/` | 정책 준비·제어 연결·자산 관리 명령 |
| `scripts/visualization/` | 공통 로컬/WebRTC 초기화·캐시·프로세스 관리 |
| `tests/` | 단위 테스트·설치 및 기본 물리 검사 |
| `third_party/` | submodule로 연결한 외부 소스 |
| `.runtime/`, `.cache/`, `.conda/` | 로컬 실행 결과·캐시·가상환경 |

장애물 위치·크기·카메라는 `scenarios/shipyard/scene_cfg.py`, Go2 모터 설정은
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
Go2 정책이 없으면 먼저 `python scripts/go2/prepare_go2_policy.py`를 실행한다.
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
