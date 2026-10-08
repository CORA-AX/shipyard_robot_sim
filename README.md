# Shipyard robot simulation

조선소 이기종 로봇 시뮬레이션을 위한 source asset 준비 프로젝트.
HUNTER 2.0 URDF 교정·정적 검증을 완료했고, Isaac Sim 5.0 / Python 3.11 환경을 사용한다.
Isaac Sim과 Go2는 `.conda/isaacsim-5.0` 환경 하나를 사용한다. Go2는 `assets/go2`의 자체 시나리오에서 공식 평지 정책을 실행하며 Isaac Lab v2.2.1을 라이브러리로 사용한다.
[Go2 설치·실행 안내](docs/go2_setup.md)에 의존성 및 테스트 범위가 있다.

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
[Python 패키지 버전 기록](requirements-isaacsim.lock.txt)에 있다.

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
| [requirements-isaacsim.txt](requirements-isaacsim.txt) | 기본 설치용: `isaacsim[all,extscache]==5.0.0`, `wheel==0.45.1`, NVIDIA 패키지 인덱스 |
| [requirements-isaacsim.lock.txt](requirements-isaacsim.lock.txt) | 통합 환경의 Python 배포 패키지 230개 버전 기록: 설치 제약·환경 비교용 |
| [conda-isaacsim-linux-64.lock.txt](conda-isaacsim-linux-64.lock.txt) | Linux용 Conda 기반 패키지 설치 기록 |
| [requirements-go2-isaaclab.txt](requirements-go2-isaaclab.txt) | 동일 환경에 추가하는 Isaac Lab·RSL-RL 의존성. [Go2 설치 절차](docs/go2_setup.md) 참고 |

두 `requirements-isaacsim` 파일은 **설치 요구사항**과 **설치 결과 기록**을 나누기 위해 유지한다.
기본 설치에는 `requirements-isaacsim.txt`를 사용하고 CUDA PyTorch는 아래 명령으로 먼저 설치한다.
`.lock.txt`는 해시와 다운로드 출처까지 고정한 lock 파일이 아니라 버전 목록이다.
기본 설치 절차는 모든 하위 의존성까지 기존 환경과 같은 버전으로 재현한다고 보장하지 않는다.

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

# 3. Isaac Sim 전체 기능과 확장 캐시 설치
python -m pip install -r requirements-isaacsim.txt

# 4. 같은 환경에 Isaac Lab과 Go2 의존성 설치
git submodule update --init --recursive
python -m pip install 'setuptools==80.9.0'
python -m pip install --no-build-isolation -r requirements-go2-isaaclab.txt
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
python scripts/check_isaacsim_install.py
python scripts/smoke_test_isaacsim.py
python scripts/go2/run_go2.py --headless --device cuda:0 --steps 500
```

첫 번째 스크립트는 Python 3.11, 프로젝트 환경 경로, 주요 패키지, CUDA tensor 연산을 검사한다.
두 번째 스크립트는 headless Kit 실행, URDF importer 명령, 큐브 낙하를 통한 PhysX 동작을 확인한다.
세 번째는 같은 환경에서 Go2 평지 정책을 실행한다. 정책이 없으면 먼저 `python scripts/go2/prepare_go2_policy.py`를 실행한다.
Isaac Sim 첫 실행 시에는 NVIDIA 라이선스 동의 안내에 직접 응답해야 한다.
검사 결과는 `docs/isaacsim_install_evidence/`에 기록된다. HUNTER의 USD 변환·주행 검증은 별도 작업이다.

서버 WebRTC 실행 명령, 연결 포트, 프로젝트 내부 cache/config 설정은
[Isaac Sim 설정 문서](docs/isaacsim_5_setup.md)를 참고한다.
Go2는 아래처럼 환경 보기, 정책 실행·내보내기, 전진 검증을 모두 WebRTC로 볼 수 있다.
`192.168.0.10`을 클라이언트에서 접근 가능한 서버 주소로 바꾼다.
명령은 한 번에 하나씩 실행하고 WebRTC Streaming Client에서 같은 IP로 연결한다.

```bash
export PUBLIC_IP=192.168.0.10
python scripts/go2/run_go2_webrtc.py view
# 이전 실행을 종료한 후 정책 데모 실행
python scripts/go2/run_go2_webrtc.py policy
# 이전 실행을 종료한 후 30초 전진 검증 실행
python scripts/go2/run_go2_webrtc.py check --steps 1500 --vx 0.4
```

`--scenario shipyard`를 추가하면 화물 상자 3개가 있는 프로젝트 환경을 선택한다.
`scenarios/shipyard/scene_cfg.py`에서 배치·크기·카메라를 수정한다.
세 모드 모두 프로젝트 실행기를 사용한다.

정책 export 위치와 연결 방법은 [Go2 실행 문서](docs/go2_setup.md)를 참고한다.
환경 사용을 마치면 `conda deactivate`로 활성화를 해제한다.

## 시나리오만 보기 및 화면 선택

로봇 없이 환경만 보는 공통 viewer다. 기본값은 서버 데스크톱의 로컬 창이다.

```bash
python scripts/visualization/view_scenario.py --scenario shipyard --no-webrtc
python scripts/visualization/view_scenario.py --scenario flat --no-webrtc
python scripts/visualization/view_scenario.py --scenario shipyard --webrtc --server-ip 192.168.1.149
```

`--webrtc`는 headless 스트리밍, `--no-webrtc`는 서버 로컬 창을 사용한다.
Go2 보기에도 같은 옵션을 사용할 수 있다.

```bash
python scripts/go2/run_go2.py --mode view --scenario shipyard --no-webrtc
python scripts/go2/run_go2.py --mode view --scenario shipyard --webrtc --server-ip 192.168.1.149
```

기존 `run_go2_webrtc.py`는 기본 WebRTC를 유지하며 `--no-webrtc`도 지원한다.
로컬 실행은 서버 데스크톱 세션의 화면 접근 권한이 필요하다.
자세한 표시 설정은 [viewer 문서](scripts/visualization/README.md)를 참고한다.

## 실행 스크립트 구성

- `scripts/go2/`: Go2 정책 준비·물리 실행과 시나리오 선택.
- `scripts/hunter2/`: 자산 준비·검증, 모델 비교, 운동학 검토.
- `scripts/visualization/`: 공통 WebRTC 연결·렌더링·실시간 표시·프로세스 종료 처리.
- `scripts/`: 공통 Isaac Sim 설치 검사와 기본 물리 검사.

Go2 실행 코드는 WebRTC 주소·포트와 스트리밍 초기화 우회 설정을 직접 관리하지 않는다.
시각화 모듈의 역할과 호출 흐름은 [구조 설명](scripts/visualization/README.md)을 참고한다.

명령은 프로젝트 루트에서 실행한다. 로봇 자산·Conda·Isaac Lab 경로는
프로젝트 루트를 기준으로 계산하므로 스크립트 이동 후에도 같은 구성을 사용한다.

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
`robots/{hunter2,go2,omni}`는 향후 제어 adapter용 관리 문서를 두었다.
`navigation`, `ros2_ws`는 향후 작업용 디렉터리다.
[시나리오 관리 기준](scenarios/README.md)과 [로봇 제어 관리 기준](robots/README.md)을 참고한다.
상위 navigation은 Goal/Path와 공통 상태 인터페이스를 사용하고 로봇별 adapter를 둔다.
프로젝트 공통 controller/adapter와 waypoint follower는 아직 구현하지 않았다.
Go2 단독 정책 실행은 `scripts/go2/run_go2.py`를 사용한다.

**HUNTER 상태: 조향축·중복 형상 교정 및 정적 검증 완료, Isaac import·물리/주행 검증 미완료.**
독립 조향 운동 검증을 정상 Ackermann 주행 controller 완성으로 해석하지 않는다.
