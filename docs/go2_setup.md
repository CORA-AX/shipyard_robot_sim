# 공통 Conda 환경에서 Go2 실행

Isaac Sim과 Go2 모두 기존 `.conda/isaacsim-5.0` 환경에서 실행한다.
복제 환경에서 검증한 Isaac Lab 의존성을 기존 환경에 통합했다.
통합한 환경에서도 CUDA 검사와 Go2 500스텝(10초) 실행을 통과했다.
통합 전 복제 환경에서 2026-10-08에 RTX 3090 한 대로 1,500 제어 스텝(시뮬레이션 30초)을 실행했다.
전진 거리 7.45m, 리셋 0회로 실행 검사를 통과했다.

## 구성과 버전 선택

| 항목 | 프로젝트에서 사용하는 구성 |
|---|---|
| 가상환경 | `.conda/isaacsim-5.0`, Python 3.11 |
| Isaac Sim / PyTorch | 5.0.0.0 / 2.7.0+cu128 |
| Isaac Lab | `third_party/IsaacLab`, 태그 `v2.2.1` |
| Isaac Lab commit | `0f00ca2b4b2d54d5f90006a92abb1b00a72b2f20` |
| Isaac Lab Python 패키지 버전 | `isaaclab==0.45.9` — 저장소 릴리스 번호와 별개 |
| RSL-RL / Warp | 2.3.3 / 1.7.1 |
| 정책 | NVIDIA Go2 평지 사전 학습 체크포인트 |

홈의 `/home/rodix-gpu/IsaacLab` 저장소를 로컬 복제한 뒤 호환 태그를 선택했다.
홈 저장소의 현재 HEAD는 `b4c321024792976150ca55fddb26fa34480d974e`이며,
`VERSION`은 2.3.2지만 태그 `v2.3.2` 자체를 체크아웃한 상태는 아니다.
현재 HEAD의 Pillow 11.3.0 / Starlette 0.49.1 요구사항은
Isaac Sim 5.0의 Pillow 11.2.1 / FastAPI 0.115.7 요구사항과 충돌한다.
따라서 홈 HEAD를 그대로 연결하는 대신, 같은 저장소의 `v2.2.1`을 프로젝트에서 고정했다.

설치 경로는 프로젝트 내부 소스에 대한 editable 설치다. 홈의 소스나 `env_isaaclab`을 수정하지 않는다.
통합 전 기본 환경의 기존 패키지 중 setuptools만 84.0.0에서 80.9.0으로 변경했다.
이는 flatdict 4.0.1 빌드에 필요한 `pkg_resources`를 제공하기 위한 것이다.

## WebRTC로 Go2 보기

프로젝트 루트에서 공통 환경을 활성화하고, 클라이언트에서 접근 가능한 서버 IP를 지정한다.
`192.168.0.10`은 실제 서버의 LAN/VPN 주소로 바꾼다.

```bash
cd /home/rodix-gpu/shipyard_robot_sim
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate "$PWD/.conda/isaacsim-5.0"
export PUBLIC_IP=192.168.0.10
python scripts/go2/prepare_go2_policy.py
```

노트북의 NVIDIA Isaac Sim WebRTC Streaming Client에서 같은 IP로 연결한다.
서버와 클라이언트 사이에 TCP 49100, UDP 47998 통신이 필요하다.
각 명령은 따로 실행하며, 다음 명령을 실행하기 전에 이전 실행을 종료한다.
실행기는 TCP 49100이 이미 사용 중이면 중복 실행을 거부한다.
Ctrl+C 또는 종료 신호를 받으면 자식 시뮬레이터를 종료하고,
10초 안에 응답하지 않으면 해당 실행의 프로세스 그룹을 강제 종료한다.

### 환경만 보기

```bash
python scripts/go2/run_go2_webrtc.py view
```

평지와 Go2 한 대를 생성한다. 학습 정책을 로드하지 않고 0 액션으로 물리를 실행한다.
따라서 보행 데모가 아니라 환경·로봇 확인용이다.
프로젝트의 `run_go2.py --mode view` 경로를 사용하며, 준비가 끝나면
`SHIPYARD_GO2_VIEW_READY`를 출력한다.

### 정책 실행 및 내보내기

```bash
python scripts/go2/run_go2_webrtc.py policy
```

평지 정책으로 실시간 보행을 실행하고 터미널에 Actor/Critic 구조를 출력한다.
프로젝트 실행기 `run_go2.py`가 Isaac Lab의 exporter API로 다음 파일을 내보낸다.

- `assets/go2/policies/exported/policy.pt`: TorchScript.
- `assets/go2/policies/exported/policy.onnx`: ONNX.

현재 Actor는 관측 48개 → 은닉층 128·128·128 → 액션 12개 구조다.
기본 시나리오와 검증 모드의 전진 명령은 0.4m/s이며 `--vx`로 변경한다.
정책 모드는 시나리오의 `commands` 설정을 따르고, 검증 모드는 고정 직진 명령을 사용한다.
환경 보기와 정책 데모는 실행 창을 종료하거나 터미널에서 Ctrl+C로 끝낸다.

### 전진 검증을 WebRTC로 보기

```bash
python scripts/go2/run_go2_webrtc.py check --steps 1500 --vx 0.4
```

Go2 한 대에 고정 전진 명령을 주고 실시간 속도로 1,500스텝(시뮬레이션 30초)을 실행한다.
정상 완료 시 `SHIPYARD_GO2_PASS`를 출력하고 자동 종료한다.
클라이언트 연결을 기다리지는 않으므로 서버 초기화가 끝나면 바로 연결한다.
결과는 `docs/go2_install_evidence/runtime_check.json`에 덮어쓴다.
`--vx`는 0.2~0.8m/s이며 기존 물리 검증값은 0.4m/s다.

`check` 결과 위치는 `--report 경로`로 바꿀 수 있다.

세 모드 모두 `--server-ip 주소`로 `PUBLIC_IP`를 대신할 수 있고,
`--device cuda:0`으로 사용할 GPU를 지정할 수 있다.
세 모드는 `--experience isaacsim.exp.full.streaming.kit`을 명시해,
단독 `isaacsim isaacsim.exp.full.streaming --no-window`와 같은 Full Streaming 구성을 사용한다.
Isaac Lab 기본 `isaaclab.python.kit` 대신 이 구성을 선택한다.
`scripts/visualization/stream_entry.py`는 Python SimulationApp의 `sync_loads=False`, `hide_ui=False`를 설정한다.
재질 로딩 중 화면을 막지 않고 스트리밍 UI를 표시하기 위한 설정이다.
`create_new_stage=False`로 Full Streaming의 초기 장면을 사용하고,
앱 초기화 후 Go2 자산 참조에 필요한 USD 동기 로딩을 켠다.
Full Streaming에 Isaac Lab의 Isaac 5.0 자산 서버 설정도 명시한다.
Go2 실행에 필요하지 않은 ROS Bridge 자동 시작은 끈다.
기존 단독 Isaac Sim은 먼저 종료해야 한다. 두 실행이 같은 스트리밍 포트를 사용한다.
WebRTC를 켜면 Isaac Lab이 서버를 headless로 실행하므로 데스크톱 디스플레이가 필요하지 않다.
캐시와 Kit 데이터는 `.cache/go2`, `.runtime/go2`에 저장하며 확장 자동 변경 감시를 끈다.
Full Streaming WebRTC 구성에서 Go2 500스텝(10초), 전진 2.48m, 리셋 0회 검증을 통과했다.
[실행 기록](go2_install_evidence/webrtc_runtime_check.json)을 참고한다.
정책 데모의 TorchScript·ONNX export, TorchScript 추론, ONNX 모델 구조 검사와
ONNX reference evaluator / TorchScript 출력 일치도 확인했다.
환경 보기의 `SHIPYARD_GO2_VIEW_READY` 출력과 Ctrl+C 이후 자식 프로세스·포트 해제도 확인했다.
실제 WebRTC 클라이언트 영상 수신은 별도 확인이 필요하다.

화면 없이 기존 검증만 실행할 때는 다음 명령을 사용한다.

```bash
python scripts/go2/run_go2.py --headless --device cuda:0 --steps 500
```

## 프로젝트 시나리오 수정

세 모드 모두 프로젝트의 `run_go2.py`에서 실행한다. 평지 설정은 `assets/go2`,
공통 조선소 장면과 Go2 연결 설정은 루트 `scenarios/shipyard`에서 읽는다.
Isaac Lab의 예제 `play.py`, Gym task registry, `isaaclab_tasks` 환경 설정을 사용하지 않는다.
Isaac Lab의 물리·환경·센서·MDP·정책 export API는 `third_party/IsaacLab`에서 계속 사용한다.

| 파일 | 수정 대상 |
|---|---|
| `assets/go2/config/robot_cfg.py` | 로봇 초기 자세, 관절 기본값, 모터 게인·제한 |
| `assets/go2/config/policy_cfg.py` | 체크포인트와 호환되는 Actor/Critic 구성 |
| `assets/go2/scenarios/flat_env_cfg.py` | 지면, 센서, 관측·액션, 명령, 리셋·종료 조건 |
| `scenarios/shipyard/scene_cfg.py` | 공통 지면·조명·화물 상자·카메라 |
| `scenarios/shipyard/go2_env_cfg.py` | 공통 장면에 Go2 로봇·센서와 제어 환경 연결 |
| `assets/go2/scenarios/__init__.py` | 새 시나리오 등록 |

기본 시나리오는 `flat`이다. 화물 상자 3개가 있는 시작 환경은 아래처럼 선택한다.

```bash
python scripts/go2/run_go2_webrtc.py view --scenario shipyard
python scripts/go2/run_go2_webrtc.py policy --scenario shipyard --vx 0.4
python scripts/go2/run_go2_webrtc.py check --scenario shipyard --steps 500
```

상자는 공통 `scene_cfg.py`의 `cargo_box()`로 생성한 정적 충돌체다.
위치 `(x, y, z)`, 크기 `(가로, 세로, 높이)`는 미터 단위이며 `z`는 상자 중심 높이다.
기본 상자는 직진 통로 옆에 배치했다. 실제 조선소 모델을 가져온 환경은 아니다.
환경 변경만으로 장애물 회피가 생기지는 않는다. 현재 평지 정책에는 장애물 관측이 없다.
관측 48개의 순서, 관절 순서, 액션 12개·스케일 0.25, 제어 주기 0.02초를 바꾸면
현재 체크포인트와 호환되지 않을 수 있으며 별도 정책이나 재학습이 필요하다.
로컬 reward 설정은 실행·확인용이며 기존 학습 환경 전체를 복제한 학습 설정은 아니다.

새 환경은 `FlatGo2EnvCfg`를 상속하고 `SCENARIOS`에 파일과 클래스 이름을 등록한다.
그 뒤 `--scenario 이름`으로 실행할 수 있다. `third_party/IsaacLab` 수정은 필요 없다.
`view`와 `policy`는 기본적으로 계속 실행하며 `--steps 100`처럼 실행 길이를 제한할 수 있다.
`view`는 0 액션 물리를 실행하므로 로봇이 내려앉을 수 있다.

## 필요한 파일

| 파일 | 역할 |
|---|---|
| [requirements-go2-isaaclab.txt](../requirements-go2-isaaclab.txt) | 공통 환경에 추가할 Isaac Lab 패키지와 버전 조건 |
| [requirements-isaacsim.lock.txt](../requirements-isaacsim.lock.txt) | 공통 환경 전체의 pip 버전 제약 목록. 해시 lock은 아님 |
| [sources.lock.json](../assets/go2/config/sources.lock.json) | 소스 commit, 모델 URL, 정책 URL·SHA-256 |
| `assets/go2/policies/flat.pt` | 로컬 정책 파일. Git에는 포함하지 않고 준비 스크립트로 복원 |
| [prepare_go2_policy.py](../scripts/go2/prepare_go2_policy.py) | 공식 정책 다운로드 및 SHA-256 검사 |
| [stream_entry.py](../scripts/visualization/stream_entry.py) | 공통 스트리밍용 앱 초기화 및 로봇 스크립트 실행 |
| [webrtc.py](../scripts/visualization/webrtc.py) | 공통 WebRTC 설정, 포트 검사, 자식 프로세스 종료 처리 |
| [runtime.py](../scripts/visualization/runtime.py) | 공통 캐시·Kit 렌더링 설정과 실시간 표시 속도 |
| [run_go2_webrtc.py](../scripts/go2/run_go2_webrtc.py) | 환경 보기·정책 실행 및 export·전진 검증의 공통 WebRTC 실행기 |
| [run_go2.py](../scripts/go2/run_go2.py) | 고정 소스·정책 검사, Go2 생성, GPU 정책 실행, 결과 기록 |

Go2 USD와 참조 모델 파일은 NVIDIA의 Isaac 5.0 자산 서버에서 로드한다.
`assets/go2`에 전체 USD/mesh를 오프라인으로 복제한 구성은 아니므로 모델 로딩에는 서버 접근이 필요할 수 있다.
정책과 Isaac Lab 소스는 해시로 고정했지만, 원격 모델의 모든 참조 파일까지 해시로 고정한 구성은 아니다.
정책은 약 0.94MiB이며 준비 스크립트가 이미 검증된 파일을 발견하면 재다운로드하지 않는다.

## 기존 환경에 Isaac Lab을 추가하거나 새로 설치하는 경우

[공통 Isaac Sim 설치](isaacsim_5_setup.md)에 따라 `.conda/isaacsim-5.0`을 준비한 뒤
프로젝트 루트에서 아래를 실행한다. Go2 전용 Conda 환경은 만들지 않는다.

```bash
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate "$PWD/.conda/isaacsim-5.0"
export PIP_CACHE_DIR="$PWD/.cache/pip"
git submodule update --init --recursive
python -m pip install 'setuptools==80.9.0'
python -m pip install --no-build-isolation -r requirements-go2-isaaclab.txt
python -m pip check
python scripts/go2/prepare_go2_policy.py
python scripts/go2/run_go2.py --headless --device cuda:0 --steps 500
```

기존 소스 폴더에 Git 메타데이터가 없으면 submodule 연결 정리가 필요하다.
[현재 연결 상태](organization_sources.md)를 참고한다.
Isaac Lab commit은 위 표의 값과 일치해야 한다. 필요한 RSL-RL extra만 설치한다.
`check_isaacsim_install.py`와 `run_go2.py` 모두 같은 공통 환경 경로를 검사한다.

## 검증 결과와 범위

프로젝트 시나리오 분리 후의 기록은 [자체 시나리오 실행 검사](go2_install_evidence/local_scenarios_check.json)에 있다.
`shipyard`의 WebRTC 환경 보기·정책·전진 검증, 화물 충돌체 생성,
정책 TorchScript·ONNX export와 원본 정책 출력 일치를 검사했다.
500스텝 전진 검증은 2.48m, 리셋 0회로 기존 결과와 일치했다.
이 검사는 스트리밍 서버 실행을 확인하며 클라이언트에서 영상 수신까지 확인한 기록은 아니다.


- [통합 전 환경 검사](go2_install_evidence/environment_check.json): 복제 환경 테스트 당시 기록.
- 통합 시 공통 환경의 패키지 230개가 검증된 복제 환경과 같은 버전임을 확인했다.
- [통합 환경 검사](environment_unification/environment_check.json): 공통 경로, 패키지 버전, 의존성 검사 결과.
- [통합 후 실행 기록](environment_unification/runtime_check.json): 기존 환경에서 500스텝, 2.48m 전진, 리셋 0회.
- [기본 물리 검사](isaacsim_install_evidence/runtime_check.json): Kit·PhysX·URDF importer 통과.
- [통합 전 30초 실행 기록](go2_install_evidence/runtime_30s.json): 1,500스텝, 7.45m 전진, 리셋 0회.
- 명령 속도 0.4m/s에 대해 평균 몸체 전방 속도는 약 0.248m/s였다.
- 몸체 높이 최저값은 약 0.151m였다. 정상 높이·속도 추종 성능을 보증하는 검사는 아니다.
- 합격 조건은 유한한 정책 출력·로봇 상태, 지정 스텝 완료, 리셋 0회, 몸체 높이 0.15m 초과, 전진 거리 0.1m 초과다.
- 초기화 중 Warp의 `cuDeviceGetUuid` 관련 메시지와 GPU P2P 경고가 남지만, 해당 실행의 GPU 정책 추론·물리 스텝·종료는 완료됐다.

현재 구성은 프로젝트 소유의 Go2 평지 및 화물 상자 시나리오 실행이다.
Hunter와 같은 장면에 배치하기, 조선소 경로 추종, ROS 2 연결, 험지 정책, 추가 학습은 아직 검증하지 않았다.

## 로컬 화면과 WebRTC 선택

Go2 실행기에도 `--webrtc` / `--no-webrtc`를 사용할 수 있다.

```bash
python scripts/go2/run_go2.py --mode view --scenario shipyard --no-webrtc
python scripts/go2/run_go2.py --mode view --scenario shipyard --webrtc --server-ip 192.168.1.149
python scripts/go2/run_go2_webrtc.py view --scenario shipyard --no-webrtc
```

WebRTC 사용 시 headless, 미사용 시 서버 데스크톱 창으로 실행한다.
기존 `run_go2_webrtc.py`는 기본 WebRTC를 유지한다.
로봇 없이 장면만 보려면 `scripts/visualization/view_scenario.py`를 사용한다.
로컬 창은 해당 데스크톱의 DISPLAY와 화면 접근 권한이 필요하다.
