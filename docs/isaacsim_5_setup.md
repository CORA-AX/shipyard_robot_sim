# Isaac Sim 5.0 설치 현황과 직접 실행

2026-10-07 실제 설치 파일 및 pip metadata 기준. 프로젝트 환경은 **하나**다.

## 설치 구조

- Conda 관리자: 기존 `/home/rodix-gpu/miniforge3` 재사용. 프로젝트에 Conda 관리자를 새로 설치하지 않았다.
- 가상환경: `/home/rodix-gpu/shipyard_robot_sim/.conda/isaacsim-5.0` (약 18 GiB).
- Conda로 Python 3.11 환경을 만들고, 그 안에 pip로 CUDA 12.8용 PyTorch와 `isaacsim[all,extscache]==5.0.0`을 설치했다.
- Docker나 별도 Isaac Sim 압축 배포판을 사용한 설치가 아니다.
- GPU 드라이버와 OS 라이브러리는 서버에 있다. 프로젝트 환경에는 PyTorch용 CUDA 라이브러리가 들어 있다.
- 다른 프로젝트의 Conda 환경은 이 프로젝트의 환경이 아니다.

| 실제 설치 항목 | 버전 / 의미 |
|---|---|
| Python | 3.11.16 |
| Isaac Sim | 5.0.0.0 (제품 버전 5.0.0), 관련 pip 배포 패키지 총 25개 |
| Omniverse Kit | 107.3.1.206797 |
| torch / torchvision / torchaudio | 2.7.0+cu128 / 0.22.0+cu128 / 2.7.0 |
| NumPy / SciPy | 1.26.0 / 1.15.3 |
| trimesh / OpenCV headless | 4.5.1 / 4.11.0.86 |
| pip / setuptools | 26.2.1 / 84.0.0 |
| packaging / wheel | 23.0 / 0.45.1 |

pip가 인식하는 Python 배포 패키지는 **155개**다. 정확한 전체 이름과 버전은
[기존 패키지 목록 파일](../requirements-isaacsim.lock.txt)에 기록했다.
Isaac Sim이 확장 폴더에 포함한 USD·PhysX·WebRTC 등의 구성은 별도 pip 항목과 일대일 대응하지 않는다.
`extscache` 3개 패키지는 실행에 쓰이는 Kit/SDK/Physics 확장 파일이며,
프로젝트 `.cache`의 pip 다운로드 캐시와 다르다.

현재 구성은 최소 설치가 아니라 **전체 기능 + 확장 캐시** 설치다.
GUI, 로봇/센서, URDF import, ROS 연동, Replicator, RL 관련 기능, 예제, 테스트 등의 패키지가 포함되어 있다.
Isaac Lab 자체는 설치되어 있지 않다. `isaacsim-ros2`는 Isaac의 ROS 연동 구성이고,
별도 ROS 2 배포판 전체를 설치한 것을 의미하지 않는다.
이번 정리는 실행 방법을 단순화한 것이며 설치 패키지를 제거하지 않았다.

Conda에는 기반 패키지 27개의 설치 기록이 남아 있다.
[Conda explicit 파일](../conda-isaacsim-linux-64.lock.txt)은 이 기반 환경의 기록이다.
**주의: pip 설치가 packaging/wheel을 각각 23.0/0.45.1로 바꿨으므로 Conda 기록의
26.3/0.48.0과 다르다.** 실제 Python 패키지 버전은 위 pip 목록을 따른다.
따라서 Conda explicit 파일만으로 현재 전체 환경을 복원할 수 없다.

## 평소 사용할 명령

새 Bash 터미널에서 프로젝트로 이동해 직접 활성화한다.

```bash
cd /home/rodix-gpu/shipyard_robot_sim
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate /home/rodix-gpu/shipyard_robot_sim/.conda/isaacsim-5.0
which python
python --version
```

`conda`가 이미 셸에 초기화되어 있다면 `source .../conda.sh`는 생략할 수 있다.
이 파일은 Conda가 제공하는 표준 셸 초기화 파일이며 프로젝트용 보조 스크립트가 아니다.
활성화를 해제할 때는 `conda deactivate`를 사용한다.

기존 프로젝트 내부 cache/config 경로를 계속 사용하려면 활성화 후 아래를 실행한다.
이 설정 없이도 환경 활성화는 되지만, Isaac 관련 도구 일부가 기본 사용자 경로를 사용할 수 있다.

```bash
export PYTHONNOUSERSITE=1
export PIP_CACHE_DIR="$PWD/.cache/pip"
export CONDA_PKGS_DIRS="$PWD/.cache/conda/pkgs"
export XDG_CACHE_HOME="$PWD/.cache/xdg"
export XDG_CONFIG_HOME="$PWD/.runtime/config"
export XDG_DATA_HOME="$PWD/.runtime/data"
export CUDA_CACHE_PATH="$PWD/.cache/cuda"
export TMPDIR="$PWD/.cache/tmp"
mkdir -p "$PIP_CACHE_DIR" "$CONDA_PKGS_DIRS" "$XDG_CACHE_HOME" "$XDG_CONFIG_HOME" "$XDG_DATA_HOME" "$CUDA_CACHE_PATH" "$TMPDIR"
```

SSH 서버에서 WebRTC로 실행한다. 아래 `192.168.0.10`은 노트북에서 접근 가능한
**서버 LAN/VPN IPv4**로 바꾼다. 기존 Isaac Sim은 해당 터미널에서 Ctrl+C로 종료한다.

```bash
isaacsim isaacsim.exp.full.streaming --no-window \
  --/app/livestream/publicEndpointAddress=192.168.0.10 \
  --/app/livestream/port=49100 \
  --/isaac/startup/ros_bridge_extension= \
  --portable --portable-root "$PWD/.runtime/kit" \
  --/renderer/multiGpu/enabled=false
```

노트북의 NVIDIA Isaac Sim WebRTC Streaming Client에서 같은 서버 IP를 입력하고 Connect한다.
TCP 49100, UDP 47998 통신이 필요하다. SSH 접속 성공만으로 이 포트의 통신을 보장하지 않는다.
이 명령은 화면 확인을 위해 ROS Bridge 자동 시작을 끈다. ROS 연동 시 환경 설정 후 활성화한다.
서버에 데스크톱 창을 만들지 않아도 원격 UI를 조작할 수 있다.

설치된 5.0 streaming experience와 설정을 확인했으며,
[NVIDIA 5.0 안내](https://developer.nvidia.com/blog/isaac-sim-and-isaac-lab-are-now-available-for-early-developer-preview/)의
주소/포트 설정 및 [인접 버전 5.1 연결 문서](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html)와 대조했다.

## 직접 확인

활성화한 환경에서:

```bash
python -m pip list
python -m pip check
python scripts/check_isaacsim_install.py
python scripts/smoke_test_isaacsim.py
```

마지막 두 스크립트는 각각 GPU 연산 확인과 Kit/PhysX/URDF importer 검사용이다.
GPU에 접근 가능한 호스트에서 실행한다. HUNTER USD 변환·주행 검사는 별도다.
2026-10-07 재확인한 `pip check`는 통과했다. 이번 정리에서 Isaac Sim을 새로 실행하지 않았다.
이전 사용자 로그에서 라이선스 직접 동의 및 Vulkan 초기화는 확인했지만,
WebRTC 실제 화면 연결 및 차량 물리 검증 완료를 뜻하지 않는다.

## 재설치가 필요할 때만

현재 환경에는 다시 실행할 필요가 없다. 새 환경을 만드는 기본 설치 순서:

```bash
conda env create --prefix ./.conda/isaacsim-5.0 -f environment.isaacsim.yml
conda activate "$PWD/.conda/isaacsim-5.0"
python -m pip install 'torch==2.7.0' 'torchvision==0.22.0' --index-url https://download.pytorch.org/whl/cu128
python -m pip install -r requirements-isaacsim.txt
python -m pip check
```

이것은 주요 버전을 지정한 설치 절차이며 전이 의존성까지 같은 버전으로 재현한다고 보장하지 않는다.
전체 버전 대조에는 `requirements-isaacsim.lock.txt`를 사용한다.

## 파일 정리 범위

프로젝트 전용 설치·활성화·실행용 `.sh` 6개를 제거했다.
환경 정의 YAML, 설치 요구사항, 전체 버전 목록, 검증용 Python 코드는 유지한다.
HUNTER 원본/교정 모델, 검증 결과, 비교·교정 근거 문서는 유지한다.

- `.conda/isaacsim-5.0`: 실제 설치 환경. 실행에 필요하다.
- `.cache`: 다운로드/컴파일 캐시 (확인 시 약 1.7 GiB). 실제 환경과 별개이며 이번에 삭제하지 않았다.
- `.runtime`: 실행 설정·로그·Kit 데이터 (약 284 MiB). 기존 설정을 이어 쓰기 위해 유지했다.
- `logs`: 설치/실행 기록 (약 92 KiB).

기존 [설치 증거](isaacsim_install_evidence/installation_status.json)는 설치 당시 기록으로 보존한다.
로봇 작업의 다음 단계는 [USD import 계획](hunter2_usd_import_plan.md)을 따른다.
