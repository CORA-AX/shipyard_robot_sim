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

## 실행 방법

Go2는 viewer가 환경·로봇·물리를 담당하고 별도 제어기가 정책을 실행한다.
두 터미널 모두 프로젝트 루트에서 공통 Conda 환경을 활성화한다.

```bash
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate "$PWD/.conda/isaacsim-5.0"
python scripts/go2/prepare_go2_policy.py
```

```bash
# 터미널 1: 로컬 화면
python scripts/viewer.py --scenario shipyard --robot go2 --no-webrtc

# 또는 WebRTC + headless: 실제 서버 IP 지정
python scripts/viewer.py --scenario shipyard --robot go2 --webrtc --server-ip 192.168.1.149
```

```bash
# 터미널 2: 이미 생성된 Go2를 제어
python scripts/go2/control.py --vx 0.4
```

환경만 표시하려면 viewer의 `--robot go2`를 생략한다. 평지는 `--scenario flat`을 사용한다.
제어기는 기본 CPU에서 체크포인트 Actor를 추론하며 별도 Isaac Sim을 시작하지 않는다.
제어기 종료 시 물리는 정지하고 viewer는 유지된다. 다시 연결하면 현재 상태에서 재개한다.
자세한 제한 실행·보고서·재접속·통신 포트는 [분리 실행 안내](viewer_control.md)를 참고한다.

## 시나리오와 로봇 설정

| 파일 | 역할 |
|---|---|
| `assets/go2/config/robot_cfg.py` | 로봇 초기 자세·모터 설정 |
| `assets/go2/config/sources.lock.json` | 고정 소스·정책 출처·체크섬 |
| `scenarios/flat/go2_env_cfg.py` | Go2 센서·관측·액션·명령·리셋 |
| `scenarios/shipyard/scene_cfg.py` | 공통 지면·조명·화물·카메라 |
| `scenarios/shipyard/go2_env_cfg.py` | 조선소 배경과 Go2 연결 |
| `control/go2/policy.py` | 관측 48개 → 액션 12개 Actor 추론 |
| `control/go2/simulation.py` | viewer 내부 Go2 물리·관측·상태 기록 |

현재 정책은 메모리 없는 평지 Actor이며 장애물 회피를 수행하지 않는다.
관측 순서·관절 순서·액션 크기·제어 주기를 변경하면 체크포인트와 호환되지 않을 수 있다.
모델 USD와 mesh는 NVIDIA 서버에서 로드하며 정책은 준비 스크립트로 다운로드한다.

## 기존 환경에 Isaac Lab을 추가하거나 새로 설치하는 경우

[공통 Isaac Sim 설치](isaacsim_5_setup.md)에 따라 `.conda/isaacsim-5.0`을 준비한 뒤
프로젝트 루트에서 아래를 실행한다. Go2 전용 Conda 환경은 만들지 않는다.

```bash
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate "$PWD/.conda/isaacsim-5.0"
export PIP_CACHE_DIR="$PWD/.cache/pip"
git submodule update --init --recursive
python -m pip install -c requirements.lock.txt setuptools wheel toml
python -m pip install --no-build-isolation -r requirements.txt
python -m pip check
python scripts/go2/prepare_go2_policy.py
python tests/check_isaacsim_install.py
```

기존 소스 폴더에 Git 메타데이터가 없으면 submodule 연결 정리가 필요하다.
[현재 연결 상태](organization_sources.md)를 참고한다.
Isaac Lab commit은 위 표의 값과 일치해야 한다. 통합 `requirements.txt`가 Isaac Sim과
Isaac Lab·RSL-RL을 함께 설치하며, `requirements.lock.txt`의 버전 제약을 적용한다.
`tests/check_isaacsim_install.py`와 viewer는 같은 공통 환경 경로를 검사한다.

## 검증 결과와 범위

현재 분리 실행에서 정책 500스텝, 일시정지, 새 제어기 재접속 후 20스텝을 확인했다.
총 520스텝·10.4초, 전진 2.585m, 리셋 0회였고 대기 중 상태 보존 검사를 통과했다.
[현재 실행 기록](go2_install_evidence/external_control_check.json)을 참고한다.
`passed`는 실행·통신 완료 결과이며 자동 보행 성능 판정은 제공하지 않는다.
WebRTC 서버 실행을 검사했으며 클라이언트 영상 수신은 별도 확인이 필요하다.

과거 설치·통합 검증 기록은 보존한다. 당시 단독 실행 도구와 export 기능은 현재 제거됐다.

- [통합 환경 검사](environment_unification/environment_check.json)
- [통합 후 500스텝 기록](environment_unification/runtime_check.json)
- [과거 자체 시나리오·export 검사](go2_install_evidence/local_scenarios_check.json)
- [통합 전 30초 실행 기록](go2_install_evidence/runtime_30s.json)
- [Kit·PhysX·URDF importer 검사](isaacsim_install_evidence/runtime_check.json)

다중 로봇 배치, 조선소 경로 추종, ROS 2 연결, 험지 정책 및 추가 학습은 아직 구현·검증하지 않았다.

## 몸체 높이 검사

2026-10-08에 각 조건을 500스텝 실행하고 마지막 100스텝의 root z 평균을 비교했다.
공식 PLAY 환경은 로봇 한 대·seed 42로 실행하고 질량·마찰 무작위화 및 외력을 끄고
초기 자세·속도 명령을 프로젝트와 맞췄다.

| 조건 | 평균 몸체 기준 높이 |
|---|---|
| 프로젝트 환경, 정책 없이 기본 관절 목표 유지 | 0.287m |
| 프로젝트 환경, 현재 평지 정책 0.4m/s | 0.162m |
| 공식 환경, 같은 평지 정책 0.4m/s | 0.162m |

현재 정책·환경 조합은 낮은 보행 자세를 보인다. 이전의 리셋 0회·전진 검사는 정상 자세나
속도 추종 성능까지 합격했다는 뜻이 아니다. 초기 z=0.4m는 생성 위치이며 유지할 몸체 높이가 아니다.
현재 정책 관측에는 높이 목표가 없고 공식 평지 보상에는 높이 유지 항목이 없다.
기존 체크포인트를 그대로 실행하면서 보상 설정만 바꿔도 정책의 출력은 바뀌지 않는다.
정상 높이의 보행은 검증된 다른 정책을 도입하거나 높이 유지 보상을 포함해 재학습한 뒤 검증해야 한다.
NVIDIA 4.5·5.0·5.1 평지 체크포인트의 SHA-256은 동일했으므로 URL 버전 변경만으로 해결되지 않는다.
[높이 비교 기록](go2_install_evidence/base_height_check.json)을 참고한다.
