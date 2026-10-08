# 로봇 담당자용 구조 및 실행 안내

전체 명령 옵션·기본값·실행 예시는 [커맨드 옵션 안내](command_options.md)를 참고한다.

현재 코드 기준: 2026-10-08. 목표는 여러 로봇이 하나의 조선소 장면과 물리 시간을 공유하는 것이다.
현재 viewer·별도 제어기 분리 실행을 지원하며, 다중 로봇 실행기와 공통 목표·상태 adapter는 아직 구현하지 않았다.

## 폴더별 책임

```text
assets/
  go2/config/                  로봇·모터 설정, 출처 기록
  go2/policies/                로컬 체크포인트 (Git 제외)
  hunter2/                     기존 URDF·mesh·USD·차량 설정
  environments/shipyard/       향후 공통 배경 USD·지도·재질 (현재 README만 있음)
control/
  control_channel.py           viewer·제어기 간 로컬 통신과 응답 검사
  go2/policy.py                별도 프로세스용 정책 추론 (PyTorch)
  go2/simulation.py            viewer 내부 Go2 생성·물리 진행·제어 대기
  hunter2/                     향후 차량 제어 adapter (현재 README만 있음)
  omni/                        향후 전방향 제어 adapter (현재 README만 있음)
simulation/                   향후 시뮬레이션 실행 계층 (현재 빈 폴더)
scenarios/
  registry.py                  공통 장면 및 Go2 환경 등록
  common/scene_cfg.py          공통 지면·조명
  flat/scene_cfg.py            로봇 없는 평지 장면
  flat/go2_env_cfg.py          Go2 기본 관측·액션·명령·리셋·종료 조건
  shipyard/scene_cfg.py        공통 조선소 장애물·카메라
  shipyard/go2_env_cfg.py      조선소 장면과 Go2 로봇·센서 연결
scripts/
  viewer.py                    환경 표시·Go2 소환, 로컬/WebRTC 선택
  go2/control.py               viewer에 연결하는 별도 정책 제어기
  go2/prepare_go2_policy.py    공식 체크포인트 다운로드·해시 검사
  hunter2/                    기존 모델 준비·정적 검증 도구
  visualization/launcher.py    로컬/WebRTC 선택·초기화·포트·프로세스 관리
  visualization/runtime.py     캐시·Kit 설정·프레임 속도
third_party/                   고정된 외부 소스; 로봇별 시나리오를 여기서 수정하지 않음
```

`assets`는 모델·설정·정책 파일, `control`은 구동 계산과 실행 로직,
`scenarios`는 공통 장면과 로봇 연결, `scripts`는 사용자 명령을 담당한다.
URDF는 로봇 원본, USD는 Isaac Sim 자산, Python은 장면 구성·제어 연결에 사용한다.
`assets/go2/scenarios` 및 이전 viewer 호환 파일은 삭제됐으므로 새 경로를 사용한다.

| 변경하려는 내용 | 수정 위치 |
|---|---|
| 장애물 위치·크기·색상, 조선소 카메라 | `scenarios/shipyard/scene_cfg.py` |
| 공통 지면·조명 | `scenarios/common/scene_cfg.py` |
| Go2 관절 기본값·모터 게인·제한 | `assets/go2/config/robot_cfg.py` |
| Go2 관측·액션·리셋·명령 | `scenarios/flat/go2_env_cfg.py` |
| 조선소에 Go2·센서 연결 | `scenarios/shipyard/go2_env_cfg.py` |
| 분리 실행의 정책 추론 | `control/go2/policy.py`, `scripts/go2/control.py` |
| 분리 실행의 물리 루프·연결 | `control/go2/simulation.py`, `control/control_channel.py` |
| 화면·WebRTC 처리 | `scripts/visualization/launcher.py`, `runtime.py` |

Go2 평지 장면은 기본 환경 정의에 지면 설정도 포함한다. 공통 지면을 변경할 때 평지 설정과도 비교한다.

## 향후 시뮬레이션 실행 계층

루트 `simulation/`은 환경 생성·물리 진행·렌더링 실행부를 분리하기 위한 빈 폴더다.
현재 Go2 실행 코드는 `control/go2/simulation.py`에 유지하며 viewer의 호출 경로도 그대로다.

다음 정리 단계에서는 해당 파일을 `simulation/go2.py`로 이동하는 정도로 시작한다.
다른 로봇이 준비되면 실제 공통 부분을 확인해 하나의 물리 실행 루프와 로봇별 adapter로 확장한다.
지금은 공통 runner·adapter 인터페이스·다른 로봇용 파일을 구현하지 않는다.
빈 폴더에는 파일을 추가하지 않았으므로 Git에는 기록되지 않는다.

## 실행 환경 준비

아래 명령은 모두 프로젝트 루트에서 실행한다. 경로는 현재 서버 기준이다.

```bash
cd /home/rodix-gpu/shipyard_robot_sim
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate "$PWD/.conda/isaacsim-5.0"
```

다른 컴퓨터에서는 프로젝트·Conda 설치 경로를 해당 위치로 바꾼다.
공통 환경은 Isaac Sim 5.0, Isaac Lab v2.2.1, RSL-RL 2.3.3이다.
Isaac Lab은 프로젝트 `third_party/IsaacLab`을 사용하며 홈의 IsaacLab을 사용하지 않는다.
새 설치는 [README 설치 절차](../README.md#새-환경-설치)와 [Go2 의존성 안내](go2_setup.md)를 따른다.

```bash
python -m pip check
python scripts/go2/prepare_go2_policy.py
```

정책 준비는 별도 Go2 제어기에 필요하다. 환경·로봇 표시만 하는 viewer에는 필요 없다.
Go2 USD와 참조 모델은 NVIDIA 서버에서 로드하므로 모델 파일이 완전히 오프라인으로 포함된 구성은 아니다.

## 화면 선택

| 옵션 | 실행 방식 | IP 필요 여부 |
|---|---|---|
| `--no-webrtc` | 서버 데스크톱 로컬 창 | 필요 없음 |
| `--webrtc` | headless 서버 + WebRTC 클라이언트 | `--server-ip` 또는 `PUBLIC_IP` 필요 |

로컬 실행은 서버 데스크톱의 DISPLAY와 X11 접근 권한이 필요하다.
DISPLAY가 비어 있고 X11 :0 소켓이 있으면 launcher가 :0과 사용자 .Xauthority를 기본으로 선택한다.
다른 세션에서는 해당 DISPLAY·인증 설정을 사용한다.
WebRTC 클라이언트는 지정한 서버 IP로 연결하고 TCP 49100·UDP 47998을 사용한다.
스트리밍 명령은 한 번에 하나씩 실행한다. 창 닫기 또는 Ctrl+C로 종료한다.
초기 렌더링 준비에 시간이 걸릴 수 있다.

`192.168.1.149`는 예시다. 클라이언트에서 접근 가능한 실제 서버 IP로 바꾼다.

```bash
export PUBLIC_IP=192.168.1.149
```

## 로봇 없이 시나리오만 보기

```bash
# 로컬 창
python scripts/viewer.py --scenario shipyard --no-webrtc
python scripts/viewer.py --scenario flat --no-webrtc

# WebRTC + headless
python scripts/viewer.py --scenario shipyard --webrtc --server-ip "$PUBLIC_IP"

# 30프레임 실행 후 종료·결과 기록
python scripts/viewer.py --scenario shipyard --webrtc --server-ip "$PUBLIC_IP" \
  --steps 30 --report .runtime/scenario_viewer/shipyard_check.json
```

viewer 기본값은 `shipyard`, 로컬 창, 종료할 때까지 표시다.
`--steps`는 렌더링 프레임 수이며 `0`은 무제한이다.
준비 완료 로그는 `SCENARIO_VIEW_READY`다. 보고서는 `--report`를 지정한 경우에만 저장한다.
로봇을 지정하지 않은 viewer는 정적 배치 미리보기이며 물리 시간이 진행하지 않는다.
현재 plane 지면과 spawn 설정이 있는 장면 자산을 지원하며 정적 장면 정의에 로봇 articulation을 직접 넣으면 거부한다. `--robot go2`는 별도 동적 환경을 생성한다.
주행 검증은 Go2 viewer와 별도 제어기를 연결해 수행한다.

## Viewer + 별도 제어 프로세스

```bash
# 터미널 1
python scripts/viewer.py --scenario shipyard --robot go2 --no-webrtc
# 터미널 2
python scripts/go2/control.py --robot-id go2 --vx 0.4
```

정책은 터미널 2에서 실행하고 물리는 viewer만 진행한다. 제어기 종료 후에는 물리를 멈추고
같은 명령으로 다시 연결할 수 있다. [자세한 분리 실행 안내](viewer_control.md)를 참고한다.

## Go2 제어 테스트

두 터미널에서 같은 Conda 환경을 활성화한다.

```bash
# 터미널 1: 500개 액션을 적용한 후 종료
python scripts/viewer.py --scenario shipyard --robot go2 --no-webrtc \
  --steps 500 --report .runtime/go2/viewer_control.json

# 터미널 2: 정책 실행 및 적용 확인
python scripts/go2/control.py --vx 0.4 --steps 500 --report .runtime/go2/control.json
```

WebRTC는 viewer의 `--no-webrtc` 대신 `--webrtc --server-ip "$PUBLIC_IP"`를 사용한다.
평지 테스트는 `--scenario flat`을 사용한다. viewer 보고서에서 리셋 횟수·전진 거리·
최저 몸체 높이와 상태 보존 결과를 확인한다. `passed`는 실행 완료 및 통신 검사 결과이며
별도의 보행 성능 합격 판정은 제공하지 않는다. 정책 export 기능도 제공하지 않는다.
정책·관측·액션 호환성과 재접속 방법은 [분리 실행 안내](viewer_control.md)를 참고한다.

## Hunter2 담당자

기존 Hunter2 코드·자산은 이번 구조 정리에서 변경하지 않았다.
현재 조향축·연결·메시의 정적 검증은 완료했으며, 주행 controller와 실제 접촉·주행 검증은 미완료다.
USD 의존성과 형상·관절 로딩은 검사했지만 WebRTC 렌더링 멈춤을 재현해 정상 viewer 표시로 확정하지 않았다.
Hunter2 전용 실행 명령이나 공통 다중 로봇 실행 명령은 아직 없다.

```bash
# 원본 lock과 입력 해시를 검사하고 파생 자산 재생성 (생성 파일 변경)
python3 scripts/hunter2/prepare_hunter2_asset.py

# 기존 파일의 정적 검증
python3 scripts/hunter2/validate_hunter2_asset.py
```

위 도구는 Python 3.10+ 표준 라이브러리와 Git으로 동작한다.
모델 변경 기준·물리 TODO는 [Hunter2 자산 안내](../assets/hunter2/README.md)를 따른다.

## 새로운 로봇 또는 시나리오 추가

1. `assets/<robot>/`에 모델·참조 파일·출처와 관절·모터 설정을 둔다.
   URDF를 USD로 변환했다면 원본과 변환 옵션·결과를 함께 기록한다.
2. `control/<robot>/`에 제어기를 구현한다. 로봇별 액션 계산과 상태를 제공하고,
   향후 공통 물리 루프가 스텝을 진행하도록 분리한다.
3. 로봇 없는 공통 장면은 `scenarios/<scene>/scene_cfg.py`에 정의한다.
   viewer가 사용할 `SceneCfg`, `CAMERA_EYE`, `CAMERA_TARGET`을 제공하고
   `scenarios/registry.py`의 `SCENARIOS`에 모듈 경로를 등록한다.
4. `scenarios/<scene>/<robot>_env_cfg.py`에서 공통 장면과 로봇·센서·제어 환경을 연결한다.
   Go2 환경은 같은 registry의 `GO2_SCENARIOS`에 등록한다.
   새 로봇용 등록·로딩 기능은 담당자가 추가해야 하며 Go2 정책 설정을 그대로 적용하지 않는다.
5. `scripts/<robot>/`에 얇은 실행 진입점을 둔다. 공통 `visualization.launcher`·`runtime`을 사용한다.
   Isaac Lab의 sim·asset·sensor import는 SimulationApp 초기화 후 수행한다.
6. 단독 모델 보기, 제어·접촉·상태 검사 후 공통 시나리오에 연결한다.
   최종 통합에서는 하나의 앱·장면·물리 시간에 로봇별 제어기를 연결한다.
   현재 Go2 `simulation.run_controlled()`는 자체 환경을 생성한다. 다중 로봇으로 확장할 때는 하나의 공통 환경·물리 루프에서 각 로봇의 액션을 적용한다.

Go2 정책은 관측 48개 순서·관절 순서·액션 12개·스케일 0.25·제어 주기 0.02초에 맞춰져 있다.
이 조건을 바꾸면 기존 정책과 호환되지 않을 수 있다.

## 테스트와 현재 검증 범위

```bash
# 시뮬레이터 없이 실행하는 정적·launcher 테스트
python3 -m unittest discover -s tests -v

# 도움말로 사용 가능한 옵션 확인
python scripts/viewer.py --help
python scripts/go2/control.py --help
```

단위 테스트의 로컬 소켓·프로세스 검사는 OS 권한이 필요하며 GPU 시뮬레이션을 대체하지 않는다.
제어기 속도 설정 프로토콜 반영 후 단위 테스트 45개가 통과했다. 별도 제어 통신 테스트는 `tests/test_control_channel.py`에 있다. Go2 구조 이동 후 500스텝 2.48m 전진·리셋 0회를 확인했다.
단일 viewer 진입점 정리 후 shipyard WebRTC 10프레임 실행을 확인했다.
이전 viewer 단계에서 flat 로컬 창과 shipyard WebRTC 각각 30프레임을 완료했다.
실제 원격 클라이언트의 영상 수신은 별도 확인이 필요하다.

새 컴퓨터 재현·정책 호환성과 상세 기록은 [Go2 설치 및 검증 안내](go2_setup.md)를 참고한다.
