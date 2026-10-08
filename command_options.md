# 명령 및 전체 사용자 옵션

현재 프로젝트의 사용자 실행 명령 기준이다. 모든 명령은 프로젝트 루트에서 실행한다.
외부 `third_party/` 도구의 옵션은 해당 저장소 문서를 따른다.

## 공통 환경

viewer·Go2 제어·설치 검사는 공통 Conda 환경에서 실행한다. 두 터미널을 사용하면 각각 활성화한다.

```bash
cd /home/rodix-gpu/shipyard_robot_sim
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate "$PWD/.conda/isaacsim-5.0"
```

Hunter2 정적 도구와 단위 테스트는 Python 3.10 이상으로 실행할 수 있다.
아래 기본값은 현재 코드 기준이다.

## 1. 환경·로봇 viewer

```bash
python scripts/viewer.py [옵션]
```

| 옵션 | 기본값 | 역할·허용 값 |
|---|---|---|
| `-h`, `--help` | — | 도움말 출력 후 종료 |
| `--scenario 이름` | `shipyard` | `flat`, `shipyard` |
| `--robot 종류` | `none` | `none`: 환경만 표시, `go2`: 환경과 Go2 생성 |
| `--device 장치` | `cuda:0` | 시뮬레이션 장치. 현재 검증 구성은 NVIDIA GPU |
| `--webrtc` | 꺼짐 | headless WebRTC 스트리밍 |
| `--no-webrtc` | 기본 표시 방식 | 서버 데스크톱에 로컬 창 표시 |
| `--server-ip 주소` | 환경변수 `PUBLIC_IP`, 없으면 미설정 | WebRTC 클라이언트가 접근할 서버 주소. WebRTC 사용 시 필요 |
| `--steps N` | `0` | 0 이상. 환경만 표시하면 프레임 수, Go2가 있으면 총 적용 제어 스텝 수. `0`은 무제한 |
| `--report 경로` | 저장 안 함 | viewer 결과 JSON 저장. 정상 종료 시 기록 |
| `--robot-id ID` | `go2` | 제어 연결 식별자, 1~64자. 제어기와 같아야 함. 여러 로봇 소환 기능은 아님 |
| `--control-port 포트` | `8765` | 제어용 localhost TCP 포트, 1~65535. 제어기와 같아야 함 |
| `--control-timeout 초` | `2.0` | 연결된 제어기의 초기 설정·유효한 액션 응답 제한 시간. 양수. 초과 시 연결 해제·물리 정지 |

로봇이 없으면 제어 관련 옵션은 표시 동작에 사용되지 않는다.
Go2는 제어기 연결 전 물리를 진행하지 않는다. 초기 연결 대기는 `--control-timeout`의 대상이 아니다.

```bash
# 조선소 환경만 표시
python scripts/viewer.py --scenario shipyard --no-webrtc

# 평지 + Go2, 로컬 화면
python scripts/viewer.py --scenario flat --robot go2 --no-webrtc

# 조선소 + Go2, 원격 화면
python scripts/viewer.py --scenario shipyard --robot go2 \
  --webrtc --server-ip 192.168.1.149

# 환경만 100프레임 표시하고 결과 기록
python scripts/viewer.py --steps 100 --report .runtime/scenario_viewer/check.json
```

WebRTC 화면 포트는 TCP 49100·UDP 47998이다. `--control-port`는 화면 포트를 변경하지 않는다.
`--worker`와 launcher의 `--entry`는 내부 실행 인자다. 내부 worker가 받는 Isaac Lab의
`--headless`, `--livestream`, `--experience`, `--kit_args`는 일반 viewer 명령 옵션이 아니다.
화면 모드는 `--webrtc` / `--no-webrtc`로 선택한다.

## 2. 별도 Go2 정책 제어기

```bash
python scripts/go2/control.py [옵션]
```

| 옵션 | 기본값 | 역할·허용 값 |
|---|---|---|
| `-h`, `--help` | — | 도움말 출력 후 종료 |
| `--robot-id ID` | `go2` | viewer와 같은 로봇 ID |
| `--control-port 포트` | `8765` | viewer와 같은 제어 포트, 1~65535 |
| `--vx 속도` | `0.4`m/s | Go2 목표 전진 속도, 0.2~0.8m/s. 연결 시 viewer에 전달 |
| `--device 장치` | `cpu` | 정책 추론 장치. `cpu`, `cuda:0` 등. viewer 장치와 독립적 |
| `--steps N` | `0` | 이번 연결에서 적용이 확인된 액션 수. 0 이상, `0`은 무제한 |
| `--connect-timeout 초` | `180.0` | 연결 시도 제한 시간과 연결 후 첫 관측의 수신 대기 제한 시간에 각각 사용. 양수 |
| `--response-timeout 초` | `30.0` | 첫 메시지 이후 다음 관측·완료 메시지 수신 제한 시간. 양수 |
| `--report 경로` | 저장 안 함 | 적용 확인 스텝·마지막 상태·오류 등 JSON 기록 |

같은 서버의 viewer에 `127.0.0.1`로 연결한다. 제어기에 WebRTC·서버 IP·시나리오 옵션은 없다.
viewer에서 선택한 로봇을 제어하며, 제어기 자체가 시뮬레이터를 시작하지 않는다.
연결할 때 `--vx`를 먼저 전달하고 그 속도가 반영된 첫 관측으로 정책을 시작한다.
재접속하면 새 제어기의 속도를 적용한다. 시나리오의 초기 속도는 실행 명령으로 사용하지 않는다.

```bash
python scripts/go2/control.py --vx 0.4
python scripts/go2/control.py --device cuda:0
python scripts/go2/control.py --vx 0.4 --steps 500 --report .runtime/go2/control.json
```

Ctrl+C로 제어기를 종료하면 viewer는 화면을 유지하고 물리를 멈춘다.
다시 실행하면 현재 상태에서 재개한다. viewer의 `--steps`는 누적 스텝이고 제어기의
`--steps`는 이번 연결 기준이다. viewer가 요청 스텝 전에 종료하면 제어기는 오류를 반환한다.

## 3. Go2 체크포인트 준비

```bash
python scripts/go2/prepare_go2_policy.py
```

사용자 옵션이 없다 (`--help`도 구현하지 않음).
`sources.lock.json`에 기록된 정책을 다운로드하고 SHA-256을 확인한다.
파일이 이미 있으면 체크섬만 검사한다. 이때 잘못된 체크섬의 파일을 자동 교체하지 않는다.

## 4. 두 터미널 제어 테스트 예시

```bash
# 터미널 1: viewer
python scripts/viewer.py --scenario shipyard --robot go2 --no-webrtc \
  --steps 500 --robot-id go2 --control-port 8765 \
  --report .runtime/go2/viewer_control.json

# 터미널 2: 제어기
python scripts/go2/control.py --vx 0.4 --robot-id go2 --control-port 8765 \
  --steps 500 --report .runtime/go2/control.json
```

viewer를 무제한으로 두고 제어기에만 `--steps 500`을 주면 실행 후 정지하고 다음 연결을 기다린다.
`passed`는 통신·실행 완료 결과이며 자동 보행 성능 판정은 아니다.

## 5. Hunter2 자산 생성

```bash
python3 scripts/hunter2/prepare_hunter2_asset.py
```

추가 옵션은 `-h`, `--help`만 있다. 원본 저장소 commit과 입력 체크섬을 확인한 뒤
파생 URDF·차량 설정·검증 보고서를 생성한다. 생성 파일을 변경하는 명령이다.

## 6. Hunter2 정적 검증

```bash
python3 scripts/hunter2/validate_hunter2_asset.py [urdf] [--structural-only]
```

| 인자·옵션 | 기본값 | 역할 |
|---|---|---|
| `-h`, `--help` | — | 도움말 |
| `urdf` (선택 위치 인자) | `assets/hunter2/source/hunter2_sim.urdf` | 검사할 URDF 경로 |
| `--structural-only` | 꺼짐 | 다른 위치로 옮긴 자산 묶음의 구조 검사. 원본·설정과의 비교 생략 |

```bash
python3 scripts/hunter2/validate_hunter2_asset.py
python3 scripts/hunter2/validate_hunter2_asset.py /경로/hunter2_sim.urdf --structural-only
```

## 7. Hunter2 모델 비교

```bash
python3 scripts/hunter2/compare_hunter2_models.py [--output-dir 경로]
```

| 옵션 | 기본값 | 역할 |
|---|---|---|
| `-h`, `--help` | — | 도움말 |
| `--output-dir 경로` | 미지정 | 지정하지 않으면 JSON 출력만 수행. 지정하면 비교 자료를 저장. 프로젝트 `docs/` 내부 경로만 허용 |

```bash
python3 scripts/hunter2/compare_hunter2_models.py
python3 scripts/hunter2/compare_hunter2_models.py --output-dir docs/hunter2_model_comparison
```

## 8. Hunter2 운동학 검토

```bash
python3 scripts/hunter2/review_hunter2_kinematics.py
```

사용자 옵션이 없다 (`--help`도 구현하지 않음). 조향·바퀴·메시의 정적 검토 결과를 JSON으로 출력한다.
물리 시뮬레이션이나 주행 제어를 실행하는 명령은 아니다.

## 9. 설치 검사 및 단위 테스트

```bash
# 공통 Conda 환경에서 패키지·CUDA 검사
python tests/check_isaacsim_install.py

# Kit·PhysX·URDF importer 검사 (화면 없이 실행)
python tests/smoke_test_isaacsim.py

# 시뮬레이터 없이 단위 테스트
python3 -m unittest discover -s tests -v

# 제어 통신 테스트만 실행
python3 -m unittest discover -s tests -p 'test_control_channel.py' -v
```

설치 검사 두 스크립트에는 사용자 옵션이 없다 (`--help`도 구현하지 않음).
GPU·설치 검사 결과는 `docs/isaacsim_install_evidence/`에 저장된다.
단위 테스트 명령의 `-s`는 검색 폴더, `-p`는 테스트 파일 패턴, `-v`는 상세 출력이다.
