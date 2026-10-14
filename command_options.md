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

## 공통 키보드 옵션

`control.py`, `ros.py`, `ros_command.py`는 아래 옵션을 공유한다.

| 옵션 | 기본값 | 역할 |
|---|---|---|
| `--keyboard` | 꺼짐 | 실행 터미널에서 키보드 이동 |
| `--keyboard-speed` | `1.5`m/s | 전후·횡이동 속도, 유한한 0 이상, 상한 없음 |
| `--keyboard-turn-speed` | `1.5`rad/s | yaw 회전 속도, 유한한 0 이상, 상한 없음 |
| `--keyboard-timeout` | `0.6`초 | 마지막 이동/정지 키 입력 후 목표 속도를 0으로 바꾸는 벽시계 시간 |

W/S(↑/↓): 전후, A/D(←/→): 횡이동, Z/C: 회전, Space/X: 정지, Q/Ctrl+C: 종료.
키를 누르고 있으면 OS 반복 입력으로 이동을 유지한다. 키를 놓으면 마지막 입력 후
timeout에 따라 정지하며, 즉시 정지는 Space를 사용한다. 한 번에 가장 최근 방향을 적용한다.
화면 창이 아닌 실행 터미널에 포커스를 둔다. SSH에서는 `ssh -t` 등으로 TTY를 할당한다.

```bash
python scripts/go2/control.py --keyboard
python scripts/go2/ros.py --scenario warehouse --keyboard
python scripts/go2/ros_command.py --keyboard --keyboard-speed 0.3 --keyboard-turn-speed 0.4
```

키보드 단위 검사와 실제 터미널/통신/물리 검사는 다음 명령으로 실행한다.

```bash
python -m unittest tests.test_go2_keyboard tests.test_control_channel -v
python tests/check_go2_keyboard.py --transport-only
python tests/check_go2_keyboard.py
```

통합 검사는 PTY로 W·Space·Z·A·Q를 입력해 제어 TCP, ROS DDS, 자동 정지와
터미널 복원을 확인한다. 기본 검사는 GPU에서 `ros.py --headless --keyboard`도 실행한다.
검사 결과는 `.runtime/go2_keyboard/checks.json`에 저장한다.

## 1. 환경·로봇 viewer

기본 배경은 `warehouse`다. NVIDIA Simple Warehouse의 Isaac Sim 5.0
USD에 고정된 배치를 사용한다. 시작 위치·카메라는 `scenarios/warehouse/scene_cfg.py`,
자산 URL·버전·원본 출처는 `assets/environments/warehouse/sources.lock.json`에서 관리한다.

```bash
python scripts/viewer.py [옵션]
```

| 옵션 | 기본값 | 역할·허용 값 |
|---|---|---|
| `-h`, `--help` | — | 도움말 출력 후 종료 |
| `--scenario 이름` | `warehouse` | `flat`, `warehouse` |
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

warehouse 바닥·벽 충돌, 시작 위치 여유 공간과 리셋 재현성은
`python tests/check_warehouse.py --headless`로 검사한다.
보행·ROS·clock 통합 검사는 `python tests/check_go2_ros.py --scenario warehouse`를 사용한다.

```bash
# 창고 환경만 표시
python scripts/viewer.py --scenario warehouse --no-webrtc

# 평지 + Go2, 로컬 화면
python scripts/viewer.py --scenario flat --robot go2 --no-webrtc

# 창고 + Go2, 원격 화면
python scripts/viewer.py --scenario warehouse --robot go2 \
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
| `--checkpoint 경로` | manifest의 `control_policy` | 추론할 체크포인트 지정. 현재 기본은 `upstream_flat_6800.pt` |
| `--steps N` | `0` | 이번 연결에서 적용이 확인된 액션 수. 0 이상, `0`은 무제한 |
| `--connect-timeout 초` | `180.0` | 연결 시도 제한 시간과 연결 후 첫 관측의 수신 대기 제한 시간에 각각 사용. 양수 |
| `--response-timeout 초` | `30.0` | 첫 메시지 이후 다음 관측·완료 메시지 수신 제한 시간. 양수 |
| `--report 경로` | 저장 안 함 | 적용 확인 스텝·마지막 상태·오류 등 JSON 기록 |

같은 서버의 viewer에 `127.0.0.1`로 연결한다. 제어기에 WebRTC·서버 IP·시나리오 옵션은 없다.
viewer에서 선택한 로봇을 제어하며, 제어기 자체가 시뮬레이터를 시작하지 않는다.
연결할 때 `--vx`를 먼저 전달하고 그 속도가 반영된 첫 관측으로 정책을 시작한다.
재접속하면 새 제어기의 속도를 적용한다. 시나리오의 초기 속도는 실행 명령으로 사용하지 않는다.
`--keyboard` 모드에서는 정지 명령으로 연결하고 키 입력에 따라 매 스텝 vx/vy/yaw 명령을
갱신한다. `--vx`는 고정 명령 모드의 옵션이며, 키보드 속도는 공통 키보드 옵션으로 정한다.
viewer에도 실행 중 명령 갱신 코드가 필요하므로 변경 적용 후 viewer를 재시작한다.

```bash
python scripts/go2/control.py --vx 0.4
python scripts/go2/control.py --device cuda:0
python scripts/go2/control.py --vx 0.4 --steps 500 --report .runtime/go2/control.json
python scripts/go2/control.py --checkpoint assets/go2/policies/flat.pt --vx 0.4
```

실제 모델 경로·SHA-256·iteration은 시작 로그 `GO2_POLICY_LOADED`와 결과 JSON의 `policy`에서 확인한다.

Ctrl+C로 제어기를 종료하면 viewer는 화면을 유지하고 물리를 멈춘다.
다시 실행하면 현재 상태에서 재개한다. viewer의 `--steps`는 누적 스텝이고 제어기의
`--steps`는 이번 연결 기준이다. viewer가 요청 스텝 전에 종료하면 제어기는 오류를 반환한다.

## ROS Go2 독립 실행

```bash
python scripts/go2/ros.py [옵션]
```

| 옵션 | 기본값 | 역할 |
|---|---|---|
| `--scenario` | `warehouse` | `flat`, `warehouse` |
| `--device` | `cuda:0` | 시뮬레이션 장치 |
| `--policy-device` | `cpu` | 정책 추론 장치 |
| `--checkpoint` | manifest의 `control_policy` | 기본 모델 `upstream_flat_6800.pt`; SHA-256 검사 |
| `--cmd-vel-topic` | `/unitree_go2/cmd_vel` | 수신할 절대 ROS topic 이름 |
| `--command-timeout` | `0.5`초 | 벽시계 기준 명령 만료 시간. 만료 시 목표 속도 0 |
| `--ros-domain-id` | `ROS_DOMAIN_ID` 또는 `0` | 0~232; 송신 노드와 같은 값 사용 |
| `--steps` | `0` | 적용할 50Hz 정책 스텝 수; 0은 무제한 |
| `--report` | 저장 안 함 | 모델 정보·명령 이력·리셋·clock 발행 수 등을 JSON 저장 |
| `--headless` | 꺼짐 | 화면 없이 물리·ROS 실행 |
| `--webrtc` | 꺼짐 | headless WebRTC 화면 |
| `--no-webrtc` | 기본 표시 방식 | 서버 로컬 화면 |
| `--server-ip` | `PUBLIC_IP` | WebRTC 클라이언트가 접근할 서버 주소 |

`--headless`와 `--webrtc` 중 하나의 화면 옵션을 사용한다. `GO2_ROS_READY` 이후 명령을
발행한다. 정책과 물리는 명령 유무와 관계없이 실행하며, 에피소드 시간에 따른 자동
리셋은 비활성화한다. 몸체 접촉 종료 조건에 의한 리셋은 보고서에 기록한다.
`passed`는 실행 완료 여부이며 보행 성능은 리셋·높이·이동량을 별도로 확인한다.
`--keyboard` 모드의 입력원은 실행 터미널이다. 이 모드에서도 `/clock`은 발행하며,
외부 cmd_vel 송신기를 사용하려면 일반 ROS 모드로 실행한다.

```bash
python scripts/go2/ros.py --scenario flat --headless \
  --steps 1000 --report .runtime/go2_ros/run.json
```

명령 송신 도구는 같은 Conda 환경에서 사용한다.

```bash
python scripts/go2/ros_command.py --vx 0.4 --vy 0 --wz 0 --duration 5
```

| 옵션 | 기본값 | 역할 |
|---|---|---|
| `--vx`, `--vy`, `--wz` | 각각 `0` | 몸체 좌표 x/y 속도(m/s), yaw 속도(rad/s) |
| `--duration` | 고정 명령 `5`초, 키보드 무제한 | 명령 반복 발행 시간; 완료·Q·Ctrl+C 시 정지 명령 전송 |
| `--rate` | `20`Hz | 벽시계 기준 발행 주기 |
| `--topic` | `/unitree_go2/cmd_vel` | simulator의 `--cmd-vel-topic`과 동일하게 지정 |
| `--ros-domain-id` | `ROS_DOMAIN_ID` 또는 `0` | simulator와 같은 domain |

내장 Python 3.11 Humble 라이브러리는 자식 프로세스에만 설정한다. 시뮬레이터 터미널에서
시스템 ROS의 Python 3.10 환경을 source할 필요가 없다. 외부 ROS 명령과 연결할 때는
별도 ROS 터미널을 사용한다. `/clock`은 50Hz best-effort/volatile이며, 수신 노드에서
`use_sim_time=true`를 설정해 사용한다. 센서·odom·TF topic은 이 실행기의 범위에 포함되지 않는다.

```bash
# 명령 유효성·만료 단위 검사
python -m unittest tests.test_go2_ros -v
# GPU 없이 실제 DDS cmd_vel/clock 통신 검사
python tests/check_go2_ros.py --transport-only
# GPU에서 최고 정책 + ROS 통합 900스텝 검사
python tests/check_go2_ros.py
```

통합 검사는 별도 domain 79에서 flat 환경을 실행해 전진·회전·횡이동·정지·후진 명령,
명령 만료 후 0 목표 속도, 관측 반영, 증가하는 clock과 리셋 0회를 확인한다.
결과는 `.runtime/go2_ros/{transport_check,integration_check}.json`에 저장한다.

## 3. Go2 체크포인트 준비

```bash
python scripts/go2/prepare_go2_policy.py
```

기본 실행은 `sources.lock.json`의 `policy`에 기록된 NVIDIA 정책을 다운로드하고 SHA-256을 확인한다.
`--control-policy`를 지정하면 `control_policy`에 등록한 현재 기본 제어 정책을 준비한다.
현재는 Zhefan-Xu/isaac-go2-ros2의 `flat_model_6800.pt`를 다운로드한다. `--help`로 옵션을 확인한다.

```bash
python scripts/go2/prepare_go2_policy.py --control-policy
```

파일이 이미 있으면 체크섬만 검사한다. 이때 잘못된 체크섬의 파일을 자동 교체하지 않는다.

## 4. 두 터미널 제어 테스트 예시

```bash
# 터미널 1: viewer
python scripts/viewer.py --scenario warehouse --robot go2 --no-webrtc \
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

## 10. Go2 평지 PPO tracking 학습·재생

```bash
python rl/go2/train.py --task Shipyard-Go2-Flat-Tracking-v0 \
  --num_envs 1024 --max_iterations 300 --seed 42 --run_name tracking --headless
python rl/go2/play.py --task Shipyard-Go2-Flat-Tracking-Play-v0 \
  --num_envs 16 --checkpoint /절대경로/model_299.pt
```

두 실행기는 프로젝트 task를 등록하고 Isaac Lab RSL-RL 실행기에 인자를 전달한다.
전체 옵션은 각각 `--help`로 확인한다. 공통 Isaac Sim Conda 환경에서 실행한다.

| 옵션 | 역할 |
|---|---|
| `--task` | tracking 학습 또는 PLAY task 선택 |
| `--num_envs` | 프로세스당 병렬 환경 수; 분산 학습에서는 GPU당 환경 수 |
| `--max_iterations` | PPO 학습 iteration 수 |
| `--seed` | 난수 seed |
| `--run_name` | 실행 폴더 이름에 붙이는 식별자 |
| `--headless` | 로컬 창 없이 실행 |
| `--distributed` | torch.distributed.run으로 띄운 여러 GPU 학습에 사용 |
| `--checkpoint` | 재생할 학습 체크포인트 경로 |

로그·모델은 `rl/rsl_rl/go2_flat_tracking/`에 저장된다.

현재 tracking v4는 대각선 trot 보상, 발 미끄러짐과 장시간 발 들기 페널티를 포함한다.
4 GPU로 처음부터 학습하는 명령은 [학습 안내](rl/README.md)를 참고한다.
처음부터 학습할 때는 `--resume`, `--load_run`, `--checkpoint`를 생략한다.

고정 명령에서 추종과 발 접촉을 평가하려면 다음 명령을 사용한다.

```bash
python rl/go2/evaluate.py \
  --checkpoint assets/go2/policies/upstream_flat_6800.pt \
  --case forward_04 --num_envs 16 --steps 1600 --seed 42 \
  --output rl/rsl_rl/go2_gait_eval/baseline_forward_04.json --headless
```

`--case`는 전진(`forward_02/04/08`), 정지(`stand`), 회전(`yaw_05/yaw_neg_05`),
전진 회전(`forward_turn`), 후진(`backward_04`), 횡이동(`lateral_04`)을 선택한다.
`--steps`는 policy step 수(기본 1,600 = 32초), `--warmup-steps`는 시작·리셋 후 제외 구간
(기본 100 = 2초)이다. `--output`과 같은 이름으로 JSON 통계, NPZ 원시 데이터,
PNG 발 접촉 타임라인을 저장한다. 전체 옵션은 `python rl/go2/evaluate.py --help`로 확인한다.
[보상 설정·4 GPU 실행·검증 안내](rl/README.md)를 참고한다.
