# Viewer와 별도 로봇 제어 실행

viewer가 환경·Go2 생성, 관측 계산, 액션 적용, 물리 스텝과 화면을 담당한다.
별도 `control.py`는 이미 생성된 로봇의 관측을 받아 정책 추론만 수행한다.
현재 Go2 한 대를 지원하며 Hunter2 및 여러 로봇 동시 실행은 아직 구현하지 않았다.

## 두 터미널에서 실행

두 터미널 모두 같은 서버에서 공통 환경을 활성화한다.

```bash
cd /home/rodix-gpu/shipyard_robot_sim
source /home/rodix-gpu/miniforge3/etc/profile.d/conda.sh
conda activate "$PWD/.conda/isaacsim-5.0"
```

터미널 1에서 환경과 로봇을 생성한다.

```bash
# 로컬 창
python scripts/viewer.py --scenario shipyard --robot go2 --no-webrtc

# 또는 WebRTC + headless (IP는 실제 서버 주소)
python scripts/viewer.py --scenario shipyard --robot go2 \
  --webrtc --server-ip 192.168.1.149
```

`ROBOT_VIEW_READY`가 출력되면 터미널 2에서 정책 제어를 시작한다.
먼저 실행해도 기본 180초 동안 viewer 연결·첫 관측을 기다린다.

```bash
# 체크포인트가 없으면 한 번 준비
python scripts/go2/prepare_go2_policy.py

# viewer 안의 Go2에 연결
python scripts/go2/control.py --robot-id go2 --vx 0.4
```

제어기는 기본 CPU에서 정책을 추론하며 Isaac Sim을 import하거나 별도 시뮬레이터를 시작하지 않는다.
검증된 공식 `flat.pt`의 Actor만 PyTorch로 로드하므로 사전 export는 필요 없다.
`--device cuda:0`으로 정책 추론 GPU를 별도로 선택할 수 있다.
viewer의 `--device`는 시뮬레이션 장치이며 제어기 장치와 같을 필요는 없다.

제어 프로세스를 Ctrl+C로 종료하면 viewer는 계속 표시되고 물리는 현재 상태에서 멈춘다.
같은 control 명령을 다시 실행하면 새 세션으로 연결해 현재 상태에서 재개한다.
viewer를 종료하면 제어기도 연결 종료를 감지하고 끝난다.
환경만 보려면 `--robot`을 생략한다.

## 제한 실행과 결과 기록

```bash
# 터미널 1: 총 500개 액션 적용 후 viewer 종료
python scripts/viewer.py --scenario shipyard --robot go2 --no-webrtc \
  --steps 500 --report .runtime/go2/viewer_control.json

# 터미널 2: 500개 액션의 적용 확인 후 종료
python scripts/go2/control.py --vx 0.4 --steps 500 --report .runtime/go2/controller.json
```

로봇 viewer의 `--steps`는 적용된 제어 스텝 수다. 연결 대기·일시정지 중에는 증가하지 않는다.
로봇 없는 viewer의 `--steps`는 렌더링 프레임 수다. 둘 다 기본값 0은 무제한이다.

viewer를 무제한 실행하고 control에만 `--steps 500`을 주면, 500스텝 후 제어기가
정지 요청을 보내고 종료한다. viewer는 물리를 멈춘 채 다음 연결을 기다린다.
제어기보다 viewer의 종료 스텝이 작으면 제어기는 요청 수를 완료하지 못했다는 오류를 반환한다.

viewer JSON에는 `control_steps`, `simulated_seconds`, `resets`, 전진 거리·최저 높이,
연결 횟수·거부 메시지 수·연결 상태 전환·대기 중 상태 보존 결과가 기록된다.
제어기 JSON에는 실제 적용이 확인된 스텝 수와 마지막 상태가 기록된다.
이 실행의 `passed`는 통신·실행 완료를 뜻한다. 보행 성능을 평가하려면 viewer 보고서의
리셋 횟수·몸체 높이·전진 거리를 확인한다. 자동 보행 합격 판정은 제공하지 않는다.
보고서는 `--report`를 지정해야 저장되며 viewer 보고서는 정상 종료 시 기록된다.

## 통신 및 시간 처리

- 제어는 같은 서버의 `127.0.0.1:8765` TCP 소켓을 사용한다. 외부 주소에는 바인딩하지 않는다.
- 두 명령에 같은 `--control-port`를 지정해 포트를 변경한다.
- `--robot-id`도 양쪽이 같아야 한다. 한 로봇에는 한 제어기만 연결할 수 있다.
- WebRTC의 TCP 49100·UDP 47998은 화면용이며 제어 포트와 별개다.
- 관측 48개, 액션 12개, 관절 순서와 0.02초 제어 주기를 확인한다.
- 프로토콜 v2는 초기 `hello → configure` 교환 후 관측·액션을 교환한다. 설정 전에는 액션을 적용하지 않는다.
- 관측에는 프로토콜 버전·로봇 ID·세션 ID·스텝·에피소드 번호가 포함된다.
- 해당 관측에 대응하는 유한한 액션 하나만 받아 적용한다. 오래된 액션·잘못된 ID·잘못된 크기는 연결 해제한다.
- viewer는 액션이 없으면 물리를 진행하지 않고 화면만 갱신한다.
- 기본 `--control-timeout 2`초 동안 유효한 응답이 없으면 제어기를 분리한다.
- 새 연결은 새 세션 ID를 사용한다. 로봇을 다시 생성하거나 임의로 초기화하지 않는다.
- 환경의 넘어짐·시간 제한으로 자동 리셋되면 에피소드 번호를 증가시키고 새 관측을 보낸다.

물리 시간 기준 정책 주기는 50Hz다. 실제 벽시계 속도는 렌더링·추론·통신 지연에 따라 느려질 수 있다.
평지 정책은 메모리 없는 Actor이므로 에피소드 변경 시 추가 은닉 상태 초기화가 필요하지 않다.
앞으로 순환 정책을 추가하면 제어기에서 에피소드 변경 시 상태 초기화도 구현해야 한다.
`--vx`는 제어기에서 지정하며 기본값은 0.4m/s, 범위는 0.2~0.8m/s다.
제어기가 연결할 때 목표 속도를 먼저 전달하고 viewer는 이를 반영한 첫 관측을 계산한다.
재접속할 때는 새 제어기의 속도를 반영하며, 에피소드 리셋 후에도 해당 목표 속도를 유지한다.
현재 평지 정책은 장애물 회피를 수행하지 않는다.

## 코드 위치

| 파일 | 역할 |
|---|---|
| `scripts/viewer.py` | 환경·로봇 선택, 화면 실행, 앱 생명주기 |
| `control/go2/simulation.py` | viewer 내부 Go2 환경·관측·액션 적용·대기 렌더링 |
| `control/control_channel.py` | 시뮬레이터에 독립적인 로컬 JSON 통신·연결·응답 검사 |
| `control/go2/policy.py` | Isaac Sim 없는 체크포인트 Actor 로딩·추론 |
| `scripts/go2/control.py` | 별도 제어 프로세스의 연결·추론 루프 |

향후 다중 로봇에서는 하나의 공통 물리 루프에 로봇별 관측·액션과 제어 연결을 추가한다.
현재 `--robot-id`는 한 Go2의 연결 식별자이며 여러 로봇 소환 옵션은 아니다.

## 검증 결과 (2026-10-08)

프로젝트 단위 테스트 45개가 통과했다. 실제 GPU·WebRTC 서버에서 별도 CPU 제어기로
500스텝을 실행하고 종료한 뒤, 정지 상태에서 새 제어기를 연결해 20스텝을 추가 실행했다.
연결 2회, 총 520스텝·물리 시간 10.4초, 전진 2.585m, 리셋 0회, 거부 메시지 0회였다.
재접속은 초기화 없이 500스텝에서 이어졌고 대기 중 로봇 상태 보존 검사도 통과했다.
WebRTC 클라이언트의 실제 영상 수신은 이번 검사에 포함하지 않았다.
[검증 결과 JSON](go2_install_evidence/external_control_check.json)을 참고한다.

제어기 속도 설정 변경 후 초기 연결 0.3m/s에서 100스텝, 재접속 0.6m/s에서 20스텝을
실행했다. 각각 첫 정책 관측에 지정한 속도가 반영됐고, 정지 상태 보존·리셋 0회를 확인했다.
[속도 설정 검증 기록](go2_install_evidence/controller_velocity_check.json)을 참고한다.
