# 시각화와 WebRTC

로봇 시뮬레이션에서 공통으로 사용하는 화면·스트리밍 기능이다.
이 모듈에는 Go2 관절, 속도 명령, 정책 로딩, 이동 거리 판정 코드가 없다.

| 파일 | 책임 |
|---|---|
| `runtime.py` | 프로젝트 내부 cache/config 경로, Kit 렌더링 옵션, 실시간 프레임 간 대기 |
| `webrtc.py` | 서버 주소·연결 포트, 스트리밍 실행 옵션, 포트 중복 검사, 자식 프로세스 종료 |
| `stream_entry.py` | Isaac Sim 5.0 공통 화면 초기화 보정 후 지정된 시뮬레이션 스크립트 실행 |

```mermaid
flowchart LR
    CLI[Go2 WebRTC 명령] --> Scenario[Go2 시나리오 선택]
    Scenario --> Streaming[공통 WebRTC 실행]
    Streaming --> Entry[공통 스트리밍 앱 초기화]
    Entry --> Robot[프로젝트 Go2 환경·정책 실행]
    Robot --> Display[공통 캐시·렌더링·프레임 대기]
```

`scripts/go2/run_go2_webrtc.py`는 `view`, `policy`, `check` 모드와 `flat`, `shipyard` 시나리오를 선택한다. 서버 주소, Kit 스트리밍 설정과 프로세스 관리는 이 패키지에 위임한다.
`scripts/go2/run_go2.py`는 물리·정책·검증 결과를 처리하고,
표시 속도와 Kit 실행 설정만 `runtime.py`에 위임한다.
화면 없이 실행할 때는 WebRTC 프로세스 실행기를 거치지 않는다.

`runtime.py`, `webrtc.py`를 import하는 것만으로 Isaac Sim을 시작하지 않는다.
Isaac Sim·Isaac Lab import와 초기화 보정은 스트리밍 자식 프로세스에서 적용한다.
`stream_entry.py`의 초기화 보정은 프로젝트의 고정된 Isaac Sim 5.0 / Isaac Lab 2.2.1 구성에 맞춘 것이다.

기존 명령은 동일하다. 프로젝트 루트에서 공통 Conda 환경을 활성화한 뒤 실행한다.

```bash
python scripts/go2/run_go2_webrtc.py view --server-ip 192.168.1.149
python scripts/go2/run_go2_webrtc.py policy --server-ip 192.168.1.149
python scripts/go2/run_go2_webrtc.py check --server-ip 192.168.1.149 --steps 500
python scripts/go2/run_go2.py --headless --steps 500
```

WebRTC 명령은 한 번에 하나씩 실행한다.

## 시나리오 전용 viewer와 표시 방식

`view_scenario.py`는 로봇·정책 없이 공통 `flat`, `shipyard` 장면만 표시한다.
`scenario_viewer.py`는 장면 로딩·렌더링 worker이고, `display.py`는 로컬 창/WebRTC 선택을 관리한다.
장면 목록은 루트 `scenarios/registry.py`에 등록한다.

공통 Conda 환경을 활성화한 뒤 실행한다. WebRTC는 headless로 실행하며 서버 IP가 필요하다.
로컬 모드는 WebRTC 포트를 사용하지 않고 서버 데스크톱에 창을 연다.
로컬 창은 데스크톱 세션의 `DISPLAY`와 X11 접근 권한이 필요하다. `DISPLAY`가 비어 있고
서버의 X11 `:0` 소켓이 있으면 `:0`과 사용자 `.Xauthority`를 기본으로 선택한다.
다른 데스크톱 세션에서는 해당 `DISPLAY`·인증 설정을 사용한다. SSH만 연결한 터미널에서는
서버 데스크톱 터미널에서 실행하거나 해당 세션의 `DISPLAY`·인증 설정을 사용한다.

```bash
# 시나리오만 보기: 기본은 로컬 창
python scripts/visualization/view_scenario.py --scenario shipyard --no-webrtc
python scripts/visualization/view_scenario.py --scenario flat --no-webrtc

# 시나리오만 보기: WebRTC + headless
python scripts/visualization/view_scenario.py --scenario shipyard --webrtc --server-ip 192.168.1.149

# Go2 포함 환경 보기: 로컬 창 / WebRTC
python scripts/go2/run_go2.py --mode view --scenario shipyard --no-webrtc
python scripts/go2/run_go2.py --mode view --scenario shipyard --webrtc --server-ip 192.168.1.149

# 기존 명령도 로컬 선택 가능; 기존 실행기의 기본값은 WebRTC 유지
python scripts/go2/run_go2_webrtc.py view --scenario shipyard --no-webrtc
```

`--steps 100`으로 보기 길이를 제한하고 `--report 경로`로 실행 결과를 기록할 수 있다.
일반 실행은 창 닫기 또는 Ctrl+C로 종료한다. WebRTC 모드는 한 번에 하나만 실행한다.
Go2의 `policy`, `check`도 같은 `--webrtc` / `--no-webrtc` 옵션을 지원한다.
기존 `run_go2.py --headless` 물리 검사 명령도 유지한다.

시나리오 전용 viewer는 정적 배치 미리보기이며 물리 시간을 진행하지 않는다.
보행·주행 및 접촉 검증은 로봇 실행기에서 수행한다.

현재 정적 viewer의 terrain은 평지(`plane`)를 지원한다. 높이맵·생성 지형은 별도 로더 확장이 필요하다.
