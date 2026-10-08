# 공통 시각화

사용자는 `scripts/viewer.py`로 시나리오를 표시하고 `--robot go2`로 로봇을 생성한다.

| 파일 | 책임 |
|---|---|
| `scripts/viewer.py` | 사용자 명령, 장면 생성·카메라·표시 루프 |
| `launcher.py` | 로컬/WebRTC 선택, 앱 초기화 보정, 포트 검사, 자식 프로세스 종료 |
| `runtime.py` | 프로젝트 캐시·Kit 설정, 프레임 간 대기 |

```bash
python scripts/viewer.py --scenario shipyard --no-webrtc
python scripts/viewer.py --scenario flat --no-webrtc
python scripts/viewer.py --scenario shipyard --webrtc --server-ip 192.168.1.149
```

공통 Conda 환경 `.conda/isaacsim-5.0`에서 실행한다.
기본값은 로컬 창이다. `--webrtc`는 headless 스트리밍을 사용하며 서버 IP가 필요하다.
로컬 창은 서버 데스크톱 DISPLAY와 X11 접근 권한이 필요하다.
DISPLAY가 비어 있고 서버 X11 :0 소켓이 있으면 :0과 사용자 .Xauthority를 기본으로 사용한다.
다른 세션은 해당 DISPLAY·인증 설정을 사용한다.

`--steps 100`으로 프레임 수를 제한하고 `--report 경로`로 결과를 기록한다.
기본은 창 닫기 또는 Ctrl+C까지 표시한다. WebRTC는 한 번에 하나씩 실행한다.
시나리오 목록은 `scenarios/registry.py`에 등록한다.
로봇 없는 viewer는 plane 지면과 정적 배치를 표시한다.

Go2는 viewer가 생성하고 별도 제어기가 정책을 실행한다.

```bash
# 터미널 1
python scripts/viewer.py --scenario shipyard --robot go2 --no-webrtc
# 터미널 2
python scripts/go2/control.py
```

launcher를 import하는 것만으로 Isaac Sim이 시작되지는 않는다.
앱 초기화 보정은 별도 자식 프로세스에서 지정 스크립트를 실행하기 전에 적용한다.
viewer 내부의 --worker와 launcher의 --entry는 이 실행 단계용 내부 인자다.
이 초기화 보정은 Isaac Sim 5.0 / Isaac Lab 2.2.1 구성에 맞춘 것이다.

전체 구조와 실행 명령은 [로봇 담당자 안내](../../docs/developer_guide.md)를 참고한다.

Viewer가 Go2를 생성하고 별도 `scripts/go2/control.py`가 정책을 실행한다.
[분리 실행 안내](../../docs/viewer_control.md)를 참고한다.
