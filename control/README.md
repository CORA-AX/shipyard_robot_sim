# 로봇 제어 및 상태 연결

로봇별 제어·실행 코드와 향후 공통 상태 adapter를 관리한다.
모델·로봇 설정·정책 파일은 `assets/<robot>/`에, 공통 장면은 `scenarios/`에 둔다.

- `go2/`: 별도 정책 추론과 viewer 내부 환경 실행·상태 기록. 명령 진입점은 `scripts/go2/`.
- `hunter2/`: 향후 조향·바퀴 속도 제어·상태 adapter. 기존 Hunter2 코드는 유지.
- `omni/`: 향후 전방향 로봇 제어·상태 adapter.

상위 목표·상태 인터페이스는 공유하고 로봇별 구동 계산은 각 adapter가 담당하도록 확장한다.
현재 공통 인터페이스 구현이나 동시 로봇 실행 지원을 의미하지 않는다.

전체 구조와 실행 명령은 [로봇 담당자 안내](../docs/developer_guide.md)를 참고한다.

Viewer가 Go2를 생성하고 별도 `scripts/go2/control.py`가 정책을 실행한다.
[분리 실행 안내](../docs/viewer_control.md)를 참고한다.
