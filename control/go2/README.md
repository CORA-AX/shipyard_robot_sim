# Go2 제어·시뮬레이션

- `policy.py`: Isaac Sim 없이 PyTorch로 체크포인트 Actor 로딩·추론.
- `simulation.py`: viewer 내부 환경 생성·관측·액션 적용·물리 진행·대기 렌더링·상태 기록.
- 공통 `../control_channel.py`: 로컬 제어 연결·세션·스텝·메시지 검사.

모델·모터 설정은 `assets/go2/config/`, 관측·액션·리셋은 `scenarios/`에서 관리한다.

```bash
# 터미널 1: 환경·Go2 생성
python scripts/viewer.py --scenario shipyard --robot go2 --no-webrtc
# 터미널 2: 정책 연결
python scripts/go2/control.py
```

제어기 종료 시 물리는 정지하고 다시 연결하면 현재 상태에서 이어진다.
[분리 실행 안내](../../docs/viewer_control.md)를 참고한다.
