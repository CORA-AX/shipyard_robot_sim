# Go2 제어·시뮬레이션

- `policy.py`: Isaac Sim 없이 PyTorch로 체크포인트 Actor 로딩·추론.
  기본 모델은 자산 manifest의 `control_policy`를 사용하고 `control.py --checkpoint`로 바꾼다.
- `simulation.py`: viewer 내부 환경 생성·관측·액션 적용·물리 진행·대기 렌더링·상태 기록.
- `ros_runtime.py`: ROS 명령을 관측에 반영하고 동일한 선택 정책·물리를 50Hz로 실행.
- `ros_bridge.py`: 내장 Humble 환경 설정, cmd_vel 수신·명령 만료, `/clock` 발행.
- `keyboard.py`: 공통 터미널 키 입력·자동 정지·터미널 설정 복원.
- 공통 `../control_channel.py`: 로컬 제어 연결·세션·스텝·메시지 검사.

정책·키보드·ROS 관련 구현의 원본은 이제 `ros2_ws/src/robot_control` 및 `robot_sim`에 있다.
이 폴더의 해당 Python 파일은 기존 실행 명령을 위한 호환 import다.
[workspace 실행 안내](../../ros2_ws/README.md)를 참고한다.

모델·모터 설정은 `assets/go2/config/`, 관측·액션·리셋은 `scenarios/`에서 관리한다.

```bash
# 터미널 1: 환경·Go2 생성
python scripts/viewer.py --scenario warehouse --robot go2 --no-webrtc
# 터미널 2: 정책 연결
python scripts/go2/control.py
```

제어기 종료 시 물리는 정지하고 다시 연결하면 현재 상태에서 이어진다.
[분리 실행 안내](../../docs/viewer_control.md)를 참고한다.

ROS 독립 실행은 `python scripts/go2/ros.py --scenario warehouse`를 사용한다.
이 실행기는 `control_channel.py`를 사용하지 않으며 정책과 물리를 직접 실행한다.
`python scripts/go2/ros_command.py --vx 0.4`로 명령을 보낼 수 있다.
명령이 없으면 속도 목표가 0이고, 자세 유지를 위한 정책 실행은 계속한다.
[ROS 사용 안내](../../README.md#ros로-go2-직접-제어)를 참고한다.

`control.py`, `ros.py`, `ros_command.py`에 `--keyboard`를 지정할 수 있다.
`control.py`는 액션과 속도 명령을 같은 메시지로 보내 viewer에 함께 반영한다.
`ros.py`는 직접 터미널 입력을 사용하고, `ros_command.py`는 키 입력으로 cmd_vel을 발행한다.
[키보드 사용 안내](../../README.md#키보드-이동)를 참고한다.
