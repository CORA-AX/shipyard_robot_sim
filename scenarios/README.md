# 공통 시나리오

배경·장애물·배치와 로봇별 환경 연결을 관리한다. 로봇 모델 파일은 `assets/<robot>/`,
향후 제어·상태 adapter는 `control/<robot>/`, 시각화는 `scripts/visualization/`에서 관리한다.

- `warehouse/scene_cfg.py`: 기본 NVIDIA warehouse USD·조명·카메라·시작 위치 설정.
- `warehouse/go2_env_cfg.py`: 공통 배경에 Go2 로봇·접촉 센서를 배치하고 기존 관측·액션 설정을 연결.
- `assets/environments/warehouse/`: 배경 자산 URL·버전·원본 레포 출처 기록.

기존 viewer/환경 연결은 Go2 한 대이며 ROS workspace의 공통 실행부는 같은 장면에
Go2+Lekiwi를 배치한다. 로봇 목록과 시작 위치는 `robot_bringup/config/go2_lekiwi.yaml`,
articulation 생성과 물리 루프는 `robot_sim/scene.py`, `multi_runtime.py`에서 관리한다.
Go2 정책의 관측·액션은 Go2에만 사용하고 Lekiwi는 바퀴 제어기를 사용한다.
Hunter2 연결과 차량 제어는 후속 작업이며 이번 정리에서는 Hunter2 파일을 수정하지 않았다.

```bash
# 터미널 1
python scripts/viewer.py --scenario warehouse --robot go2 --webrtc --server-ip <서버_IP>
# 터미널 2
python scripts/go2/control.py
```

새 로봇을 연결할 때 `WarehouseSceneCfg`를 상속해 로봇·센서를 추가하고 별도 환경 설정에 연결한다.
동시 실행 환경에서는 하나의 장면에 여러 로봇을 배치하고 각자의 제어기를 같은 물리 시간으로 실행한다.

로봇 없는 장면 viewer는 `scripts/viewer.py`다.
`scenarios/registry.py`에 장면 모듈을 등록하며 모듈은 `SceneCfg`, `CAMERA_EYE`, `CAMERA_TARGET`을 제공한다.
`--webrtc`로 headless 스트리밍, `--no-webrtc`로 로컬 창을 선택한다.

Go2 환경 설정도 모두 이 폴더에서 관리한다. `flat/go2_env_cfg.py`는 기본 Go2 환경,
`warehouse/go2_env_cfg.py`는 기본 창고 연결이다. `registry.py`의 `GO2_SCENARIOS`에 등록한다.
`assets/go2`에는 로봇 설정·정책 파일과 자산 출처만 둔다.

Go2 정책 추론과 viewer 내부 물리 루프는 `control/go2/`, 사용자 실행 명령은 `scripts/go2/`에 둔다.


전체 구조와 실행 명령은 [로봇 담당자 안내](../developer_guide.md)를 참고한다.

Viewer가 Go2를 생성하고 별도 `scripts/go2/control.py`가 정책을 실행한다.
[분리 실행 안내](../docs/viewer_control.md)를 참고한다.

기본 시나리오는 warehouse다. 기본 seed는 42이며 warehouse 배치는
USD에 고정되어 있다. 시작 위치는 `(0, 0, 0.4)`m, yaw는 0이고 리셋 후에도 동일하다.
별도 ROS 실행은 `python scripts/go2/ros.py --keyboard`를 사용한다.
[자산 출처와 실행 안내](../assets/environments/warehouse/README.md)를 참고한다.
