# Warehouse 배경 자산

기본 배경은 NVIDIA Simple Warehouse다.
[원본 레포의 create_warehouse_env](https://github.com/Zhefan-Xu/isaac-go2-ros2/blob/f201692a5519e6e6ed058637ce93a2f603f016f2/env/sim_env.py)는
`Isaac/Environments/Simple_Warehouse/warehouse.usd`를 참조한다.
이 프로젝트는 같은 경로의 Isaac Sim **5.0** 배포 자산을 사용한다.
URL과 출처는 `sources.lock.json`에 기록하며, 자산은 NVIDIA 서버에서 불러온다.
첫 실행은 종속 USD와 텍스처 다운로드 때문에 시간이 걸릴 수 있다.

USD에 저장된 벽·선반·상자·재질·충돌 형상을 그대로 사용하며, 임의의 지형 샘플링은 없다.
USD 자체의 바닥 collision plane을 사용해 별도 바닥과 겹치지 않게 했다.
실험 seed는 기존 Go2 환경과 동일한 42, 시작 위치는 `(0, 0, 0.4)`m, yaw는 0이다.
리셋도 같은 위치를 사용한다. 위치와 카메라는 `scenarios/warehouse/scene_cfg.py`에서 수정한다.

```bash
# 터미널에서 바로 조작
python scripts/go2/ros.py --keyboard

# ROS 입력
python scripts/go2/ros.py
python scripts/go2/ros_command.py --keyboard

# viewer + 별도 제어기
python scripts/viewer.py --robot go2 --no-webrtc
python scripts/go2/control.py --keyboard

# ROS 보행/정지/회전/clock 검사
python tests/check_go2_ros.py --scenario warehouse

# 실제 USD 충돌·시작 위치·리셋 검사
python tests/check_warehouse.py --headless
```

viewer와 ROS의 기본 시나리오는 `warehouse`이며 `flat`도 선택할 수 있다.
