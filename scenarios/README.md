# 공통 시나리오

배경·장애물·배치와 로봇별 환경 연결을 관리한다. 로봇 모델 파일은 `assets/<robot>/`,
향후 제어·상태 adapter는 `robots/<robot>/`, 시각화는 `scripts/visualization/`에서 관리한다.

- `shipyard/scene_cfg.py`: 로봇에 독립적인 지면·조명·화물 상자·카메라 설정.
- `shipyard/go2_env_cfg.py`: 공통 배경에 Go2 로봇·센서를 배치하고 기존 관측·액션 설정을 연결.
- `assets/environments/shipyard/`: 향후 공통 배경 USD·지도·재질 파일 위치.

현재 실행 가능한 연결은 Go2 한 대다. 여러 로봇을 동시에 실행하는 환경과 공통 실행기는
아직 구현하지 않았다. 공통 장면을 확장할 때 Go2 정책의 관측·액션을 다른 로봇에 적용하지 않는다.
Hunter2 연결과 차량 제어는 후속 작업이며 이번 정리에서는 Hunter2 파일을 수정하지 않았다.

기존 실행 명령은 유지한다.

```bash
python scripts/go2/run_go2_webrtc.py view --scenario shipyard --server-ip <서버_IP>
python scripts/go2/run_go2_webrtc.py policy --scenario shipyard --server-ip <서버_IP>
```

새 로봇을 연결할 때 `ShipyardSceneCfg`를 상속해 로봇·센서를 추가하고 별도 환경 설정에 연결한다.
동시 실행 환경에서는 하나의 장면에 여러 로봇을 배치하고 각자의 제어기를 같은 물리 시간으로 실행한다.

로봇 없는 장면 viewer는 `scripts/visualization/view_scenario.py`다.
`scenarios/registry.py`에 장면 모듈을 등록하며 모듈은 `SceneCfg`, `CAMERA_EYE`, `CAMERA_TARGET`을 제공한다.
`--webrtc`로 headless 스트리밍, `--no-webrtc`로 로컬 창을 선택한다.
