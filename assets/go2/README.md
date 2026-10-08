# Go2

프로젝트 소유의 Go2 로봇·정책 설정과 자산 기록이다. 환경 설정은 루트 `scenarios/`에서 관리한다. Isaac Sim 5.0 + Isaac Lab v2.2.1을 라이브러리로 사용한다.

- `config/robot_cfg.py`: 로봇 초기 자세와 모터 설정.
- 루트 `scenarios/flat/go2_env_cfg.py`: 평지 환경의 센서·관측·액션·리셋 정의.
- 루트 `scenarios/shipyard/scene_cfg.py`: 모든 로봇에 공유할 화물 상자·지면·카메라 설정.
- 루트 `scenarios/shipyard/go2_env_cfg.py`: 공통 환경과 Go2 연결.
- 루트 `scenarios/registry.py`: 공통 장면과 Go2 환경 등록.
- `config/sources.lock.json`: Isaac Lab commit, 원격 USD URL, 정책 출처와 SHA-256.
- `policies/flat.pt`: `python scripts/go2/prepare_go2_policy.py`로 복원하는 공식 정책.
- 로봇 USD와 참조 mesh는 NVIDIA 서버에서 로드하며, 이 폴더에는 전체 모델을 복제하지 않았다.

프로젝트 루트에서 공통 Conda 환경 `.conda/isaacsim-5.0`을 활성화한 뒤 실행한다.

```bash
python scripts/go2/prepare_go2_policy.py
# 터미널 1
python scripts/viewer.py --scenario shipyard --robot go2 --webrtc --server-ip 192.168.1.149
# 터미널 2
python scripts/go2/control.py
```

viewer가 로봇·환경·물리를 담당하고 제어기는 체크포인트 Actor를 추론한다.
정책 네트워크 구성은 `control/go2/policy.py`에서 관리한다. 평지 정책은 장애물 회피를 수행하지 않는다.
[설치 안내](../../docs/go2_setup.md)와 [분리 실행 안내](../../docs/viewer_control.md)를 참고한다.

로봇·정책 기본 설정은 Isaac Lab v2.2.1의 BSD-3-Clause 코드에서 가져왔다.
원본 저작권·라이선스는 [Isaac Lab LICENSE](LICENSE.isaaclab)를 참고한다.
