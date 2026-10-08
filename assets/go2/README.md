# Go2

프로젝트 소유의 Go2 로봇·정책 설정과 시나리오다. Isaac Sim 5.0 + Isaac Lab v2.2.1을 라이브러리로 사용한다.

- `config/robot_cfg.py`: 로봇 초기 자세와 모터 설정.
- `config/policy_cfg.py`: 관측 48개·액션 12개 체크포인트용 정책 구성.
- `scenarios/flat_env_cfg.py`: 평지 환경의 센서·관측·액션·리셋 정의.
- 루트 `scenarios/shipyard/scene_cfg.py`: 모든 로봇에 공유할 화물 상자·지면·카메라 설정.
- 루트 `scenarios/shipyard/go2_env_cfg.py`: 공통 환경과 Go2 연결.
- `scenarios/shipyard_env_cfg.py`: 기존 import 경로 호환용.
- `scenarios/__init__.py`: 시나리오 등록.
- `config/sources.lock.json`: Isaac Lab commit, 원격 USD URL, 정책 출처와 SHA-256.
- `policies/flat.pt`: `python scripts/go2/prepare_go2_policy.py`로 복원하는 공식 정책.
- 로봇 USD와 참조 mesh는 NVIDIA 서버에서 로드하며, 이 폴더에는 전체 모델을 복제하지 않았다.

프로젝트 루트에서 공통 Conda 환경 `.conda/isaacsim-5.0`을 활성화한 뒤 실행한다.

```bash
export PUBLIC_IP=192.168.0.10  # 클라이언트에서 접근 가능한 실제 서버 IP
python scripts/go2/run_go2_webrtc.py view --scenario shipyard
python scripts/go2/run_go2_webrtc.py policy --scenario shipyard
```

`view`는 정책 없이 환경 보기, `policy`는 정책 실행·내보내기, `check`는 전진 검증이다.
각 모드는 WebRTC Streaming Client에서 서버 IP로 연결해 본다.

[설치·실행·검증 범위](../../docs/go2_setup.md)를 참고한다.

세 모드는 프로젝트 `scripts/go2/run_go2.py`를 실행한다. Isaac Lab 예제 스크립트는 실행하지 않는다.
기본 `--scenario flat`도 지원한다. 화물 상자가 있어도 현재 평지 정책은 장애물을 감지하거나 회피하지 않는다.

로봇·정책 기본 설정은 Isaac Lab v2.2.1의 BSD-3-Clause 코드에서 가져왔다.
원본 저작권·라이선스는 [Isaac Lab LICENSE](LICENSE.isaaclab)를 참고한다.
