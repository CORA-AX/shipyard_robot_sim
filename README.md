# Shipyard robot simulation

조선소 이기종 로봇 시뮬레이션을 위한 source asset 준비 프로젝트.
HUNTER 2.0 URDF 교정·정적 검증을 완료했고, Isaac Sim 5.0 / Python 3.11 환경을 사용한다.
환경 설치 결과와 실행 방법은 [Isaac Sim 설정 문서](docs/isaacsim_5_setup.md)에 있다.

```bash
cd /home/rodix-gpu/shipyard_robot_sim
python3 scripts/prepare_hunter2_asset.py
python3 scripts/validate_hunter2_asset.py
python3 -m unittest discover -s tests -v
```

위 URDF 정적 검사에는 Python 3.10 이상 표준 라이브러리와 Git만 필요하다. 생성 스크립트는 네트워크 접속,
ROS/Isaac 설치, USD 생성 없이 동작한다. 다른 위치에서는 해당 프로젝트 경로로 이동한다.

- [최신 조향축·중복 형상 교정 결과](docs/hunter2_source_correction.md)
- [HUNTER 분석 및 변경 내역](assets/hunter2/README.md)
- [향후 USD import 계획](docs/hunter2_usd_import_plan.md)
- [Isaac Sim 입력 적합성 추가 검토](docs/hunter2_isaac_source_review.md): 조향축 및 중복 wheel mesh 정량 검증
- [공식 source commit 및 SHA-256](assets/hunter2/config/sources.lock.json)
- [구조 검사와 좌표·mesh 분석 결과](assets/hunter2/config/validation_report.json)

`robots/{hunter2,go2,omni}`, `navigation`, `scenarios`, `ros2_ws`는 향후 작업용 빈 디렉터리다.
상위 navigation은 Goal/Path와 공통 상태 인터페이스를 사용하고 로봇별 adapter를 둔다.
컨트롤러와 waypoint follower는 아직 구현하지 않았다.

**현재 상태: 조향축·중복 형상 교정 및 정적 검증 완료, Isaac import·물리/주행 검증 미완료.**
독립 조향 운동 검증을 정상 Ackermann 주행 controller 완성으로 해석하지 않는다.
