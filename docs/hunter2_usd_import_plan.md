# HUNTER 2.0 USD import 계획 — 실행 전

source 교정·정적 검증은 완료했다. Isaac Sim은 5.0 / Python 3.11로 선택했으며
[환경 설치 상태](isaacsim_5_setup.md)를 확인한다. USD 생성, 차량 PhysX 설정,
controller 구현은 아직 수행하지 않았다. Isaac Lab/ROS 2 설치는 별도 단계다.

## 시작 조건

- 팀의 Isaac Sim 버전과 Ubuntu 22.04 지원 여부, GPU/driver 조건을 확정한다.
- 해당 버전 공식 URDF Importer 문서의 URL·버전·확인일을 이 문서에 추가한다.
- `prepare_hunter2_asset.py`와 `validate_hunter2_asset.py`를 실행하고 source lock을 기록한다.
- [asset README](../assets/hunter2/README.md)의 물리 TODO를 먼저 검토한다.

파생 source의 조향축과 중복 geometry는 교정됐고 정적 검증을 통과했다.
[교정 보고서](hunter2_source_correction.md)를 확인한 뒤 초기 import를 **진단용**으로 진행한다.
import 성공만으로 정상 주행 완료로 처리하지 않는다. mass/inertia와 제어·접촉은 별도 검증한다.

## 버전별 초기 설정 후보

| 항목 | 초기 후보 | 확정 조건 |
|---|---|---|
| Robot Type | Wheeled | 해당 버전에서 실제 항목 지원·의미 확인 |
| Base Type | Mobile / fix_base=False | world에 base 고정 joint가 생기지 않는지 확인 |
| Merge Fixed Joints | OFF | 원본 link 추적용; zero-mass fixed link 처리는 별도 확인 |
| Self Collision | OFF | 겹친 mesh 가능성 확인 후 재검토 |
| Mesh URI | URDF 파일 기준 `meshes/...` | 해당 importer의 상대경로 resolution 확인 |

UI 명칭/API 키는 버전에 따라 다를 수 있다. 위 표현을 현재 실행 가능한 API 코드로
간주하지 않는다. importer가 절대경로를 요구하면 버전별 임시 staging 파일에서 변환하고
canonical source에는 절대경로나 PhysX schema를 넣지 않는다.

## 변환 및 검토 순서

```text
hunter2_sim.urdf + meshes/
  → 선택 버전 URDF Importer
  → hunter2_imported.usd (진단용)
  → articulation/root/mobile base 확인
  → steering joint 축·부호·범위 확인
  → wheel joint 축·부호·자유 회전 확인
  → visual/collision/접지 확인
  → mass/inertia와 fixed link 처리 확인
  → 근거가 있는 source 수정 및 재import (필요 시)
  → drive 설정 및 저속 실험
  → hunter2.usd (검증 완료 후)
```

1. import 당시 source commit/hash, importer 옵션, 출력 위치를 별도 manifest에 기록한다.
2. geometry 단위, 전면 +X, 좌측 +Y, 위쪽 +Z, wheel 위치를 시각 검증한다.
3. joint 실제 이름 8개와 root/free base를 비교하고, 고정 joint 병합 여부를 기록한다.
4. 좌우 steering을 각각 작은 각도로 움직여 수직축 yaw인지 확인한다. 교정 source의
   수학 검증 결과가 USD에도 보존됐는지 검사하고 반대쪽 wheel은 움직이지 않는지 확인한다.
5. wheel을 개별 회전시켜 축과 전진 부호를 확인한다. 원본 좌우 axis 부호는 반대다.
6. 제거한 steering collider가 다시 생성되지 않았는지, wheel collider와 접지를 확인한다.
   triangle collision의 지원과 성능, 생략된 비바퀴 외형도 검토한다.
   collision 대체가 필요하면 원본 보존 하에 변경 근거와 영향 범위를 기록한다.
7. zero-mass `rear_wheel_link`, 작은 장식 link inertia, 총 mass 차이와 접지를 확인한다.
   importer의 자동 mass/inertia 재계산 여부도 기록하고 조용히 원본 값으로 간주하지 않는다.
8. 비교 보고서의 공식 CAN 후륜 motor mapping을 근거로 다음 역할을 구현하고 검증한다.
   전륜 steering position drive, 후륜 wheel velocity drive, 전륜 wheel passive rolling.
9. 원본 rear velocity=1 rad/s와 nominal 1.5 m/s의 차이를 해소하고, drive gain/torque/
   friction은 근거 있는 범위에서 별도 calibration한다.
10. 안정된 정지·전진·후진·좌우 선회, steering limit, 접지·slip을 확인하고 결과를 남긴다.

## 후속 제어 및 프로젝트 확장

주행 가능한 asset 검증 후에만 아래 경로를 구현한다.

```text
Desired linear velocity + desired center steering angle
  → Ackermann kinematics/controller
  → left/right steering angle + rear left/right wheel velocity
  → robot adapter → 검증된 HUNTER articulation
```

0.461 rad는 중심각, 0.58 rad는 individual steering 범위다.
실제 URDF geometry와 nominal YAML geometry의 차이를 명시적으로 해결한 뒤 사용한다.
상위 navigation은 HUNTER 전용으로 만들지 않는다. 공통 `set_goal`, `set_path`,
`get_pose`, `get_velocity`, `goal_reached` 인터페이스를 정의하고 Hunter/Go2/Omni adapter가
각각 Ackermann controller / locomotion policy / holonomic controller로 변환한다.
ROS 2 연동 시 Goal/Path 수준의 `PoseStamped`, `nav_msgs/Path` 공유를 검토한다.
저수준 `/cmd_vel` 하나로 각 로봇의 이동 특성을 강제 통일하지 않는다.
waypoint following 이후 planning, multi-robot, task allocation, scheduling,
traversability/time/energy cost, MARL은 별도 단계로 진행한다.
