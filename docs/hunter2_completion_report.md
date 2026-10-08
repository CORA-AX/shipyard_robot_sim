# HUNTER 2.0 source 준비 완료 보고

> 이 문서는 교정 전 조사 기록입니다. 아래의 Current/미수정 판정은 당시 baseline에 해당합니다.
> 2026-10-06 실제 교정 결과는 [소스 교정 보고서](hunter2_source_correction.md)를 참고하세요.

작업일: 2026-10-02. 프로젝트: `/home/rodix-gpu/shipyard_robot_sim`.

1. **원본 commit**: `27633a956c845903ee630538afeb17fe70afdd84`.
   공식 hunter_ros2 참고 commit: `7bb146dd40fcea757b3516292319aa14ace41577`.
2. **사용한 URDF**: `third_party/ugv_gazebo_sim/hunter/hunter2_base/urdf/hunter2_base.urdf`.
   원본 snapshot과 STL 9개는 byte 단위 보존했다.
3. **joint 구조**: 조향 `front_steer_left_joint`, `front_steer_right_joint`는 revolute;
   바퀴 `front_left_wheel_joint`, `front_right_wheel_joint`, `left_rear_joint`,
   `right_rear_joint`는 continuous; `front_steer_joint`, `rear_wheel_joint`는 fixed.
   전체 link 9개, joint 8개. parent/child/axis 표는 asset README에 있다.
4. **원본 대비 모든 변경**: 파생 URDF의 mesh URI 18곳을 상대경로로 변경하고,
   두 조향 joint의 lower/upper 총 4개 값을 0에서 −0.58/+0.58 rad로 변경했다.
   그 외 모든 URDF byte와 STL은 동일하다. 원본에는 제거할 Gazebo/transmission 요소가 없다.
5. **변경 이유**: ROS package 의존 경로 제거와 잠긴 steering limit 해제.
   0.58은 공식 V2 driver의 individual/inner-wheel 관련 상수,
   0.461은 별도의 중심/bicycle 조향 상수로 기록했다.
6. **발견한 차이**: wheelbase 0.65142/0.65160 대 nominal 0.650 m,
   앞/뒤 track 0.584/0.585 대 0.605 m, 질량 합계 52.22747 대 매뉴얼 65/70 kg.
   rear velocity limit 1 rad/s는 nominal 1.5 m/s와 맞지 않는다.
   driver/매뉴얼 사이 속도 단위 및 감속비 불일치도 README에 기록했다.
7. **미검증 사항**: 조향축은 현재 base ±Y 방향이라 수직 yaw 조향과 불일치한다.
   CAD 외형 전면, mesh 겹침, 실제 구동축, 기계적 조향 stop, 실물 mass/inertia,
   dynamics 및 실제 importer 경로 해석을 확인하지 않았다. 추측 보정은 하지 않았다.
8. **향후 Isaac 확인**: version/importer 확정 → 진단용 import → 좌표·조향축 시각 검증
   → 필요 시 파생 source 수정 → articulation/wheel/collision/mass/inertia 확인
   → drivetrain 근거 확정과 drive 설정 → 저속 주행 검증 → 최종 USD.

검증 결과:

- `python3 -m unittest discover -s tests -v`: 12개 통과.
- 생성기 및 provenance 검사 통과. 반복 생성 결과 asset 파일 16개 동일.
- vendor 두 저장소 `git status --porcelain` 출력 없음.
- 다른 폴더에 URDF+meshes만 복사한 구조 검사 통과.
- 잘못된 XML, 중복 이름, 없는 parent, 잠긴 steering, 잘못된 joint type,
  없는 mesh, package URI, 경로 이탈, 끊어진 cycle의 거부 확인.
- `assets/hunter2/usd/`는 `.gitkeep`만 존재.

**판정: canonical source 준비 및 정적 검증 완료. 물리·Ackermann 주행 검증 미완료.**

근거와 한계: [asset README](../assets/hunter2/README.md),
[재현성 검사 기록](asset_checks.json), [USD 계획](hunter2_usd_import_plan.md).
