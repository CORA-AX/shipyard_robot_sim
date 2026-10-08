# HUNTER 2.0 canonical source asset

최종 교정: 2026-10-06 (Asia/Seoul). **조향축·중복 형상 교정 및 정적 검증 완료.**
Isaac Sim import와 물리/주행 검증은 미실행이다. 사용할 버전은 5.0이며,
[환경 설치 상태 및 실행 방법](../../docs/isaacsim_5_setup.md)을 참고한다.
[교정 결과 보고서](../../docs/hunter2_source_correction.md)를 현재 상태의 기준으로 사용한다.

## 원본과 파생 모델

- 원본: AgileX `ugv_gazebo_sim`, commit `27633a956c845903ee630538afeb17fe70afdd84`.
- 공식 수치: AgileX `hunter_ros2` humble, commit `7bb146dd40fcea757b3516292319aa14ace41577`.
- LCAS 비교: `hunter_robot`, commit `e073b273674459a13ec57b8731d06ddc8770546f`.
- 입력 해시: [sources.lock.json](config/sources.lock.json).
- `source/original/hunter2_base.urdf`와 STL 9개는 AgileX 원본 그대로 보존한다.
- 실행 입력 후보는 `source/hunter2_sim.urdf`다. 생성 스크립트를 통해 재생성한다.

원본 저작자 주석과 BSD license 표기가 있는 package 메타데이터는 vendor에 유지한다.
LCAS의 코드/mesh를 복사하지 않고, 수직 조향축 및 geometry ownership 설계를 참고했다.

## 실제 적용한 변경

| 변경 | 내용 |
|---|---|
| Mesh 경로 | ROS package 접두어를 제거하고 `meshes/...` 상대경로 사용 |
| Steering 범위 | 기존 단계의 좌우 ±0.58 rad 유지; driver 기반 임시 범위 |
| Steering axis | 좌 `0 0 1`, 우 `0 0 -1`을 모두 `0 1 0`으로 변경 |
| Steering geometry | `front_steer_left_link`, `front_steer_right_link`의 visual/collision 각각 제거 |

조향 link는 inertial과 parent/child 연결을 가진 무형상 frame으로 남는다.
바퀴 visual/collision은 해당 wheel link에 남긴다. 원본 STL 자체는 수정·삭제하지 않는다.
현재 참조는 14개(visual 7 + collision 7), 참조하는 STL은 7개, 보관하는 STL은 9개다.
9 links / 8 joints(조향 revolute 2, wheel continuous 4, fixed 2)는 유지한다.

이름·origin xyz/rpy·wheel axis·질량·COM·관성·effort·velocity는 변경하지 않았다.
Gazebo/ros2_control/plugin/mimic은 도입하지 않았다. 조향 mesh의 비바퀴 부분도 생략되므로
완전한 외형 복원이나 정확한 질량 분포까지 완료한 모델로 해석하지 않는다.

## 좌표와 검증 결과

+X forward, +Y left, +Z up 기준이다. 조향 origin rpy `1.5708 0 0`을 유지했으므로
local +Y는 base에서 약 `(0, -0.0000036732, 0.99999999999)`로 변환된다.
roll의 반올림 오차를 허용해 수직 +Z를 확인한다. 좌우 wheel axis 부호는 기존처럼 반대다.

검증기는 각 조향을 독립적으로 −10°, 0°, +10° 움직이는 FK를 계산한다.
해당 wheel 진행 방향은 같은 각도로 바뀌며 반대쪽 wheel heading은 0°로 유지된다.
이는 독립 관절 운동 검사이며 Ackermann controller 또는 실제 접촉 검증이 아니다.

[validation_report.json](config/validation_report.json)에 축, 조향 샘플, mesh bounds,
질량 및 잔여 warning이 기록된다. 잘못된 축·조향 연결·중복 geometry·mimic은 실패 처리한다.

## 아직 유효한 제한

- 총 질량 52.2274695954 kg 및 모든 원본 inertial 유지. 실제 65/70 kg과 차이가 있다.
  geometry를 없애도 mass가 자동으로 줄지 않으며 mass/inertia calibration은 미완료다.
- 질량/관성 0인 fixed `rear_wheel_link`의 import/병합 처리를 확인해야 한다.
- 후륜 limit 1 rad/s 유지. nominal r=.165 m에서 직진 약 .165 m/s에 해당한다.
  목표 1.5 m/s에 필요한 9.0909 rad/s 및 선회 시 좌우 속도 정책은 후속 작업이다.
- driver ±.58 rad는 manual inner 33°보다 약 .23155° 크다. 진단용 기존 범위를 유지했으며
  최대 조향 시험/정상 주행 전에 hard limit 정책을 확정해야 한다. 중심각 .461은 별도 값이다.
- [hunter2.yaml](config/hunter2.yaml)의 L=.650, T=.605, r=.165는 nominal driver 수치다.
  실제 URDF wheelbase 좌/우 .65142/.65160, front track .584, rear track .585와 구분한다.
- 좌우 조향과 후륜 속도 분배 controller, drive/contact/friction 설정은 미구현이다.

공식 매뉴얼 간 차이와 driver 근거는 [비교 보고서](../../docs/hunter2_model_comparison.md)에 있다.
CAN motor mapping은 후륜 traction을 뒷받침한다. 실제 구동·접촉 결과는 Isaac에서 확인한다.

## 재생성과 검증

프로젝트 루트에서 실행한다. Python 3.10+ 표준 라이브러리와 Git만 필요하다.

```bash
python3 scripts/prepare_hunter2_asset.py
python3 scripts/validate_hunter2_asset.py
python3 -m unittest discover -s tests -v
```

생성기는 vendor commit/입력 해시를 확인하고 원본 snapshot과 mesh를 보존한다.
동일 입력에는 byte 단위 동일한 URDF/YAML/검사 JSON이 생성된다. CRLF도 유지한다.
`source/hunter2_sim.urdf`, YAML, 검사 JSON은 재생성 대상이므로 생성기를 통해 변경한다.

URDF와 meshes 폴더를 함께 옮겨 다음으로 경로 이동성을 확인할 수 있다.

```bash
python3 scripts/validate_hunter2_asset.py /path/to/copied/source/hunter2_sim.urdf --structural-only
```

`--structural-only`는 vendor provenance 검사를 생략하며 조향/ownership 검사는 유지한다.
Isaac importer에서의 상대경로 처리까지 검증한 것은 아니다.
USD 폴더에는 `.gitkeep`만 있으며 [USD import 계획](../../docs/hunter2_usd_import_plan.md)을 따른다.
