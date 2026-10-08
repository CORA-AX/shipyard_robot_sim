# HUNTER 2.0/V2 source model 교차검증

> 이 문서는 교정 전 조사 기록입니다. 아래의 Current/미수정 판정은 당시 baseline에 해당합니다.
> 2026-10-06 실제 교정 결과는 [소스 교정 보고서](hunter2_source_correction.md)를 참고하세요.

자료 조사일: 2026-10-02, 검증 마무리: 2026-10-06 (Asia/Seoul).
**Audit only: 기존 asset/생성기/설정은 수정하지 않았다.**

결론: **LCAS의 조향-knuckle-wheel 분리 구조를 primary structural reference로 삼되,
그 모델 전체를 채택하지 않는다.** 공식 AgileX mesh와 원본 provenance를 보존하고,
실차 수치·구동계는 공식 매뉴얼/driver를 우선한다. LCAS의 ±0.461 individual limit,
1:1 mimic, 100.01 kg 질량, friction/effort는 그대로 가져오면 안 된다.
현재 `hunter2_sim.urdf`는 조향축과 중복 wheel 형상 문제가 있어 주행용 source freeze 불가다.
재현용 audit baseline을 고정하는 것은 가능하지만 기능 검증 완료 모델과 구분해야 한다.

## 1. 조사 repository와 정확한 commit

| Repository | Local path | HEAD |
|---|---|---|
| [AgileX ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/27633a956c845903ee630538afeb17fe70afdd84) | `third_party/ugv_gazebo_sim/` | `27633a956c845903ee630538afeb17fe70afdd84` |
| [LCAS hunter_robot](https://github.com/LCAS/hunter_robot/tree/e073b273674459a13ec57b8731d06ddc8770546f) | `third_party/hunter_robot/` | `e073b273674459a13ec57b8731d06ddc8770546f` |
| [AgileX hunter_ros2, humble](https://github.com/agilexrobotics/hunter_ros2/tree/7bb146dd40fcea757b3516292319aa14ace41577) | `third_party/hunter_ros2_reference/` | `7bb146dd40fcea757b3516292319aa14ace41577` |

각 저장소에서 `git rev-parse HEAD`를 실행했다. 원본 working tree는 clean 상태다.
매뉴얼은 GitBook 웹 자료라 commit이 없으며 URL·확인일·제원표/좌표계 이미지로 근거를 남겼다.

## 2. 실제 비교 파일과 재현 방법

AgileX repository 내부:

- `hunter/hunter2_base/urdf/hunter2_base.urdf`
- `hunter/hunter2_base/urdf/hunter2_base_gazebo.xacro`
- `hunter/hunter2_base/meshes/*.STL`
- `hunter/hunter2_gazebo/launch/hunter2_gazebo.launch`, `hunter2_empty_world.launch`
- `hunter/hunter2_control/launch/hunter2_v_contol.launch`
- `hunter/hunter2_control/config/ctrl_ackermann_steering_controller.yaml`
- `hunter/hunter2_control/config/ctrl_steer_bot_hardware_gazebo.yaml` 및 관련 설정
- `hunter/steer_bot_hardware_gazebo/src/steer_bot_hardware_gazebo.cpp`와 해당 header

LCAS repository 내부:

- `hunter_description/description/robot.urdf.xacro` — 실제 launch가 사용하는 entry
- `robot_core.xacro`, `meshes.xacro`, `inertial_macros.xacro`, `ros2_control.xacro` — 같은 description 폴더
- `robot_core_real_hack.xacro` — 별도 조사. 기본 entry에서 include하지 않으므로 기본 LCAS 열에는 제외
- `hunter_description/meshes/{chassis,front_left_wheel,front_right_wheel,rear_left_wheel,rear_right_wheel}.dae`
- `hunter_description/config/ackermann_like_controller.yaml`
- `hunter_description/launch/robot_view.launch.py`, `hunter_gazebo/launch/launch_sim.launch.py`

Driver는 `hunter_base/include/hunter_base/hunter_params.hpp`의 **HunterV2Params**를 읽었다.
현재 모델은 `assets/hunter2/source/hunter2_sim.urdf`이며 별도 열로 비교했다.

```bash
python3 scripts/compare_hunter2_models.py --output-dir docs/hunter2_comparison_evidence
python3 -m unittest discover -s tests -v
```

스크립트 기본 실행은 JSON stdout만 출력한다. 옵션 사용 시 docs 아래에만 evidence를 쓴다.
Python 3.10+ 표준 라이브러리만 필요하며 ROS/xacro/Isaac 설치나 실행은 하지 않았다.
LCAS의 실제 include/property/macro, pi와 사칙연산, package 경로 치환만 지원하는
**제한적 Xacro 전개기**다. 조건문·복잡한 macro 등 미지원 구문은 거부한다.
공식 xacro 실행 결과와의 byte 비교는 하지 않았으며, include 결과와 수치·인터페이스를 별도 테스트했다.
일반 Xacro 사용에는 ROS의 `xacro`와 `$(find ...)`를 해석할 package 환경이 필요하다.
AgileX Gazebo 파일은 확장자가 xacro지만 현 입력에는 xacro macro가 없어 XML로 직접 읽었다.

DAE는 실제 파일의 Z_UP, metre scale, identity scene matrix, triangle index를 확인해 분석했다.
비identity DAE transform 또는 미지원 primitive는 조용히 무시하지 않고 중단한다.
생성한 `lcas_expanded_for_analysis.xml`은 비교용 자료이며 실행용 asset이 아니다.

근거 산출물:

- [전체 수치·입력 SHA-256·joint/mesh/control 데이터](hunter2_comparison_evidence/comparison.json)
- [모든 joint와 link의 COM/inertia/visual/collision 상세표](hunter2_comparison_evidence/model_details.md)
- [Base와 전개 LCAS의 전체 XML diff](hunter2_comparison_evidence/base_vs_lcas.diff)
- [역할을 대응시킨 297개 필드 차이와 A–E 분류](hunter2_comparison_evidence/semantic_diff.md)
- [반복 실행·원본 보존·patch 적용 가능성 검증 기록](hunter2_comparison_evidence/audit_checks.json)

최종 검증: 테스트 19개 통과. 위 6개 자동 생성 자료(JSON, 상세표, 전개 XML,
XML diff, semantic diff, axis patch)는 반복 생성 시 byte 단위로 동일했다.
기존 `docs/asset_checks.json`의 SHA-256과 비교한 asset 16개가 모두 동일하고,
세 vendor repository의 working tree는 clean이다. Patch는 `git apply --check`만 실행했다.
테스트에는 **현재 조향축이 수직이 아니라는 경고가 검출되는지** 확인하는 항목도 포함한다.
따라서 테스트 통과는 현재 모델의 주행 적합성이나 물리 검증 완료를 뜻하지 않는다.

XML diff는 주석/서식 대신 파싱된 전체 모델을 비교한다. semantic diff는 이름이 다른 link/joint를
역할별로 대응시키며, raw XML diff와 함께 사용해야 한다. mesh binary 차이는 SHA·triangle 수·bounds로 기록한다.

## 3. 공식 실차 specification과 source priority

우선순위: manual/protocol → driver → AgileX Gazebo → LCAS → raw Base.
단 visual 형상은 원본 CAD mesh, 올바른 회전축/분리 topology는 수학 검증과 simulation 구현을 함께 판단한다.

| 항목 | 공식 reference | 근거/해석 |
|---|---|---|
| Steering | front-wheel Ackermann | 영문/중문 매뉴얼 일치 |
| Wheelbase | 0.650 m | manual 및 V2 driver |
| Front/rear track | 0.605 m | manual 및 V2 driver |
| Wheel radius | 0.165 m | driver; 확인한 manual 제원표에는 radius 직접 표기 없음 |
| Vehicle mass | 65/70 kg | manual, battery 옵션과 함께 표기; 정확한 옵션별 mass 대응은 확인 필요 |
| Payload | 150 kg | 중문 공식 manual; 차량 자체 mass와 합쳐 입력하지 않음 |
| Max speed | 무부하 1.5 m/s | GitBook 영문 이미지·중문 표·driver 일치 |
| Minimum turning radius | 1.6 m | 중문 표 m; 영문 이미지에 `mm`라고 적힌 단위 충돌은 그대로 기록 |
| Maximum inner-wheel steer | 33° = 0.5759586532 rad | 중문 표는 명확히 inner; 영문은 maximum steering angle 표현 |
| Driver individual/inner-related limit | 0.58 rad = 33.23155° | 약 33° 반올림값으로 가까우나 정확히 같지 않음 |
| Driver center/bicycle limit | 0.461 rad = 약 26.41° | individual limit와 다름 |
| Coordinate | +X forward, +Y left, +Z up | 진행 방향 본문과 좌표 그림; 좌표 원점 위치는 별도 문제 |
| Traction | 2 × 400 W, 좌·우 후륜 motor ID | CAN 설명의 motor mapping으로 후륜 구동 근거 확인 |
| Steering motor | 영문 이미지 200 W / 중문 400 W | source 간 충돌; 출력·gear calibration에 사용하지 않음 |
| Reduction | manual 1:40 / driver 1:30 | 하드웨어 revision/문서 차이 미확인 |

공식 근거:
[영문 제원](https://agilexrobotics.gitbook.io/hunter-2.0/1-hunter-2.0-introduction),
[중문 제원](https://agilexrobotics.gitbook.io/hunter-2-0/1-hunter-2.0-jian-jie-introduction),
[좌표·inner command 정의](https://agilexrobotics.gitbook.io/hunter-2.0/2-the-basics),
[CAN motor mapping](https://agilexrobotics.gitbook.io/hunter-2.0/3-getting-started),
[V2 driver](https://github.com/agilexrobotics/hunter_ros2/blob/7bb146dd40fcea757b3516292319aa14ace41577/hunter_base/include/hunter_base/hunter_params.hpp).
이미지는 [제원표](hunter2_comparison_evidence/manual_spec.png), [좌표 그림](hunter2_comparison_evidence/manual_coordinate.png)으로 보존했다.
CAN 영문 문장은 “four motors”라고 쓰면서 ID 1 조향, ID 2 우후륜, ID 3 좌후륜만 열거한다.
motor 개수 문구의 불일치는 남기되 후륜 ID mapping과 2개 traction motor를 근거로 삼는다.

## 4. 전체 비교표

S=steering, W=wheel. limit 미지정은 0 또는 무한대라고 단정하지 않는다.

| Parameter / Structure | AgileX Base | AgileX Gazebo | LCAS | Current hunter2_sim | Official Spec | Recommendation / 이유 |
|---|---|---|---|---|---|---|
| 좌표 방향 | 배치상 X전/Y좌, wheel 수평 axle | 동일 | 동일, pi/2 사용 | Base 동일 | X전/Y좌/Z상 | 방향 유지, frame 원점 별도 검토 |
| Root/body frame | base_link=차체 CAD frame | 동일 | 빈 base_link→chassis, Z+0.37 | Base 동일 | 원점 위치는 확인 안 됨 | LCAS의 +0.37을 실차 원점으로 자동 채택 금지 |
| Link/joint 수 | 9/8 | 9/8 | 8/7 | 9/8 | 역할은 2S+4W | LCAS 단순 구조 참고, rename 강제 불필요 |
| Fixed joint | front_steer_joint, rear_wheel_joint | 없음; 두 보조 joint 가동 | chassis_joint | Base 동일 | 명칭/보조 joint 규정 없음 | Gazebo 제어용 보조 DOF는 이식하지 않음 |
| S parent→child | base→S link→front W | 동일 | chassis→S link→front W | Base 동일 | yaw S와 spin W 분리 | LCAS의 link 역할 분리 참고 |
| S local axis 좌/우 | +Z / −Z | +Y / +Y | +Y / +Y | +Z / −Z | 수직 조향 필요 | Current 두 축 local +Y 수정 필수 |
| S base-frame axis | 약 −Y / +Y | 약 +Z / +Z | +Z / +Z | 약 −Y / +Y | +Z yaw | Gazebo와 LCAS가 타당 |
| S 범위 | 0 / 0 | ±0.69 | ±0.461 | ±0.58 | inner33°, driver .58, center .461 | zero 해제 필요; center .461을 개별 바퀴의 실차 최대각으로 간주하지 않음 |
| S velocity/effort | 1 / 100 | 0 / 0 | 1 / 10 | 1 / 100 | joint rate/torque 미확인 | Gazebo zero는 직접 position 제어 맥락; 숫자 복사 금지 |
| W types | 4 continuous | 4 continuous + 보조 continuous | 4 continuous | Base 동일 | 연속 회전 | 실제 4개 W 유지 |
| W local axis FL/FR/RL/RR | +Z/−Z/+Z/−Z | 모두 −Z | 모두 −Z | Base 동일 | 부호 정의 규정 없음 | 모두 유효한 transverse axle; sign map 또는 통일 선택 |
| W base-frame axis | 좌−Y/우+Y 근사 | 모두 +Y 근사 | 모두 +Y | Base 동일 | 좌우 axle 방향 | 축 부호를 무조건 오류로 보지 않음 |
| Wheelbase 좌/우 (m) | .65142/.65160 | .65142/.65160 | .65142/.65142 | Base 동일 | .650 | nominal과 CAD 차이 문서화, 위치를 임의 이동하지 않음 |
| Front track (m) | .584 | .584 | .585 | .584 | .605 | LCAS도 official 불일치; steering pivot/wheel center 정의 확인 |
| Rear track (m) | .585 | .585 | .585 | .585 | .605 | 공통 −20mm 차이; calibration 단계에서 geometry source 선택 |
| Wheel mesh radius | 약 .1642 | 약 .1642 | 약 .1643~.1644 | Base 동일 | driver .165 | mesh 최대 반경과 loaded rolling radius 구분 |
| Rear W velocity/effort | 1 / 100 | limit 없음 | 10 / 10 | 1 / 100 | 직진 최소 9.0909 rad/s | Current rate 재설계; LCAS 10은 직진 가능하나 최대 선회 충분성 별도 |
| Front W limit | 없음 | 없음 | 없음 | 없음 | passive 후보 | 명시적 drive 없음과 importer 기본 drive 차이 검증 |
| S visual | wheel 포함 STL | 작은 cylinder | 투명 cylinder | wheel 포함 STL | 실제 knuckle 형상 미확인 | 중복 wheel 제거; 투명 marker는 CAD 진실 아님 |
| S collision | wheel 포함 STL | 작은 cylinder | 없음 | wheel 포함 STL | tire는 W collider 소유 | LCAS의 collision ownership 참고 |
| Chassis/W collision | visual STL 공유 | 대부분 STL 공유 | visual DAE 공유 | STL 공유 | 정밀 제원 없음 | LCAS도 chassis/W primitive 단순화 모델은 아님 |
| FR visual/collision 일치 | 동일 right STL | visual은 left STL+rpyY3.14, collision right STL | 동일 right DAE | Base 동일 | 좌우 형상 근거 필요 | Gazebo visual 대체까지 자동 적용하지 않음 |
| Total mass (kg) | 52.22747 | 67.23239 | 100.01 | 52.22747 | 65/70 | Gazebo 합이 가깝다고 link 분포까지 신뢰하지 않음 |
| S inertia | 큰 wheel 유사값 | 거의 동일, off-diagonal 0 | mass .005, inertia diag .1 | Base 동일 | 부품별 미확인 | LCAS 값 복사 금지 |
| Zero mass/inertia | rear_wheel_link 0 | 작은 양수로 대체 | 빈 root에 inertial 없음 | 0 | 해당 link 제원 없음 | importer 처리 확인, 임의 mass 추가 금지 |
| Front driven | 명령 없음 | 양쪽 SetVelocity | command 없음, state만 | 명령 없음 | 후륜 traction 근거 | LCAS의 passive 전륜 역할이 더 일관됨 |
| Rear driven | 명령 없음 | 양쪽 같은 속도 직접 설정 | 좌 velocity, 우 1:1 mimic | 명령 없음 | 좌·우 후륜 모터 | 각 후륜 독립 속도 계산 필요 |
| Steering control | 없음 | SetPosition, 각도 분배식 | 좌 position, 우 1:1 mimic | 없음 | Ackermann 좌우 각도 다름 | 두 S 독립 position target 제어로 구현 |
| Control framework | 없음 | ROS1/Gazebo custom plugin | ros2_control + TricycleController | 없음 | CAN velocity+inner steer | framework 구문은 이식하지 않음 |
| Ackermann coupling | 없음 | 분배식·geometry·index 문제 있음 | 평행 조향 mimic | 없음 | common ICC 필요 | 어느 controller도 그대로 복사하지 않음 |
| Minimum radius | 정상 조향 불가 | raw limit/제어식과 spec 미일치 | 1:1 mimic이라 이상적 ICC 없음 | 정상 조향 불가 | 1.6m 정의 미확인 | 반경 정의 확인 후 비교 |
| URI | ROS package | ROS package | file://$(find...) | relative meshes/ | 해당 없음 | Current portable path 유지 |

## 5. Joint tree와 상세 joint 정의

Base/Current:

```text
base_link
├─ front_steer_left_joint [revolute] → front_steer_left_link
│  └─ front_left_wheel_joint [continuous] → front_left_wheel_link
├─ front_steer_right_joint [revolute] → front_steer_right_link
│  └─ front_right_wheel_joint [continuous] → front_right_wheel_link
├─ left_rear_joint [continuous] → left_rear_link
├─ right_rear_joint [continuous] → right_rear_link
├─ front_steer_joint [fixed] → front_steer_link
└─ rear_wheel_joint [fixed] → rear_wheel_link
```

AgileX Gazebo는 동일 tree에서 `front_steer_joint`가 revolute(±0.72, effort/velocity 0),
`rear_wheel_joint`가 continuous로 바뀐다. plugin의 집계/명령용 보조 joint이며
실물에 steering actuator와 별개의 추가 yaw 자유도가 있다고 해석하면 안 된다.

LCAS:

```text
base_link [inertial/geometry 없음]
└─ chassis_joint [fixed, xyz 0 0 .37] → chassis
   ├─ front_left_steering_joint [revolute] → front_left_steering_link
   │  └─ front_left_wheel_joint [continuous] → front_left_wheel
   ├─ front_right_steering_joint [revolute, mimic left] → front_right_steering_link
   │  └─ front_right_wheel_joint [continuous] → front_right_wheel
   ├─ rear_left_wheel_joint [continuous] → rear_left_wheel
   └─ rear_right_wheel_joint [continuous, mimic left] → rear_right_wheel
```

각 joint의 **실제 이름·type·parent/child·xyz·rpy·axis·lower/upper·velocity·effort**는
[자동 생성 상세표](hunter2_comparison_evidence/model_details.md)에 네 모델 모두 기록했다.
없어진 joint와 새 root/chassis도 diff에 포함한다. Current topology 자체의 link 연결은
유효한 tree이며, 현재 반드시 고쳐야 하는 기구학 오류는 연결보다 steering axis다.

## 6. Steering-axis 수학 검증

joint origin 변환은 `T_parent × Trans(xyz) × Rz(yaw) Ry(pitch) Rx(roll)`이다.
joint motion은 이 뒤에 `Rot(axis_local,q)`를 적용한다. 모든 ancestor 회전을 누적한다.

```text
Base/Current left:  Rx(1.5708) × (0,0,+1) ≈ (0,−1,−3.6732e−6)
Base/Current right: Rx(1.5708) × (0,0,−1) ≈ (0,+1,+3.6732e−6)
Gazebo both:       Rx(1.5708) × (0,+1,0) ≈ (0,−3.6732e−6,+1)
LCAS both:         Rx(pi/2)   × (0,+1,0) ≈ (0,0,+1)
```

wheel axle `a(q)`와 지면 법선 `z`로 rolling direction을 `a(q) × z`로 계산하고,
부호를 +X 쪽으로 정규화했다. heading=`atan2(d_y,d_x)`.

| steering q (rad) | Base left/right | Gazebo left/right | LCAS left/right | Current left/right |
|---:|---:|---:|---:|---:|
| −0.3 | 0 / 0 (limit 밖 가상 계산) | 약 −.3 / −.3 | −.3 / −.3 | 0 / 0 |
| 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| +0.3 | 0 / 0 (limit 밖 가상 계산) | 약 +.3 / +.3 | +.3 / +.3 | 0 / 0 |

여기서는 각 S를 독립 구동해 축만 검사했다. LCAS mimic을 적용하면 두 heading이 같아져
축 검사는 통과해도 Ackermann 연동 검사는 실패한다. 두 검증을 혼동하면 안 된다.

## 7. Wheel geometry, ownership, collision

STL binary triangle와 DAE triangle index를 읽고 실제 link/world 영점 변환을 적용했다.
1 µm 반올림 vertex triple(정점 순서·normal 제외)의 multiset 교집합을 계산했다.

| 모델 | 좌/우 steering visual 형태 | 좌/우 steering collision | 앞 wheel과 중복 triangle 좌/우 |
|---|---|---|---:|
| Base | wheel 포함 STL | 동일 STL | 36,470 / 19,714 (각 wheel 전체) |
| Gazebo | cylinder r=.005, length=.001 | 같은 cylinder | mesh triangle 중복 0; primitive와 wheel의 미세 교차까지 증명한 것은 아님 |
| LCAS | 투명 cylinder r=.01, length=.01 | 없음 | 중복 wheel mesh 0, steering collider 0 |
| Current | Base 동일 | Base 동일 | 36,470 / 19,714 |

Base/Current는 steering link와 wheel link가 동일 tire geometry를 따로 소유한다.
wheel만 spin하면 steering 쪽 tire가 같이 spin하지 않으며, 이 collider가 지면과 접촉하면
주행 결과를 왜곡할 수 있다. Self Collision OFF만으로 지면과의 접촉 문제는 해결되지 않는다.
LCAS의 **steering은 joint frame, tire는 wheel link 소유** 원칙을 채택할 근거가 있다.
투명 cylinder는 가시화 marker일 뿐 실제 knuckle geometry로 입증되지 않았다.

| Visual mesh | AgileX 원본 STL triangles | LCAS DAE triangles | 판단 |
|---|---:|---:|---|
| Chassis | 129,044 | 38,703 | triangle 수 약 70% 감소 |
| Front left | 36,470 | 10,940 | 약 70% 감소 |
| Front right | 19,714 | 5,914 | 약 70% 감소 |
| Rear left | 29,308 | 8,791 | 약 70% 감소 |
| Rear right | 36,686 | 11,005 | 약 70% 감소 |

LCAS는 DAE와 material을 사용하지만 chassis/wheel collision은 visual DAE를 그대로 공유한다.
따라서 “LCAS는 모든 collision을 단순 primitive로 바꿨다”는 해석은 틀리다.
triangle 수 감소와 bounds는 확인했지만 Hausdorff distance/실물 형상 정합은 unverified다.
Gazebo FR visual은 left STL을 Y=3.14 회전해 사용하고 collision은 원래 right STL을 사용한다.
이 visual/collision 차이도 형상 fidelity가 검증된 수정으로 자동 채택하지 않는다.

## 8. 실제 joint 좌표 기반 geometry와 wheel radius

| 모델 | FL xyz | FR xyz | RL xyz | RR xyz | 기준 frame |
|---|---|---|---|---|---|
| Base/Gazebo/Current | .37142 .29199 −.1955 | .37142 −.29201 −.1955 | −.28 .29249 −.19558 | −.28018 −.29251 −.19558 | base_link |
| LCAS | .37142 .2925 .17442 | .37142 −.2925 .17442 | −.28 .2925 .17442 | −.28 −.2925 .17442 | base_link, chassis +.37 누적 |

좌/우 wheelbase는 앞·뒤 wheel joint X 차, front/rear track은 좌·우 Y 차로 계산했다.
LCAS controller parameter .65를 URDF 측정값으로 사용하지 않았다.
LCAS의 .65142는 공식보다 1.42 mm 길고, .585 track은 공식보다 20 mm 좁다.
Base/Gazebo/Current front track은 21 mm 좁으며 우측 wheelbase는 1.60 mm 길다.
이는 joint-origin 기준 비교다. tire의 실제 접촉 중심, steering pivot offset,
타이어 두께/비대칭이 포함된 track 정의는 별도 확인이 필요하다.
LCAS의 frame +.37은 base를 지면 가까이 옮기지만 wheel bottom이 정확히 Z=0임을 뜻하지 않는다.

| Wheel radius 측정 | Base / Current | Gazebo | LCAS | Driver |
|---|---:|---:|---:|---:|
| FL mesh 최대 axle 거리 | .1641580 | .1641580 | .1643108 | .165 |
| FR mesh 최대 axle 거리 | .1641580 | .1641731 (회전된 left visual) | .1643901 | .165 |
| RL mesh 최대 axle 거리 | .1642746 | .1642746 | .1643970 | .165 |
| RR mesh 최대 axle 거리 | .1642746 | .1642746 | .1643506 | .165 |

단위는 m. local XY AABB 반폭도 JSON에 기록했으며 대체로 .164 m다.
최대 mesh 반경은 tyre loaded rolling radius의 실측이 아니다. driver .165와 차이가
약 0.6~0.8 mm라고 해서 mesh scaling을 자동 적용하지 않는다.

## 9. Mass, COM, inertia

| Link 역할 | Base | Gazebo | LCAS | Current |
|---|---:|---:|---:|---:|
| Chassis | 6.0294516954 | 21.0294516954 | 72 | 6.0294516954 |
| Left steering | 7.8526 | 7.8526 | .005 | 7.8526 |
| Right steering | 7.696 | 7.696 | .005 | 7.696 |
| Front left wheel | 7.7007 | 7.7007 | 7 | 7.7007 |
| Front right wheel | 7.5613 | 7.5613 | 7 | 7.5613 |
| Rear left wheel | 7.6692 | 7.6692 | 7 | 7.6692 |
| Rear right wheel | 7.7133 | 7.7133 | 7 | 7.7133 |
| front_steer_link 보조 | .0049179 | .0049179 | 없음 | .0049179 |
| rear_wheel_link 보조 | 0 | .0049179 | 없음 | 0 |
| Root 추가 | 없음 | 없음 | inertial 미지정 | 없음 |
| **총합 kg** | **52.2274695954** | **67.2323874954** | **100.01** | **52.2274695954** |

65/70 kg와 비교하면 Base −12.77/−17.77 kg, Gazebo +2.23/−2.77 kg,
LCAS +35.01/+30.01 kg다. root의 미지정 mass는 합계에 더하지 않았으며 importer가
자동 질량을 만들면 실행 시 mass는 달라질 수 있다.

LCAS chassis는 Base와 동일 COM과 inertia tensor를 두고 mass만 72로 바꿨다.
각 wheel은 7 kg이며 FL 계열 공통 COM/tensor를 반복한다.
steering link는 .005 kg인데 diag inertia=.1 kg·m²다. 회전반경 `sqrt(I/m)≈4.47 m`로
작은 marker의 실제 관성이라고 받아들이기 어렵다. 수치 안정화 의도인지도 근거가 없으므로 E로 분류한다.
Gazebo는 off-diagonal 항을 다수 0으로 만들고 body mass를 변경하지만 실측 검증은 없다.

**모든 link의 mass, COM xyz/rpy, inertia 6개 항을 생략 없이**
[상세표의 각 모델 inertia 표](hunter2_comparison_evidence/model_details.md)에 기록했다.
차이는 CAD 부품 집계, 중복 assembly 질량, 배터리/탑재물, simulation tuning에서 생길 수 있으나
이번 소스만으로 원인을 확정할 수 없다. LCAS 100.01 kg를 “payload 포함”이라고 단정하지 않는다.

## 10. Steering angle와 turning radius 일관성

이상적인 평면 no-slip Ackermann에서 L=.650, T=.605, 후륜축 중심 반경을 R로 정의한다.

```text
R = L / tan(delta_center)
delta_inner = atan(L / (R - T/2))
delta_outer = atan(L / (R + T/2))
cot(delta_outer) - cot(delta_inner) = T/L
```

| 입력/가정 | R (후륜축 중심,m) | Center rad | Inner rad | Outer rad | 외측 전륜 중심 반경(m) |
|---|---:|---:|---:|---:|---:|
| Driver center .461 | 1.308651 | .461000 | .573577 | .383467 | 1.737327 |
| Driver inner .58 | 1.294611 | .465311 | .580000 | .386513 | 1.724316 |
| Manual inner33° | 1.303412 | .462600 | .575959 | .384598 | 1.732471 |
| Manual 1.6 m를 후륜축 중심 반경으로 가정 | 1.600000 | .385883 | .464418 | .329222 | 2.010474 |
| Manual 1.6 m를 외측 전륜 중심 반경으로 가정 | 1.159519 | .510928 | .648883 | .418346 | 1.600000 |
| LCAS .461을 inner로만 가정 | 1.611151 | .383467 | .461000 | .327438 | 2.021029 |
| Gazebo .69를 inner로만 가정 | 1.090058 | .537693 | .690000 | .436709 | 1.536788 |

마지막 두 행은 limit 숫자의 의미를 비교한 가상 계산으로 해당 simulation의 실제 회전반경이 아니다.
33°와 .58 차이는 .23155°다. .461 center로 필요한 inner는 약32.8635°로 manual33° 이내다.
이 세 각도는 근접하지만 정확한 동일 parameter가 아니다.
개별 조향각을 .461로 제한하는 보수적 운용 자체는 가능하다. 다만 이를 실차 최대각이라고
해석하면 안 되며, 좌우 1:1 mimic의 Ackermann 불일치는 제한을 낮춰도 남는다.
반면 1.6 m는 어떤 지점 궤적·turning circle 측정 조건인지 확인되지 않았고,
위 두 흔한 정의로도 각도 제원과 정확히 일치하지 않는다. **반경1.6에 맞추기 위해
LCAS의 .461을 individual steering의 정답으로 역선택하면 안 된다.**

LCAS의 같은 좌우 각도 δ는 `cot δ − cot δ = 0`이므로 T/L>0 조건을 만족하지 않는다.
실제 LCAS L=.65142,T=.585,q=.461에서 좌측 바퀴가 요구하는 ICC Y=1.604009 m,
우측은 1.019009 m로 0.585 m 다르다(후륜축 X를 기준). 동일한 회전 중심이 없다.
따라서 마찰/slip을 허용해 돌아갈 수는 있어도 정밀 Ackermann 기구학의 증거는 아니다.

## 11. Drivetrain/control 및 최대 wheel speed

공식 CAN은 linear velocity와 **inner-wheel angle** command를 정의하고, motor feedback에는
steering/right-rear/left-rear를 지정한다. front steering position, rear traction,
front passive rolling이라는 목표 역할이 공식 자료와 더 일관된다.
motor 출력·감속비의 source 충돌과 실제 firmware 내부 속도 분배는 별도 미확인이다.

AgileX Gazebo의 `writeSim` 코드 확인:

- 좌/우 rear wheel에 `SetVelocity(0, rear_wheel_jnt_vel_cmd_)`.
- 좌/우 front wheel에도 동일 `SetVelocity`; passive rolling 구현이 아니다.
- 좌/우 steering에는 분배식 뒤 `SetPosition`.
- 해당 분기의 effort `SetForce` 코드는 주석 처리되어 있다.
- `enable_ackermann_link=true`이나 plugin geometry h=.60986,w=.47은 제원/URDF와 다르다.
- 식은 `atan2(2h tanδ, 2h ± (w/2)tanδ)`다. h=L,w=T로 해석하면 표준 분모
  `2L ± T tanδ`와 다르다. w 의미 재해석 없이는 그대로 채택 불가다.
- header는 INDEX_RIGHT=0, INDEX_LEFT=1인데 config는 `[left,right]` 순서다.
  δ=+.461일 때 실제 이름에 매핑된 left≈.425607, right≈.502273 rad로 계산된다.
  +Z 양의 좌회전에서 좌측 inner가 더 커야 하는 기하와 반대다. runtime 실행 전에도
  소스상의 불일치가 보이며, plugin 전체를 “공식이므로 정확”하다고 취급하면 안 된다.
- 상위 controller는 L=.650,r=.165를 사용해 plugin geometry와도 다르고 max linear 설정도 10 m/s다.

LCAS `ros2_control` command/state:

| Joint | Command | State | Coupling |
|---|---|---|---|
| front_left_steering_joint | position | position | 기준 |
| front_right_steering_joint | 직접 command 없음 | position | left와 multiplier=1 |
| rear_left_wheel_joint | velocity | velocity,position | 기준 |
| rear_right_wheel_joint | 직접 command 없음 | velocity,position | left와 multiplier=1 |
| front_left/right_wheel_joint | 없음 | velocity,position | 기본 모델에 mimic 없음 |

컨트롤러 이름은 ackermann_like_controller이지만 type은 **tricycle_controller/TricycleController**다.
traction joint를 rear_left, steering joint를 front_left로 연결한다.
공식 tricycle controller는 하나의 구동·조향 wheel 모델을 설명하므로, 이를 HUNTER의
정확한 좌우 Ackermann actuator 분배로 볼 수 없다.
[ROS2 controller 문서](https://control.ros.org/humble/doc/ros2_controllers/tricycle_controller/doc/userdoc.html),
[Gazebo ROS2 mimic 설명](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html).
front에 command interface가 없다는 것을 확인했으며 실제 Gazebo backend의 passive 상태는 runtime 검증이 남는다.
별도 `robot_core_real_hack.xacro`는 front wheel까지 rear_left에 mimic한다. 기본 launch에 쓰이지 않으므로 혼동 금지다.

`omega=v/r=1.5/.165=9.090909 rad/s`는 직진에서의 요구 속도다.

| 모델 | Rear URDF limit | 직진 1.5m/s 수용 | 판단 |
|---|---:|---|---|
| Base/Current | 1 rad/s | 아니오 (nominal .165m/s) | 주행 전에 제한 정책 수정 필요 |
| Gazebo | 미지정 | limit만으로 판정 불가 | SetVelocity direct 동작과 importer default를 구분 |
| LCAS | 10 rad/s | 예 (nominal1.65m/s) | 정확한 hardware max로 입증되지 않음 |

후륜축 중심 v=1.5m/s, centerδ=.461을 유지하는 이상적 선회에서는 outer≈11.19 rad/s,
inner≈6.99 rad/s가 필요하다. LCAS 10 rad/s는 이 조건의 외측 wheel에 부족하다.
outer/inner speed ratio≈1.6013인데 LCAS rear mimic은1이다. 선회 중 전체 속도를 줄일지,
개별 wheel limit를 달리 둘지 controller 정책을 정해야 한다.
front wheel은 모든 기본 비교 모델에서 URDF velocity/effort limit가 미지정이다.

## 12. LCAS 차이 분류 A–E와 채택 여부

A=명백한 bug fix, B=공식 제원 수용, C=simulation-specific, D=구조/표현 정리,
E=근거 부족 가정. 상세 diff는 각 변경 필드에 primary class 한 개와 이유를 부여한다.
하나의 설계 변경에 여러 의미가 있으면 아래 표처럼 분해해서 판단한다.

| LCAS 변경 | 분류 | 평가 / 프로젝트 적용 |
|---|---|---|
| S local ±Z→+Y | A | 수직 yaw가 되므로 우선 채택 후보 |
| zero range 해제 | A | 실제 조향 자유도를 허용하는 개선 |
| 해제 범위를 individual±.461로 선택 | E | center/inner 혼동 또는 bicycle 근사. 채택하지 않음 |
| steering wheel mesh/collider 제거 | A | 중복 ownership 문제 해결, 우선 채택 후보 |
| 투명 marker cylinder | C | 시각화용. 실제 geometry로 채택할 필요 없음 |
| rear limit1→10 rad/s | B | 공식 최대 직진 속도 수용을 개선. 정확한10의 근거는 unverified |
| 전륜은 state-only, 후륜 traction 분리 | B | official rear motor mapping과 역할상 일관됨 |
| ros2_control/GazeboSystem/plugin | C | Isaac에 복사 금지, 해당 runtime adapter로 구현 |
| rear/steering 1:1 mimic | E | 정확한 Ackermann 조건과 불일치, 복사 금지 |
| TricycleController 선택 | C | Gazebo/ROS2 제어 근사; 기구학 타당성에는 E 위험도 있음 |
| pi/2 사용·이름 정리·macro화 | D | 가독성/일관성 개선, 꼭 rename할 필요 없음 |
| 9→8 links, 집계용 보조 link 제거 | D | 유효 tree 단순화, 보조 geometry 의미는 확인 필요 |
| root/chassis 분리 Z+.37 | E | 유용할 수 있으나 official frame 정의로 입증 안 됨 |
| 좌우 대칭화 .2925/−.28 | E | symmetry 개선이나 nominal .605/.650와 여전히 다름 |
| 좌 wheel axis를 −Z로 통일 | D | 부호 통일이지 원래 transverse axle의 오류 수정은 아님 |
| STL→DAE, triangle 수 감소·색상 | D | 구조/표현 변경, geometry fidelity는 검증 필요 |
| chassis72kg·wheel7kg·S .005kg | E | 총100.01kg, official65/70와 불일치 |
| 기존 chassis inertia 유지·S diag .1 | E | 물리 calibration 근거 없음 |
| damping .2, friction15, effort10 | E | 실행 안정화 설정일 수 있으나 실물 근거 없음 |
| Gazebo mu/kp/kd/fdir1 | C | PhysX와 동일 값/의미라고 가정 금지 |

**명백히 개선한 것은 수직 조향과 중복 wheel 소유 제거다.** LCAS README의 정확한 kinematics
주장을 무비판적으로 받아들이지 않았으며 코드·기하 검증을 우선했다.

## 13. Current 모델 patch proposal — 적용하지 않음

작업 위치는 vendor가 아니라 파생 생성기 `scripts/prepare_hunter2_asset.py`와 그 검증이다.
현재 joint 이름을 유지해도 LCAS의 구조적 개선을 반영할 수 있다.

1. `front_steer_left_joint`의 axis `0 0 1` → `0 1 0`.
2. `front_steer_right_joint`의 axis `0 0 -1` → `0 1 0`.
   origin xyz/rpy는 유지한다. [axis-only 검토용 patch](hunter2_comparison_evidence/proposed_steering_axis.patch)는 생성만 했다.
3. 두 steering link에서 wheel 포함 visual/collision을 분리한다. 우선 collision 제거와
   순수 knuckle/무형상 frame 설계를 검토한다. 실제 knuckle mesh가 필요하면 근거 있는 별도 geometry가 필요하다.
   mesh에서 wheel triangle를 빼는 것만으로 남은 assembly가 올바른 부품이라고 단정하지 않는다.
4. 좌우 S는 독립 revolute로 유지하며 1:1 mimic을 추가하지 않는다. rear W도 독립으로 유지한다.
5. angle 정책: manual hard limit를 따를 경우 ±33°(±.5759586532 rad)를 채택하고,
   driver .58은 반올림된 command reference로 기록한다. center .461은 별도 상한으로 유지한다.
   현재 .58을 그대로 쓸 경우 manual보다 .23155° 큰 이유/허용오차를 명시해야 한다.
6. rear velocity는 직진 최소9.0909rad/s와 선회 속도 정책을 만족하도록 정한다.
   LCAS10을 hardware truth로 바로 복사하지 않는다.
7. 기준 wheelbase/track, COM, inertia 변경은 별도 geometry/dynamics 검증 단계로 둔다.
8. 새 검증에는 steering heading 변화, collider ownership, 각도 정의, wheel speed 실현 가능성,
   좌우 Ackermann common ICC 조건을 추가한다. 기존 byte 제한 검사는 새 허용 변경 목록에 맞춰 갱신해야 한다.

## 14. 수정 우선순위

### MUST FIX BEFORE ISAAC IMPORT

- 주행용 입력 기준: 두 steering axis를 수직 yaw가 되도록 교정하고 수학 검증한다.
- zero range는 Current에서 이미 해제했다. Base를 재사용하면 해제를 빠뜨리지 않는다.
- LCAS의1:1 steering/rear mimic을 도입하지 않고 독립 DOF/각도 의미를 명시한다.
- manual33°와 driver.58의 limit 정책을 선택·기록한 뒤 source를 확정한다.

이 목록은 **주행용 source 승인 기준**이다. 오류를 관찰하기 위한 진단 import 자체가 기술적으로
불가능하다는 의미는 아니다. Current tree 연결은 이미 유효하므로 근거 없는 topology 재작성은 필요 없다.

### SHOULD FIX BEFORE DRIVING TEST

- 중복 wheel visual/collision ownership 해결. 특히 steering tire collider 제거/분리.
- rear1rad/s 제한과 목표 속도 불일치 해결; wheel 부호 mapping과 front passive drive 확인.
- 실제 모델 geometry와 controller nominal geometry를 명시적으로 구분하고 사용할 값을 정한다.
- zero-mass fixed link와 빈 root 등 importer 처리, 최종 collider/mass 속성을 확인한다.

### DEFER UNTIL ISAAC PHYSICS VALIDATION

- mass/inertia의 실차 calibration, friction, restitution/contact, drive gain, torque/rate 실측.
- collision 근사 정밀도·성능, fixed-link 병합 및 density/inertia 자동 계산 정책.
- 실물 최소회전반경 측정 정의와 속도/하중/tire deformation에 따른 turning/slip 재검증.

## 15. 아직 알 수 없는 부분과 TODO

- manual1.6m의 정확한 측정 지점/방법: **unverified**.
- 영문/중문 steering motor 출력 및 manual/driver 감속비 차이: **unverified revision**.
- 실제 unit의 배터리/부가장치에 따른 mass 및 link별 tensor/COM: **unverified**.
- LCAS DAE가 원본 CAD를 얼마나 정확히 보존하는지, 제거된 보조 형상의 실물 의미: **unverified**.
- 공식 nominal track과 URDF joint-origin 간격 차이의 기구학적 이유: **unverified**.
- 각 ROS/Gazebo backend의 mimic·limit·friction 실제 적용: 소스 비교만 했으며 **runtime unverified**.
- Isaac importer의 상대경로, zero inertia, drive default, collision cooking, 좌표 보존 및 주행:
  **requires Isaac Sim validation**. 버전 미정, 이번에는 install/import/USD 생성을 하지 않았다.

## 16. 최종 질문 Q1–Q6

**Q1. Base와 LCAS 중 primary structural reference는?**

LCAS의 **chassis→독립 steering frame→wheel** 역할 및 collider 소유 구조가 더 적절하다.
다만 mimic·제어·mass·limit를 제거/재검토한 구조 참고라는 뜻이다. 구현의 원본 출처와 visual mesh는
AgileX를 유지하며 현재 파생 URDF에 필요한 개선만 적용할 수 있다. LCAS 전체를 새로운 source of truth로 삼지 않는다.

**Q2. LCAS의 명백한 개선은?**

수직 조향축, zero range 해제, steering의 중복 tire mesh/collider 제거다.
rear10rad/s는 공식 직진 속도를 수용하며, front state-only/rear command 구분은 공식 구동계에 더 가깝다.
다만 angle 값과 rate10의 정확한 hardware 근거는 따로 검토해야 한다.

**Q3. 그대로 복사하면 안 되는 Gazebo/ROS2 설정은?**

GazeboSystem/plugin, ros2_control 인터페이스 구문, TricycleController, Gazebo 접촉 파라미터다.
1:1 mimic은 표준 URDF 요소이기도 하지만 여기서는 기구학적으로 부적절하므로 이식하지 않는다.
Isaac의 drive/controller로 역할을 재구현한다.

**Q4. Current에서 반드시 수정할 부분은?**

정상 yaw 조향을 위해 두 steering axis가 필수다. 주행 승인 전에는 중복 wheel collider와
rear velocity 상한을 해결해야 한다. manual/driver angle 정책을 명확히 하고 mass/inertia는 검증 없이 바꾸지 않는다.

**Q5. Official spec와 simulation 중 무엇을 어디에 따르나?**

wheelbase/track, 최고속도, inner steer, vehicle mass/payload, motor arrangement는 official 자료가 우선이다.
joint frame·분리 topology·collider ownership은 기하 검증을 통과한 Gazebo/LCAS 구현을 참고한다.
visual은 AgileX CAD, driver radius·center angle은 역할을 구분해 사용한다.
drive/friction/contact는 선택한 Isaac 버전에서 검증한다.

**Q6. 현재 source를 freeze해도 되나?**

**정상 주행용 입력으로는 안 된다.** 조향축 오류가 확정됐고 collision/velocity 이슈가 남았다.
지금 commit/hash를 audit baseline으로 보존한 뒤 제안된 수정과 검증을 완료하고,
Isaac import/기본 주행 검증의 통과 범위를 명시해 단계적으로 freeze해야 한다.
