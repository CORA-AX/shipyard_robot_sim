# HUNTER 2.0：Isaac Sim 입력 모델 적합성 검토

> 이 문서는 교정 전 조사 기록입니다. 아래의 Current/미수정 판정은 당시 baseline에 해당합니다.
> 2026-10-06 실제 교정 결과는 [소스 교정 보고서](hunter2_source_correction.md)를 참고하세요.

검토일: 2026-10-02. 대상은 현재 `assets/hunter2/source/hunter2_sim.urdf`.
**판정: 원본 보존/경로 정리용 source로는 유효하지만, 주행용 USD의 완성된 입력으로는 부적합하다.**
조향축과 중복 wheel collision 해결이 우선이다. 이번 검토는 모델 수정이나 USD 생성 없이 수행했다.

## 검증 방법과 한계

1. 원본, 파생 URDF, 공식 Gazebo xacro를 XML로 읽어 좌표·joint·mesh를 비교했다.
2. 각 조향 joint의 −0.58, −0.3, 0, +0.3, +0.58 rad에서 wheel axle 방향을 계산했다.
3. STL의 삼각형 vertex를 1 µm 단위로 반올림해 순서/normal을 제외한 동일 triangle을 비교했다.
4. NVIDIA 5.1.0 importer 문서, API, OpenUSD mass 규칙 및 AgileX driver/plugin과 대조했다.
5. 기존 구조 validator도 다시 실행했다. 문법 통과와 주행 적합성을 별개로 판정했다.

재현:

```bash
python3 scripts/review_hunter2_kinematics.py
python3 scripts/validate_hunter2_asset.py
```

[계산 JSON](hunter2_kinematic_review.json)에 대상 URDF/xacro SHA-256과 수치를 저장했다.
첫 스크립트는 표준 라이브러리만 사용하고 JSON을 stdout으로 출력한다. asset에 쓰지 않는다.
회전 계산은 Rodrigues 공식 및 URDF의 Rz Ry Rx 순서를 사용했다.
기존 validator의 회전 구현을 재사용하지 않아 별도 구현으로 비교했다.
원본의 nonzero-angle probe는 limit를 무시한 가상 계산이며 원본이 그 각도를 허용한다는 뜻이 아니다.

Isaac Sim을 실행하거나 importer를 통해 USD를 생성하지 않았다. 실제 importer 성공,
PhysX 접촉·안정성·drive 동작·주행은 아직 검증하지 않았다. 팀의 기준 버전도 선택하지 않았다.
기존 로컬 설치 `~/isaacsim/VERSION`은 `6.0.0-rc.59+release.41464.5f2772bc.gl`로 확인했으며,
일부 코드는 읽기만 했다. 이를 팀 기준 버전이나 5.1.0 동작의 증명으로 사용하지 않는다.

## 현재 변경에 대한 판정

| 항목 | 판정 | 근거 / 필요한 조치 |
|---|---|---|
| vendor와 snapshot 분리, mesh 무변경 | 적절 | 현재 원본 추적과 재현성 유지 |
| `package://...` → `meshes/...` | portable source 설계로 적절, importer 검증 필요 | 파일 실재/상대경로 검사는 통과; 선택 버전에서 경로 resolution 확인 |
| 조향 limit ±0.58 | 공식 driver 근거 있음 | V2 상수이며 실측 mechanical stop 검증은 아님; ±0.461 중심각과 분리 |
| 기존 조향 axis 보존 | 원본 보존에는 맞으나 주행 요구에는 실패 | 현재 모델로는 바퀴 진행 방향이 조향되지 않음 |
| 기존 steering visual/collision 보존 | 보존은 맞으나 물리용 모델에는 추가 작업 필요 | 앞바퀴 표면 triangle 전체가 steering mesh와 중복 |
| 질량/관성 보존 | 임의 보정 방지는 적절 | zero mass/inertia와 52.23 kg 합계는 physics 승인 상태가 아님 |
| rear velocity=1 유지 | 원본 기록에는 맞으나 1.5 m/s 목표와 충돌 | nominal r=0.165 m에서 0.165 m/s; 직진 1.5 m/s에는 약 9.09 rad/s 필요 |
| Gazebo plugin 제외 | 적절 | 해당 plugin의 직접 joint 조작은 USD 변환으로 이식되지 않음 |
| 기존 구조 검사 PASS | 검사 범위 내에서는 정확 | Ackermann 기능/물리 적합성 판정으로 사용할 수 없음 |

따라서 이전의 “source 준비 완료”는 **보존 및 최소 수정 산출물이 완성됨**을 뜻한다.
“즉시 정상 주행용 USD로 변환 가능한 모델 완성”으로 읽으면 과도한 표현이다.

## 1. 조향축: 수정 필요가 수학적으로 확인됨

현재 두 steering joint는 origin rpy가 `1.5708 0 0`이다.
왼쪽 axis는 local +Z, 오른쪽은 local −Z로, base 기준 약 −Y/+Y다.
두 축은 자식 wheel 회전축과 각각 평행하다. 조향 joint를 움직여도 wheel axle 방향은
변하지 않으므로 바퀴 평면과 지면의 교선(굴러가는 방향)이 바뀌지 않는다.

공식 Gazebo 모델은 두 steering joint에 local +Y (`0 1 0`)를 사용한다.
같은 origin 회전을 거치면 base 기준 약 +Z가 된다.

| 입력 steering position | 현재 좌/우 wheel heading | Gazebo 좌/우 wheel heading |
|---:|---:|---:|
| −0.58 rad | 0 / 0 | 약 −0.58 / −0.58 |
| −0.3 rad | 0 / 0 | 약 −0.3 / −0.3 |
| 0 rad | 0 / 0 | 0 / 0 |
| +0.3 rad | 0 / 0 | 약 +0.3 / +0.3 |
| +0.58 rad | 0 / 0 | 약 +0.58 / +0.58 |

각 joint에 같은 각도를 넣는 것은 축 검사용이다. 좌우 Ackermann 각도 분배를 검증한 것이 아니다.
두 steering axis를 `0 1 0`으로 바꾸는 것은 근거 있는 수정 후보다. origin rpy를 0으로
만드는 방식은 mesh와 wheel frame까지 바꾸므로 같은 수정이 아니다.
수치 검증 뒤에도 mesh 이동·간섭·실제 actuator 부호는 렌더링과 physics에서 확인해야 한다.

NVIDIA importer는 joint의 axis/origin/limit 정보를 읽어 joint 구조를 변환한다.
차량 조향 의미를 추론하여 축을 자동으로 고친다고 가정할 수 없다.
참고: [NVIDIA importer 구조 설명](https://docs.omniverse.nvidia.com/kit/docs/omniverse-urdf-importer/latest/source/extensions/omni.importer.urdf/docs/Overview.html),
[5.1.0 UrdfJoint API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/api/structisaacsim_1_1asset_1_1importer_1_1urdf_1_1_urdf_joint.html).
앞의 구조 설명은 구버전 문서이므로 mimic 지원 등의 항목을 최신 버전에 일반화하지 않는다.

## 2. 중복 wheel geometry: 추정에서 정량 확인으로 갱신

현재 모델에서 wheel joint origin과 visual/collision origin이 모두 0이므로,
wheel position=0일 때 steering link와 wheel link의 mesh 좌표는 같은 frame에 놓인다.

| 측면 | steering mesh triangle | wheel mesh triangle | 일치 triangle (좌표 반올림 1 µm) |
|---|---:|---:|---:|
| 왼쪽 | 53,442 | 36,470 | 36,470 |
| 오른쪽 | 30,924 | 19,714 | 19,714 |

각 wheel mesh의 표면 전체가 steering mesh에도 포함되어 있다. 파일 자체가 같다는 뜻은 아니다.
현재 visual과 collision은 동일 STL을 사용하므로 양쪽에 중복이 존재한다.
바퀴가 회전하면 wheel link는 회전하지만 steering mesh에 포함된 바퀴 표면은 그 회전을 따르지 않는다.
지면 접촉에 steering 쪽 tire 형상까지 관여할 가능성이 있어 정상 wheel contact를 보장할 수 없다.
Self Collision OFF는 로봇 내부 충돌 설정이며, steering collider와 지면의 접촉을 없애는 해결책은 아니다.

공식 Gazebo xacro는 steering visual/collision을 작은 cylinder로 대체한다.
이는 해당 mesh를 그대로 쓰지 않았다는 근거지만, 이 작은 cylinder가 실물 steering 부품의
정확한 형상이라는 증거는 아니다. 후속 단계에서 steering link는 frame/물리 연결 역할을
유지하면서 중복 wheel visual/collision을 분리하는 방안을 검토해야 한다.
mass를 즉시 삭제하거나 임의의 값으로 바꾸지 않는다.

NVIDIA는 collision 근사와 겹친 collider의 안정성을 별도 검토하도록 안내한다.
[5.1.0 importer의 Colliders 설명](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html).

## 3. Gazebo 모델을 그대로 가져오면 안 되는 이유

`.xacro` 확장자가 물리를 자동 보정하는 것은 아니다. 해당 파일에는 수정된 robot XML이
따로 작성되어 있고, launch가 그 모델을 선택한다. 현재 파일에는 xacro macro/include가 없다.

Gazebo 모델은 축 외에도 steering limit ±0.69, effort/velocity 0, mesh 대체,
질량·관성, wheel axis 부호, 가동 보조 joint, friction 및 plugin 설정이 다르다.
총 mass는 67.2323874954 kg이고 현재 모델은 52.2274695954 kg이다.
Gazebo 값이 모두 실제 차량의 정답이라고 판단할 수 없다.

더구나 plugin `writeSim`은 rear wheel뿐 아니라 **front wheel에도 SetVelocity**를 호출하고,
steering에는 **SetPosition**을 직접 호출한다. 물리 drive로 토크를 만들어 수렴시키는 설정과
동일하지 않으며, front wheel passive 구동의 공식 증거로 삼을 수도 없다.
Gazebo에서 동작한다는 사실만으로 PhysX drive 적합성 또는 실물 drivetrain을 증명할 수 없다.

고정 commit의 근거:

- [Gazebo 모델](https://github.com/agilexrobotics/ugv_gazebo_sim/blob/27633a956c845903ee630538afeb17fe70afdd84/hunter/hunter2_base/urdf/hunter2_base_gazebo.xacro)
- [모델을 선택하는 launch](https://github.com/agilexrobotics/ugv_gazebo_sim/blob/27633a956c845903ee630538afeb17fe70afdd84/hunter/hunter2_gazebo/launch/hunter2_gazebo.launch)
- [plugin writeSim](https://github.com/agilexrobotics/ugv_gazebo_sim/blob/27633a956c845903ee630538afeb17fe70afdd84/hunter/steer_bot_hardware_gazebo/src/steer_bot_hardware_gazebo.cpp)

이 세 파일은 로컬 공식 clone에서 직접 읽었다.

## 4. limit, drive, mass와 경로

공식 [HunterV2Params](https://github.com/agilexrobotics/hunter_ros2/blob/7bb146dd40fcea757b3516292319aa14ace41577/hunter_base/include/hunter_base/hunter_params.hpp)의
L=0.650, T=0.605, center=0.461로 계산한 inner angle은 0.57357688 rad로 0.58 안에 든다.
따라서 현재 limit 선택에는 근거가 있다. Gazebo의 0.69를 따라야 한다는 결론은 나오지 않는다.

rear wheel 1 rad/s는 nominal 0.165 m/s에 해당한다. 실제 목표 1.5 m/s를 위해서는
직진 때 약 9.09 rad/s가 필요하고, 선회 바깥쪽 wheel의 상한 정책은 별도로 정해야 한다.
NVIDIA 문서에는 effort와 velocity를 물리 drive/velocity 제한으로 변환하는 설명이 있다.
[현재 importer 변환 표](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/ext_isaacsim_asset_importer_urdf.html).
이 페이지는 latest라 향후 달라질 수 있다. 로컬 6.0 RC 코드에서도 rad/s 제한을
degree/s로 변환하는 코드를 읽었지만, 아직 선택하지 않은 모든 버전에 같은 동작을 보증하지 않는다.

NVIDIA 5.1.0 API에는 fixed base, joint 병합, inertia import, 기본 drive 등의 옵션이 있다.
기본값을 그대로 사용하는 대신 mobile base와 각 joint의 position/velocity/none 목적을 정해야 한다.
[5.1.0 ImportConfig](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/api/structisaacsim_1_1asset_1_1importer_1_1urdf_1_1_import_config.html).

`rear_wheel_link`의 0 mass/0 inertia가 반드시 import 실패를 뜻하지는 않는다.
USD에서는 0 mass/inertia가 명시 값으로 사용되지 않는 경우가 있고 importer의 재계산/병합 정책도
영향을 준다. 최종 USD와 실행 시 유효 mass/inertia를 점검해야 한다.
[OpenUSD MassAPI](https://openusd.org/dev/api/class_usd_physics_mass_a_p_i.html).
따라서 Merge Fixed Joints OFF는 진단을 위한 시작 후보이지 이 모델의 최종 최적 설정이 아니다.

상대경로는 현재 파일 묶음에서 유효하다. 로컬 6.0 RC의 `urdf_utils.py`에는 임시 staging 시
relative mesh URI를 기준 디렉터리에 대해 절대경로로 바꾸는 함수도 있다. 공식 구현이 상대경로를
고려한다는 참고 근거다. 해당 함수를 실행하거나 canonical source를 절대경로로 바꾸지 않았다.
최종 importer의 STL 변환과 상대경로 해석은 버전 확정 후 검증한다.

## 권장 후속 수정 순서 (이번 검토에서는 적용하지 않음)

1. 파생 source 생성기에 두 steering axis local +Y 수정과 비영점 heading 검사를 추가한다.
2. 중복 steering/wheel visual·collision을 분리하는 모델링 방안을 확정한다.
3. 실제 drivetrain 근거와 joint sign mapping을 확정하고, 속도 상한 정책을 정한다.
4. 질량·관성·fixed link 병합/자동 계산 정책을 정한다.
5. Isaac 버전 확정 후 진단 import → 결과 USD 속성 점검 → 접지/drive/저속 조향 검증을 한다.

현재 파일·원본은 그대로 보존했다. 이번에 생성한 것은 검토 스크립트, 계산 JSON, 이 문서다.
