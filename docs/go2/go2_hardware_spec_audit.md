# Unitree Go2 하드웨어 제원 및 프로젝트 모델 정리

조사 시작: 2026-10-08, 정리: 2026-10-09 (Asia/Seoul). 대상: `~/shipyard_robot_sim`의 Go2. 기존 모델·설정 변경, 패키지 설치, 시뮬레이션 실행 없이 소스와 USD를 읽어 정리했다.

## 1. 핵심 결론

현재 프로젝트의 Go2는 **NVIDIA Go2 USD + Isaac Lab DC 모터 모델 + 평지 보행 정책**이다. 실물 AIR/PRO/X/EDU 중 어느 구성과 일치하는지 명시되어 있지 않다. 원격 파일 이름만으로 EDU라고 확정하지 않는다.

- USD 질량 합계는 **15.019 kg**이며 공식 약 15 kg에 가깝다.
- 네 다리마다 hip/thigh/calf 3개씩, 총 **12개 능동 회전관절**을 갖는다.
- 원본 USD 토크 상한은 hip/thigh **23.7 N·m**, calf **45.43 N·m**이다. 프로젝트 정책 실행 설정은 **모든 관절 23.5 N·m**의 DC 모터 모델이다.
- `velocity_limit=30 rad/s`는 DC 모터의 토크–속도 곡선 설정이다. 물리 엔진의 모든 관절 속도 상한이 30으로 바뀐다는 뜻이 아니다. USD에는 hip/thigh 약 30.1, calf 약 15.7 rad/s가 저장되어 있다.
- 공식 관절 범위와 USD hip/thigh 범위에 차이가 있다. 좌표계·각도 영점·하드웨어 버전 확인 없이 실차 제어에 그대로 사용하면 안 된다.
- 프로젝트는 평지 속도 명령을 받아 보행하는 구성이다. LiDAR 기반 지도 작성, 장애물 회피, waypoint 계획·추종은 현재 정책 자체의 기능이 아니다.

**[공식]** 제조사 공개값, **[USD]** 원격 파일 실값, **[설정]** 프로젝트 코드, **[계산]** 수치로부터 유도, **[기록]** 기존 실행 기록, **[미확인]** 이번에 검증할 수 없는 항목을 구분한다. 실행 설정은 소스 해석이며 이번 새 실행으로 확인한 결과가 아니다.

## 2. 실물 Go2 하드웨어 제원

아래는 조사 시점 [Unitree 공식 Go2 제품표](https://www.unitree.com/go2/)의 요약이다. 구성·지역·시점에 따라 달라지며 특정 구매 장비의 사양서는 별도 확인해야 한다.

| 항목 [공식] | 값 |
| --- | --- |
| 기립 외형 L×W×H | 0.70 × 0.31 × 0.40 m |
| 웅크린 외형 | 0.76 × 0.31 × 0.20 m |
| 배터리 포함 질량 | 약 15 kg |
| 전압 / 최대 작동 전력 | 28–33.6 V / 약 3000 W |
| 구동 관절 | 12개 |
| 최대 관절 토크 | 약 45 N·m: 가장 큰 모터의 최대값; 전 관절 공통 정격 아님 |
| 각도 범위 body / thigh / shank | ±0.837758 / [−3.490659, 1.570796] / [−2.722714, −0.837758] rad |

| 항목 [공식] | AIR | PRO | X | EDU |
| --- | --- | --- | --- | --- |
| 속도 (m/s) | 0–2.5 | 0–3.5 | 0–3.7, 최대 약 5 | 0–3.7, 최대 약 5 |
| 적재 (kg) | 약 7, 최대 약 10 | 약 8, 최대 약 10 | 약 8, 최대 약 12 | 약 8, 최대 약 12 |
| 단차 / 경사 | 약 0.15 m / 30° | 약 0.16 m / 40° | 약 0.16 m / 40° | 약 0.16 m / 40° |
| 배터리 / 지속시간 | 8 Ah / 약 1–2 h | 8 Ah / 약 1–2 h | 8 Ah / 약 1–2 h | 15 Ah / 약 2–4 h |

공식 5 m/s는 실험실 조건의 최대값이다. 전체 관절에 45 N·m를 지속 적용할 수 있다는 뜻도 아니다. 공식 페이지에는 4D LiDAR와 카메라가 소개되어 있으나 실물 기능이 시뮬레이션 모델·정책에 자동 포함되지는 않는다. 제조사 표는 현재 L2를 소개하므로 과거 L1 구성과 혼동하지 않는다. 위 표의 숫자는 동일 [공식 제품 페이지](https://www.unitree.com/go2/)에서 확인했다.

연속 관절 토크, 관절별 실제 토크–속도 곡선, 감속비·효율, 링크별 실측 관성/CoM, 발바닥 마찰계수는 해당 제품 페이지로 확정할 수 없다. 아래 USD 값은 이런 미공개 하드웨어 수치를 대신 입증하지 않는다.

## 3. 프로젝트 파일과 출처

| 파일 | 역할 |
| --- | --- |
| `assets/go2/config/robot_cfg.py` | spawn 설정, 초기 자세, 액추에이터 |
| `assets/go2/config/sources.lock.json` | Isaac Lab commit, NVIDIA 모델 URL, 정책 SHA-256 |
| `scenarios/flat/go2_env_cfg.py` | 관측·액션·시간 간격·평지 마찰·리셋 |
| `scenarios/shipyard/go2_env_cfg.py` | 평지 정책 환경과 조선소 공통 장면 결합 |
| `scenarios/common/scene_cfg.py` | 공통 지면 Physics Material |
| `control/go2/policy.py` | 관측 48 → action 12의 Actor 및 관절 순서 |
| `control/go2/simulation.py` | viewer 내부 물리·관측 처리 |
| `docs/go2_install_evidence/base_height_check.json` | 과거 자세 높이 비교 기록; 이번 재실행 아님 |

Isaac Lab 고정 기록: `v2.2.1`, commit `0f00ca2b4b2d54d5f90006a92abb1b00a72b2f20`. [upstream 저장소](https://github.com/isaac-sim/IsaacLab/tree/0f00ca2b4b2d54d5f90006a92abb1b00a72b2f20). 현재 실제 로딩 설정은 프로젝트 소유 `robot_cfg.py`이므로 upstream 기본값만 보고 판단하지 않았다.

프로젝트에는 Go2 원본 URDF를 로딩하는 구성이 없다. 다음 [NVIDIA Go2 USD](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/IsaacLab/Robots/Unitree/Go2/go2.usd)와 [참조 mesh USD](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/IsaacLab/Robots/Unitree/Go2/Props/instanceable_meshes.usd)를 사용한다. 조사 중 `/tmp`로 내려받아 기존 프로젝트 캐시와 SHA-256이 각각 일치함을 확인했다. 기존 캐시/Asset은 바꾸지 않았다.

`pxr`로 root와 참조 메시를 오프라인 composition하여 검사했다. Kit나 PhysX stepping은 실행하지 않았다. texture 렌더링은 검사하지 않았다. 아래 USD 값은 runtime override 전 파일 값이다.

| 감사 파일 | SHA-256 |
| --- | --- |
| go2.usd | ba171c972b987d8c8fb7157ccad2ba9c0c1fed105755d2d8af46bef96cc11c6d |
| Props/instanceable_meshes.usd | 2902646d0f4c13c9ecae3ac9046e32c7d18902eef9679de9829f642a15c93fb5 |

## 4. 기구 구조와 USD 관절

USD defaultPrim은 `/go2_description`, 단위는 m, Z-up이다. Articulation Root는 `/go2_description/base`다. 파일에는 질량을 가진 body 19개, revolute joint 12개, fixed joint 6개가 있다. 물리 엔진 초기화 시 fixed-link 처리 결과를 별도 측정한 것은 아니다.

각 다리의 체인은 `base → hip → thigh → calf → foot`이다. foot은 fixed link이며 발을 구동하는 별도 모터는 없다. FL/FR은 앞 왼쪽/오른쪽, RL/RR은 뒤 왼쪽/오른쪽이다.

| 항목 [USD/계산] | 값 | USD 위치 |
| --- | --- | --- |
| 전후 hip 장착점 | X=±0.1934 m | `base/*_hip_joint.physics:localPos0` |
| 좌우 hip 장착점 | Y=±0.0465 m | 동일 |
| 전후 장착 간격 | 0.3868 m | 위 X 차이; 차량 wheelbase와 다름 |
| 좌우 hip 장착 간격 | 0.093 m | 위 Y 차이; 발 접지 폭과 다름 |
| hip → thigh 측면 offset | 좌우 ±0.0955 m | `*_hip/*_thigh_joint.physics:localPos0` |
| thigh → calf / calf → foot 길이 | 각 0.213 m | 각 joint localPos0 Z=−0.213 |
| 발 collision | 반지름 0.022 m sphere | `*_foot/collisions/mesh_0` |
| 바퀴/조향 구조 | 없음 | 4족 로봇; Ackermann 기구학 적용 대상 아님 |

hip의 관절축은 parent 기준 X다. thigh/calf의 USD `physics:axis`도 X로 저장되지만 joint localRot의 Z 90° 회전과 조합하면 parent 기준 Y축이다. axis 토큰만 읽으면 실제 다리 회전방향을 오해할 수 있다.

아래 범위는 raw USD degree를 rad로 변환한 값이다. 경로는 hip=`/go2_description/base/<leg>_hip_joint`, thigh=`/<leg>_hip/<leg>_thigh_joint`, calf=`/<leg>_thigh/<leg>_calf_joint`이며 후자는 `/go2_description` 아래다. 속성은 `physics:lowerLimit/upperLimit`, `physxJoint:maxJointVelocity`, `drive:angular:physics:maxForce`다.

| joint | USD hard range (rad) | soft factor 0.9 결과 (rad) | USD 속도 (rad/s) | USD torque cap (N·m) |
| --- | --- | --- | --- | --- |
| FL_hip_joint | -1.047200 … 1.047200 | -0.942480 … 0.942480 | 30.100 | 23.70 |
| FR_hip_joint | -1.047200 … 1.047200 | -0.942480 … 0.942480 | 30.100 | 23.70 |
| RL_hip_joint | -1.047200 … 1.047200 | -0.942480 … 0.942480 | 30.100 | 23.70 |
| RR_hip_joint | -1.047200 … 1.047200 | -0.942480 … 0.942480 | 30.100 | 23.70 |
| FL_thigh_joint | -1.570800 … 3.490700 | -1.317725 … 3.237625 | 30.100 | 23.70 |
| FL_calf_joint | -2.722700 … -0.837760 | -2.628453 … -0.932007 | 15.700 | 45.43 |
| FR_thigh_joint | -1.570800 … 3.490700 | -1.317725 … 3.237625 | 30.100 | 23.70 |
| FR_calf_joint | -2.722700 … -0.837760 | -2.628453 … -0.932007 | 15.700 | 45.43 |
| RL_thigh_joint | -0.523600 … 4.537900 | -0.270525 … 4.284825 | 30.100 | 23.70 |
| RL_calf_joint | -2.722700 … -0.837760 | -2.628453 … -0.932007 | 15.700 | 45.43 |
| RR_thigh_joint | -0.523600 … 4.537900 | -0.270525 … 4.284825 | 30.100 | 23.70 |
| RR_calf_joint | -2.722700 … -0.837760 | -2.628453 … -0.932007 | 15.700 | 45.43 |

soft factor 0.9는 범위 **중앙을 기준으로 폭을 90%로 줄인 내부 soft limit**이다. 모든 각도에 0.9를 곱하는 방식도 아니고, PhysX hard stop이 자동으로 그 값으로 바뀌는 것도 아니다. 프로젝트 `JointPositionActionCfg`에는 별도 명시적 clip이 없으므로 soft limit 존재만으로 action이 자동 제한된다고 가정하지 않는다.

공식 body ±48°와 USD hip ±60°는 차이가 있고, soft ±54°도 여전히 ±48°보다 넓다. 공식 thigh [−200°,90°]와 USD 앞 thigh [−90°,200°]는 부호 반전 관계지만, 뒤 thigh [−30°,260°]에는 추가 차이가 있다. 실물 각도 영점/축 방향 확인 없이 하나의 범위를 다른 쪽에 복사하지 않는다. calf [−156°,−48°]는 반올림 범위에서 일치한다.

## 5. 질량·관성·CoM

USD 질량 합계 [계산] **15.018999692 kg**. 공식 약 15 kg에 대한 수치 차이는 약 **+0.019 kg / +0.127%**다. 공식 값 자체가 근삿값이므로 이를 제조 공차 측정처럼 해석하지 않는다.

mass 구성: base 6.921 kg + 다리당 (0.678+1.152+0.154+0.040) kg × 4 + head 0.001 kg × 2. 임의 적재물은 포함하지 않는다.

아래는 각 body의 `physics:mass`, `physics:centerOfMass`, `physics:diagonalInertia`, `physics:principalAxes`다. CoM는 body local, 관성은 CoM 기준 주관성 kg·m², quaternion은 `(w,x,y,z)`다. 텐서는 quaternion 회전 R을 사용해 `I = R diag(Iprincipal) Rᵀ`로 복원한다. 모든 숫자가 실차 실측이라는 뜻은 아니다.

| body | kg | local CoM (m) | 주관성 (kg·m²) | principalAxes |
| --- | --- | --- | --- | --- |
| base | 6.921 | (0.021111997, 0, -0.0053659999) | (0.024453087, 0.098077118, 0.10702679) | (0.999958, 0.0016130266, 0.008992232, -0.0008444425) |
| FL_hip | 0.678 | (-0.0054000001, 0.00194, -0.000105) | (0.00047996719, 0.00088402943, 0.00059600337) | (0.9999788, -0.0024969478, 0.0047282153, 0.0037199105) |
| FL_thigh | 1.152 | (-0.0037400001, -0.022299999, -0.032699998) | (0.0058414871, 0.0059497268, 0.00087878696) | (0.97662413, 0.074123584, 0.045750134, -0.19651426) |
| FL_calf | 0.154 | (0.0054799998, -0.00097499997, -0.115) | (0.001080271, 0.0011000754, 3.2553424e-05) | (0.999887, 0.003973435, -0.008161128, -0.011986799) |
| FL_foot | 0.04 | (0, 0, 0) | (9.6000003e-06, 9.6000003e-06, 9.6000003e-06) | (1, 0, 0, 0) |
| FR_hip | 0.678 | (-0.0054000001, -0.00194, -0.000105) | (0.00047996719, 0.00088402943, 0.00059600337) | (0.9999788, 0.0024969478, 0.0047282153, -0.0037199105) |
| FR_thigh | 1.152 | (-0.0037400001, 0.022299999, -0.032699998) | (0.0058414871, 0.0059497268, 0.00087878696) | (0.97662413, -0.074123584, 0.045750134, 0.19651426) |
| FR_calf | 0.154 | (0.0054799998, 0.00097499997, -0.115) | (0.001080271, 0.0011000754, 3.2553424e-05) | (0.999887, -0.003973435, -0.008161128, 0.011986799) |
| FR_foot | 0.04 | (0, 0, 0) | (9.6000003e-06, 9.6000003e-06, 9.6000003e-06) | (1, 0, 0, 0) |
| Head_upper | 0.001 | (0, 0, 0) | (9.6000003e-06, 9.6000003e-06, 9.6000003e-06) | (1, 0, 0, 0) |
| Head_lower | 0.001 | (0, 0, 0) | (9.6000003e-06, 9.6000003e-06, 9.6000003e-06) | (1, 0, 0, 0) |
| RL_hip | 0.678 | (0.0054000001, 0.00194, -0.000105) | (0.00047996719, 0.00088402943, 0.00059600337) | (0.9999788, -0.0024969478, -0.0047282153, -0.0037199105) |
| RL_thigh | 1.152 | (-0.0037400001, -0.022299999, -0.032699998) | (0.0058414871, 0.0059497268, 0.00087878696) | (0.97662413, 0.074123584, 0.045750134, -0.19651426) |
| RL_calf | 0.154 | (0.0054799998, -0.00097499997, -0.115) | (0.001080271, 0.0011000754, 3.2553424e-05) | (0.999887, 0.003973435, -0.008161128, -0.011986799) |
| RL_foot | 0.04 | (0, 0, 0) | (9.6000003e-06, 9.6000003e-06, 9.6000003e-06) | (1, 0, 0, 0) |
| RR_hip | 0.678 | (0.0054000001, -0.00194, -0.000105) | (0.00047996719, 0.00088402943, 0.00059600337) | (0.9999788, 0.0024969478, -0.0047282153, 0.0037199105) |
| RR_thigh | 1.152 | (-0.0037400001, 0.022299999, -0.032699998) | (0.0058414871, 0.0059497268, 0.00087878696) | (0.97662413, -0.074123584, 0.045750134, 0.19651426) |
| RR_calf | 0.154 | (0.0054799998, 0.00097499997, -0.115) | (0.001080271, 0.0011000754, 3.2553424e-05) | (0.999887, -0.003973435, -0.008161128, 0.011986799) |
| RR_foot | 0.04 | (0, 0, 0) | (9.6000003e-06, 9.6000003e-06, 9.6000003e-06) | (1, 0, 0, 0) |

파일에 저장된 pose에서 계산한 총 CoM는 **(0.0088445232, -6.9301159e-19, -0.030518839) m**이며 USD world/root 기준이다. 식은 `Σ m_i(T_i c_i) / Σ m_i`다. 이 값은 프로젝트 initial joint pose나 정책 보행 중 CoM가 아니다. 다리가 움직이면 전체 CoM도 바뀐다. 공식 하드웨어 CoM·관성 상세 자료는 확인하지 못했으므로 정확도 대조는 보류한다.

공식 기립 외형과 비교할 USD mesh 전체 AABB는 이번에 자세를 맞춰 측정하지 않았다. 파일의 authored pose, 프로젝트 초기 pose, 실물 기립 자세가 다를 수 있어 0.70×0.31×0.40 m를 USD 측정값으로 쓰지 않는다.

## 6. 실제 프로젝트 모터 및 제어 설정

`assets/go2/config/robot_cfg.py`의 `GO2_CFG.actuators['base_legs']`:

| 항목 [설정] | 현재값 | 의미 |
| --- | --- | --- |
| 모델 | DCMotorCfg | explicit PD + 속도 의존 토크 포화 |
| 관절 | hip/thigh/calf 전체 12개 | 동일 모터 파라미터 사용 |
| effort_limit / saturation_effort | 23.5 / 23.5 N·m | 토크 cap와 곡선 포화 토크 |
| velocity_limit | 30 rad/s | DC 모터 곡선의 속도 파라미터 |
| stiffness Kp | 25 N·m/rad | 정책 target position 오차에 작용 |
| damping Kd | 0.5 N·m·s/rad | 속도 오차에 작용 |
| joint friction | 0 | 시뮬레이션 근사; 발바닥 마찰 아님 |
| effort_limit_sim / velocity_limit_sim | 프로젝트에서 명시하지 않음 | Isaac Lab 기본/상속 처리 확인 필요 |

확인한 로컬 구현은 `third_party/IsaacLab/source/isaaclab/isaaclab/actuators/{actuator_base,actuator_pd}.py`와 `assets/articulation/articulation.py`다. explicit actuator는 USD spring drive K/D를 0으로 설정하고 외부에서 PD torque를 계산한다. torque solver cap은 기본 큰 값으로 처리하며 actuator가 직접 포화시킨다. 따라서 raw USD의 calf 45.43 N·m가 그대로 정책 실행 토크 cap이 되는 것은 아니다.

기본 속도 solver limit은 USD에서 상속하는 코드 경로다. 프로젝트가 `velocity_limit_sim`을 지정하지 않으므로 calf 약 15.7 rad/s와 DC 모터 곡선 30 rad/s를 구분해야 한다. 이는 코드 해석 결과이며 실제 초기화된 PhysX tensor를 이번에 읽은 것은 아니다.

```text
τrequested = 25 × (qtarget − q) + 0.5 × (q̇target − q̇) + τfeedforward
τupper = min(23.5, 23.5 × (1 − q̇/30))
τlower = max(−23.5, 23.5 × (−1 − q̇/30))
τapplied = clip(τrequested, τlower, τupper)
```

예를 들어 양의 q̇=15 rad/s에서는 같은 방향 가속 토크 상한이 11.75 N·m다. 23.5를 어떤 속도에서도 제공하는 모터가 아니다. 이 계산은 구현의 해당 속도 구간에 대한 설명이며 실물 모터 성능 측정이 아니다.

USD calf cap 45.43 대비 프로젝트 cap 23.5는 약 **48.27% 작다**. 이것만으로 모델 오류라 단정할 수 없다. 정책이 그 모터 설정으로 학습되었을 수 있으므로 실제 하드웨어와 가깝게 만들겠다고 45로 즉시 올리면 기존 정책의 동작 분포가 바뀐다.

### 6.1 자세·시간·정책 입출력

| 항목 | 설정값 |
| --- | --- |
| 생성 위치 | root (0,0,0.4) m; 유지할 높이 목표 아님 |
| 초기 hip | 왼쪽 +0.1, 오른쪽 −0.1 rad |
| 초기 thigh | 앞 +0.8, 뒤 +1.0 rad |
| 초기 calf / 관절속도 | −1.5 rad / 0 rad/s |
| action | qtarget = default joint position + 0.25 × policy action |
| 관측 / 출력 | 48개 / 12개 |
| 관절 순서 | hip FL/FR/RL/RR → thigh FL/FR/RL/RR → calf FL/FR/RL/RR |
| physics dt / decimation | 0.005 s / 4 → 물리 200 Hz, 정책 50 Hz |
| 기본 command | vx=0.4 m/s, vy=0, yaw rate=0; runtime command로 변경 가능 |
| policy 구조 | 48→128→128→128→12, ELU, 메모리 없는 Actor |

관측은 base 선속도·각속도·중력 방향·속도 명령 각 3개, 상대 관절각·관절속도·이전 action 각 12개로 구성된다. 현재 actor 입력에 LiDAR/카메라/지도/waypoint는 없다. policy 관절 순서·관측 순서·action scale·dt를 바꾸면 같은 checkpoint라도 의미가 달라진다.

## 7. Collision·마찰·센서·물리

composed USD의 collision은 **27개**: Cylinder 17, Cube 5, Sphere 5다. 발 4개는 반지름 0.022 m sphere이며 타이어가 없다. Cube `size=1`은 transform scale과 함께 해석해야 하므로 실제 1 m 상자라고 읽으면 안 된다. 단순 collision은 실물 발 탄성/압력 분포를 그대로 재현하지 않는다.

로봇 USD에 PhysicsMaterialAPI는 발견되지 않았다. 프로젝트 평지와 공통 조선소 지면은 static/dynamic friction **1.0/1.0**, friction combine mode **multiply**, restitution combine mode **multiply**를 명시한다. 반발계수 자체는 프로젝트에서 명시하지 않는다. 로봇과 바닥의 실제 조합값은 runtime material binding/fallback 확인이 필요하며, 지면 1.0을 모든 접촉의 실차 마찰이라고 부르면 안 된다.

프로젝트 override: gravity 활성, rigid linear/angular damping=0, max depenetration velocity=1 m/s, self collision=False, articulation solver position/velocity iterations=4/0. `max_linear_velocity=1000`, `max_angular_velocity=1000`은 rigid-body solver 제한이며 실물 보행 최고속도가 아니다.

센서는 모든 robot body를 대상으로 ContactSensor가 설정되어 있고 history length=3, air-time 추적, update period=0.005 s다. 이는 시뮬레이션 접촉력 관측이며 실물 foot force sensor의 정확도/대역폭 모델이 아니다. 현재 환경에는 경로 계획에 사용할 LiDAR·depth camera 센서가 별도로 구성되어 있지 않다. USD 외형에 장치가 보이는 것과 기능 센서가 동작하는 것은 구분해야 한다.

## 8. 비교 판정과 현재 한계

| 항목 | 판정 | 이유 |
| --- | --- | --- |
| 총질량 | 허용 가능한 근사 | 15.019 kg; 공식 명목값에 가까움 |
| 다리/관절 구성 | 일치 | 4다리 × 3능동관절 |
| calf 각도 범위 | 반올림 수준 일치 | USD 약 −156°…−48° |
| hip 각도 범위 | 추가 확인 필요 | USD ±60°, 공식 ±48°; soft도 ±54° |
| thigh 각도 범위 | 좌표계 대조 필요 | 전후 USD 범위가 다르고 공식 부호/영점도 다를 수 있음 |
| 토크·속도 한계 | 정책용 근사 | 파일상 관절별 값과 runtime DC 모델이 다름 |
| 기립 외형 | 비교 미완료 | 동일 관절 자세로 USD mesh 측정하지 않음 |
| 링크 관성·CoM | 정보 부족 | USD 수치는 있으나 실차 상세 근거 없음 |
| 센서·자율주행 | 부분 구현 | 접촉 센서와 속도 정책; 경로/회피 센서는 별도 필요 |
| 최고속도·경사·적재 능력 | 이번 검증 범위 밖 | 프로젝트 시험으로 제조사 성능을 검증하지 않음 |

기존 높이 검사 기록 `docs/go2_install_evidence/base_height_check.json`은 root Z 평균을 정책 없이 약 0.287 m, 현 정책에서 약 0.162 m로 보고한다. 비교용 공식 PLAY 환경도 약 0.162 m였다. 이는 **과거 기록**이며 이번에 재실행하지 않았다. root Z와 전체 로봇 외형 높이 0.40 m는 서로 다른 측정량이다. 전진 성공·리셋 0회만으로 정상 자세나 속도 추종까지 검증되었다고 해석하지 않는다.

## 9. 권장 우선순위

1. **필수 — 실차 연계 전:** 구매/대상 Go2 버전과 하드웨어 revision 확인, joint 이름·부호·영점·각도 범위 대조. 동일한 45 N·m를 모든 관절에 적용하지 않는다.
2. **필수 — 모델 재현성:** 원격 USD와 dependency hash를 프로젝트 관리 절차에 고정하는 방안 검토. 현재 lock은 policy 해시는 있으나 model content hash는 없다. 이번 보고서에 확인한 두 해시를 남겼다.
3. **필수 — 정책 변경 전:** 현재 checkpoint와 모터·관측·action 계약 유지. 토크/각도/질량을 변경하면 자세·속도 추종·포화·낙상 검증을 다시 수행한다.
4. **권장:** stance/보행 중 body 높이, CoM, foot slip, joint torque/speed를 기록해 낮은 자세 원인을 검토한다. 보상 코드만 변경해도 이미 학습된 정책이 바뀌지는 않는다.
5. **권장 — waypoint 개발:** 위치추정/지도 → 경로 계획 → vx/vy/yaw-rate 명령 생성 → 기존 보행 actor 계층을 분리한다. Go2에 차량식 Ackermann 각도 변환을 적용하지 않는다.
6. **추후 보정:** 실제 발바닥 마찰·탄성, 관절 마찰·백래시·모터 지연, 관절별 torque–speed 곡선, payload 질량/관성을 식별한다.

저속 waypoint 개발은 현재 검증 이력을 참고해 0.2–0.4 m/s 수준의 시험 후보부터 검토할 수 있으나 이는 실차 성능값이나 새로 검증한 권장 한계가 아니다. 장애물 인식과 경로 추종기가 없으므로 command만 주면 장애물을 피해 간다고 가정하지 않는다.

## 10. 조사 범위 및 산출물

공식 제품 페이지, 현재 프로젝트 config/환경/정책 코드, 기존 실행 기록, NVIDIA USD와 참조 mesh를 읽었다. USD 수치·질량합·단위변환·모터 구현을 검토했으며 실차 측정이나 새 시뮬레이션은 수행하지 않았다. 모든 기존 모델/코드는 유지하고 이 보고서만 추가했다. 원격 감사용 파일과 계산 중간 결과는 `/tmp`에 두었다.
