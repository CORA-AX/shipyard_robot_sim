# HUNTER 2.0 하드웨어·물리 파라미터 감사

분석일: **2026-10-08, Asia/Seoul**. 대상: `~/shipyard_robot_sim`, Isaac Sim 5.0용 HUNTER 2.0. **분석과 이 문서 작성만 수행했으며 URDF·USD·YAML을 수정하거나 패키지를 설치하거나 시뮬레이션을 실행하지 않았다.**

## 1. 결론과 증거의 범위

현재 모델은 **외형과 전륜 조향·후륜 구동 기구학을 검토하고 저속 경로 추종을 개발할 수 있는 기반**이다. 실차 동역학을 검증한 디지털 트윈으로 보기는 어렵다.

- 조향축은 현재 URDF와 USD에서 수직 방향으로 교정되어 있다. 앞바퀴 중복 visual/collision은 제거되어 있다.
- 현재 작업 USD에는 앞바퀴 자유 회전, 전륜 Position Drive, 후륜 Velocity Drive, 후륜 무한 회전 및 왼쪽 후륜 회전 부호 교정이 저장되어 있다.
- 총질량 **52.22747 kg**은 공식 **65/70 kg**보다 작다. 링크별 관성의 실제 하드웨어 대응은 미확인이다.
- 후륜 최대 속도는 **약 1 rad/s**이다. 반지름 약 0.1643 m에서 직진 속도는 약 **0.1643 m/s**이므로 YAML의 1.5 m/s와 일치하지 않는다.
- 공식 track 0.605 m와 비교할 때 **joint 원점 간격과 바퀴 형상 중심 간격을 혼동하면 안 된다.** 전자는 전륜 0.584 m, 후륜 0.585 m이나, 후자는 약 0.603/0.604 m이다. 이 차이를 이유로 바퀴 joint를 곧바로 바깥쪽으로 옮기는 것은 근거가 부족하다.
- USD에는 아커만 제어기가 구성되어 있지 않다. 두 조향축에 같은 각도를 주는 것은 방향 확인용이며, 정확한 회전에는 안쪽 바퀴의 조향각을 더 크게 계산해야 한다.
- 로컬 USD의 관절 연결점은 일치하지만, 외부 바닥 참조를 이번 오프라인 USD 검사에서 해석하지 못했다. 바닥 접촉, 최종 마찰, 충돌 cooking 결과, 주행 안정성은 확인하지 않았다.

표기: **[파일]** 실제 파일에 저장된 값, **[공식]** 제조사 자료의 값, **[계산]** 파일/제원에서 유도한 값, **[제안]** 개발 초기 설정 후보, **[미확인]** 실측·실행 또는 추가 자료가 필요한 값. `미설정`은 명시적 authoring이 없다는 뜻이며 엔진의 유효 값이 반드시 0이라는 뜻은 아니다.

## 2. 조사 파일과 모델 선택

아래 경로는 프로젝트 루트 기준이다. USD 경로 중 `hunter2_sim.urdf`는 **파일이 아니라 디렉터리**다.

| 역할 | 경로 | 판정 |
| --- | --- | --- |
| 현재 입력 URDF | `assets/hunter2/source/hunter2_sim.urdf` | 9 links, 8 joints: 가동 6 + 고정 2 |
| 원본 보존본 | `assets/hunter2/source/original/hunter2_base.urdf` | 공식 Gazebo 저장소의 기본 URDF와 바이트 일치 |
| 메시 | `assets/hunter2/source/meshes/*.STL` | 9개 보관, 현재 활성 geometry는 7종; 원본 복사본 일치 |
| 명목 제원 | `assets/hunter2/config/hunter2.yaml` | driver 기반 숫자이며 USD를 자동 제어하거나 갱신하지 않음 |
| 출처 잠금 | `assets/hunter2/config/sources.lock.json` | 등록 14개 파일 SHA-256 일치 |
| 과거 검증 결과 | `assets/hunter2/config/validation_report.json` | 이번 재검증으로 대체 해석; 동역학 검증 증거 아님 |
| 현재 작업 USD | `assets/hunter2/usd/hunter2_sim.urdf/hunter2_sim/hunter2_sim.usd` | 이번 감사의 주 USD; 사용자 수동 변경을 포함한 작업본 |
| 이전 USD | `assets/hunter2/source/hunter2_sim/hunter2_sim.usd` | 고정 base/root joint 및 바퀴 Position Drive가 남은 이전 결과; 현재 주행 모델로 혼용 금지 |
| USD 구성 레이어 | 각 USD 옆 `configuration/hunter2_sim_{base,physics,robot,sensor}.usd` | root와 함께 보존해야 함; root만 복사하면 의존 파일 누락 가능 |

현재 작업 USD를 하드웨어 검증이 끝난 “최종 모델”이라고 선언할 근거는 없다. 파일에 저장된 동작 설정과 수정 내역을 기준으로 선택한 **최신 작업 후보**이다. assets 안에는 새로 조립할 활성 Xacro가 없으며, 외부 Gazebo/LCAS Xacro는 참고 자료다.

기존 `docs/hunter2_completion_report.md`, `hunter2_isaac_source_review.md`, `hunter2_model_comparison.md`, `hunter2_source_correction.md`, `hunter2_usd_import_plan.md`, `hunter2_kinematic_review.json` 및 준비·검증·비교 스크립트를 검토했다. 과거의 “USD 미생성”, “축 교정 예정” 등의 상태는 현재 파일과 다를 수 있다. 이번에는 XML, STL vertex, USD composition을 직접 읽어 수치를 재확인했다.

스크립트의 현재 위치는 `scripts/hunter2/{prepare_hunter2_asset,validate_hunter2_asset,review_hunter2_kinematics,compare_hunter2_models}.py`이다. 예전 문서의 `scripts/validate_hunter2_asset.py` 경로와 구분한다.

### 2.1 외부 모델 출처와 재현성

| 로컬 디렉터리 | upstream / 기록된 commit | 역할 |
| --- | --- | --- |
| `third_party/ugv_gazebo_sim` | [AgileX](https://github.com/agilexrobotics/ugv_gazebo_sim), `27633a956c845903ee630538afeb17fe70afdd84` | 원본 URDF/STL 및 Gazebo Xacro |
| `third_party/hunter_ros2_reference` | [AgileX hunter_ros2](https://github.com/agilexrobotics/hunter_ros2), `7bb146dd40fcea757b3516292319aa14ace41577` | `HunterV2Params` 및 명령 변환 참고 |
| `third_party/hunter_robot` | [LCAS](https://github.com/LCAS/hunter_robot), `e073b273674459a13ec57b8731d06ddc8770546f` | 비공식 시뮬레이션 구현 비교 |

`docs/source_repositories.json`에는 CORA-AX fork와 위 upstream/commit이 기록되어 있다. 현재 Hunter 관련 third_party 디렉터리는 자체 `.git`이 없는 복사본이다. 여기서 `git rev-parse HEAD`를 실행하면 상위 프로젝트 commit `31cddd8e69849099b6b9e51bb2642e6c70030c49`가 나온다. 따라서 이를 vendor HEAD로 간주하면 안 된다. LCAS 전체 checkout이 기록 commit과 완전히 같은지는 이번에 증명하지 않았다.

재검증 결과:

- `PYTHONDONTWRITEBYTECODE=1 python3 scripts/hunter2/validate_hunter2_asset.py --structural-only`: **통과**.
- 기본 validator: **실패**, `Unreviewed vendor commit` 오류. 현재 복사본 구조와 `verify_sources()`의 git HEAD 검사 방식이 맞지 않는다.
- 잠금 파일의 파일별 해시 14/14 일치, 원본 URDF와 보존본 일치, 9개 STL 복사본 일치.
- `derived_bytes(original, 0.58)`를 메모리에서 계산한 결과와 현재 URDF 바이트가 일치한다. 생성 스크립트의 쓰기 작업은 실행하지 않았다.

즉, 기본 validator 실패를 숨기거나 “전체 검증 통과”라고 표현하지 않는다. 현재 원본 파일 내용은 잠금 해시로 별도 확인했다.

### 2.2 AgileX와 LCAS에서 무엇을 채택할 수 있는가

공식 Gazebo Xacro는 단순히 기본 URDF를 include한 파일이 아니라 별도 설정을 갖는다. `hunter/hunter2_base/urdf/hunter2_base_gazebo.xacro`는 조향 axis `0 1 0`, 왼쪽 후륜 axis `0 0 -1`, 축약 steering geometry, base mass 21.02945 kg 등을 사용한다. 일부 조향 effort/velocity는 0이며 Gazebo 플러그인도 포함한다. `mu1=0.8`, 후륜 `mu2=10` 같은 Gazebo 설정은 현재 URDF/USD에 자동 승계되지 않는다.

LCAS `hunter_description/description/robot_core.xacro`는 조향 axis `0 1 0`, 후륜 양쪽 `0 0 -1` 등 유용한 방향 설정을 보여 준다. 그러나 오른쪽 조향을 왼쪽에 1:1 mimic하고 후륜도 1:1 mimic한다. `config/ackermann_like_controller.yaml`은 `TricycleController`를 사용한다. 이 구성 전체를 정확한 좌우 아커만 분배로 간주할 수 없다.

LCAS `inertial_macros.xacro`는 chassis 72 kg + 바퀴 7 kg × 4 + steering 0.005 kg × 2로 **100.01 kg**이며, chassis 관성은 작은 기존 값을 그대로 사용한다. 따라서 LCAS의 질량·관성을 실차 정답으로 복사하지 않는다. 축 방향·geometry 소유권은 참고하되, 질량·mimic·Gazebo 마찰·제어기는 별도 근거로 선택해야 한다.

## 3. 공식 하드웨어 자료와 버전 차이

자료 접근일: 2026-10-08. 다음 출처 구분을 이후 표에서도 사용한다.

- **S1**: [AgileX 중국어 HUNTER 2.0 Introduction](https://agilexrobotics.gitbook.io/hunter-2-0/1-hunter-2.0-jian-jie-introduction), 온라인 표. 명시적인 문서 revision/해당 실물 일련번호는 확인되지 않는다.
- **S2**: [AgileX 영어 The Basics](https://agilexrobotics.gitbook.io/hunter-2.0/2-the-basics), 조향 명령이 **내측 바퀴 각도**라고 설명한다.
- **S3**: [AgileX 영어 Getting Started](https://agilexrobotics.gitbook.io/hunter-2.0/3-getting-started), 조향 및 좌우 후륜 모터 식별. 본문에 모터 개수 표현의 불일치가 있어 모터 ID 설명과 다른 자료를 함께 본다.
- **S4**: [제조사 작성 영어 매뉴얼 PDF, Generation Robots 배포본](https://static.generation-robots.com/media/hunter-user-manual-en.pdf). 표지 V2.0.2 / 2023.08이나 개정 이력에는 V2.0.4 / 2023-09-08까지 있다. 단일 일관된 revision으로 단정하지 않는다. PDF 9–10쪽의 제원표를 참조했다.
- **S5**: 로컬 공식 driver `third_party/hunter_ros2_reference/hunter_base/include/hunter_base/hunter_params.hpp`, `struct HunterV2Params`. [기록된 upstream 파일](https://github.com/agilexrobotics/hunter_ros2/blob/7bb146dd40fcea757b3516292319aa14ace41577/hunter_base/include/hunter_base/hunter_params.hpp). 로컬 파일은 SHA-256 검증 완료. 주석은 manual v1.2.6_S를 언급하지만 해당 매뉴얼 원문을 열람했다고 주장하지 않는다.

S1의 명목 값은 길이/너비/높이 0.980/0.745/0.380 m, 축간 0.650 m, 전후 track 0.605 m, 자체질량 65/70 kg, 적재 150 kg, 무부하 최고속도 1.5 m/s, 최소 회전반경 1.6 m, 최소 지상고 0.105 m이다. 구동 모터는 brushless 400 W × 2, 조향은 brushless 400 W, 감속비는 1:40이며 최대 내측 조향은 33°다. 조향 정밀도 0.5°는 **조향속도가 아니다**. 질량 65/70 kg를 특정 배터리 선택과 일대일 대응시키는 것은 보류한다.

S4는 높이 0.370 m, 지상고 0.100 m를 기재한다. 속도 표의 `km/h: 1.5`, 회전반경 표의 `mm: 33°`는 다른 근거와 불일치하거나 차원이 잘못되어 그대로 채택하지 않았다. S1/S5와 일치하는 1.5 m/s를 명목 기준으로 삼되 실물 버전 확인이 필요하다. S4의 조향 감속비는 `-`이며 구동 감속비 1:40이다.

S5의 수치는 wheel radius 0.165 m, reduction 30, max inner steer 0.58 rad, max center steer 0.461 rad다. 감속비 1:30은 S1/S4의 1:40과 충돌한다. 드라이버 상수만으로 실제 감속기를 확정하지 않는다. `0.75 for inner wheel` 같은 오래된 주석보다 실제 상수 0.58을 읽되, 33°와의 작은 차이도 남긴다.

실차의 바퀴 폭, 최대 조향속도, 연속/피크 출력축 토크, 조향 감속비, 링크별 질량·관성, 차체 CoM, 타이어 마찰은 확인한 자료만으로 확정할 수 없다. 모터 400 W만으로 토크를 계산할 수 없으며 회전속도와 감속 효율 등이 추가로 필요하다.

## 4. 현재 URDF의 외형과 기구학

기준: 조향/바퀴 각도 0, `base_link`에서 +X 전방, +Y 좌측, +Z 상방. STL의 모든 vertex를 link/joint 변환으로 base 좌표계에 옮겨 AABB를 계산했다. STL 단위는 URDF의 무배율 mesh와 전체 크기 일치에 따라 m로 해석한다. 원본 CAD 설계 공차나 실차 실측을 의미하지 않는다.

| 항목 | 값 [계산] | 추출 위치 / 의미 |
| --- | --- | --- |
| 전체 길이·너비·높이 | (0.979809135, 0.745566147, 0.37664415) m | 활성 visual mesh 전체 AABB 크기 |
| AABB 최소 | (-0.444231685, -0.372793071, -0.359644149) m | base 좌표 |
| AABB 최대 | (0.53557745, 0.372773076, 0.0170000009) m | base 좌표 |
| 왼쪽/오른쪽 wheelbase | 0.65142 / 0.65160 m | 전후 wheel joint origin의 X 차이 |
| 평균 wheelbase | 0.65151 m | 두 축의 평균 X 차이 |
| 전륜 조향 pivot track | 0.58400 m | front_steer_left/right_joint origin Y 차이 |
| 후륜 joint-origin track | 0.58500 m | left/right_rear_joint origin Y 차이 |
| 전륜/후륜 mesh 축방향 중간면 간격 | 약 0.60300 / 0.60400 m | wheel STL 축방향 min/max 중간면을 base로 변환; 접지압 중심 실측 아님 |
| 최소 지상고 후보 | 0.108458 m | body 최저 Z − 전체 wheel 최저 Z; 평면 바닥·정적 형상 가정 |
| 4바퀴 최저점 편차 | 약 0.000484 m | 메시 각도/형상 차이; 동적 안착 후 값 아님 |

### 4.1 Track과 바퀴 크기의 해석

wheel mesh는 joint 원점을 기준으로 축 방향에 약 0.0095 m씩 바깥쪽으로 치우쳐 있다. 따라서 joint 간격 0.584/0.585 m와 외형 중심 간격 0.603/0.604 m가 동시에 성립한다. 형상 중심은 축방향 AABB 중간면으로 정의했으며 타이어 접지 중심과 완전히 같다고 입증한 것은 아니다. 공식 0.605 m의 측정 기준도 도면/실측으로 확인해야 한다.

조향각 계산에는 조향 pivot 간격을, 후륜 지면 선속도 분배에는 타이어 유효 구름 중심 간격을 사용하는 모델이 더 적합하다. 단일 track을 쓰는 제어기는 이 차이를 근사한다. 0.605를 맞추려고 joint 위치만 일괄 수정하면 기존 mesh offset까지 더해져 오히려 폭이 달라질 수 있다.

| 바퀴 | 축으로부터 최대 vertex 반경 (m) | 축방향 전체 mesh 폭 (m) | 근거 |
| --- | --- | --- | --- |
| front_left | 0.164158002 | 0.141565721 | `assets/hunter2/source/hunter2_sim.urdf` wheel link `collision/geometry/mesh` + STL |
| front_right | 0.164158007 | 0.141565721 | `assets/hunter2/source/hunter2_sim.urdf` wheel link `collision/geometry/mesh` + STL |
| rear_left | 0.164274644 | 0.141565721 | `assets/hunter2/source/hunter2_sim.urdf` wheel link `collision/geometry/mesh` + STL |
| rear_right | 0.164274644 | 0.141565721 | `assets/hunter2/source/hunter2_sim.urdf` wheel link `collision/geometry/mesh` + STL |

이 반경은 CAD의 최대 외곽값이며 하중·타이어 변형이 반영된 rolling radius가 아니다. 폭 0.141566 m 역시 전체 wheel assembly의 mesh 폭이며 트레드 접촉 폭이라고 확정하지 않는다. 명목 제어 반경 0.165 m와 메시 반경은 약 0.44–0.51% 다르다.

### 4.2 Joint 위치·축·제한

아래 모든 XML 위치는 `hunter2_sim.urdf`의 `/robot/joint[@name="이름"]/{origin,axis,limit}`다. position과 axis는 **base 좌표계, zero pose**로 변환한 결과다. 소수점의 극소 성분은 `rpy=1.5708`이 정확한 π/2와 조금 다른 데서 생긴다.

| joint | 유형 | 위치 (m) | 축 단위벡터 | limit: rad, N·m, rad/s |
| --- | --- | --- | --- | --- |
| `front_steer_left_joint` | revolute | (0.37142, 0.29199, -0.1955) | (0, -3.6732051e-06, 1) | lower=-0.58, upper=0.58, effort=100, velocity=1 |
| `front_left_wheel_joint` | continuous | (0.37142, 0.29199, -0.1955) | (0, -1, -3.6732051e-06) | — |
| `front_steer_right_joint` | revolute | (0.37142, -0.29201, -0.1955) | (0, -3.6732051e-06, 1) | lower=-0.58, upper=0.58, effort=100, velocity=1 |
| `front_right_wheel_joint` | continuous | (0.37142, -0.29201, -0.1955) | (0, 1, 3.6732051e-06) | — |
| `left_rear_joint` | continuous | (-0.28, 0.29249, -0.19558) | (0, -1, -3.6732051e-06) | effort=100, velocity=1 |
| `right_rear_joint` | continuous | (-0.28018, -0.29251, -0.19558) | (9.76153107e-09, 1, 3.67319213e-06) | effort=100, velocity=1 |
| `front_steer_joint` | fixed | (0.44792, -1.1431e-05, -0.121) | (0, 0, 0) | — |
| `rear_wheel_joint` | fixed | (-0.42408, -1.1431e-05, -0.085581) | (0, 0, 0) | — |

가동 joint 모두 URDF `<dynamics>`와 `<mimic>`가 없다. joint 수동 damping/friction은 **미설정**이다. 앞 wheel의 effort/velocity limit도 미설정이며, 뒷 wheel은 effort 100 N·m, velocity 1 rad/s다. 이는 시뮬레이션 상한으로, 실제 모터의 정격 토크·회전속도 증거가 아니다.

## 5. 질량·관성·무게중심

전체 질량 [계산] **52.2274695954 kg**. 전체 CoM [계산], base 기준 **(0.137848884, -0.000298215, -0.188608260) m**. 계산식은 `c = Σ(m_i · (R_i c_i + t_i)) / Σm_i`다. 위에서 가정한 최저점 접지 평면 기준 CoM 높이는 약 **0.171036 m**이나 실제 하중 상태의 CoM가 아니다.

아래 XML 위치는 `/robot/link[@name="이름"]/inertial/{mass,origin,inertia}`. 관성은 각 링크의 **inertial frame CoM 기준**, 단위 kg·m². 모든 inertial origin rpy는 0이다. 대칭 텐서는 `[[Ixx,Ixy,Ixz],[Ixy,Iyy,Iyz],[Ixz,Iyz,Izz]]`로 복원한다.

| link | 질량 (kg) | local CoM (m) | Ixx, Iyy, Izz | Ixy, Ixz, Iyz |
| --- | --- | --- | --- | --- |
| `base_link` | 6.02945169537 | (0.0266989328, 0.000409203501, -0.135901125) | (0.033237096, 0.111072002, 0.126608628) | (-1.01315049e-06, -0.00188859201, -5.13308151e-07) |
| `front_steer_left_link` | 7.8526 | (-0.012614, 1.8116e-05, 0.0023132) | (0.063827, 0.063827, 0.11091) | (1.5154e-07, 3.1921e-07, 2.9854e-08) |
| `front_left_wheel_link` | 7.7007 | (-6.5774e-06, 2.0242e-05, -0.0096386) | (0.063724, 0.063724, 0.11072) | (-3.8745e-08, 2.2753e-07, -4.88e-08) |
| `front_steer_right_link` | 7.696 | (-8.9374e-08, 1.0997e-07, 0.0095958) | (0.063704, 0.063704, 0.11068) | (-6.3892e-08, -2.0159e-08, 6.4721e-08) |
| `front_right_wheel_link` | 7.5613 | (-9.1148e-08, 1.1284e-07, 0.0095049) | (0.063626, 0.063626, 0.11054) | (-6.4092e-08, -2.0164e-08, 6.4721e-08) |
| `left_rear_link` | 7.6692 | (-8.826e-05, 6.4342e-05, -0.0096491) | (0.063699, 0.063699, 0.11067) | (-2.542e-07, 2.638e-08, -5.9934e-08) |
| `right_rear_link` | 7.7133 | (8.6344e-05, 7.4488e-05, 0.0097824) | (0.06373, 0.06373, 0.11073) | (2.5408e-07, 2.6379e-08, 5.9941e-08) |
| `front_steer_link` | 0.0049179 | (0.049836, 8.1046e-15, 0.017912) | (1.6846e-07, 7.549e-07, 7.334e-07) | (7.6396e-23, -1.0974e-08, -1.8957e-22) |
| `rear_wheel_link` | 0 | (0, 0, 0) | (0, 0, 0) | (0, 0, 0) |

질량이 양수인 링크의 관성 고유값은 양수이며 주관성 모멘트의 삼각 부등식을 만족한다. 이는 수학적 유효성 검사이지 하드웨어 관성의 정확성 검증이 아니다. `rear_wheel_link`는 0 질량/0 관성의 고정 부속이며 현재 USD에서는 base에 병합되어 독립 동적 0질량 바디가 되지 않는다.

특히 geometry를 제거한 두 steering link에 약 7.85/7.70 kg가 그대로 남아 있다. virtual knuckle의 물리적 의미와 wheel 질량의 중복 가능성을 재검토해야 한다. 증거 없이 그 질량을 삭제하거나 두 배 계산이라고 단정하지 않는다. 총질량을 65 kg에 맞추기 위해 base mass만 더하고 관성을 그대로 두는 방식도 적절하지 않다.

### 5.1 USD 변환 후 질량

현재 USD의 rigid body는 7개다. fixed link 두 개가 base에 병합되어 base mass가 6.034369946 kg가 되었다. 아래는 각 prim의 `physics:mass`, `physics:diagonalInertia`, `physics:principalAxes`를 직접 읽은 결과다. principalAxes quaternion 순서는 `(w,x,y,z)`이며, 대각 관성과 함께 해석해야 한다.

| `/hunter2_base/` 아래 body | 질량 (kg) | 주관성 (kg·m²) | principalAxes |
| --- | --- | --- | --- |
| `base_link` | 6.034369946 | (0.0332017131, 0.112168416, 0.127740577) | (0.999946, 0.000014270459, -0.010393011, 4.5700398e-7) |
| `front_steer_left_link` | 7.852600098 | (0.0638271496, 0.0638268515, 0.110909998) | (0.92406636, 0.0000010027434, 0.0000032536466, 0.38223222) |
| `front_left_wheel_link` | 7.700699806 | (0.0637239516, 0.063724041, 0.110720001) | (0.91730684, 0.0000014401528, 0.0000020138245, 0.39818102) |
| `front_steer_right_link` | 7.696000099 | (0.0637039393, 0.0637040585, 0.110679999) | (0.9185627, -7.1758683e-7, 7.520047e-8, 0.3952754) |
| `front_right_wheel_link` | 7.561299801 | (0.0636259392, 0.0636260659, 0.110540003) | (0.923218, -7.1940383e-7, 6.666345e-8, 0.38427672) |
| `left_rear_link` | 7.669199944 | (0.0636987388, 0.0636992455, 0.11067) | (0.92387956, 6.968899e-7, 1.5290837e-8, 0.38268343) |
| `right_rear_link` | 7.713300228 | (0.0637302548, 0.0637297481, 0.11073) | (0.92387956, -4.817391e-7, 5.032957e-7, 0.38268343) |

USD 총질량 **52.22746992111 kg**, base 좌표 CoM **(0.137848878, -0.000298215641, -0.188608261) m**. URDF 총질량과 차이는 약 3.26×10⁻⁷ kg이며 float 저장 정밀도 수준이다. CoM도 약 10⁻⁸ m 범위에서 일치한다. base 관성은 fixed-link 병합으로 달라지므로 원래 base의 대각 원소만 직접 비교하면 안 된다. USD의 주축 표현을 실차 관성 측정값으로 취급하지 않는다.

## 6. 현재 USD의 물리 구성과 구동

검사 방식: 기존 Isaac Sim 5.0 환경의 `omni.usd.libs`에 포함된 `pxr`로 USD를 읽었다. Kit/SimulationApp/PhysX stepping은 실행하지 않았다. 로컬 root와 configuration 레이어의 composed 속성을 검사했다.

- `metersPerUnit=1`, `upAxis=Z`, `defaultPrim=hunter2_base`.
- variant: `Physics=PhysX`, `Robot=Robot`, `Sensor=Sensors`.
- Articulation Root: `/hunter2_base/base_link`. 별도 world fixed joint 없는 movable 구성.
- 7 rigid bodies, 6 revolute joints. 모든 `physics:body0/body1`가 존재하는 rigid body를 가리킨다.
- 6 joint 모두 양쪽 local anchor를 world로 변환한 위치 차이는 저장 pose에서 **0 m**다. 연결 누락이나 anchor 위치 분리는 발견되지 않았다.
- `physxArticulation:enabledSelfCollisions=False`, solver position iterations 32, velocity iterations 1.
- 왼쪽 후륜 축은 USD override로 base의 +Y를 향한다. 오른쪽 후륜도 +Y여서 두 양의 속도 명령이 +X 전진 방향이다. **URDF의 왼쪽 후륜은 -Y이므로 재import 시 부호가 달라진다.**

### 6.1 Drive와 limit 실제 값

경로: `assets/hunter2/usd/hunter2_sim.urdf/hunter2_sim/hunter2_sim.usd`의 composed `/hunter2_base/joints/<joint>`. K/D는 `drive:angular:physics:stiffness/damping`, torque cap은 `...:maxForce`, 속도 limit은 `physxJoint:maxJointVelocity`다. raw USD 각도/각속도는 degree/degree·s⁻¹이며 Python articulation 제어의 rad/rad·s⁻¹와 구별한다.

| joint | 역할 | K raw | D raw | maxForce (N·m) | 속도 상한 (rad/s) | 각도 범위 (rad) |
| --- | --- | --- | --- | --- | --- | --- |
| `front_steer_left_joint` | Position | 903.072815 | 72.245827 | 100 | 0.999999945 | -0.579999934 … 0.579999934 |
| `front_left_wheel_joint` | free (K=D=0) | 0.000000 | 0.000000 | 3.40282e+38 | inf | -inf … inf |
| `front_steer_right_joint` | Position | 907.908813 | 72.632706 | 100 | 0.999999945 | -0.579999934 … 0.579999934 |
| `front_right_wheel_joint` | free (K=D=0) | 0.000000 | 0.000000 | 3.40282e+38 | inf | -inf … inf |
| `left_rear_joint` | Velocity | 0.000000 | 619.264221 | 100 | 0.999999945 | -inf … inf |
| `right_rear_joint` | Velocity | 0.000000 | 621.344238 | 100 | 0.999999945 | -inf … inf |

모든 drive type은 `force`다. 앞 wheel에는 DriveAPI가 남아 있지만 K=D=0이므로 active motor로 볼 수 없다. 3.402823×10³⁸은 사실상 무제한 수치이며 실차 토크가 아니다. 저장된 조향 targetPosition과 후륜 targetVelocity는 0이다. 최대 조향속도 약 1 rad/s는 **모델 limit**이며 실차 제원이 아니다.

[OpenUSD DriveAPI 단위 정의](https://openusd.org/dev/api/class_usd_physics_drive_a_p_i.html)에 따르면 각 drive의 stiffness/damping은 degree 기준이다. SI의 rad 기준 계수로 표현할 때는 raw K/D에 `180/π`를 곱한다. 아래 환산값은 파일 단위의 변환이며 실제 액추에이터 식별 결과가 아니다. importer의 Natural Frequency/ratio 입력을 그대로 실차 응답으로 해석하지 않는다.

| joint | K (N·m/rad) | D (N·m·s/rad) |
| --- | --- | --- |
| front_steer_left_joint | 51742.261 | 4139.381 |
| front_steer_right_joint | 52019.343 | 4161.547 |
| left_rear_joint | 0.000 | 35481.226 |
| right_rear_joint | 0.000 | 35600.402 |

위 D는 velocity error에 작용하는 **drive gain**이며 URDF의 passive joint damping이나 타이어 마찰계수와 다른 양이다. 낮은 속도에서도 torque cap에 쉽게 도달할 수 있는 높은 gain이므로 응답 측정 후 조정해야 한다. 이번에는 안정성 시험을 하지 않았으므로 적정 gain을 확정하지 않는다.

### 6.2 Collision, 마찰, 바닥

URDF는 활성 7 link의 visual/collision에 같은 STL을 사용한다. steering 좌우 link는 geometry가 없고, body와 고정 부속 2개 및 바퀴 4개에 collision이 있다. 현재 USD에서 7 collider는 모두 `physics:approximation=convexHull`이다. base rigid body에 collider 3개, 각 wheel에 1개씩 붙는다. 실제 엔진의 convex hull cooking 결과를 오프라인에서 생성하거나 시험하지 않았다.

로컬 composed 로봇 prim에서 PhysicsMaterialAPI와 명시적인 static/dynamic friction, restitution 설정을 찾지 못했다. Looks/DefaultMaterial 바인딩은 시각 재질이며 타이어 마찰의 근거가 아니다. passive joint friction도 명시되어 있지 않다. 엔진 fallback 및 바닥 재질 조합의 최종 유효 값은 **확인 불가**다. Gazebo의 mu1/mu2를 현재 적용값으로 보고하지 않는다.

현재 root에는 `/FlatGrid`와 `/Render`도 저장되어 있다. `/FlatGrid`는 NVIDIA의 원격 `Assets/Isaac/5.0/Isaac/Environments/Grid/default_environment.usd`를 참조한다. standalone USD resolver는 이 참조를 열지 못했다. 이는 Isaac Sim GUI에서도 반드시 실패한다는 뜻은 아니다. 원본 URI는 `https://`이며 경고 출력의 `https:/`만으로 파일 URI를 잘못 작성했다고 단정하지 않는다.

로봇 root translation Z=-0.430274231 m, FlatGrid translation Z=-1.001130042 m이다. 두 값만으로 실제 바닥 collider 높이를 확정할 수 없다. 원격 환경 내부 transform과 PhysicsScene이 이번 검사에서 해석되지 않았으므로 “정상 접지한다”고 증명할 수 없다. defaultPrim을 reference하면 로봇 subtree만 가져오며 sibling FlatGrid/Render가 자동 포함되는 것으로 가정하지 않는다. 재사용 로봇과 테스트 world를 분리하는 것이 권장된다.

## 7. 하드웨어 대조표

오차는 `(Asset − 기준)` 및 `|Asset − 기준| / |기준| × 100`이다. “허용 가능한 근사”는 초기 기구학 개발 관점의 판단이며 제조사 공차 판정이 아니다. 측정 정의가 다르면 오차를 억지로 계산하지 않는다. U=현재 URDF, D=현재 작업 USD, Y=`config/hunter2.yaml`; 모두 2절 경로 참조.

| 파라미터 | 실제 HUNTER 2.0 | 현재 Asset | 차이 | 판단 | 근거 파일/출처 |
| --- | --- | --- | --- | --- | --- |
| 길이 | 0.980 m | 0.979809 m | −0.000191 m; 0.0195% | 허용 가능한 근사 | S1; U/STL AABB |
| 너비 | 0.745 m | 0.745566 m | +0.000566 m; 0.0760% | 허용 가능한 근사 | S1; U/STL AABB |
| 높이 | 0.380 m (S1) / 0.370 m (S4) | 0.376644 m | −0.003356 m, 0.883% / +0.006644 m, 1.796% | 정보 부족 | S1/S4 버전·기준 차이; U/STL |
| Wheelbase | 0.650 m | 0.651510 m | +0.001510 m; 0.2323% | 허용 가능한 근사 | S1/S5; U joint/origin |
| Front track | 0.605 m | mesh 중간면 0.603 / pivot 0.584 m | 중간면 기준 −0.002 m, 0.331%; pivot은 기준 다름 | 허용 가능한 근사 | S1/S5; STL + U steer/origin; 접지 중심 미확인 |
| Rear track | 0.605 m | mesh 중간면 0.604 / joint 0.585 m | 중간면 기준 −0.001 m, 0.165%; origin은 기준 다름 | 허용 가능한 근사 | S1/S5; STL + U rear/origin |
| Wheel radius | driver 명목 0.165 m | 전 0.164158 / 후 0.164275 m | −0.000842 / −0.000725 m; 0.510 / 0.440% | 허용 가능한 근사 | S5; STL 최대 반경; rolling radius는 미확인 |
| Wheel width | 확인 불가 | mesh 폭 0.141566 m | 산출 불가 | 정보 부족 | U/STL; 실차 트레드 폭 자료 없음 |
| 최소 지상고 | 0.105 / 0.100 m | 평면 접지 가정 0.108458 m | +0.003458 / +0.008458 m; 3.29 / 8.46% (조건부) | 비교 불가 | S1/S4; 하중·자세 미동일 |
| 자체 질량 | 65 / 70 kg | 52.227470 kg | −12.772530 / −17.772530 kg; 19.650 / 25.389% | 수정 필요 | S1/S4; U inertial/mass, D physics:mass |
| 최대 적재 | 150 kg | payload 모델 없음 | 미설정 | 정보 부족 | S1; 별도 payload 질량/관성 필요 |
| 최대 주행속도 | 1.5 m/s | Y 1.5; D limit 기반 약 0.164275 m/s | 약 −1.335725 m/s; 89.05% | 수정 필요 | S1/S5; D maxJointVelocity × mesh r; 무미끄럼 가정 |
| 바퀴 각속도 | 직진 1.5/0.165=9.09091 rad/s [계산] | 후륜 약 1 rad/s | −8.09091 rad/s; 89.0% | 수정 필요 | S5 계산; U limit/@velocity, D |
| 최대 개별 조향 | 내측 33°=0.575959 rad | ±0.58 rad | +0.004041 rad; 0.702% | 수정 필요 | S1, U limit; full-range 운전 전 정책 통일 |
| 중심 조향 | S5 상수 0.461 rad | Y 0.461; pivot/내측limit 기반 0.468797 | +0.007797 rad; 1.69% (계산 정의 차이) | 허용 가능한 근사 | S5; U 기하학; 보수적 0.461 제한 가능 |
| 최소 회전반경 | 1.6 m, 기준점 불명 | 후륜중앙 1.286416 m; 외측 전륜 pivot 1.707590 m | 기준점 불명으로 직접오차 확정 불가 | 비교 불가 | S1; 8절 계산 |
| 최대 회전반경 | 확인 불가 | 직진에서 ∞ | 유한 최대값 없음 | 비교 불가 | 아커만 계산 |
| 구동 형태 | 전륜 조향·후륜 좌우 구동 | 현재 D 동일, 전 wheel passive | 구조상 차이 없음 | 일치 | S1/S3; D Drive |
| 구동 모터 | brushless 400 W × 2 | 전기적 motor 모델 없음 | 미설정 | 정보 부족 | S1/S4; D는 joint drive만 |
| 조향 모터 | brushless 400 W | 두 독립 position drive | 실차 linkage/단일 모터를 추상화 | 허용 가능한 근사 | S1/S4; torque 분배·linkage 미재현 |
| 구동 감속비 | S1/S4 40; S5 30 | URDF/USD 미설정 | 출처 충돌 | 정보 부족 | S1/S4/S5 |
| 최대 구동·조향 토크 | 확인 불가 | 각 active drive cap 100 N·m | 하드웨어 대응 불명 | 정보 부족 | U effort, D maxForce |
| 최대 조향속도 | 확인 불가 | 1 rad/s | 실차와 비교 불가 | 정보 부족 | U limit, D maxJointVelocity |
| CoM·관성·링크 질량 | 공식 상세값 미확인 | 5절 수치 명시 | 차이 산출 불가 | 정보 부족 | U inertial, D MassAPI |
| 타이어 마찰·반발 | 실측/공식값 미확인 | 로봇에 명시적 재질 없음 | 미설정 | 정보 부족 | D MaterialAPI 검사 |
| 수동 joint damping/friction | 확인 불가 | URDF 미설정; D drive gain만 존재 | 직접 비교 불가 | 정보 부족 | U dynamics 없음; D Drive |
| 서스펜션 | 공식 rocker arm 설명 | 서스펜션 DOF 없음 | 접지/요철 응답 미재현 | 수정 필요 | S1/S4; U joint tree; 평지 개발에서는 명시적 단순화 가능 |

## 8. Ackermann 기구학 검증

### 8.1 축·부호·직선 주행

URDF 전륜 조향 local axis `(0,1,0)`에 origin 회전 `Rx(1.5708)`을 적용하면 base axis는 약 `(0,-3.6732e-6,1)`이다. 양쪽 모두 수직 +Z이고 양의 각도로 좌회전한다. wheel 회전축은 zero pose에서 ±Y로, +X 구름 방향과 직교한다. 조향각 0에서 직진 가능한 기구학 구조다. 오른쪽 후륜의 작은 origin 회전은 axle의 전후 방향 오차를 약 10⁻⁸ 수준으로만 만든다.

독립 조향 probe는 한쪽 조향 joint만 변할 때 다른 앞바퀴가 움직이지 않는 것을 검증한다. 이는 linkage가 없다는 뜻이며 rear wheel을 가리키는 표현이 아니다. 두 조향 joint에 +30°를 주면 두 바퀴가 모두 왼쪽을 향하는 부호는 정상이지만, 유한 반경에서 정확한 아커만 각도 조합은 아니다.

current USD의 rear 두 축은 모두 +Y이다. URDF는 left rear=-Y, right rear=+Y이므로 URDF 기반으로 새로 import한 모델에서는 `left=-ω, right=+ω`가 필요하거나 재현 가능한 축 교정이 필요하다. 현재 USD를 제어하면서 이 부호 보정을 다시 적용하면 오히려 반대로 돈다.

### 8.2 각도와 회전반경 계산

평면·강체·무미끄럼 근사. L=0.65151 m, 전륜 pivot 간격 Tₛ=0.584 m, R=후륜축 중앙에서 ICR까지의 거리. 좌회전 R>0에서:

```text
R = L / tan(δcenter)
δleft  = atan(L / (R − Tₛ/2))   # 안쪽
δright = atan(L / (R + Tₛ/2))   # 바깥쪽
cot(δouter) − cot(δinner) = Tₛ/L
Rmin = L / tan(δinner,max) + Tₛ/2
```

우회전은 안쪽/바깥쪽 좌우를 교환하고 두 조향각 부호를 음으로 한다. 같은 각도로 1:1 mimic하는 것으로 위 비선형 관계를 정확히 만들 수 없다. 현재 독립 joint 두 개는 이 관계를 **만족하도록 명령할 수 있는 구조**지만 자동으로 만족시키지는 않는다.

| 조건 | 내측각 (rad) | 중심각 (rad) | 외측각 (rad) | 후륜중앙 R (m) | 외측 전륜 pivot 궤적반경 (m) |
| --- | --- | --- | --- | --- | --- |
| 현재 개별 limit | 0.580000000 | 0.468797136 | 0.391459378 | 1.286416018 | 1.707589648 |
| 현재 기하 + 공식 33° | 0.575958653 | 0.466048164 | 0.389497724 | 1.295237423 | 1.715747043 |
| 공식 명목 L/T + 33° | 0.575958653 | 0.462599894 | 0.384598245 | 1.303412226 | 1.732470513 |

공식 1.6 m를 후륜중앙 반경으로 **가정할 경우에만** 현재 1.286416 m와 차이는 −0.313584 m(19.599%)다. 외측 전륜 pivot 기준으로 가정하면 1.707590 m와 +0.107590 m(6.724%)다. 공식 기준점이 불명하므로 두 수치를 “실차 오차”로 확정할 수 없다. tire 중간면 궤적이나 차체 외곽 sweep 반경은 pivot 반경과도 다르다.

공식 최대 33°는 **개별 내측 바퀴 각도**이며 중심 bicycle steering 33°가 아니다. driver의 중심 상한 0.461 rad를 현재 기하에 적용하면 R=1.311691 m, 내측 0.568553 rad, 외측 0.385888 rad다. 현재 개별 limit보다 보수적이다. δ→0일 때 R→∞이며 아커만 차량에는 유한한 “최대 회전반경”이나 제자리 회전 능력이 없다.

### 8.3 후륜 속도와 Isaac Sim 제어기 연결

후륜 접지 중심 간격의 초기 근사 Tᵣ=0.604 m, 반지름 r를 사용하면 좌회전 시:

```text
ωleft  = (v/r) × (1 − Tᵣ/(2R))
ωright = (v/r) × (1 + Tᵣ/(2R))
```

후륜 두 wheel에 같은 속도를 강제하면 정확한 무미끄럼 회전과 맞지 않는다. 앞 wheel은 구름에 따라 자유 회전시키고, rear 두 개만 velocity 제어한다. 안쪽/바깥쪽 rear 속도를 따로 계산하고 최대속도를 넘을 때 공통 비율로 낮추면 곡률을 보존할 수 있다.

[Isaac Sim 5.0 Ackermann Controller](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/py/source/extensions/isaacsim.robot.wheeled_control/docs/ogn/OgnAckermannController.html)는 좌우 조향각과 4개 wheel 속도를 출력한다. 실제 설치된 `isaacsim.robot.wheeled_control/.../controllers/ackermann_controller.py`와 `.../nodes/OgnAckermannController.py`도 읽었다. 적용할 때 다음 제한을 고려한다.

- 조향 출력 2개를 steering joints에, wheel 출력 중 rear 2개만 rear joints에 연결한다. 이름/DOF 순서를 명시하고 front passive wheel에 drive 명령을 넣지 않는다.
- `invertSteering=False`. node/Python 제어는 rad와 rad/s를 사용한다. USD Property 창의 degree 표기와 혼합하지 않는다.
- 설치된 구현은 입력 중심각과 출력 개별각을 같은 `max_wheel_rotation_angle`로 clamp한다. 중심 상한과 개별 상한을 외부에서 분리 제한해야 최대 조향에서 관계가 깨지는 것을 피할 수 있다.
- 설치된 구현은 `abs(steering_angle)<0.0157 rad`에서 직진 처리하고, wheel 속도를 각각 clip한다. 미세한 경로 추종 및 포화 시 회전반경에 영향을 줄 수 있다.
- 한 개 track을 조향과 속도 분배에 공통 사용한다. 현 모델의 pivot track 0.584 m와 후륜 유효 중심 간격 약 0.604 m를 모두 정확히 표현하지 못한다. 초기에는 단순화 오차를 명시하고, 정밀 추종에는 두 간격을 분리한 변환이 권장된다.
- standalone controller source를 읽었을 뿐 실제 node 실행, DOF 매핑, 방향, 응답은 시험하지 않았다.

## 9. URDF에서 정의된 값과 USD/제어에서 필요한 값

| 항목 | 현재 URDF | 현재 USD / 별도 설정 책임 |
| --- | --- | --- |
| 링크·관절·축 | 정의됨 | import + 후륜 left 수동 부호 override; 재현 방법 필요 |
| Mass/inertia | 정의됨, 하드웨어 보정 전 | fixed merge 및 principal axes 저장; 실차 일치 검증 별도 |
| 각도/effort/속도 limit | steer/rear 정의됨 | 단위 변환되어 저장; 실제 제한·target 포화 확인 필요 |
| drive stiffness/damping/type | 미정의 | USD에서 설정; 액추에이터 tuning 필요 |
| Collision 형상 | STL | 현재 convexHull; 접촉 품질 시험 필요 |
| 마찰/반발/합성 규칙 | 미정의 | Physics Material과 타이어/바닥 바인딩 별도 필요 |
| Articulation Root, solver | URDF 고유 설정 아님 | current root/solver 저장됨; world PhysicsScene 별도 |
| 바닥/중력/초기높이 | 로봇 URDF 범위 밖 | world 정의·의존성 해석·초기 관통 검사 필요 |
| 아커만·모터/기어·rate limit | 제어 로직 없음 | joint target 변환·토크/rate 제한 별도 구현 |
| 경로·자세 피드백 | 없음 | planner/tracker·odometry 및 frame 정의 필요 |

## 10. 수정 권장사항과 우선순위

이번 감사에서는 아래 항목을 **적용하지 않았다**. “필수”는 해당 목적 달성 전에 해결해야 한다는 뜻이며, 모두 해결해야 파일을 열 수 있다는 뜻은 아니다.

| 우선순위 | 항목 | 이유 / 완료 판단 기준 |
| --- | --- | --- |
| 필수 — 경로 추종 전 | 사용할 USD·joint 부호를 고정하고 재현 절차 기록 | current USD와 URDF 재import 결과가 다름; 동일 command의 forward 방향 확인 |
| 필수 — 경로 추종 전 | 독립 좌우 조향각·후륜 속도 분배, rad/degree 변환, DOF 매핑 | 같은 각도/같은 후륜속도는 정확한 아커만을 보장하지 않음 |
| 필수 — 실차 속도 재현 전 | URDF/USD/YAML의 최고속도 및 wheel limit 정책 통일 | 1 rad/s와 1.5 m/s 모순; 명목 직진 9.09 rad/s이나 급히 상향하지 말고 응답 검증 |
| 필수 — full-range 전 | 33° 내측 / 0.461 중심 / 0.58 개별 정책 구분 | 현재 개별 상한은 매뉴얼보다 약 0.00404 rad 큼 |
| 필수 — 재현 가능한 물리 평가 전 | 타이어·바닥 Physics Material, ground 및 PhysicsScene 명시 | 기본값/원격 의존성에 따른 유효 마찰과 접촉을 통제 |
| 필수 — 하드웨어 동역학 비교 전 | 실물 버전별 질량·CoM·관성 모델 재작성 | 52.23 kg를 실제 65/70 kg와 동일 모델로 평가하면 안 됨; knuckle/wheel 중복 검토 |
| 권장 | 로봇 USD와 world 분리, 의존 configuration 함께 관리 | root에 환경·render·위치 override가 혼합되어 있음 |
| 권장 | convex hull wheel 접촉과 drive gain 시험 후 필요 시 단순 collision 사용 | 원통/근사 hull은 계산 안정성 후보이나 단위·폭·축 보존 필요; 현재 문제라고 확정하지 않음 |
| 권장 | vendor 복사본에 맞는 검증기 수정 | git parent HEAD 때문에 기본 validator 실패; 파일 잠금 검증 유지 |
| 권장 | 후륜축 중앙을 tracking 기준점으로 명시 | base 원점과 다름; offset 없이 bicycle 식에 넣으면 추종 위치 오차 발생 |
| 추후 보정 | rolling radius, friction, torque curve, 기어비, steering rate/latency | 실물/벤치 측정 필요; 임의값을 제조사 스펙으로 쓰지 않음 |
| 추후 보정 — 요철 평가 전 필수 | rocker/suspension 및 payload 모델 | 평지 강체 근사 한계를 넘는 접지·하중 연구 시 필요 |

## 11. Waypoint 주행 권장 초기 파라미터

아래는 **[제안] 저속 개발용 초기값**이며 검증된 튜닝값 또는 실차 제원이 아니다. 현 후륜 1 rad/s 제한을 유지하는 단계에 맞춘다. 숫자만 추가하면 제어기가 생성되는 것은 아니다.

| 파라미터 | 초기 후보 | 근거 / 제한 |
| --- | --- | --- |
| wheelbase | 0.65151 m | 현재 joint 기하; 명목 실차 .650과 구분 |
| steering pivot track | 0.584 m | 현재 조향축 위치 |
| rear rolling-center track | 약 0.604 m | mesh 중간면 추정; 접지 중심 보정 대상 |
| front/rear radius | 0.164158 / 0.164275 m | mesh 기반 초기값; 실측 rolling radius 아님 |
| 직진 속도 | 0.05–0.10 m/s | 현재 1 rad/s 제한에서 저속 시작 |
| rear wheel speed cap | 1 rad/s 유지 | 회전 시 outer wheel 포함, 공통 비율 포화 |
| 중심 조향 운전 범위 | ±0.30 rad로 시작 | 초기 급회전 회피; 실차 상한 아님 |
| 개별 조향 hard cap | ±0.575959 rad 후보 | S1 내측 33° 기준; 현재 USD .58과 정책 정합 필요 |
| 중심 조향 최종 명목 cap | ±0.461 rad 후보 | S5; 개별 제한과 함께 적용 |
| 조향 rate | 0.2 rad/s 후보 | 개발용 slew rate; 실차 최대속도 아님 |
| 선가속도 | 0.1 m/s² 후보 | 개발용 제한 |
| control / physics | 50 Hz / 120 Hz 후보 | 시뮬레이션 시간 dt 사용; 현재 실행 설정을 읽어 확정한 값 아님 |
| Pure Pursuit lookahead | 0.7 m 후보 | 속도·경로 간격 따라 조정; 짧은 경로 코너 절삭 확인 |
| goal 허용 위치/방향 | 0.10 m / 0.10 rad 후보 | 작업 요구에 맞춰 변경 |
| tracking frame | rear axle center ≈ (−0.28009, −0.00001) m in base | bicycle model 기준; base pose를 그대로 쓰지 않음 |
| planner footprint | 현재 약 0.980 × 0.746 m 및 steering sweep | 추가 여유 0.10 m는 개발 가정; 임무별 보정 |
| friction/restitution | 실측 전에는 명시적 시험값과 출처를 별도 기록 | 현 Asset 값/실차 값으로 임의 확정하지 않음 |

±0.30 rad 중심각에서 R=L/tan(0.30)≈2.106 m이므로 planner도 이 단계의 곡률 제한에 맞춰야 한다. 최고 조향 능력으로 경로를 만든 뒤 tracker만 ±0.30으로 제한하면 따라갈 수 없다. 전진 경로는 곡률 제한을 반영한 Dubins/Hybrid A* 등, 후진이 필요하면 후진 가능한 모델을 검토한다. 지점에서 제자리 yaw 회전을 요구하는 waypoint 연결은 피한다.

후속 실행 검증 순서(이번에는 미실행): (1) 무명령 안착과 관통 확인 → (2) 직진 0.05 m/s 및 부호 확인 → (3) 조향 양/음 각각 일정 원 주행 → (4) 측정 R·yaw rate·wheel slip과 계산 비교 → (5) waypoint 직선/완만한 곡선 → (6) gain·limit·질량 보정 후 속도 확대. 각 단계에서 pose, joint position/velocity, target, applied effort를 기록해야 토크 포화와 접촉 문제를 구분할 수 있다.

## 12. 추출·검증 재현 메모와 한계

계산법:

```text
URDF joint transform: T_parent_child(q) = T_origin · R(axis_local, q)
base axis = R_base_parent · R_origin · axis_local
wheelbase = mean(front joint x) − mean(rear joint x)
AABB = min/max over all transformed active STL vertices
wheel radius = max sqrt(x_local² + y_local²)  # 해당 mesh의 wheel axis는 local Z
wheel width = max(z_local) − min(z_local)
clearance candidate = lowest(non-wheel collision z) − lowest(wheel collision z)
mass = Σ link mass
CoM = Σ mass_i · transformed local CoM_i / total mass
inertia validity: eigenvalues > 0, each principal moment ≤ sum(other two)
USD joint anchor check: T_body0 · localPos0 == T_body1 · localPos1
```

URDF XML 및 STL 수치는 기존 비교 코드의 파싱/변환 함수를 읽은 뒤 실제 파일에서 다시 계산했다. USD는 `Usd.Stage.Open`과 `UsdGeom.XformCache`/`UsdPhysics`를 사용해 읽었다. GUI 세션의 저장하지 않은 변경은 이 보고서의 범위 밖이다. 실제 PhysX contact manifold, 마찰 결합, 출력 torque, 관절 속도 limit 집행, controller 동작은 실행 검증을 거치지 않았다. 공식 자료에 없는 값은 실물 특성으로 추정하지 않았다.

### 12.1 감사 대상 주요 파일 SHA-256

| 파일 | SHA-256 |
| --- | --- |
| `assets/hunter2/source/hunter2_sim.urdf` | dd089214f9968b2d47fbf7fa90e3300c3d404dfcb8c814cd5913ecf95e60a93f |
| `assets/hunter2/usd/hunter2_sim.urdf/hunter2_sim/hunter2_sim.usd` | aae0d79aff1312c026d276ac78a1d6b7c009111dd7224bdca7ea7b971b35ab11 |
| `assets/hunter2/source/hunter2_sim/hunter2_sim.usd` | 83db36a265414beaf9a2d24240a2e211c76af1c6ed2513f5cfe386e48374bd79 |
| `assets/hunter2/config/hunter2.yaml` | d0c1d2b348186ba34cf17f5511083b0f0fdfe61ea474025b5751d8df3cb32784 |

본 감사에서 추가한 프로젝트 산출물은 이 Markdown 보고서 하나다. 기존 모델/설정은 그대로 두었다. 보고서는 하드웨어 제원 일치와 동작 가능성을 구분하며, 사용자에게 관찰된 주행 성공을 이번 감사의 물리 시험 통과로 대체하지 않는다.
