# HUNTER 2.0 소스 교정 결과

작업일: 2026-10-06 (Asia/Seoul).
**수정 제안만 남긴 단계에서 실제 파생 URDF 교정 단계로 진행했다.**
AgileX 원본을 기반으로 조향축과 중복 형상을 수정하고 정적 검증을 완료했다.
Isaac Sim import, USD 생성, 접촉/주행 검증 및 controller 구현은 아직 하지 않았다.

## 실제 파일에 적용한 내용

입력: `assets/hunter2/source/original/hunter2_base.urdf`와 고정 vendor.
출력: [hunter2_sim.urdf](../assets/hunter2/source/hunter2_sim.urdf).
생성 규칙은 [prepare_hunter2_asset.py](../scripts/prepare_hunter2_asset.py)에 반영했다.

| 항목 | 교정 전 | 교정 후 |
|---|---|---|
| 왼쪽 조향 local axis | `0 0 1` | `0 1 0` |
| 오른쪽 조향 local axis | `0 0 -1` | `0 1 0` |
| base 기준 조향축 | 좌우 방향 ±Y 근처 | 양쪽 모두 수직 +Z 근처 |
| steering link visual | 바퀴 포함 mesh | 제거 |
| steering link collision | 바퀴 포함 mesh | 제거 |
| wheel link visual/collision | 각 바퀴가 별도 소유 | 그대로 유지 |
| mesh 참조 / 참조 STL | 18 / 9 | 14 / 7 |
| 보관 STL 수 | 9 | 9 (원본 그대로) |
| link / joint 수 | 9 / 8 | 9 / 8 |
| 총 질량 | 52.2274695954 kg | 동일 |

조향 link 두 개는 **관절 연결과 inertial을 가진 무형상 frame**으로 남는다.
전체 STL을 편집한 것이 아니라 해당 link의 visual/collision 요소만 제거했다.
joint 이름, parent/child, origin xyz/rpy, wheel 축, 범위 ±.58, 속도·effort는 유지했다.
모든 inertial(mass/COM/tensor), 원본 snapshot, vendor와 STL 파일도 보존했다.

이 구조는 AgileX Gazebo/LCAS의 올바른 수직 조향과 LCAS의 collider 소유 분리를 참고했다.
LCAS의 DAE, ros2_control, controller, 1:1 mimic 및 질량값은 복사하지 않았다.
기존의 package URI 제거와 ±.58 조향 범위 해제는 계속 적용된다.

[이번 변경만의 실제 diff](hunter2_source_correction_evidence/applied_source_changes.diff)
에는 조향축 2곳과 visual/collision 블록 4개의 변경이 기록돼 있다.
이전 비교 보고서의 axis-only patch는 과거 제안이며 현재 파일에 다시 적용할 필요가 없다.

## 어떻게 검증했나

[validate_hunter2_asset.py](../scripts/validate_hunter2_asset.py)는 조향축 문제를
경고로 넘기던 기존 동작을 수정해 다음 조건을 필수 검사한다.

1. 모든 부모 joint의 origin 회전을 누적하고, 조향축이 base의 +Z를 향하는지 검사한다.
2. 바퀴 axle이 영점에서 차체 좌우 방향인지 검사한다.
3. 각 조향 joint를 독립적으로 −10°, 0°, +10° 움직이는 회전행렬을 계산한다.
4. 바퀴 axle과 지면 법선의 외적으로 진행 방향을 구해 목표 조향각과 비교한다.
5. 반대쪽 바퀴의 heading은 0으로 유지되는지 검사한다.
6. 조향 link의 visual/collision 부재, wheel geometry 소유권, 관절 연결, mimic 부재를 검사한다.
7. 원본과 허용 변경 목록만 다르며 모든 inertial·origin·wheel axis가 보존됐는지 테스트한다.

| 독립 조향 입력 | 왼쪽 시험의 왼쪽 heading | 오른쪽 시험의 오른쪽 heading | 각 시험의 반대쪽 heading |
|---:|---:|---:|---:|
| −10° | 약 −10° | 약 −10° | 0° |
| 0° | 0° | 0° | 0° |
| +10° | 약 +10° | 약 +10° | 0° |

원본 roll=1.5708을 유지하므로 최종 축은 약 `(0, -3.6732e-6, 0.99999999999)`다.
회전 계산은 이 반올림 오차를 허용한다. source validator의 heading 허용오차는 1e-6 rad다.
별도 비교 스크립트의 ±.3 rad 샘플과 kinematic review의 ±.58 범위 샘플도 확인했다.

기존 steering/wheel 중복 triangle은 좌 36,470개, 우 19,714개였고,
현재는 visual/collision 양쪽에서 각각 0개다. 이는 **해당 조향 link와 wheel 사이의 중복**
검사 결과다. 차체와 다른 부품의 모든 가능한 교차·접촉까지 검사한 것은 아니다.

테스트 **28개 통과**. 잘못된 축·부호, 수직 wheel axle, 조향을 우회한 wheel 연결,
중복 collider 재삽입, wheel collider 누락, 1:1 mimic을 넣으면 실패하는 회귀 테스트를 포함한다.
반복 재생성, 다른 폴더로 옮긴 bundle 검증, vendor 보존도 확인했다.

## 이 교정으로 해결하지 않은 것

- 조향 mesh의 비바퀴 부분도 함께 생략될 수 있다. 필요하면 순수 knuckle 등 외형을 별도로 복원한다.
- 형상을 제거했지만 질량·관성은 그대로다. 실제 부품별 inertial로 검증되지 않은 진단용 상태다.
  모델 총 질량도 실차 65/70 kg과 다르다. importer의 자동 재계산 여부를 확인해야 한다.
- 질량/관성 0인 fixed `rear_wheel_link`의 import/병합 정책은 미확정이다.
- 후륜 velocity=1 rad/s는 유지했다. nominal 최고속도 1.5 m/s를 지원하는 모델이 아니다.
- 기존 ±.58 rad는 **driver 기반 임시 범위**로 유지했다. manual inner 33°와 약 .23155° 차이가
  있으므로 정적 관절 확인 이후 전체 조향 범위 주행 전에는 제한 정책을 확정한다.
- 좌우 wheel axis 부호가 다르다. controller가 전진 부호와 좌우 선회 속도를 구분해야 한다.
- YAML nominal geometry와 URDF joint-origin geometry 차이는 유지했다.
- 독립 조향 운동만 검증했다. 좌우 Ackermann 연동, 마찰, 접지, 안정된 주행은 미검증이다.

따라서 이번 완료 판정은 **교정된 source로 진단 import를 진행할 수 있음**이다.
최종 실차 동역학 모델 또는 주행 검증이 끝난 asset으로 freeze한 것이 아니다.

## 재현 및 산출물

프로젝트 루트에서 실행한다. source 재생성은 아래 명령을 사용한다.

```bash
python3 scripts/prepare_hunter2_asset.py
python3 scripts/validate_hunter2_asset.py
python3 -m unittest discover -s tests -v
python3 scripts/compare_hunter2_models.py --output-dir docs/hunter2_source_correction_evidence/comparison
python3 scripts/review_hunter2_kinematics.py
```

- [현재 source 검사 결과](../assets/hunter2/config/validation_report.json)
- [교정 전 파일 해시](hunter2_source_correction_evidence/before_sha256.json)
- [보존·반복 생성·테스트 확인 기록](hunter2_source_correction_evidence/checks.json)
- [교정 후 4개 모델 비교 JSON](hunter2_source_correction_evidence/comparison/comparison.json)
- [교정 후 joint·inertial·geometry 상세표](hunter2_source_correction_evidence/comparison/model_details.md)
- [별도 kinematic review](hunter2_source_correction_evidence/kinematic_review.json)

교정 후 comparison의 `proposed_steering_axis.patch`는 비어 있다. 축 수정이 이미 반영돼
추가 제안이 없다는 뜻이다. `docs/hunter2_comparison_evidence/`의 과거 증거는 덮어쓰지 않았다.
이전 보고서의 Current 열과 테스트 19개 기록은 교정 전 상태로 구분한다.

다음은 [USD import 계획](hunter2_usd_import_plan.md)에 따라 Isaac 버전을 선택하고,
변환된 관절·collision·질량을 확인하는 단계다. 처음에는 작은 조향과 제한된 속도로 시험한다.
