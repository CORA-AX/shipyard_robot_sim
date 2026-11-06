# Hunter 2.0 주행 개발

Hunter 전용 모델, Ackermann 제어, ROS2 연결, 경로 계획·추종을 이 폴더에서 관리한다.
실행 파일은 **`scripts/`**, 실제 구현은 역할별 Python 모듈에 둔다.

- [실행 방법](docs/usage.md): 준비, 터미널별 명령, RViz 조작, 취소, 시간 cost 확인.
- [파일 안내](docs/files.md): 각 파일의 역할, 알고리즘, 사용하는 라이브러리.

## 실행 모드

| 실행 파일 (`scripts/` 아래) | 목적 | 사용 방식 |
| --- | --- | --- |
| `simulate.py` | 시뮬레이터 실행 | 저장된 USD + Isaac Sim/IsaacLab |
| `teleop.py` | 수동 운전 | 키보드 → ROS Ackermann 명령 |
| `waypoint.py` | 자체 단일 목표 주행 | 터미널 좌표 → 자체 Dubins → Pure Pursuit |
| `nav2.py` | RViz 클릭 목표 주행 | 위치/자세 입력 → Smac Hybrid-A* → RPP |
| `patrol.py` | 저장 경로 순차 왕복 | waypoint 목록 → 같은 Nav2 제어 → 정지·다음 지점 |
| `export_map.py` | 정적 지도 생성 | OpenUSD 충돌 형상 → 2D occupancy grid |

명령은 프로젝트 루트에서 실행한다. 예전 `hunter/nav2.py` 등의 실행 파일은 `hunter/scripts/`로 이동했다.
시뮬레이터와 운전/내비게이션 실행기는 별도 터미널에서 실행한다. 차량 명령 발행기는 한 번에 하나만 사용한다.

## 구조

```text
hunter/
├── scripts/                 실행·설치·ROS 환경 설정
├── control/                 Ackermann 기구학·명령 검사·키보드
├── navigation/
│   ├── custom/              자체 Dubins·Pure Pursuit
│   ├── nav2/                Nav2 설정·도착 방향 후보 계산
│   ├── state.py             공통 평면 위치·속도·곡률 적분
│   ├── routes.py            저장 경로 검증·순찰용 여유 폭
│   └── maps.py              지도 읽기·차량 영역 검사
├── ros/                     ROS 메시지·odometry·수동 운전
│   └── nav2/                Nav2 목표·순찰·프로세스·RViz 관리
├── sim/                     USD 배치·물리 루프·관절 목표 적용
├── environments/            환경 프로필·지도 생성
├── config/                  차량·제어기·환경·waypoint 설정
├── assets/                  운용 로봇 USD·환경 참조 레이어
├── maps/                    생성된 지도와 원본 해시
├── tools/                   의존성 설치 내부 도구
├── tests/                   CPU·USD·ROS 연결 검사
└── docs/                    사용법·파일 설명
```

## 현재까지 구현한 것

1. **모델·구동**: 운용 USD 사본, 네 바퀴의 숨겨진 원통 collider, 앞바퀴 조향·후륜 속도 제어.
2. **차량 기준**: 위치·속도 기준을 후륜 축 중앙으로 통일. 차체 원점은 `chassis_link`로 구분.
3. **수동 운전**: ROS2 `AckermannDriveStamped`, 키보드 전후진·조향, 명령 수신 만료 시 정지.
4. **자체 경로 추종**: 직접 구현한 전진 Dubins 경로와 Pure Pursuit 단일 목표 주행 유지.
5. **Nav2 주행**: RViz 위치/자세 목표, Ackermann 제약을 반영한 Smac Hybrid-A*와 RPP.
6. **조선소**: 기존 로컬 환경 참조, 지지면·정적 충돌체에서 지도 생성, 저장 경로 순차 왕복.
7. **시간 cost**: 목표별·순찰 전체의 시뮬레이션 경과 시간 기록. 도착 후 실제 정지도 확인.

주행 흐름은 `목표 입력 → 경로 계획·추종 → 속도·조향 명령 → Ackermann 제어 → 관절 목표`다.
`patrol.py`는 방문 순서를 관리하며 Nav2를 재사용한다. 별도의 계획 알고리즘은 추가하지 않는다.

## 현재 범위와 외부 의존성

- 자체 waypoint 및 Nav2 자동 주행은 전진 방식이다. 수동 운전은 후진도 지원한다.
- 위치는 Isaac ground truth를 사용한다. SLAM·센서 기반 위치 추정·움직이는 장애물 인지는 포함하지 않는다.
- 조선소 자동 주행은 연결된 평지 영역이 대상이다. 경사로·계단 주행은 검증 범위에 포함하지 않는다.
- 위치 전용 목표는 유한한 도착 방향 후보 중 경로 거리를 비교한다. **시간 최소화 최적화는 다음 단계**다.
- Isaac Sim 5.0 Conda, ROS2 Humble, 설치된 IsaacLab을 사용한다. `third_party/IsaacLab`은 실행 의존성이므로 유지한다.
- 다른 로봇의 제어 모듈은 import하지 않는다. 공용 `assets/environments/shipyard/`는 읽기 전용으로 참조한다.
  `hunter/`를 다른 저장소로 옮기면 이 환경 에셋도 준비해야 한다.
- `.runtime/`, `outputs/`, 캐시는 실행 산출물이며 Git에서 제외한다.
