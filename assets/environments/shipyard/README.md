# Shipyard and Dry Dock

원본 GLB의 도크·선체·장비 배치를 유지한 로컬 USD 환경이다.
원본: https://3dassets.dev/packs/shipyard-and-dry-dock
입력 파일과 SHA-256은 `sources.lock.json`에 기록한다.

## 실행

프로젝트 루트에서 실행한다. 기존 기본 시나리오는 warehouse이며,
조선소는 `--scenario shipyard`로 선택한다.

```bash
# 로봇 없이 로컬 창
.conda/isaacsim-5.0/bin/python scripts/viewer.py --scenario shipyard --no-webrtc

# WebRTC (서버 IP를 실제 주소로 변경)
.conda/isaacsim-5.0/bin/python scripts/viewer.py --scenario shipyard --webrtc --server-ip <서버_IP>

# 기존 Go2 제어기 연결: 첫 터미널
.conda/isaacsim-5.0/bin/python scripts/viewer.py --scenario shipyard --robot go2 --no-webrtc
# 두 번째 터미널
.conda/isaacsim-5.0/bin/python scripts/go2/control.py
```

환경 로더는 `scenarios/shipyard/scene_cfg.py`다. Go2 시작 위치는
`(-16, 18, 0.4)`m이며 왼쪽 육상 바닥 위다. Hunter2 모델과 ROS/Nav2의
기존 warehouse 지도·실행 설정은 이 시나리오와 별개다.

## 서비스 도로·경사로·계단

`--scenario shipyard` 실행 시 `routes.usda`가 함께 로드된다.
기존 조선소 서쪽에 정비·물류용 서비스 구역을 확장했다. 기존 선박·도크·설비,
로봇 시작 위치, 제어기와 다른 시나리오 파일은 변경하지 않는다.
`world.usda`에 이미 있던 동쪽 warehouse 참조도 그대로 유지한다.

| 구역 | 위치와 치수 (m, 기존 환경의 Z-up 좌표) |
| --- | --- |
| 진입 연결 | 시작점 `(-16,18,0)`에서 서쪽으로 이동, 원래 apron의 `x=-18`에 단차 없이 연결 |
| 평지 우회로 | 중심 `x=-25`, 폭 6, 남북 교차로 중심 `y=-20 / 18` |
| 북쪽 경사로 | `x=-40…-34`, `y=14→2`, 높이 `0→1.2`, 길이 12, 경사 10% (약 5.71°) |
| 작업 데크 | `x=-40…-34`, `y=-4…2`, 높이 1.2, 크기 6×6 |
| 남쪽 경사로 | `x=-40…-34`, `y=-4→-16`, 높이 `1.2→0`, 길이 12, 경사 10% |
| 계단 | 동쪽 `x=-30.8`에서 데크 `x=-34`로 상승, 중심 `y=-1`, 폭 2.4, 8단; 단높이 0.15 / 디딤폭 0.4 |

경사로·데크·계단에 노란 난간을 설치하고, 도로에 차선·횡단보도·계단 끝 표시,
외곽에 낮은 연석을 배치했다. 표시선에는 충돌체를 넣지 않았다.
새 지면은 유한한 서비스 구역에만 있으며 기존 드라이 도크와 수면을 덮지 않는다.
경사로는 실제 경사면을 가진 정적 삼각형 메시, 계단은 각 단별 정적 충돌체다.
새 충돌체의 정지/동마찰은 0.9/0.8, 반발계수는 0이다. 로봇 접촉 재질에 따라
실효 마찰과 등판 성능은 달라질 수 있다.

Isaac Sim에서 전체 배치는 `/World/ServiceRoadOverview` 카메라로 볼 수 있다.
viewer가 환경을 참조로 로드하면 해당 환경 prim 아래 같은 이름의 카메라가 있다.
기존 Overview 및 viewer 기본 카메라는 변경하지 않았다.
추가 구역 편집은 Layers에서 **`routes.usda`를 Edit Target으로 선택**하고 저장한다.

### 공개 에셋 출처

- [Shipyard and Dry Dock](https://3dassets.dev/packs/shipyard-and-dry-dock):
  기존 로컬 원본의 배관 랙, 컨테이너, 소화 설비, 조명탑을 추가 구역에 재배치했다.
  배포 페이지는 **CC0 1.0 Universal**로 명시한다(2026-11-02 확인).
  원본 prim 경로·원본 GLB SHA-256은 `routes.sources.json`에 기록했다.
- 도로·경사로·계단·난간·차선은 위 배치와 치수에 맞춰 자체 생성한 USD 형상이다.
  NVIDIA 공식 에셋으로 표시하거나 제공하지 않는다.
- [NVIDIA 공식 SimReady Warehouse 01](https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html)에도
  경사판·발판 등이 있다. 공개 팩 용량이 13.9GB이므로 이번에는 기존 조선소와
  재질·규모가 맞는 로컬 공개 에셋을 재사용했다. 새 원격 의존성은 추가하지 않았다.

### 이후 waypoint 연결

`route_hints.json`은 환경용 **경로 후보 데이터**다. 자동 주행이나 Nav2 연동은 아직 하지 않는다.

- `nodes`: `[x,y,z]`는 **지지면 높이**다. 로봇 base/spawn 높이는 각 로봇에 맞게 더해야 한다.
  좌표는 `world.usda`의 `/World` 기준이며, 환경 자체를 이동/회전하면 함께 변환해야 한다.
- `edges`: 양방향 연결, 폭, `flat` / `ramp` / `stairs` 구분 및 요구 능력을 기록한다.
  바퀴형 로봇은 계단 간선을 제외하고 등판 능력이 확인된 경우만 경사로를 사용한다.
- `suggested_routes.flat_only`: 평지 경로. `service_loop`: 경사로를 포함한 순환 경로.
  `stair_access`: 계단 접근 경로로 각 디딤판 중심 좌표를 포함한다.
- 계단 좌표를 직선 보간해서 경사면처럼 주행시키면 안 된다. 계단 보행 제어가 별도로 필요하다.
  로봇별 footprint, 회전 반경, 높이·경사 제한을 반영해 경로를 생성해야 한다.
  기존 warehouse용 2D 지도는 이 확장 구역이나 높이 변화를 표현하지 않는다.

### 추가 구역 생성·검사

```bash
# 별도 경로에서 재생성해 비교 (이 명령은 world/layout/physics를 수정하지 않음)
.conda/isaacsim-5.0/bin/python scripts/environments/build_shipyard_routes.py --output-dir /tmp/shipyard-routes

# 추가 구역 3개 파일만 재생성: 수동 편집한 routes.usda가 있다면 먼저 보관
.conda/isaacsim-5.0/bin/python scripts/environments/build_shipyard_routes.py --replace

# CPU: 실제 world 구성에서 지지면, 경로 여유 공간, 충돌·재질, 재생성 일치 검사
.conda/isaacsim-5.0/bin/python tests/check_shipyard_routes.py

# GPU: 시작점·연결부·우회로·양쪽 경사로·데크·계단에서 낙하 테스트 및 RTX 캡처
.conda/isaacsim-5.0/bin/python tests/check_shipyard_routes_sim.py
```

CPU 검사는 원래 조선소와 새 구역을 함께 구성하되, 기존 원격 warehouse는
population mask로 제외한다. 간선마다 약 15cm 간격, 좌우 65cm와 중앙에서
지지면 및 높이 1.5m 이내 장애물을 검사한다. 이는 경로 중심부의 기하 검사이며,
차량의 회전 궤적 검증이나 로봇의 실제 등판·계단 보행 성공을 의미하지 않는다.
GPU 검사는 실제 `world.usda` 전체를 로드하므로 기존 warehouse 원격 참조에 접근할 수 있어야 한다.
출력은 `outputs/shipyard/routes_report.json`, `routes_simulation_report.json`,
`routes_preview.png`, `routes_detail.png`다.
별도 출력 폴더의 USD를 렌더링하려면 기존 `textures/`도 같은 폴더 아래에 복사한다.

## 환경을 계속 꾸미고 저장하기

| 파일 | 역할 | 직접 편집 |
| --- | --- | --- |
| `source/model.glb` | 다운로드 원본 | 보관 |
| `models/shipyard.usd` | 생성한 모델·재질·오브젝트 계층 | 재생성 가능하므로 원본 취급 |
| `textures/image_0.png` | 원본 WebP를 PNG로 변환한 콘크리트 텍스처 | 생성 파일 |
| `layout.usda` | 모델 참조, 배치, 조명, Overview 카메라 | 배치 작업 저장 |
| `physics.usda` | 정적 충돌체와 마찰 설정 | 물리 작업 저장 |
| `routes.usda` | 추가 도로·경사로·계단·공개 설비 및 해당 충돌·재질 | 추가 구역 편집 |
| `route_hints.json` | 높이·주행 능력을 구분한 waypoint 후보 연결 | 배치 변경 시 함께 갱신 |
| `routes.sources.json` | 추가 구역의 제작·공개 에셋 출처 | 출처 기록 |
| `world.usda` | layout·physics·routes를 조합하는 진입점 | 레이어 구성 |

1. Isaac Sim 편집기에서 **`world.usda`를 직접 연다**.
2. Layers 창에서 **`layout.usda`를 편집 대상(Edit Target)으로 선택**한다.
3. 기존 장비를 이동하거나 새 USD 에셋을 Reference로 추가한다.
   새 모델은 이 폴더의 `models/` 아래에 넣고 상대 경로로 참조한다.
4. **수정한 레이어를 저장한다(Save All Layers).** 루트 파일만 저장하는 것으로
   다른 레이어의 변경이 저장됐다고 가정하지 말고 dirty 표시가 사라졌는지 확인한다.
5. 다음 작업에서 같은 `world.usda`를 연다. viewer를 재시작하면 저장된 배치를 읽는다.

물리 설정을 바꿀 때는 `physics.usda`를 편집 대상으로 선택한다.
새 에셋에는 필요한 Collision API를 추가한다. 새 물체의 충돌은 자동으로 생성되지 않는다.
Session Layer와 실행 중 생성되는 viewer의 임시 stage에만 변경을 남기면 재실행 시 사라진다.
시뮬레이션을 정지한 상태에서 위 환경 파일을 편집하고 저장한다.
`world.usda`를 flatten해서 원본에 덮어쓰면 레이어 구분이 사라지므로,
단일 파일이 필요하면 별도 이름으로 내보낸다.

배치 레이어와 모델·텍스처를 함께 버전 관리한다. 현재 전체 환경은 약 3.3 MB로
일반 Git에 보관할 수 있다. 향후 큰 자산을 추가하면 Git LFS 등을 적용한다.
새 배치 버전은 별도 브랜치/커밋으로 기록하면 이전 상태로 돌아갈 수 있다.

## 변환과 검사

GPU 없이 설치된 Isaac Sim의 USD 라이브러리로 변환한다.
현재 GLB에 사용된 normalized 정수 좌표, 삼각형, TRS 계층, 재질 및 텍스처를 처리하는
전용 변환기다. 임의의 glTF 범용 변환기는 아니다.

```bash
# 누락된 초기 파일 생성. 기존 layout/physics/world는 항상 보존.
.conda/isaacsim-5.0/bin/python scripts/environments/prepare_shipyard.py

# 변환 모델만 재생성. 기존 편집 레이어 보존.
.conda/isaacsim-5.0/bin/python scripts/environments/prepare_shipyard.py --rebuild-model

# CPU: 좌표, 메시, 충돌, 시작 위치, 상대 참조, 재실행 보존 검증
.conda/isaacsim-5.0/bin/python tests/check_shipyard_asset.py

# GPU: 장면·Go2 설정 로딩, 바닥 낙하 테스트, 화면 캡처
.conda/isaacsim-5.0/bin/python tests/check_shipyard_sim.py
```

원본을 새 버전으로 교체하면 노드 경로와 기존 물리 override가 달라질 수 있다.
새 폴더에서 변환해 기존 배치와 비교한 다음 이전한다. SHA-256이 다르면 변환기는 중단한다.
`check_shipyard_asset.py`는 초기 배치의 전체 경계·메시 수·단일 폴더 의존성을 가정하므로
warehouse 참조 및 routes가 추가된 현재 world 전체에는 그대로 적용할 수 없다.
현재 추가 구역은 위의 `check_shipyard_routes.py` / `check_shipyard_routes_sim.py`로 검사한다.

## 원본 조선소 변환 결과와 범위 (추가 구역 제외)

- 단위 m, Z-up. 원본 `(x, y, z)`를 `(x, -z, y)`로 변환한다.
- 전체 경계 약 `(-19, -30.30, -6.20)` ~ `(22.00, 26, 17.00)`m.
- 955개 메시, 930개 정적 삼각형 충돌체, 물 표면 25개는 충돌 제외.
- 도크를 덮는 무한 평면을 추가하지 않았다. 물은 시각 요소이며 부력/유체 시뮬레이션은 없다.
- 모든 장비는 정적 배경이다. 크레인·선박의 동작/관절은 구성하지 않았다.
- 콘크리트 텍스처와 기본 PBR 색상·금속성·거칠기를 보존한다.
  원본 유리의 `KHR_materials_transmission`은 불투명 Preview Surface로 근사한다.
- 산출물/검사 보고서는 `outputs/shipyard/`에 저장된다.
