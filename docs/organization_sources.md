# CORA-AX 소스 저장소 구성

프로젝트 코드와 로봇 소스를 CORA-AX organization에서 관리하고, shipyard가 submodule commit을 고정한다.

| 저장소 | shipyard 내 경로 | 역할 |
|---|---|---|
| [shipyard_robot_sim](https://github.com/CORA-AX/shipyard_robot_sim) | 프로젝트 루트 | 실행 코드, 환경 정의, 교정된 Hunter 자산, Go2 정책 출처·검증 기록 |
| [IsaacLab](https://github.com/CORA-AX/IsaacLab) | `third_party/IsaacLab` | Isaac Lab v2.2.1 소스 |
| [ugv_gazebo_sim](https://github.com/CORA-AX/ugv_gazebo_sim) | `third_party/ugv_gazebo_sim` | 원본 Hunter URDF·mesh |
| [hunter_ros2](https://github.com/CORA-AX/hunter_ros2) | `third_party/hunter_ros2_reference` | 공식 Hunter 치수·제한값 참조 |
| [hunter_robot](https://github.com/CORA-AX/hunter_robot) | `third_party/hunter_robot` | Hunter 구조 비교용 소스 |

원본 저장소의 저작권·라이선스 파일과 commit 이력을 유지한다.
검증한 commit과 upstream 주소는 [source_repositories.json](source_repositories.json)에 기록한다.
2026-10-08에 네 CORA-AX 저장소 모두에서 해당 commit이 조회됨을 확인했다.

## 다른 컴퓨터에서 가져오기

이 연결 설정이 shipyard 원격 저장소에 반영된 뒤 다음 명령으로 소스를 함께 받는다.

```bash
git clone --recurse-submodules https://github.com/CORA-AX/shipyard_robot_sim.git
cd shipyard_robot_sim
```

이미 clone했다면 아래 명령을 사용한다.

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

그 다음 [기본 Isaac Sim 설치](isaacsim_5_setup.md)와 [Go2 환경 설치](go2_setup.md)를 진행한다.
Conda 환경, pip/Kit 캐시, 실행 로그는 Git에 포함하지 않는다.
Go2 정책은 SHA-256으로 검증해 NVIDIA 서버에서 받으며, 전체 Go2 USD 참조 파일도 현재 원격 자산을 사용한다.
따라서 이 구성은 **소스 저장소를 organization에 모으는 구성**이며 완전한 오프라인 시뮬레이터 배포판은 아니다.

## 현재 작업 폴더의 연결 상태

현재 Hunter 참조 폴더 3개에는 파일이 있지만 개별 Git 메타데이터가 없다.
IsaacLab의 shipyard gitlink 등록과 `.gitmodules`의 commit/push도 아직 완료되지 않았다.
따라서 위 새 컴퓨터용 clone 명령은 연결 정리가 원격에 반영된 뒤 사용할 수 있다.
이 도구 정리는 기존 Git 연결을 복원하거나 변경하지 않는다.

IsaacLab을 업데이트할 때는 먼저 CORA-AX/IsaacLab에서 검증하고,
shipyard의 gitlink·소스 manifest·환경 버전 기록을 함께 갱신한다.
