# Lekiwi 이동 베이스

자산 원본은 `lekiwi_isaacsim/urdf/lekiwi/lekiwi.usd`와 그 configuration USD다.
기존 URDF/mesh와 USD는 수정하지 않고 `robot_sim/lekiwi_scene.py`에서 참조 후 stage override를 적용한다.
원본 제공 저장소는 https://github.com/kabilankb/lekiwi_isaacsim 이며 자산 자체 안내는
`lekiwi_isaacsim/README.md`를 참고한다. 현재 파일의 SHA-256은 `config/sources.lock.json`에 기록한다.

## 확인한 원본 상태와 실행 시 보정

- `/LeKiwi/root_joint`가 월드와 연결된 fixed joint다. 이를 비활성화하고 base link에 articulation root를 적용한다.
- 구동 바퀴 3개 중 2개의 `physics:axis`가 빈 token이다. 실제 배치와 축을 설정 파일에 명시하고
  3개 바퀴 모두 joint frame을 다시 구성한다. 임의 축을 나타내던 빈 token에 의존하지 않는다.
- 팔/그리퍼 회전 관절은 6개다. 이번 이동 베이스 실행에서는 원래 자세를 유지하는 fixed joint로 잠근다.
- 원본 바퀴는 단일 convex hull이며 별도의 자유 회전 롤러가 없다. 해당 접촉만 비활성화하고,
  각 바퀴에 capsule roller 12개와 자유 회전 관절을 추가한다. 몸체·팔의 충돌은 유지한다.

구동 관절은 다음 순서다.

1. `ST3215_Servo_Motor_v1_1_Revolute_62`
2. `ST3215_Servo_Motor_v1_2_Revolute_60`
3. `ST3215_Servo_Motor_v1_Revolute_64`

`robot_control/config/lekiwi.yaml`은 축·배치·hub offset·반경 0.0508m·모터 gain/토크 및 롤러 설정이다.
`robot_control/lekiwi/controller.py`는 몸체 기준 vx/vy/wz를 구동 바퀴 rad/s로 바꾼다.
이후 위치/속도를 매 스텝 직접 지정하지 않고 velocity drive와 실제 물리 접촉으로 움직인다.
출력 관절 상태는 구동 3개 + 수동 롤러 36개, 총 39개다. 팔 고정 관절은 포함하지 않는다.

이 모델은 이동 제어 검증용 근사다. 원본 CAD 질량/관성을 사용하며 실제 로봇의 실측 질량·관성,
roller 형상·접촉 마찰 및 모터 토크 곡선으로 보정하지 않았다. 실제 하드웨어 성능 예측용으로 검증된 모델은 아니다.

## 실행과 검사

[ROS workspace 사용 안내](../../ros2_ws/README.md)의 Lekiwi 절을 따른다.
키보드 초기 검증 속도는 0.2m/s, 회전은 0.5rad/s다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch robot_bringup simulation.launch.py \
  robots_file:="$PWD/ros2_ws/src/robot_bringup/config/lekiwi_single.yaml"

# 별도 터미널에서 source 후
ros2 run robot_control teleop --robot-id lekiwi_0 --keyboard \
  --keyboard-speed 0.2 --keyboard-turn-speed 0.5
```

물리 통합 검사는 `tests/check_lekiwi_ros.py`이며 보고서는 `.runtime/lekiwi/<scenario>_<display>/`에 저장한다.

2026-10-15: flat/headless 및 warehouse/WebRTC에서 각각 900스텝·리셋/전복 0회.
0.2m/s 전후·횡이동과 ±0.5rad/s 회전, 정지·명령 만료 후 실제 정지 및 ROS 상태 출력을 확인했다.
WebRTC는 서버 실행·signaling 포트까지 검사했으며 원격 영상 수신은 별도 확인이 필요하다.
