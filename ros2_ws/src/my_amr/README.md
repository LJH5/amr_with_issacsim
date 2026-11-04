# my_amr

ROS 2 Jazzy / ament_cmake / C++17 기반 AMR 제어 패키지입니다. 라이선스는 Apache-2.0입니다.

## 역할과 명령 흐름

`keyboard_teleop` → `/cmd_vel` (Twist) → `my_amr_node` → `/wheel_commands` (JointState) → Isaac Sim Bridge → 네 휠 조인트.

| 파일 | 역할 |
| --- | --- |
| `src/keyboard_teleop.cpp` | WASD·Q/E·J 키 입력과 주기적인 Twist 발행 |
| `src/drive_node.cpp`, `include/my_amr/drive_node.hpp` | ROS 구독·발행, 파라미터, 주기 갱신 |
| `src/mecanum_drive.cpp`, `include/my_amr/mecanum_drive.hpp` | ROS에 독립적인 메카넘 계산·속도/가속도 제한·timeout |
| `src/my_amr_node.cpp` | ROS 초기화와 제어 노드 실행 |
| `launch/drive.launch.py` | amr_description의 공통 JSON 제원을 ROS 파라미터로 전달 |

운영 제어는 이 C++ 패키지에서 수행합니다. Isaac Sim에는 바퀴 명령 수신·이름 매핑·단위 변환·통신 watchdog만 둡니다. 기존 Python 기구학은 `--control cli`의 초기 검증용입니다.

## 빌드와 실행

```bash
# 터미널 1: 저장소 루트에서 빌드하고 제어 노드 실행
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select amr_description my_amr --symlink-install
source install/setup.bash
ros2 launch my_amr drive.launch.py
```

```bash
# 터미널 2: 저장소 루트에서 Isaac Sim 실행
cd isaac_sim
bash run.sh --control ros
```

```bash
# 터미널 3: 저장소 루트에서 키보드 노드 실행
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run my_amr keyboard_teleop
```

세 터미널의 `ROS_DOMAIN_ID`를 맞추고 키보드 터미널에 포커스를 둡니다. 실행 순서는 자유이며 명령이 없으면 정지합니다. 키보드는 대화형 터미널에서 직접 실행합니다. 일반 ROS launch는 키보드 stdin을 전달하지 않으므로 제어 노드와 별도 실행합니다.

| 키 | 동작 (로봇 좌표계) |
| --- | --- |
| W | 전진, +X |
| S | 후진, −X |
| A | 좌측 횡이동, +Y |
| D | 우측 횡이동, −Y |
| Q | 제자리 좌회전, +yaw (위에서 볼 때 반시계) |
| E | 제자리 우회전, −yaw (위에서 볼 때 시계) |
| J | 정지, 프로그램은 계속 실행 |
| Ctrl+C | 정지 명령을 보내고 종료 |

대소문자 모두 지원하며 Enter는 필요하지 않습니다. **방향은 다음 키까지 유지됩니다. 키를 놓는 것만으로 정지하지 않습니다.** J와 미지원 키는 속도를 0으로 설정합니다. 기본 병진 속도는 0.3 m/s, 회전 속도는 0.3 rad/s입니다. 예를 들어 다음과 같이 낮출 수 있습니다.

```bash
ros2 run my_amr keyboard_teleop --ros-args \
  -p speed_m_s:=0.1 -p yaw_rate_rad_s:=0.2
```

외부 `/cmd_vel` 명령을 사용할 때는 키보드 노드를 종료하고 발행자를 하나만 사용합니다. 제자리 회전에서는 병진 명령을 0으로, 병진 이동에서는 회전 명령을 0으로 설정합니다.

## 제원과 메시지

`amr_description/config/compact_amr.json`의 `wheels`, `motion_targets`, `control` 항목을 사용합니다. 형상 제원을 C++에 별도로 복제하지 않습니다. 필수 파라미터를 전달하는 `drive.launch.py`로 실행하세요. 파라미터는 시작 시 읽습니다.

- `/cmd_vel`: geometry_msgs/Twist, linear.x·linear.y [m/s], angular.z [rad/s]. 다른 축은 사용하지 않습니다.
- `/wheel_commands`: sensor_msgs/JointState, 네 휠의 name과 velocity [rad/s]. 목표 명령이며 position/effort는 비웁니다.
- `/joint_states`: Isaac Sim이 발행하는 실제 관절 위치·속도. name으로 휠을 찾습니다.

제어 노드는 50 Hz로 바퀴 속도를 발행합니다. `/cmd_vel` 수신이 0.25초 끊기면 C++ 노드가 속도 목표를 0으로 발행합니다. `/wheel_commands` 수신이 0.25초 끊기면 Isaac Sim 어댑터도 독립적으로 속도 목표를 0으로 적용합니다. 두 watchdog은 ROS 시각이 아닌 steady/monotonic clock을 사용합니다. 물리적인 정지 거리와 감속은 관성·접촉·토크 제한의 영향을 받습니다.

일정한 명령도 메시지가 새로 수신되면 유효합니다. Isaac Sim은 native subscriber의 새 메시지 실행 신호를 Counter로 세어 수신 여부를 판정합니다. 잘못된 휠 이름·누락·중복·NaN/Inf·제원 초과 명령은 정지 처리합니다. 정지 중에도 0 속도 명령을 발행합니다.

`/odom`, 폐루프 보정, 장애물 기반 자동 정지, Nav2, ros2_control 하드웨어 인터페이스는 후속 작업입니다. 현재 설치 환경에는 mecanum_drive_controller와 topic 기반 하드웨어 연결이 준비되어 있지 않아 기존 검증한 기구학을 C++ 계산기로 이식했습니다.

## 검사와 주석

```bash
# ros2_ws/에서 시스템 Jazzy 및 install/setup.bash를 source한 뒤 실행
colcon test --packages-select my_amr amr_description
colcon test-result --verbose
ROS_DOMAIN_ID=42 python3 src/my_amr/test/keyboard_integration.py
```

통합 검사는 실제 C++ 키보드 프로세스의 가상 터미널에 WASD·Q/E·J를 입력합니다. `--sim`은 같은 도메인에서 실행 중인 Isaac Sim의 실제 휠 피드백까지 검사합니다. 이 옵션은 로봇을 움직이므로 빈 테스트 장면에서 사용하고 기존 키보드·제어 노드를 종료하세요.

```bash
# Isaac Sim 터미널: isaac_sim/에서
ROS_DOMAIN_ID=42 bash run.sh --control ros
# 별도 시스템 ROS 터미널: ros2_ws/에서
ROS_DOMAIN_ID=42 python3 src/my_amr/test/keyboard_integration.py --sim
```

검증 기록과 네트워크 제한 환경의 XML 스키마 검사 방법은 [키보드 제어 기록](../../../docs/keyboard-control.md)에 정리합니다.

C++ 클래스·함수는 Doxygen 형식으로 문서화합니다. 헤더에는 @brief·@param·@return·@details·@throws와 단위·좌표계·배열 순서·timeout 의미를 적고, 헤더가 없는 함수는 정의부에 작성합니다.
