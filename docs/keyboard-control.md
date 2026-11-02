# ROS 2 C++ 키보드·메카넘 제어

날짜: 2026-11-02. 브랜치: `feature/keyboard-control`.

## 구현 범위

키보드 WASD 이동, Q/E 제자리 회전, J 정지를 ROS 2 C++ 패키지 `my_amr`에서 구현한다. `isaac_sim/`은 로봇·센서·물리 환경과 바퀴 명령 적용 어댑터를 담당한다. 제어 노드와 키보드는 `ament_cmake` / C++17 / 시스템 Jazzy로 빌드·실행한다.

명령 흐름은 `keyboard_teleop → /cmd_vel → my_amr_node → /wheel_commands → Isaac Sim ROS Bridge → 휠 조인트`다. 초기 제원은 amr_description의 `compact_amr.json`에서 launch가 읽어 ROS 시작 파라미터로 전달한다. 시뮬레이터 운영 경로에서 메카넘 계산과 가속도 제한을 수행하지 않는다.

- `keyboard_teleop.cpp`: 비정규·비에코 터미널 입력, WASD/Q/E/J 변환, 주기적인 명령 발행, 종료 시 정지·터미널 복원.
- `mecanum_drive.hpp/.cpp`: 차체 속도 제한, 벡터 가속도 제한, 메카넘 계산, 휠 각속도 제한, 수신 timeout.
- `drive_node.hpp/.cpp`: ROS 구독·타이머·JointState 목표 명령 발행. 단일 스레드 executor에서 실행한다.
- `my_amr_node.cpp`: ROS 초기화·노드 실행·오류 처리.
- `isaac_sim/ros_commands.py`: native ROS2SubscribeJointState와 Counter, 휠 이름 매핑, 통신 watchdog.
- `isaac_sim/drive.py`: 목표 rad/s를 USD 각도 단위 deg/s로 변환해 네 angular drive에 적용.

이전 `feature/ros-cmd-vel`의 수신 횟수 기반 watchdog 개념을 재사용하되, Twist 계산은 C++에 두고 시뮬레이터는 바퀴 명령만 받도록 변경했다. 전체 구형 브랜치를 병합하지 않았다.

## 키와 정지 의미

W/S는 전후진, A/D는 좌우 횡이동, Q/E는 제자리 좌·우회전이다. J는 정지하며 프로그램은 종료하지 않는다. Ctrl+C는 정지 명령을 보내고 종료한다. 모든 키는 대소문자를 지원한다. 방향은 다음 키까지 유지하므로 키를 놓는 것만으로는 정지하지 않는다. 기본 병진 0.3 m/s, 회전 0.3 rad/s이며 ROS 파라미터로 변경할 수 있다.

C++ 노드가 `/cmd_vel` 수신이 끊긴 것을 감지하면 속도 목표를 0으로 발행한다. 제어 노드 자체가 종료되어 `/wheel_commands`가 끊긴 경우는 Isaac Sim 어댑터가 별도로 정지시킨다. 현재 timeout은 각각 0.25 s다. steady/monotonic clock을 사용해 ROS /clock 정지·변경에 watchdog이 의존하지 않게 한다. 다만 시뮬레이터 앱 자체가 막혀 있으면 다음 갱신 때 정지가 적용되며 하드웨어 수준의 실시간 정지를 보장하지 않는다.

물리적 속도·정지 거리는 접촉·슬립·관성·토크 제한에 따라 목표와 다를 수 있다. 짧은 저속 회전의 한 시점에서 관절 속도 부호가 일시적으로 목표와 달랐으므로, 고정된 한 표본만으로 기구학 오류라고 판정하지 않는다. 최종 검사는 목표 도달 후 반복 수신 유지 및 실제 휠 피드백을 확인한다. 정밀한 폐루프 속도 보정은 이번 범위에 포함하지 않는다.

## 검증 방법

실행 명령은 [my_amr README](../ros2_ws/src/my_amr/README.md)를 따른다. 세 터미널에서 제어 launch, Isaac Sim, 키보드를 각각 실행한다. 기존 명령 발행자와 테스트가 동시에 로봇을 제어하지 않도록 분리한다.

오프라인 검증:

- C++ gtest: 전진·횡이동·회전 기구학, 명시적 정지, timeout·반복 수신, NaN, 벡터 가속도·휠 속도 제한.
- `isaac_sim/test/check_wheel_commands.py`: 입력 이름 순서 변경, timeout·counter reset, 누락·중복·NaN·초과 속도 거부.
- `isaac_sim/test/check_simulation.py`: CLI 기존 동작 및 ROS 바퀴 명령 경로, 장면 변경·재생 상태 처리.
- 기존 LiDAR·카메라·모델 계산 검사.

`ros2_ws/src/my_amr/test/keyboard_integration.py`는 실제 설치된 C++ 키보드 프로세스의 PTY에 키를 주입하고 DDS 토픽을 수신한다. 방향별 목표 속도, 유지된 동일 값의 반복 수신, J 정지 후 프로세스 유지, 키보드 프로세스 강제 종료 시 C++ timeout을 검사한다. `--sim`에서는 실제 `/joint_states` 피드백과 C++ 제어 노드 종료 시 시뮬레이터 transport watchdog 정지까지 확인한다.

기본 통합 검사는 로봇을 직접 구동하지 않지만 `--sim`은 실제 시뮬레이션 로봇을 움직인다. 같은 ROS_DOMAIN_ID의 빈 테스트 장면에서 실행한다.

2026-11-02 최종 결과: C++ 패키지/모델 검사 보고서는 48 tests, 0 errors, 0 failures, 7 skipped였다. Python 재생 검사 13개, 바퀴 어댑터 검사 2개, 기존 카메라·LiDAR 검사 각 3개도 통과했다. GPU와 별도 ROS 프로세스의 최종 PTY 검사는 W/S/A/D/Q/E 방향, J 정지, 반복 명령 유지, 키보드 종료 시 C++ watchdog, 제어 노드 종료 시 Isaac Sim watchdog을 모두 통과했다. 시험 병진 속도는 0.1 m/s, 회전 속도는 0.3 rad/s였다. 바퀴 목표는 병진 ±1.3333 rad/s, 회전 ±1.32 rad/s였고 실제 관절 피드백에서도 방향 변화를 확인했다. 실제 값과 목표의 일치는 보장하지 않는다. 두 최종 GPU/DDS 프로세스는 종료 코드 0이며 GUI 키보드 조작과 차체 위치·방향 오차의 정량 검증은 별도다.

## 형식 검사와 문서화

C++ 클래스·함수에 Doxygen `@brief`, `@param`, `@return`, `@details`, `@throws`를 작성했다. 형상·시간·속도의 단위와 휠 순서를 명시하며 헤더와 구현에 같은 설명을 중복하지 않는다. 이 규칙은 AGENTS.md에도 반영했다.

ROS 패키지 XML 검사는 기존 manifest가 참조하는 download.ros.org의 네트워크·인증서 문제로 실패했다. 스키마 검사를 생략하지 않고 공식 `ros-infrastructure/rep`의 XSD를 임시 디렉터리에 받아 XML Catalog로 검증했다.

```bash
# 공식 스키마: https://github.com/ros-infrastructure/rep/tree/master/xsd
# package_format3.xsd와 package_common.xsd를 /tmp에 저장한 환경의 예:
XML_CATALOG_FILES=/tmp/amr-package-catalog.xml \
  colcon test --packages-select my_amr amr_description
colcon test-result --verbose
```

Catalog는 `http://download.ros.org/schema/package_format3.xsd`를 로컬 공식 XSD로 매핑한다. 이 임시 환경 설정은 소스 코드나 manifest의 유효성 검사를 완화하지 않는다. 검증 과정에서 cppcheck는 배포된 2.13.0의 알려진 성능 문제 때문에 도구 자체가 검사를 생략했고, 기존 다른 패키지 테스트의 skipped 결과도 보고서에 포함된다.

## 남은 작업

오도메트리, 실제 위치·방향 오차 정량 측정, 폐루프 보정, 장애물에 따른 자동 정지와 Nav2 통합은 후속 작업이다. 사용자 수정 `omni_amr.usd`는 보존하고 자동 저장하지 않는다. Git 커밋·push와 Notion 외부 게시를 수행하지 않았다.

커밋 메시지 초안: `update(control): ROS C++ 메카넘 제어와 WASD·Q/E·J 키보드 연결`

공식 API 근거: [Isaac Sim 5.1 ROS2 Subscribe Joint State](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2SubscribeJointState.html), [Jazzy Twist](https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/Twist.html).
