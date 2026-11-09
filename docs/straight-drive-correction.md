# 휠·IMU 융합과 저속 직선 보정

작업 브랜치: `update/straight-drive-correction`. 기준: `develop`의 `66fe67d`.
2026-11-09 사용자 요청으로 시작했다. Isaac Sim 센서·모델은 유지하고 제어는 ROS 2 C++로 구현한다.

## 구현 범위

`robot_localization`의 EKF에 휠 오도메트리의 차체 `vx/vy`와 IMU의 yaw·yaw 각속도를
입력한다. 같은 엔코더에서 얻은 위치·방향·속도를 중복 융합하지 않는다. 메카넘은
횡이동이 가능하므로 `vy=0` 제약을 강제하지 않는다. IMU 가속도는 이번 단계에서 사용하지 않는다.

`bringup.launch.py localization:=true`는 센서 융합만 켠다. `straight:=true`는 융합과
유한 거리 직선 시험 노드를 켜며, 시작 서비스를 호출하기 전에는 주행 명령을 발행하지 않는다.
기존 기본 실행과 자동 주행 검증 모드는 유지한다. `straight`와 `validation`을 함께 켜면
명령 발행 충돌을 막기 위해 launch가 실패한다.

| 입력·출력 | 용도 |
| --- | --- |
| `/joint_states` | 실제 휠 위치·속도 입력 |
| `/odometry/wheel` | 융합 모드의 휠 오도메트리. 위치는 EKF에 융합하지 않음 |
| `/imu/data` | Isaac Sim의 원본 IMU |
| `/imu/filtered_input` | IMU 측정·시각은 유지하고 미지정 공분산만 채운 EKF 입력 |
| `/odom` | 융합 모드에서는 EKF 결과, 기본 모드에서는 기존 휠 오도메트리 |
| `/cmd_vel` | 직선 시험의 차체 보정 명령 |
| `/wheel_commands` | 기존 MecanumDrive가 계산한 네 휠 목표 |

융합 모드의 `odom → base_footprint` TF는 EKF만 발행한다. 휠 오도메트리의 TF는 끈다.
`robot_state_publisher`는 기존 URDF 내부 TF를 유지한다. IMU의 상대 yaw 옵션은 시작 방향을
지역 odom 방향과 맞춘다. 지도상의 절대 위치나 방위가 생기는 것은 아니다.

## 보정 방식

시작 서비스가 호출된 시점의 `/odom` pose로 목표 직선을 고정한다. 시작 방향을 기준으로
전진 잔여 거리, 좌우 이탈 거리, 방향 오차를 계산한다. 목표 직선은 매 주기 다시 설정하지 않는다.

```text
along_speed = min(speed, distance_gain * remaining_distance)
lateral_speed = clamp(-lateral_gain * lateral_error)
yaw_rate = clamp(-heading_gain * heading_error)
```

직선 기준 속도는 현재 차체 방향으로 회전해 `linear.x/linear.y`에 기록한다.
시작 방향과 현재 방향이 다를 때도 같은 직선을 향하도록 하는 처리다.
목표 근처에서는 전진 속도를 줄이고 목표를 지나면 더 전진하지 않는다.
위치·방향 허용값과 실제 추정 속도 조건을 0.3초 유지하면 완료한다.

제어 계산은 `straight_controller.hpp/.cpp`, ROS 서비스·시각·센서 상태 검사는
`straight_drive_node.hpp/.cpp`, 진입점과 종료 처리는 `straight_drive_main.cpp`에 분리했다.
IMU 공분산 relay도 별도 클래스와 실행 파일이다. 이득·제한·허용값은 YAML에 둔다.

## 실행

시스템 ROS 환경에서 의존성을 설치한다. `requirements-apt.txt`에도 기록했다.

```bash
sudo apt install ros-jazzy-robot-localization clang-format
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select my_amr --symlink-install
source install/setup.bash

# Isaac Sim을 이미 실행했다면 sim:=true를 생략
ros2 launch my_amr bringup.launch.py sim:=true straight:=true
```

빈 시험 공간과 센서 수신을 확인한 뒤 별도 터미널에서 시작한다. 키보드·다른 `/cmd_vel`
발행자는 종료한다. 기본 목표는 시작 위치에서 전방 1 m, 속도 0.1 m/s다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 service call /straight_drive/start std_srvs/srv/Trigger '{}'

# 중간에 정지
ros2 service call /straight_drive/stop std_srvs/srv/Trigger '{}'
```

시작은 건강한 센서, 정지 상태, 단일 명령 발행자를 요구한다. 원본 휠 오도메트리·IMU·EKF
출력 각각의 ROS timestamp와 steady clock 수신 공백을 확인한다. EKF가 입력 단절 후 예측을
계속 발행하더라도 원본 센서 단절을 놓치지 않는다. 중복·과거 시각으로 생존 시간을 갱신하지 않는다.
IMU·휠 단절, 시각 역행, 일시정지, 다른 명령 발행자, 과도한 경로 이탈, 시간 초과는 0 명령으로
시험을 종료한다. 센서 복구만으로 재시작하지 않는다. Ctrl+C도 종료 전 0 명령을 발행한다.
이는 속도 목표 정지이며 무거운 차체가 물리적으로 즉시 멈춤을 보장하지 않는다.

robot_localization 3.8.3 소스에서는 `reset_on_time_jump`의 자동 재설정 경로가 구현되지 않아
시간 역행 뒤 이전 측정 시각까지 입력을 무시할 수 있다. 이 경우 주행은 정지한 상태로 두고
다음 서비스를 호출한 뒤 센서·필터가 안정되면 `/straight_drive/start`를 다시 호출한다.
재설정은 지역 odom 원점을 바꾸므로 실행 중 경로를 계속 추종하는 용도로 사용하지 않는다.

```bash
ros2 service call /ekf_filter/reset std_srvs/srv/Empty '{}'
```

## 설정과 한계

- `config/localization.yaml`: EKF 입력·프레임·융합 항목, IMU 초기 분산.
- `config/straight_drive.yaml`: 거리·속도·P 이득·출력 제한·종료 허용값·단절 제한.
- `straight_params:=/절대경로/설정.yaml`로 직선 시험 설정을 교체할 수 있다.
- `localization.launch.py localization_params:=/절대경로/설정.yaml`로 융합 설정을 교체할 수 있다.
- 공분산과 이득은 시작값이며 교정된 값이 아니다. IMU relay는 공분산 전체가 0일 때만 초기 분산을
  채우고, 기존 공분산이 있으면 유지한다. 유효한 orientation·angular velocity가 필요하다.
- 휠·IMU만으로 관측할 수 없는 횡방향 미끄러짐과 누적 병진 오차는 남는다. 이 작업은
  LiDAR 스캔 매칭·SLAM·AMCL 또는 Nav2 경로 추종을 구현한 것이 아니다.
- 800 kg으로 편집한 USD의 접촉·모터 토크·가감속·정지 거리에서 물리 검증해야 한다.
  기존 사용자 USD나 질량 설정은 이번 작업에서 변경하지 않았다.

## 검증

단위 검사는 부호, 시작 방향 회전, yaw 래핑, 출력 제한, 목표 종료, 잘못된 입력, 합성 폐루프
수렴을 확인한다. DDS 검사는 실제 EKF 실행 파일과 실제 C++ 노드를 합성 센서로 실행한다.

```bash
# ros2_ws/에서, ROS와 install/setup.bash를 source한 뒤
XML_CATALOG_FILES=/tmp/amr-package-catalog.xml colcon test --packages-select my_amr
colcon test-result --test-result-base build/my_amr/test_results --verbose

# 시뮬레이터와 분리한 도메인. 실제 로봇이 없는 환경에서 실행
ROS_DOMAIN_ID=78 python3 src/my_amr/test/straight_integration.py
```

XML Catalog는 이전 [검증 문서](keyboard-control.md)의 공식 ROS XSD 매핑이며 XML 검사를 생략하지 않는다.
sudo 설치를 할 수 없는 현재 에이전트 환경에서는 robot_localization 3.8.3과 필요한 바이너리 의존성을
`/tmp/amr-localization-runtime`에 추출해 합성 DDS 검사를 실행했다. 일반 사용자는 위 apt 설치를 사용한다.

공식 설정 근거:
[robot_localization 3.8.3 센서 구성 문서](https://github.com/cra-ros-pkg/robot_localization/blob/3.8.3/doc/configuring_robot_localization.rst),
[공식 EKF 설정 예제](https://github.com/cra-ros-pkg/robot_localization/blob/3.8.3/params/ekf.yaml).

### 2026-11-09 검사 결과

- `my_amr` 빌드 성공. 패키지 결과 107 tests, 오류·실패 0, skip 25.
- 실제 EKF와 C++ 노드의 합성 DDS 검사 통과. 0.3 m 목표에 최종 fixture 위치
  `x=0.2909154 m`, `y=0.0011419 m`, `yaw=0.00012293 rad`였다. 1 cm 위치 허용값에 따른
  종료이며 실제 AMR의 주행 정밀도를 의미하지 않는다.
- 원본 IMU 공분산 설정, TF 단일 소유자, 시작 전 정지, 부호에 맞는 보정, 목표 완료,
  중복 시작 거부, 미갱신·NaN IMU, 휠 단절, 일시정지, 시각 역행, 명령 발행 충돌,
  센서 복구 후 자동 재개 금지, 명시적 EKF 초기화와 재시작·정지 서비스 확인.
- 기존 `odometry_integration.py --bringup` 3가지 구성과
  `validation_integration.py --bringup`의 합성 18회 시험·단절·리셋·충돌 회귀 검사 통과.
  오도메트리 오류 주입 검사 중 RSP가 NaN 관절 TF를 거부하는 로그는 의도한 잘못된 입력에 따른 것이다.
- 검증 중 원본 오도메트리 include의 `publish_tf=false` 인자가 EKF에도 전파되는 문제를 발견해
  `GroupAction`의 범위로 격리했다. 표준 EKF의 시각 역행 제한은 `/ekf_filter/reset` 복구 절차로 처리했다.
- 최종 DDS 로그: `/tmp/amr-straight-dds-5d33r7u9/bringup.log`.
  패키지 검사 로그: `/tmp/amr-straight-final-tests.log`.
  기존 검증 회귀 결과: `/tmp/amr-validation-dds-v0_mttqa/`.
- 이번 작업에서 Isaac Sim GPU 물리 주행은 실행하지 않았고 사용자 USD·질량은 변경하지 않았다.
  LiDAR 위치 추정, 실제 마찰·센서 교정과 800 kg 모델의 주행 성능은 후속 검증 대상이다.
