# 실제 휠 오도메트리와 TF 연동

작업일: 2026-11-05. 브랜치: `feature/wheel-odometry`.
기준 커밋: `origin/develop`의 `fa64021`(AMR 모델 수정).

## 목적과 구현

SLAM에서 사용할 연속적인 로봇 위치 정보를 준비한다. 기존 ROS 독립 계산기
`MecanumOdometry`를 재사용하여 실제 휠 위치 증분을 SE(2) 방식으로 적분한다.
코드는 `ros2_ws/src/my_amr/`의 C++ 노드로 구현하며 Isaac Sim 구동 코드는 변경하지 않는다.

```text
Isaac Sim /joint_states (측정 위치 rad, 시뮬레이션 시각)
  -> wheel_odometry
     -> /odom (nav_msgs/Odometry)
     -> /tf: odom -> base_footprint
  -> robot_state_publisher
     -> URDF 내부 TF: base_footprint -> base_link -> 휠·센서
```

원본 JSON에서 형상·handedness·네 휠 이름·속도 한계를 읽는다.
`/cmd_vel`, `/wheel_commands`, `/ground_truth/pose`로 위치를 추정하지 않는다.
IMU 융합·슬립 보정은 이번 단계에 포함하지 않는다.

## 실행

저장소 루트에서 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select amr_description my_amr --symlink-install
source install/setup.bash
```

## 통합 실행

빌드·환경 source 후 다음 명령 하나로 주행 제어·오도메트리·로봇 TF를 함께 실행한다.

```bash
ros2 launch my_amr bringup.launch.py
```

Isaac Sim도 함께 실행하려면:

```bash
ros2 launch my_amr bringup.launch.py sim:=true
```

`drive.launch.py`와 `odometry.launch.py`를 재사용한다. 기존 SDK의 `run.sh`가
Isaac Sim 전용 Python 환경을 유지한다. 키보드는 별도 입력 터미널에서
`ros2 run my_amr keyboard_teleop`로 실행한다. `display.launch.py`는 가상 JointState를
발행하므로 포함하지 않는다. 통합 실행 중 개별 launch를 중복 실행하지 않는다.

| 인자 | 기본값 | 역할 |
| --- | --- | --- |
| `sim` | false | Isaac Sim도 함께 시작. 이미 실행 중이면 생략 |
| `headless` | false | 함께 시작하는 시뮬레이터의 GUI를 끔 |
| `sim_script` | 저장소의 isaac_sim/run.sh 자동 탐색 | SDK 폴더가 다른 배치이면 절대 경로 지정 |
| `validation` | false | 자동 검증 launch로 제어기 실행 책임 전환 |
| `use_sim_time` | true | 오도메트리·로봇 TF의 시뮬레이션 시간 설정 |
| `publish_tf` | true | 휠 오도메트리 TF 발행 여부 |
| `params_file` | my_amr/config/odometry.yaml | 오도메트리 설정 파일 |

자동 검증 모드는 `validation.launch.py`가 자체 제어기를 실행하므로 기본 drive launch를
추가 실행하지 않는다. 정상 주행·검증 모드 모두 제어기는 한 개다.
아래 명령은 로봇을 자동으로 움직이며 키보드와 함께 사용하지 않는다.
검증의 repetitions·speed_m_s·output_prefix 등 기존 인자를 그대로 전달할 수 있다.

```bash
ros2 launch my_amr bringup.launch.py sim:=true validation:=true \
  repetitions:=1 speed_m_s:=0.1 output_prefix:=/tmp/amr_validation
```

통합 launch의 Ctrl+C는 그 launch가 시작한 프로세스를 함께 종료한다.
시뮬레이터 창을 닫거나 자동 검증이 끝나도 해당 통합 실행을 종료한다.
다른 터미널에서 독립 실행한 시뮬레이터는 소유하지 않는다.
원본 USD를 저장하거나 제어 알고리즘을 변경하지 않는다.

## 개별 실행

아래는 통합 launch 대신 선택하는 분리 실행이다.
각 터미널은 저장소 루트에서 시작하며 `ROS_DOMAIN_ID`를 맞춘다.

```bash
# 터미널 1: 시뮬레이터
cd isaac_sim
bash run.sh --control ros
```

```bash
# 터미널 2: 기존 주행 제어기
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch my_amr drive.launch.py
```

```bash
# 터미널 3: 새 오도메트리 + 실제 관절 기반 로봇 TF
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch my_amr odometry.launch.py
```

```bash
# 터미널 4: 키보드
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run my_amr keyboard_teleop --ros-args -p speed_m_s:=0.1
```

WASD 이동, Q/E 좌우 회전, J 정지다. 오도메트리 launch는 제어기를 추가로 실행하지 않는다.
`display.launch.py`와 `joint_state_publisher_gui`는 가상 관절 값을 발행하므로 함께 사용하지 않는다.
현재 wheel JointState는 단일 측정 발행자를 사용해야 한다.

확인은 별도 ROS 터미널에서 실행한다.

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo odom lidar_front_left_scan
```

RViz2는 Fixed Frame을 `odom`으로 지정한다. 기존 모델 표시 설정을 재사용할 수 있다.

```bash
ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix amr_description)/share/amr_description/rviz/compact_amr.rviz" \
  -f odom --ros-args -p use_sim_time:=true
```

LaserScan의 `/scan/front`, `/scan/rear`, Odometry의 `/odom` 표시를 추가할 수 있다.
이번 검증은 headless로 수행했으며 RViz GUI 화면 확인은 별도다.

## 메시지·TF 책임

| 항목 | 기준 또는 발행 주체 |
| --- | --- |
| `/odom.header.frame_id` | `odom`, 노드 시작/시뮬레이션 시간 초기화 시점의 지역 원점 |
| `/odom.child_frame_id` | `base_footprint`, URDF 루트 |
| pose | `odom` 좌표계의 x/y/yaw, z=0 |
| twist | 차체 `base_footprint` 좌표계의 vx/vy/wz |
| 시간·발행 주기 | 수신한 실제 JointState 시각·주기. 현재 Isaac Sim 약 60 Hz |
| `odom -> base_footprint` | C++ `wheel_odometry`, `/odom`과 같은 시각·pose |
| `base_footprint -> base_link` | robot_state_publisher, URDF 높이 0.19 m |
| URDF 내부 휠·센서 TF | robot_state_publisher, 실제 `/joint_states` 사용 |
| `base_link -> lidar_*_scan` | Isaac Sim Bridge, scan 전용 이름 |
| `base_link -> camera_front_rgbd_optical_frame` | Isaac Sim Bridge, RGB-D 전용 이름 |
| `map -> odom` | 후속 SLAM/위치 추정에서 발행, 현재는 없음 |

`odom -> base_link`를 따로 발행하면 URDF의 `base_footprint -> base_link`와 부모가
겹치므로 현재 모델에서는 `odom -> base_footprint`를 사용한다.

## 입력 처리와 파라미터

`my_amr/config/odometry.yaml`에서 토픽·프레임·공분산과 최대 표본 간격을 설정한다.
기하학 수치는 `amr_description/config/compact_amr.json`에서 자동 전달한다.

- 휠은 FL/FR/RL/RR 순서로 이름 매핑한다. 다른 조인트가 섞여도 처리한다.
- 실제 관절 위치의 차이를 사용한다. 속도 배열은 없어도 된다.
  설치된 Isaac Sim 5.1의 실제 관절 값은 `+6.2827 -> -6.2616 rad`처럼
  ±2π 경계에서 약 4π만큼 바뀐다. `position_wrap_period_rad`를 4π로 지정하여
  이 차이를 실제 작은 회전량으로 복원한다. 일반적인 2π 래핑과 구분한다.
  연속 각도를 발행하는 하드웨어에서는 이 파라미터를 0으로 설정한다.
- 첫 표본은 위치 원점·휠 기준이다. 초기 절대 encoder 각도를 이동으로 적분하지 않는다.
- 같은 시각은 무시한다. 시간 역행은 시뮬레이션 재시작으로 보고 원점을 초기화한다.
  SLAM이 이미 실행 중이면 그 지도 세션과 원점도 함께 재설정해야 한다.
- 누락·중복 휠 이름, 위치 배열 길이 불일치, NaN/Inf, 잘못된 시각은 무시한다.
- 0.5초를 넘는 표본 간격이나 속도 한계를 넘는 위치 점프는 pose를 유지하고 기준만
  갱신한다. 그 표본을 발행하지 않으며 다음 유효 표본부터 재개한다.
  손실 구간의 이동은 복구하지 않는다. 위치 양자화 여유는 표본당 0.01 rad다.
  래핑 사용 시 속도 한계×시간 간격+여유가 반주기 이상이면 이동 방향이 모호하므로
  같은 방식으로 건너뛴다. 현재 35 rad/s·4π에서는 간격 약 0.179초 이상이 해당한다.
  래핑 입력은 ±반주기 범위여야 한다.
- 피드백이 끊기면 `/odom`과 해당 TF도 중단한다. 실제 정지를 확인하지 않고 0 속도나
  새 시각으로 pose를 발행하지 않는다.
- `pose_covariance_diagonal`, `twist_covariance_diagonal`은 x/y/z/roll/pitch/yaw 순서의
  분산이다. 현재 `[0.01, 0.01, 1e6, 1e6, 1e6, 0.04]`는 초기 튜닝값으로,
  실측 오차의 통계값이나 누적 오차 모델이 아니다. 센서 융합 전 보정해야 한다.

`use_sim_time`은 기본 true다. 실제 ROS 시각을 사용할 때는 false로 실행한다.
향후 EKF가 같은 TF를 발행하면 아래와 같이 wheel TF를 끄고 EKF의 차체 프레임도
`base_footprint`로 맞춘다. `/odom` 메시지는 계속 발행한다.

```bash
ros2 launch my_amr odometry.launch.py publish_tf:=false
```

다른 설정 파일은 `params_file:=/absolute/path/odometry.yaml`로 전달한다.
설정은 시작 시 읽는다.

## 검증 결과

통합 launch 추가 후:

- 빌드 성공, 패키지 검사 98개 / 오류·실패 0 / skip 16.
- `odometry_integration.py --bringup`: 제어기·오도메트리·RSP 각각 한 개,
  TF 켜기/끄기·시각·입력 오류·각도 래핑 검사 통과.
- `validation_integration.py --bringup`: 6방향×3회 합성 측정과 단절·시간 초기화·
  중복 명령에 대한 중단·정지 확인. 실제 물리 정지 성능 시험과는 별개다.
- 실제 `bringup.launch.py sim:=true headless:=true`: 네 방향 명령,
  391개 측정 시각 일치와 센서 TF, Ctrl+C 후 SDK·ROS 프로세스 동반 종료 확인.
- 존재하지 않는 sim_script는 오류로 중단하며 SDK 자동 실행은 기본 false다.

재현은 시뮬레이터가 없는 별도 도메인에서 실행한다.

```bash
ROS_DOMAIN_ID=68 python3 ros2_ws/src/my_amr/test/odometry_integration.py --bringup
ROS_DOMAIN_ID=69 python3 ros2_ws/src/my_amr/test/validation_integration.py --bringup
```

통합 실행 로그는 `/tmp/amr_bringup_*.log`, native 요약은
`ros2_ws/log/odometry/bringup_summary.json`에 보존했다(Git 제외).

오도메트리 단독 구현 시점:

- `colcon build`: amr_description·my_amr 성공.
- `colcon test-result --verbose`: 96개, 오류·실패 0, skip 16.
- `test/odometry_integration.py`: 실제 C++ 프로세스와 DDS에서 전진·횡이동·yaw·회전 후
  이동, body-frame twist, 조인트 순서 변경·추가, 측정 시각, 공분산, RSP 연결,
  TF 켜기/끄기, 입력 오류, 시간 초기화, 손실·encoder reset, 양방향 각도 래핑과
  모호한 표본 간격 거부 처리를 통과했다.
  의도적인 NaN JointState 주입에서는 RSP/TF2의 거부 로그가 발생할 수 있다.
- Isaac Sim 5.1 실제 피드백: 391개 `/odom` 전부 `/joint_states`와 시각이 일치했다.
  전진·좌측·좌우 회전의 적분 방향과 아래 6개 프레임의 TF 경로를 확인했다.
  `base_footprint`, `base_link`, `imu_link`, 두 `lidar_*_scan`, RGB-D optical frame.
- 래핑 보정 후 8초 연속 직진: 약 0.78443 m 적분, 541개 전부 조인트 측정 시각 일치.
  각도 래핑이 관측된 2개 표본에서도 `/odom`이 누락되지 않았다.

1초 simulation-time 명령의 실제 휠 적분값은 다음과 같다. 제어기의 가속도 제한을
포함하므로 목표 속도×1초와 정확히 같지는 않다. 차체 참값과의 오차 표가 아니다.

| 명령 | 시작 차체 좌표계 dx m | dy m | yaw rad |
| --- | --- | --- | --- |
| 전진 0.1 m/s | 0.09081 | 0.00006 | 0.00040 |
| 좌측 0.1 m/s | -0.00148 | 0.09162 | -0.00078 |
| 좌회전 0.3 rad/s | 0.00020 | -0.00053 | 0.26548 |
| 우회전 -0.3 rad/s | 0.00021 | 0.00007 | -0.26726 |

합성 통합 검사는 시뮬레이터가 없는 별도 DDS 도메인에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ROS_DOMAIN_ID=63 python3 ros2_ws/src/my_amr/test/odometry_integration.py
```

원시 native 결과·측정 스크립트는 `ros2_ws/log/odometry/`, 로그는
`/tmp/amr_odometry_*.log`에 보존했다(Git 제외). 원본 USD와 마찰 설정은 변경하지 않았다.
이번 확인은 ROS 오도메트리·TF 경로의 검증이다. 이전에 남은 물리 정지 진동과
장거리 슬립 보정이 해결됐다는 의미는 아니다.

다음은 IMU 융합과 SLAM Toolbox의 scan 입력·지도 생성을 연결하는 단계다.
실행 후 commit/push는 별도이며 이번 작업에서는 수행하지 않았다.

Notion 개발 기록: [06. 실제 휠 오도메트리와 TF](https://www.notion.so/40fe0bf3f92981e286e1e298a817ec5e).

공식 메시지 기준: [ROS 2 Odometry](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html),
[Nav2 오도메트리 연결](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/configuration_and_development/first_time_robot_setup_guide/odom/setup_odom.md).
관절 래핑 근거: [PhysX Articulations](https://nvidia-omniverse.github.io/PhysX/physx/5.3.0/docs/Articulations.html).
4π 점프와 실제 표본 간격 0.016666668초는 해당 호스트의 원시 JointState로 확인했다.
