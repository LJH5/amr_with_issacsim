# amr_navigation

저장한 점유 지도, Nav2/AMCL과 `robot_localization`으로 메카넘 AMR의 목표 이동을 실행한다.
휠 오도메트리와 IMU로 국소 움직임을 추정하고, LiDAR와 지도를 맞춰 전역 위치를 보정한다.

## 실행

**매핑 launch와 키보드 주행을 먼저 종료한다.** SLAM과 AMCL의 `map→odom` 발행 및
키보드와 Nav2의 `/cmd_vel` 발행이 중복되면 안 된다.

```bash
cd /home/buddha/my_workspace/amr_with_issacsim/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to amr_navigation --symlink-install
source install/setup.bash

# Isaac Sim은 이미 실행 중이며 Play 상태일 때
ros2 launch amr_navigation navigation.launch.py

# Isaac Sim도 함께 시작하려면 위 명령 대신
ros2 launch amr_navigation navigation.launch.py sim:=true
```

기존 robot bringup이 실행 중이면 `start_bringup:=false`를 사용한다. 이때 기본값은
외부 bringup의 `/odom`과 TF를 그대로 사용한다. 외부 bringup을 사용하면서 EKF도 실행할
경우 `my_amr bringup.launch.py`를 `publish_tf:=false odom_topic:=/wheel/odom`으로 실행하고
navigation에서 `start_bringup:=false use_ekf:=true`를 지정한다.
기본 지도는 `amr_mapping/map/map.yaml`과 상대 경로 `map.pgm`이며 지도 폴더도 함께 설치한다.
다른 지도는 `map:=/절대/경로/map.yaml`로 선택한다.

1. RViz2에서 Fixed Frame=`map`을 확인한다.
2. **2D Pose Estimate**로 실제 로봇의 지도상 위치와 방향을 지정한다.
3. 지도와 라이다 점이 맞는지 확인한다. 이번 구성의 AMCL은 `/initialpose`를 실제로 구독한다.
4. **Nav2 Goal**로 빈 공간의 목적지와 방향을 지정한다.
5. 필요하면 오른쪽 접힘 영역을 열어 Navigation 2 패널의 **Cancel**로 목표를 취소한다.

기본 화면에는 Displays만 표시한다. Views와 Tool Properties는 자동으로 열지 않는다.
Navigation 2 패널은 Nav2 Goal 연결을 위해 로드하되 오른쪽 도크에 숨긴다.
상단의 초기 위치·목표 지정 및 화면 이동 도구는 유지한다.

시작 시 AMCL이 먼저 활성화되고, 로컬·전역 costmap에 필요한 TF가 생기면 Nav2가 자동 활성화된다.
초기 위치를 늦게 지정해도 이 순서로 시작한다. `ros2 lifecycle get /bt_navigator`가
`active [3]`이면 목표를 보낼 수 있다.
명령한 목표가 없을 때에는 자동으로 주행하지 않는다. 새 지도 작성은 기존
`amr_mapping`을 별도로 실행하며 navigation은 저장된 지도를 사용한다.

## 연결 및 메카넘 설정

```text
/wheel/odom (vx, vy, wz) + /imu/data (relative yaw, wz) → 2D EKF → /odom, odom→base_footprint
/map + /scan/front + odom→base_footprint → AMCL → map→odom

Nav2 planner/controller → /cmd_vel_nav → velocity_smoother
    → /cmd_vel_smoothed → collision_monitor → /cmd_vel (Twist)
    → 기존 my_amr_node → /wheel_commands → Isaac Sim
```

- 기본 navigation launch에서 휠 오도메트리는 `/wheel/odom`을 발행한다. EKF는 휠로 계산한
  차체 속도 `vx`, `vy`, `wz`, IMU의 `angular_velocity.z`, 센서 시작 자세 기준 상대 yaw를
  융합해 `/odom`과 `odom→base_footprint`를 발행한다. 상대 yaw는 정지 중 작은 각속도
  오차가 누적되는 것을 줄인다. 설정은 `config/ekf.yaml`에 있다.
  `imu_covariance`는 센서가 공분산을 모두 0으로 보낸 경우에만 각속도와 자세의 초기 분산을 채운다.
  실제 장비에서 IMU의 yaw가 신뢰하기 어려우면 `ekf_params_file`에서 `imu0_config`의 yaw 항목을 끈다.
- AMCL은 `/scan/front`와 저장 지도의 일치도를 이용해 `map→odom`을 보정한다. 시뮬레이터의
  ground truth는 시험의 비교 기준으로 사용한다. 미끄러져 바퀴 회전과 실제 차체 이동이
  어긋나는 경우, 수평 위치 보정은 LiDAR가 지도에서 위치를 구분할 수 있는 정도에 좌우된다.
  기본 `sigma_hit: 0.10`과 `alpha3: 0.5`는 5 cm 지도와 휠 슬립을 고려한 초기 튜닝 값이다.
  제자리 회전에서 차체 중심이 조금만 움직인 Isaac Sim 측정 결과를 반영해
  회전에 따른 위치 불확실성은 `alpha4: 0.02`로 조정했다.
- DWB는 보정된 위치와 속도로 경로를 추종한다. 수정된 `vx`, `vy`, `wz`는 속도 제한과
  충돌 감시를 거쳐 기존 메카넘 제어기에서 네 바퀴의 목표 속도로 변환된다.
  목표 회전은 `yaw_goal_tolerance: 0.08` rad와 `RotateToGoal.lookahead_time: 0.5` s를
  사용한다. 짧은 평가 시간으로 목표 근처에서 회전 속도가 너무 일찍 줄어드는 것을 완화한다.
- DWB는 X/Y 양방향 속도를 샘플링한다. 초기 평면 최대 속도0.3m/s, 회전0.6rad/s로
  기존 하위 제어기의 최고 속도보다 낮게 시작한다. 회전 정렬 없이도 횡이동할 수 있다.
- costmap의 직사각형 footprint는 `amr_description/config/compact_amr.json`에서 읽는다.
  별도 복제한 로봇 크기를 사용하지 않는다. costmap padding은1cm다.
- 전·후방 스캔을 costmap과 Collision Monitor에 각각 입력한다. 배열 병합은 하지 않는다.
  기본 Collision Monitor는 두 스캔이 모두 필요하며 오래된 입력에서는 정지한다.
- `enable_stamped_cmd_vel: false`로 기존 C++ `geometry_msgs/Twist` 입력에 맞춘다.
- 필수 Nav2 서버만 실행하며 docking·route·waypoint 서버는 시작하지 않는다.
  초기 파라미터는 실제 Isaac Sim의 하중·슬립·정지 거리 검증 후 보정해야 한다.

## 실행 인자

| 인자 | 기본값 | 의미 |
| --- | --- | --- |
| `map` | 설치된 `amr_mapping/map/map.yaml` | 사용할 저장 지도 |
| `params_file` | 패키지 `config/nav2.yaml` | Nav2 튜닝 |
| `scan_topic` | `/scan/front` | AMCL 및 전방 장애물 입력 |
| `rear_scan_topic` | `/scan/rear` | 후방 장애물 입력 |
| `use_sim_time` | `true` | Isaac Sim `/clock` 사용 |
| `start_bringup` | `true` | 기존 주행·odom·RSP 실행 |
| `use_ekf` | `start_bringup` 값 | 휠 속도와 IMU Z축 각속도를 융합. `false`이면 기존 `/odom` 사용 |
| `ekf_params_file` | 패키지 `config/ekf.yaml` | EKF의 토픽 및 융합 설정 |
| `sim` | `false` | 기존 Isaac Sim 실행 스크립트 호출 |
| `headless` | `false` | 함께 실행하는 Isaac Sim GUI 생략 |
| `rviz` | `true` | RViz2 표시 |

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 topic info /initialpose -v
ros2 topic info /odom -v
ros2 topic echo /wheel/odom --once
ros2 topic echo /imu/data --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 action list
```

초기 위치를 준 뒤 AMCL pose 및 TF가 연결되고 `/navigate_to_pose` 액션이 있어야 한다.
다른 ROS domain, 시간 설정, 센서 TF 및 양쪽 scan 수신을 확인한다.

## 검사

```bash
colcon test --packages-select amr_navigation
colcon test-result --test-result-base build/amr_navigation --verbose
```

별도 ROS domain에서 실제 EKF·AMCL·Nav2와 C++ 주행·odom 노드를 실행한다. 합성 스캔,
IMU, 바퀴 피드백으로 초기 위치, 의도적으로 삽입한 슬립 중 횡이동, 90° 제자리 회전,
정지 상태의 각속도 바이어스, 목표 취소 및
Collision Monitor 정지를 확인한다. 실제 Isaac Sim의 물리 주행과 도착 오차는 별도로 검증한다.

저장 장면에서 좌·우 90° `NavigateToPose`를 한 번씩 비교했다. 교정 후 실제 회전각은
좌 86.9°, 우 85.6°였고 지도상 차체 중심 이동량은 좌 3.4 cm, 우 1.8 cm였다.
교정 전에는 좌 80.6°, 우 78.0°였고 지도상 이동량은 좌 6.0 cm, 우 9.5 cm였다.

정지 후 상대 yaw 융합 검증에서는 좌회전 목표 완료 0.45초 뒤부터 10초간 측정했다.
그 구간의 EKF yaw 변화량은 1.17°에서 0.04°로 줄었고, 차체의 실제 방향은 약 0.01° 이내로 유지됐다.

공식 참고: https://docs.nav2.org/ 및 https://api.nav2.org/nav2-jazzy/html/index.html
