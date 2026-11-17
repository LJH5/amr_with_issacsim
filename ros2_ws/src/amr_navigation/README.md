# amr_navigation

저장한 점유 지도와 설치된 Nav2/AMCL로 메카넘 AMR의 목표 이동을 실행한다.
Apache-2.0 launch·설정 패키지이며 위치 추정·계획·제어 알고리즘은 기존 Nav2를 사용한다.

## 실행

**매핑 launch와 키보드 주행을 먼저 종료한다.** SLAM과 AMCL의 `map→odom` 발행 및
키보드와 Nav2의 `/cmd_vel` 발행이 중복되면 안 된다.

```bash
cd /home/buddha/my_workspace/amr_with_issacsim/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to amr_navigation --symlink-install
source install/setup.bash

# Isaac Sim은 이미 실행 중이며 Play 상태일 때
ros2 launch amr_navigation navigation.launch.py

# Isaac Sim도 함께 시작하려면 위 명령 대신
ros2 launch amr_navigation navigation.launch.py sim:=true
```

기존 robot bringup이 실행 중이면 `start_bringup:=false`를 사용한다. 기본 지도는
`amr_mapping/map/map.yaml`과 상대 경로 `map.pgm`이다. 지도 폴더도 함께 설치한다.
다른 지도는 `map:=/절대/경로/map.yaml`로 선택하며 원본 지도 파일을 수정하지 않는다.

1. RViz2에서 Fixed Frame=`map`을 확인한다.
2. **2D Pose Estimate**로 실제 로봇의 지도상 위치와 방향을 지정한다.
3. 지도와 라이다 점이 맞는지 확인한다. 이번 구성의 AMCL은 `/initialpose`를 실제로 구독한다.
4. **Nav2 Goal**로 빈 공간의 목적지와 방향을 지정한다.
5. 필요하면 오른쪽 접힘 영역을 열어 Navigation 2 패널의 **Cancel**로 목표를 취소한다.

기본 화면에는 Displays만 표시한다. Views와 Tool Properties는 자동으로 열지 않는다.
Navigation 2 패널은 Nav2 Goal 연결을 위해 로드하되 오른쪽 도크에 숨긴다.
상단의 초기 위치·목표 지정 및 화면 이동 도구는 유지한다.

초기 위치를 지정하기 전에는 `map` TF 관련 대기 메시지가 나올 수 있다.
명령한 목표가 없을 때에는 자동으로 주행하지 않는다. 새 지도 작성은 기존
`amr_mapping`을 별도로 실행하며 navigation은 저장된 지도를 사용한다.

## 연결 및 메카넘 설정

```text
map_server ── /map ── AMCL(front scan, OmniMotionModel) ── map→odom
                                                               │
wheel_odometry ── odom→base_footprint ── RSP/sensor TF ───────────┘

Nav2 planner/controller → /cmd_vel_nav → velocity_smoother
    → /cmd_vel_smoothed → collision_monitor → /cmd_vel (Twist)
    → 기존 my_amr_node → /wheel_commands → Isaac Sim
```

- AMCL은 `/scan/front` 하나로 위치를 추정한다. 휠 odom TF는 기존 노드가 유지한다.
  IMU 융합은 추가하지 않는다. `map→odom`은 AMCL만 발행한다.
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
| `sim` | `false` | 기존 Isaac Sim 실행 스크립트 호출 |
| `headless` | `false` | 함께 실행하는 Isaac Sim GUI 생략 |
| `rviz` | `true` | RViz2 표시 |

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 topic info /initialpose -v
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

로봇 없는 별도 ROS domain에서 저장 지도를 읽고 실제 AMCL/Nav2와 기존 C++ 주행·odom
노드를 실행한다. 합성 스캔과 바퀴 피드백으로 초기 위치 입력·목표 이동·횡이동·취소·
Collision Monitor 정지를 확인한다. 실제 Isaac Sim의 물리 주행·도착 오차 검증과는 구분한다.

공식 참고: https://docs.nav2.org/ 및 https://api.nav2.org/nav2-jazzy/html/index.html
