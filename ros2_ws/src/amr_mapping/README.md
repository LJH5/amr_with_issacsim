# amr_mapping

설치된 ROS 2 Jazzy SLAM Toolbox의 `async_slam_toolbox_node`로 2D 지도를 만든다.
이 패키지는 Apache-2.0의 launch·설정이며 SLAM 알고리즘을 재구현하지 않는다.
SLAM Toolbox와 Nav2는 각각의 원래 라이선스를 따른다.

## 빌드 및 실행

```bash
cd /home/buddha/my_workspace/amr_with_issacsim/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to amr_mapping --symlink-install
source install/setup.bash

# Isaac Sim은 이미 실행 중이고 Play 상태일 때
ros2 launch amr_mapping mapping.launch.py

# Isaac Sim도 함께 시작하려면 위 명령 대신
ros2 launch amr_mapping mapping.launch.py sim:=true
```

기본으로 주행 제어·휠 오도메트리·robot_state_publisher·SLAM·RViz를 시작한다.
기존 bringup이 켜져 있다면 `start_bringup:=false`로 중복 실행을 피한다.
자동 검증·다른 SLAM 노드는 종료하고 하나의 제어기만 사용한다.
키보드는 별도 실제 터미널에서 ROS 환경을 source한 뒤 실행한다.

```bash
ros2 run my_amr keyboard_teleop --ros-args -p speed_m_s:=0.1 -p yaw_rate_rad_s:=0.2
```

WASD 이동, Q/E 회전, J 정지다. 키를 놓는 것만으로 정지하지 않는다.
물리적으로 안정된 로봇과 충돌 설정된 벽·비대칭 장애물이 있는 공간에서 저속으로 탐색한다.

## 인자 및 데이터

| 인자 | 기본값 | 의미 |
| --- | --- | --- |
| `scan_topic` | `/scan/front` | 실제 SLAM 파라미터에 전달할 LaserScan 토픽 |
| `use_sim_time` | `true` | Isaac Sim `/clock` 사용 |
| `params_file` | 패키지 `config/slam.yaml` | SLAM 설정 파일 |
| `start_bringup` | `true` | 주행·오도메트리·로봇 TF 시작 |
| `sim` | `false` | bringup에서 기존 Isaac Sim run.sh 실행 |
| `headless` | `false` | 함께 시작하는 시뮬레이터 GUI 생략 |
| `rviz` | `true` | RViz 지도·스캔·로봇 표시 |

예: `ros2 launch amr_mapping mapping.launch.py scan_topic:=/scan/rear rviz:=false`.
현재 선택한 한 라이다만 사용하며 두 스캔을 자동 병합하지 않는다.
`params_file`에 토픽이 있어도 명시적인 launch 인자 `scan_topic`이 우선한다.
거리 한계는 20m, 지도 해상도는 5cm, 지도 발행 주기는 시뮬레이션 시간 기준 1초다.
라이다 거리 설정을 바꾸면 `max_laser_range`도 맞춘다. 이 값들은 초기 설정이며
물리 주행·지도 정확도와 별개로 검증해야 한다.

```text
map --SLAM--> odom --wheel_odometry--> base_footprint --RSP--> base_link
                                                               └--Isaac Sim--> lidar_front_left_scan
```

SLAM은 선택한 LaserScan과 그 시각의 TF를 사용한다. IMU·카메라는 직접 입력하지 않는다.
시작하면 lifecycle을 configure→activate하여 수동 활성화가 필요 없다.
정상 상태에서 `/map`을 발행하고 `map→odom`을 보정한다. 정지 상태에서도 유효한
최초 스캔이 있으면 초기 지도를 만들 수 있다. 새 공간은 이동·회전하며 탐색한다.

## RViz 및 연결 확인

동봉한 RViz 설정의 Fixed Frame은 `map`, Map 토픽은 `/map`, LaserScan 토픽은
선택한 `scan_topic`이다. Map은 Transient Local, LaserScan은 Best Effort를 사용한다.

상단 도구에 Move Camera와 **2D Pose Estimate**를 표시한다. 2D Pose Estimate는
지도에서 클릭·드래그한 위치·방향을 `/initialpose`로 발행한다. 현재 async mapping
노드는 이 토픽을 사용한 위치 초기화를 제공하지 않는다. AMCL 또는 SLAM Toolbox
localization 구성에서 사용하며 시뮬레이터의 로봇을 직접 이동시키지는 않는다.

```bash
ros2 lifecycle get /slam_toolbox
ros2 param get /slam_toolbox scan_topic
ros2 topic info /scan/front
ros2 run tf2_ros tf2_echo map odom
```

기본 실행에서 active, `/scan/front`, 스캔 발행자·구독자 각 1개 이상을 확인한다.
active인데 map이 없으면 스캔 연결과 `odom→base_footprint→scan` TF를 먼저 확인한다.
TF를 임의의 정적 `map→odom`으로 대체하지 않는다.

## 지도 저장

점유 지도는 설치된 Nav2 저장기로, pose graph는 SLAM Toolbox 서비스로 저장한다.
아래 경로는 예시이며 다른 이름으로 기존 지도를 보존한다.

```bash
mkdir -p /tmp/amr_maps
ros2 run nav2_map_server map_saver_cli -f /tmp/amr_maps/room --ros-args \
  -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/tmp/amr_maps/room'}"
```

CLI 성공 종료, 서비스 응답 result=0과 `.yaml`/`.pgm`, `.posegraph`/`.data` 파일을 확인한다.
`/tmp`는 재부팅 시 지워질 수 있으므로 보존할 지도는 별도 경로에 저장한다.
기본 실행은 저장 지도를 불러오지 않는 새 mapping이다.

현재 설치 조합의 통합 시험에서 SLAM 내부 `/save_map` 서비스가 실행하는 저장기는
지도 수신 시간 초과로 실패했다. 별도 Nav2 CLI로 저장을 검증했으며, 이 구성에서는
`use_map_saver: false`로 내부 저장 서비스를 끈다. 설치된 SLAM 소스는 수정하지 않는다.

## 검증

```bash
colcon test --packages-select amr_mapping
colcon test-result --test-result-base build/amr_mapping --verbose
```

검사는 로봇 없는 별도 ROS domain에서 native SLAM을 실행하고 합성 room 스캔으로
자동 활성화·토픽 인자·초기 지도·횡이동 위치 추정·TF·지도 저장을 확인한다.
실제 Isaac Sim 장면의 정확도나 차체 정지 안정성을 증명하는 검사는 아니다.

2026-11-16 검증: 의존 패키지 포함 빌드 성공, 모델 검사와 매핑 통합 검사 통과.
전방 기본 입력 및 후방 토픽 인자 변경에서 모두 초기 지도·자동 활성화·TF·네 가지
지도 파일 저장을 확인했다. 합성 횡이동 -0.8m에 대한 map 기준 변위는 각각 약
-0.799m / -0.800m였다. 시험 자료는 `/tmp/amr-mapping-x040lzwa/`에 있다.

공식 참고: https://github.com/SteveMacenski/slam_toolbox
