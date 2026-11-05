# Compact mecanum AMR — Isaac Sim

이 폴더가 시뮬레이터 실행 진입점입니다. ROS용 원본 모델과 제원은 `../ros2_ws/src/amr_description/`에 두고, 여기서 불러와 지면·조명·물리 장면 및 바퀴 구동을 구성합니다. USD와 실행 스크립트를 ROS 패키지로 옮길 필요는 없습니다.

ROS 노드와 함께 시작하려면 Jazzy 및 워크스페이스 환경을 source한 ROS 터미널에서
`ros2 launch my_amr bringup.launch.py sim:=true`를 사용합니다.
기존 run.sh를 호출하면서 주행 제어·오도메트리·로봇 TF를 함께 실행합니다.
이미 시뮬레이터를 띄웠다면 `sim:=true`를 생략하고 ROS 노드만 시작합니다.
키보드는 별도 터미널에서 실행하며 [통합 실행 옵션](../docs/odometry-integration.md#통합-실행)을 참고하세요.

이 폴더는 **시뮬레이터 구성과 연결**을 담당합니다. 로봇 생성·가져오기, LiDAR·카메라·IMU, 센서 발행, ROS에서 받은 바퀴 명령의 조인트 적용은 여기서 개발합니다. `/cmd_vel` 처리·메카넘 계산·속도 제한·주행 timeout·오도메트리 등 **AMR 제어 로직은 `../ros2_ws/src/`의 ROS 2 C++ 패키지**에서 개발합니다. 기존 `mecanum.py`와 CLI 주행은 초기 검증용이며 운영 제어의 기준 구현으로 확장하지 않습니다. 상세 규칙은 [AGENTS.md](../AGENTS.md)의 개발 위치와 책임 경계를 따릅니다.

## Physics 편집 메뉴

GUI 실행에서는 `scene.py`가 `omni.kit.property.physx`와 의존 확장 `omni.physx.ui`를 매번 활성화합니다. 재실행 후에도 **Create → Physics**와 Property의 **Add → Physics**에서 물리 요소를 편집할 수 있도록 합니다. `--headless`에서는 편집 확장을 추가로 켜지 않습니다. 확장 활성화는 실행 설정이며, USD 저장만으로 다음 실행의 메뉴 활성화가 보장되지는 않습니다.

이미 열린 창에서는 **Window → Extensions**에서 `omni.kit.property.physx`를 검색하여 활성화하거나, 프로그램을 종료한 뒤 `bash run.sh --control ros`로 다시 실행하세요. Physics Material을 생성한 다음에는 실제 Collider의 Physics Material 항목에 연결하고 USD를 저장해야 접촉 마찰 설정이 유지됩니다.

## 휠·바닥 마찰과 회전 표시

`../ros2_ws/src/amr_description/config/compact_amr.json`의 `contact_materials`를
수정한 뒤 시뮬레이터를 재시작합니다. 현재 롤러와 바닥 각각 정지 마찰 **0.8**,
운동 마찰 **0.6**, 반발 **0**, 결합 방식 **average**를 사용합니다.
우레탄·에폭시 접촉을 위한 시뮬레이션 초기값이며 실측 물성값은 아닙니다.
`materials.py`가 32개 롤러의 Collider와 `/World/Ground`의 Collider에 서로 다른
Physics Material을 연결합니다. 새 장면과 저장 장면 모두 실행 시 JSON 값을 적용하므로
GUI에서만 바꾼 마찰값은 다음 실행에서 JSON 값으로 덮어씁니다.

`robot.py`는 저장된 USD의 롤러 축도 공통 URDF와 맞춥니다. 롤러는 자유 회전을
유지하며, 네 휠 허브 바깥쪽의 주황색 표시가 실제 휠과 함께 회전합니다.
Q/E에서는 좌우 휠이 서로 반대로 돌아야 합니다. 물리 조인트 적용과 자료형,
검증 결과는 [AMR 모델 수정 기록](../docs/amr-model-fix.md)을 참고하세요.

## 차체 참값과 자동 주행 검증

ROS 실행에서는 `ground_truth.py`가 실제 `base_link`의 월드 위치·자세를
`/ground_truth/pose` (`geometry_msgs/PoseStamped`, `frame_id=world`)로 발행합니다.
이 값은 측정 전용이며 `/odom`이나 로봇 TF를 발행하지 않습니다.
ROS C++ 자동 검증 실행과 결과 해석은 [주행 검증 기록](../docs/drive-validation.md)을 따릅니다.
검증 중 다른 키보드·제어 노드를 함께 실행하지 마세요.

실제 `/joint_states`에서 `/odom`을 계산하는 코드는 ROS C++ 패키지에 있습니다.
`ros2 launch my_amr odometry.launch.py`를 별도 ROS 터미널에서 실행하면
`odom → base_footprint → base_link`와 URDF 센서 프레임이 연결됩니다.
Isaac Sim의 두 scan 프레임과 RGB-D 광학 프레임은 서로 다른 이름으로 유지하여
TF 발행이 겹치지 않습니다. `/ground_truth/pose`는 오도메트리 입력으로 사용하지 않습니다.
[오도메트리 연동 기록](../docs/odometry-integration.md)에 실행·검증 방법을 정리했습니다.

## 소형 모델 제원

ROS 주행은 `bash run.sh --control ros`를 사용합니다(Bridge 활성화·CLI 속도 미지정 시 기본 모드). 이 경로는 `/cmd_vel`을 직접 구독하거나 메카넘 계산을 하지 않습니다. `ros2_ws`의 C++ 제어 노드가 발행한 `/wheel_commands`의 네 휠 이름·각속도 [rad/s]를 받아 조인트에 적용합니다. [my_amr 실행 안내](../ros2_ws/src/my_amr/README.md)에 따라 C++ 제어와 키보드를 별도 터미널에서 실행하세요. 키는 WASD 이동, Q/E 제자리 좌·우회전, J 정지, Ctrl+C 종료입니다.

`/World/AMRWheelCommands`에 명령 구독 그래프를 자동 생성합니다. 명령은 이름으로 매핑하고 통신 단절 시 독립적인 monotonic watchdog으로 정지합니다. 기존 `--vx/--vy/--wz`는 초기 검증용 CLI 모드를 선택하고, `--no-ros2`도 CLI 모드를 사용합니다. `--control ros`와 CLI 속도를 동시에 지정하면 오류를 표시합니다. ROS 모드에서는 `--drive-seconds`를 사용하지 않습니다.

| 항목 | 초기 설계값 |
| --- | --- |
| 전체 공간 예약 크기 | 길이 0.48 × 폭 0.44 × 높이 0.34 m (센서·휠 돌출 여유 포함) |
| 차체 본체 | 0.42 × 0.28 × 0.16 m |
| 공차 질량 | 20 kg, 센서·바퀴 포함 |
| 적재량 목표 | 10 kg. 현재 모델에는 적재물이 없으며 강도·모터 용량 미검증 |
| 휠 | 메카넘 4개, 직경 0.15 m, 바퀴당 수동 롤러 8개 |
| 휠베이스 / 좌우 휠 중심 거리 | 0.30 / 0.36 m |
| 최고 병진 속도 | **1.5 m/s**, `sqrt(vx² + vy²)` 기준 |
| 초기 테스트 속도 | 0.3 m/s |
| 최대 회전 속도 / 병진 가속도 목표 | 1.2 rad/s / 0.5 m/s² |
| 휠 속도·토크 한계 | 35 rad/s (약 334 rpm), 8 N·m. 시뮬레이션 초기값 |
| 지면 | 4 × 4 m 테스트 바닥 |

모든 수치는 하드웨어 선정 결과가 아닌 초기 시뮬레이션 설계값입니다. 최고 속도에서의 실제 주행 성능은 물리 시뮬레이션으로 검증해야 합니다.

센서 위치는 지면의 차체 중심을 원점으로 하고, X 전방·Y 좌측·Z 위쪽입니다.

| 센서 | 위치 `(x, y, z)` m | 목표 출력 |
| --- | --- | --- |
| 전방 좌측 LiDAR | `(0.205, 0.135, 0.30)`, yaw +45° | 270° / 20 Hz / 0.05–20 m |
| 후방 우측 LiDAR | `(-0.205, -0.135, 0.30)`, yaw −135° | 270° / 20 Hz / 0.05–20 m |
| 전방 RGB-D | `(0.225, 0, 0.235)` | 640×480 / 30 Hz, RealSense 계열을 가정한 일반 모델 |
| 중앙 IMU | `(0, 0, 0.19)` | 100 Hz |

ROS 2가 활성화된 실행에서는 물리 IMU, 실제 조인트 피드백, 두 PhysX LiDAR의 스캔과 전방 RGB-D 영상·카메라 정보를 발행합니다. 두 LiDAR의 배치는 수평 시야를 넓히기 위한 것이며, 차체 가림·근접 사각·낮은 장애물 검출을 보장하지 않습니다.

## 전방 RGB-D 카메라

`bash run.sh` 실행 시 `camera.py`가 전방 장착 링크에 실제 USD Camera와 `/World/AMRCamera` 그래프를 구성합니다. 수동 그래프 편집이나 RealSense ROS 드라이버 설치는 필요하지 않습니다. RealSense 계열의 배치를 가정한 **일반 pinhole RGB-D 모델**이며, 특정 제품의 스테레오 매칭·노이즈·왜곡을 구현한 것은 아닙니다.

| 토픽 | 타입 / 인코딩 | 내용 |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image`, `rgb8` | 렌더링한 컬러 영상 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image`, `32FC1` | 영상 평면 기준 Z 깊이, **미터** 단위 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 컬러 영상의 내부 파라미터 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 같은 광학계에 정렬된 깊이 영상의 내부 파라미터 |

해상도는 640×480, 수평 FOV는 87°, clipping range는 0.2–10 m입니다. 장착 위치는 기존 `(0.225, 0, 0.235)` m입니다. 로봇 +X 방향을 바라보며 메시지의 `frame_id`는 `camera_front_rgbd_optical_frame`(+Z 전방·+X 우측·+Y 아래)입니다. `base_link`에서 이 프레임으로 정적 TF를 발행합니다. 기존 URDF optical 링크와 충돌하지 않도록 별도 프레임 이름을 사용합니다.

RGB와 depth는 하나의 render product를 공유하므로 같은 시야에 정렬됩니다. `check_topics.py`는 두 영상이 같은 timestamp를 갖는 표본도 확인합니다. 가까운 물체가 없으면 깊이 값이 `inf` 또는 무효 값일 수 있으며, 검사 출력의 `center_depth_m: null`은 중앙 픽셀에 유효한 깊이가 없다는 뜻입니다.

제원·토픽은 `../ros2_ws/src/amr_description/config/compact_amr.json`의 `sensors.camera_front`에서 변경합니다. 목표 주기는 30 Hz이며, 현재 60 Hz Stage에서 `frameSkipCount=1`로 설정합니다. 제원의 주기는 Stage 렌더 기준 주기를 정수로 나눌 수 있어야 합니다. 실제 수신 속도는 GPU 처리량·초기화·DDS 전송에 따라 낮아질 수 있습니다.

```bash
# isaac_sim/에서 실행
bash run.sh

# 별도 시스템 Jazzy 터미널: 카메라를 포함한 모든 센서 검사
source /opt/ros/jazzy/setup.bash
python3 check_topics.py
```

RViz에서 Image 표시를 추가하고 `/camera/color/image_raw`를 선택합니다. Reliability는 Best Effort로 설정합니다. Camera 표시를 사용할 때는 Fixed Frame을 `base_link`로 두고 컬러 영상과 CameraInfo 토픽을 지정합니다. 정면에 시각 형상이 있는 물체를 두면 영상과 깊이를 확인할 수 있습니다. 카메라는 시각 형상을 렌더링하므로 LiDAR와 달리 Collider가 없어도 영상에 나타납니다.

센서와 그래프 설정은 메모리의 Stage에 적용하며 원본 `omni_amr.usd`는 자동 저장하지 않습니다. `--new-scene`도 같은 카메라를 구성합니다. `--no-ros2`는 자동 센서 구성을 생략합니다. GPU/ROS 검증 절차는 [카메라 연동 기록](../docs/camera-integration.md)을 참고하세요.

## 두 LiDAR 연동

기본 `bash run.sh` 실행 시 `lidar.py`가 센서와 `/World/AMRLidar` 그래프를 자동 구성합니다. 수동 Action Graph 편집은 필요하지 않습니다. Isaac Sim 5.1의 **PhysX raycast**를 사용하므로 감지할 장애물에 Collision이 설정되어 있어야 합니다. RTX 재질 반사·투과나 센서 노이즈를 모델링한 구현은 아닙니다.

| 토픽 | 타입 | frame_id |
| --- | --- | --- |
| `/scan/front` | `sensor_msgs/msg/LaserScan` | `lidar_front_left_scan` |
| `/scan/rear` | `sensor_msgs/msg/LaserScan` | `lidar_rear_right_scan` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `base_link` → 두 `*_scan` 프레임 |

각 스캔은 270°, 해상도 0.5°(540개 빔), 거리 0.05–20 m, 목표 20 Hz입니다. 설정은 `../ros2_ws/src/amr_description/config/compact_amr.json`의 두 `lidar_*` 항목에서 변경합니다. 토픽의 timestamp는 시뮬레이션 시간이며, 실제 초당 발행 수는 앱 실행 속도에 따라 달라집니다.

5.1 전용 LaserScan 발행 노드의 첫 메시지 시간 계산 오류를 피하기 위해 Bridge의 범용 `ROS2Publisher`를 사용합니다. 거리 배열은 실제 PhysX 데이터를 연결하며 `intensities`는 빈 배열입니다. 초기 빈 스캔은 발행하지 않습니다.

센서는 각 `lidar_*_link/LidarSensor`에 생성합니다. 장착 원통 자체를 감지하지 않도록 스캔 평면을 링크 중심보다 0.025 m 높게 두며, 초기 제원 기준 지면에서 0.325 m입니다. PhysX는 수직 FOV가 양수여야 하므로 수직 FOV/해상도를 각각 1°로 설정합니다. 설치된 5.1에서 실제 데이터가 **수평 1행, zenith 0**임을 확인했습니다. 이 높이보다 낮은 물체는 검출되지 않을 수 있습니다.

센서 TF는 Stage의 실제 장착 위치·방향에서 계산합니다. 별도의 `*_scan` 프레임을 사용하므로 `robot_state_publisher`가 발행하는 기존 장착 링크 TF와 중복되지 않습니다. `/odom`, `map` TF나 두 스캔의 병합은 아직 구현하지 않았습니다.

```bash
# 터미널 1: isaac_sim/에서 실행하고 Play 상태 유지
bash run.sh

# 터미널 2: 같은 ROS_DOMAIN_ID의 시스템 Jazzy 환경
source /opt/ros/jazzy/setup.bash
python3 check_topics.py
ros2 topic echo /scan/front --once --qos-reliability best_effort
ros2 topic echo /scan/rear --once --qos-reliability best_effort
ros2 topic hz /scan/front
ros2 topic hz /scan/rear
rviz2
```

RViz의 Fixed Frame을 `base_link`로 설정하고 LaserScan 표시 두 개에 `/scan/front`, `/scan/rear`를 지정합니다. Reliability는 Best Effort로 설정합니다. 기본 장면은 바닥만 있어 거리 값이 모두 최대 거리 20 m여도 정상입니다. 스캔 평면 높이를 가로지르는 Cube를 추가하고 Collision을 적용한 뒤 확인하세요.

원본 `omni_amr.usd`는 자동으로 덮어쓰지 않습니다. GUI에서 저장해도 다음 실행 시 센서·그래프를 재사용합니다. `--no-ros2`는 자동 구성을 생략하는 옵션이며, 이미 USD에 저장된 그래프를 삭제하는 옵션은 아닙니다.

`check_topics.py` 하나로 IMU·네 휠·두 LiDAR·센서 TF를 함께 검사합니다. 기존의 개별 토픽 검사 파일은 이 파일로 통합했습니다. 검증 기록과 장애물 이동 테스트 재현 방법은 [LiDAR 연동 기록](../docs/lidar-integration.md)을 참고하세요. 공식 근거: [5.1 PhysX LiDAR](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physx_lidar.html).

## IMU와 실제 휠 피드백

기본 `bash run.sh` 실행 시 `feedback.py`가 다음을 자동 구성합니다. `--no-ros2` 실행에서는 구성하지 않습니다.

| 토픽 | 타입 | 데이터 |
| --- | --- | --- |
| `/imu/data` | `sensor_msgs/msg/Imu` | 중앙 물리 IMU의 각속도·가속도·시뮬레이션 기준 자세, frame `imu_link` |
| `/joint_states` | `sensor_msgs/msg/JointState` | 로봇 articulation의 실제 관절 위치(rad)·속도(rad/s). 수동 롤러도 포함될 수 있음 |

센서는 `.../imu_link/ImuSensor`에 생성됩니다. 그래프는 `/World/AMRFeedback`에 두며 기존 Clock/명령 그래프를 삭제하지 않습니다. 휠 피드백은 드라이브 목표값을 복사하는 것이 아니라 Isaac Sim의 `ROS2PublishJointState`가 articulation의 실제 상태를 읽습니다. 메시지 배열 순서를 가정하지 말고 `name`으로 네 휠 조인트를 찾습니다.

설치된 5.1의 물리 센서 API를 사용합니다. IMU 센서 주기는 제원의 `sensors.imu.rate_hz`(현재 100 Hz)로 설정하고 필터 폭은 1입니다. 센서는 물리 시뮬레이션을 측정하고, 발행 그래프는 On Playback Tick에서 읽습니다. 따라서 토픽 발행은 앱 갱신 속도에도 제한되며 정확한 100 Hz 발행을 보장하지 않습니다. 실제 토픽 주기·중복 표본 여부는 `ros2 topic hz`와 timestamp로 확인하세요. 두 메시지의 header stamp는 Isaac Read Simulation Time을 사용합니다. 기존 `/clock`도 동일한 시뮬레이션 시간 설정으로 유지하세요.

`readGravity=True`이므로 평평한 바닥에서 정지한 IMU의 가속도 Z는 약 +9.81 m/s²가 정상입니다. 각속도 단위는 rad/s입니다. 센서는 현재 노이즈·바이어스를 추가하지 않은 이상적인 시뮬레이션 센서이며, 자세 출력은 실제 하드웨어의 자세 융합기를 대체하는 구현이 아닙니다. 오도메트리, ROS TF 발행 및 직진 방향 보정 제어기는 이번 단계에 포함하지 않습니다.

```bash
# 터미널 1: isaac_sim/에서 실행
bash run.sh

# 터미널 2: 시스템 Jazzy 환경. Isaac Sim과 ROS_DOMAIN_ID를 맞춥니다.
source /opt/ros/jazzy/setup.bash
ros2 topic echo /imu/data --once
ros2 topic echo /joint_states --once
python3 check_topics.py
# 주행 중 실제 바퀴 속도가 0이 아닌지도 검사하려면:
python3 check_topics.py --require-motion

# 각 명령은 따로 실행합니다.
ros2 topic hz /imu/data
ros2 topic hz /joint_states
```

위 통합 검사 스크립트는 `isaac_sim/` 경로에서 시스템 Python으로 실행하며 IMU·휠뿐 아니라 두 LiDAR와 센서 TF도 모두 수신해야 통과합니다. Play 상태에서 측정합니다. 휠 배열에서 `front_left_wheel_joint`, `front_right_wheel_joint`, `rear_left_wheel_joint`, `rear_right_wheel_joint`를 골라 표시합니다.

저속 전진 검증에는 `bash run.sh --vx 0.2 --drive-seconds 10`을 사용하고 바닥 밖으로 벗어나기 전에 종료합니다. 바퀴 속도 부호는 모두 양수, 감속 후에는 0 부근이어야 합니다. 회전 검증에는 `--wz 0.3`을 사용하며 IMU Z 각속도의 변화와 부호를 확인합니다. 목표값과 실제 값이 완전히 일치한다고 가정하지 않습니다.

설정은 실행 중 메모리의 Stage에 적용하며 기존 `omni_amr.usd`를 자동으로 덮어쓰지 않습니다. GUI에서 저장하면 다음 실행 시 같은 센서와 그래프 노드를 재사용합니다. 그래프 입력이 다른 곳에 연결됐거나 노드 타입이 다르면 덮어쓰지 않고 오류를 표시합니다. 실행 중 다른 Stage를 여는 경우에는 그 파일로 다시 실행해야 자동 피드백 구성이 적용됩니다.

오프라인 검증: `test/check_feedback.py`는 실제 USD와 설치된 5.1 노드 스키마를 이용해 그래프 연결·센서 주기·중복 생성 방지·잘못된 모델 거부를 검사합니다. 실제 물리 측정과 DDS 발행 검증은 별도로 수행해야 합니다.

2026-11-02 검증 결과: 오프라인 피드백 검사 7개와 기존 재생 검사 11개, 메카넘/모델 검사가 통과했습니다. 저장된 `omni_amr.usd`를 GPU 헤드리스 모드에서 열어 0.1 m/s 저속 명령을 주고, 별도 시스템 Jazzy 프로세스에서 IMU와 네 휠의 실제 상태를 수신했습니다. 가속 초기 휠 속도는 약 0.28–0.29 rad/s, IMU 가속도는 약 `[0.54, -0.03, 9.92]` m/s²였습니다. 두 프로세스 모두 종료 코드 0입니다. 이 값은 가속 중 한 표본이며 성능·오차 보정 완료를 뜻하지 않습니다. 회전 시 각속도 부호, 센서 주기 및 정지 상태 오차는 후속 검증 대상입니다.

공식 근거: [5.1 IMU 센서](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physics_imu.html), [ROS 2 Joint State](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html).

## 여기서 실행

Isaac Sim 자체 Python 환경을 사용합니다. 프로젝트 `.venv`나 ROS 시스템 Python으로 실행하지 않습니다.

```bash
cd /home/buddha/my_workspace/amr_with_issacsim/isaac_sim

# 기본값: ../../IsaacSim/python.sh (my_workspace/IsaacSim 설치)
# 다른 경로에 설치했다면 실행 파일을 지정합니다.
# export ISAAC_SIM_PYTHON=/path/to/isaac-sim/python.sh

# pip 설치라면 Isaac Sim을 설치한 환경의 Python을 지정합니다.
# export ISAAC_SIM_PYTHON=/path/to/isaac-venv/bin/python

bash run.sh                              # 저장한 omni_amr.usd 로딩 + ROS 2 Bridge 활성화
bash run.sh --scene scenes/other.usda     # 다른 저장 장면 선택
bash run.sh --new-scene --no-ros2         # 새 모델 생성, Bridge 비활성화
bash run.sh --vx 0.3                     # 전진, 2초 후 명령 정지
bash run.sh --vy 0.3                     # 횡이동, 2초 후 명령 정지
bash run.sh --wz 0.4                     # 제자리 회전
bash run.sh --vx 0.2 --vy 0.2             # 대각 이동
bash run.sh --headless --steps 240        # 지정 업데이트 횟수 후 종료
bash run.sh --help
```

`ISAAC_SIM_PYTHON`을 생략하면 프로젝트와 같은 상위 폴더의 `IsaacSim/python.sh`를 사용합니다. 현재 `/home/buddha/my_workspace/IsaacSim`에서 Isaac Sim 5.1.0 설치를 확인했습니다. 다른 설치를 사용할 때만 환경변수를 지정합니다. 런처는 `PYTHONPATH`와 `PYTHONHOME`을 해제해 기존 ROS Python 경로 유입을 막습니다.

Standalone 설치의 내장 `exts/isaacsim.ros2.bridge/jazzy/lib`가 있으면 시스템 ROS의 `LD_LIBRARY_PATH`·ament/colcon 경로도 분리하고, 내장 Jazzy 라이브러리와 기본 Fast DDS를 사용합니다. 다른 Python 실행 파일을 지정하면서 동일한 standalone 설치를 쓸 때는 `ISAAC_SIM_ROOT`에 해당 설치 루트를 지정할 수 있습니다. 외부 ROS 터미널은 시스템 Jazzy를 계속 사용합니다. Bridge 확장 등록 직후에는 앱 갱신을 완료한 다음 Stage를 교체해 초기화되지 않은 그래프에 접근하지 않도록 합니다.

기본 실행은 이 폴더의 `omni_amr.usd`를 직접 엽니다. 저장된 로봇과 Action Graph를 사용하며 URDF 재생성·로봇 중복 가져오기·지면 중복 생성·저장 파일 덮어쓰기를 하지 않습니다. 편집 후 `Ctrl+S`로 저장하세요. 파일이 없으면 오류를 표시하고 종료하며 빈 장면으로 대체하지 않습니다. Bridge가 필요 없는 테스트에는 `--no-ros2`를 사용합니다. Bridge는 Isaac Sim 내장 Jazzy 라이브러리를 사용하고 외부 ROS 노드와 `ROS_DOMAIN_ID`를 맞춥니다.

`--new-scene`을 지정할 때만 제원에서 URDF를 재생성하고 새 로봇·지면을 구성합니다. 생성된 정지 장면은 `build/amr_scene.usda`로 내보냅니다. `omni_amr.usd`는 변경하지 않습니다. 모터 4개만 속도 제어하며 롤러는 수동 회전합니다. 4.5/5.x URDF 명령과 6.x 새 importer API를 구분하는 코드가 있지만 실제 설치 버전별 실행 검증은 아직 하지 않았습니다.

실행 중 `File → Open/New`로 Stage를 바꾸면 기존 휠 핸들을 해제하고 CLI 제어를 중단한 채 UI를 유지합니다. 새 장면에서 CLI 주행을 하려면 그 파일을 `--scene`으로 지정해 다시 실행하세요. 이전 코드는 장면 변경 후 무효화된 휠 핸들에 접근해 예외로 종료될 수 있었습니다.

운영 ROS 모드의 메카넘 계산과 속도·가속도 제한은 C++ 제어 노드에서 수행합니다. 레거시 CLI 모드도 속도·가속도를 제한하고 `--drive-seconds` 경과 후 목표를 0으로 바꿉니다. 목표 0이 물리적인 정지 거리가 0이라는 뜻은 아닙니다. 작은 바닥이므로 최고 속도·장시간 주행 전 바닥 크기를 확장해야 합니다.

## ROS launch와의 관계

GUI 실행 시 `omni.graph.window.action`(OmniGraph Action Graph Editor)을 명시적으로 활성화합니다. `Window → Graph Editors → Action Graph`에서 편집기를 엽니다. 메뉴가 없으면 `Window → Extensions`에서 해당 확장의 활성화 상태를 확인하세요. ROS 2 Bridge와 IMU·휠·LiDAR 발행 그래프는 기본 활성화합니다. 기존 USD의 다른 그래프는 유지합니다.

- `ros2 launch amr_description display.launch.py`: RViz에서 형상·TF 확인용. Isaac Sim을 실행하지 않습니다.
- `bash run.sh`: Isaac Sim 모델·물리 주행 확인용. ROS launch 없이 실행합니다.
- Isaac Sim 센서와 Bridge를 검증한 다음 `amr_bringup` launch에서 이 실행 파일과 ROS 노드를 묶습니다. 현재 통합 launch는 없습니다.

## 코드 구성

| 파일 | 담당 기능 |
| --- | --- |
| `run_amr.py` | 제원 로딩·URDF 재생성, SimulationApp 시작·종료 |
| `cli.py` | 실행 인자 파싱·검증 |
| `scene.py` | 편집기 확장, Stage·지면·조명·물리 환경 구성 |
| `robot.py` | URDF 가져오기, 저장 장면의 롤러 축 정합, 휠 회전 표시 |
| `materials.py` | JSON의 롤러·바닥 마찰값을 실제 Collider에 연결 |
| `drive.py` | 휠 드라이브 설정, 수동 롤러 처리, 가속도 제한·속도 적용 |
| `ros_commands.py` | ROS 바퀴 명령 구독·이름 매핑·통신 watchdog. 메카넘 계산 없음 |
| `feedback.py` | 물리 IMU 생성, 실제 조인트 상태·IMU ROS 발행 그래프 구성 |
| `check_topics.py` | 별도 ROS 터미널에서 IMU·네 휠·두 LiDAR·RGB-D·CameraInfo·센서 TF를 한 번에 검사 |
| `camera.py` | 전방 USD Camera, RGB·깊이·카메라 정보·광학 프레임 TF 구성 |
| `lidar.py` | 두 PhysX 라이다 생성, LaserScan·센서 TF 발행 그래프 구성 |
| `test/run_lidar_fixture.py` | GPU/DDS 통합 검증용 이동 장애물 장면, 원본 USD 저장 안 함 |
| `test/run_model_fix_fixture.py` | SDK에서 실제 롤러 축·마찰 연결·회전 표시·중복 적용 검사 |
| `mecanum.py` | 시뮬레이터에 의존하지 않는 메카넘 역기구학·속도 제한 |
| `simulation.py` | 장면 조립·저장, 재생 루프와 종료 시 정지 처리 |

Kit/USD 모듈은 SimulationApp 생성 후에만 불러옵니다. 실행 명령과 CLI 옵션은 그대로 유지합니다. 기존 메카넘·모델 검사 외에 재생 루프의 시간 제한·가속도·단위 변환·오류 시 정지 처리를 오프라인으로 검사합니다.

## Action Graph 생성 시 빈 경로 오류

`Failed to create ComputeGraph ""`와 `MakeAbsolutePath(): anchor is the empty path`가 함께 나오면 Stage의 `defaultPrim`을 확인합니다. 실제 저장된 장면에서 `defaultPrim = "compact_mecanum_amr"`였지만 로봇은 `/World/compact_mecanum_amr`에 있어 유효하지 않은 루트 Prim을 가리켰습니다. 편집기는 이 빈 기준 경로로 새 그래프 경로를 계산하다 실패했습니다.

`robot.py`에서 URDF 가져오기 후 기본 Prim을 `/World`로 복원하고, 기존 importer의 `make_default_prim`도 비활성화합니다. 이는 `--new-scene`으로 생성할 때 적용됩니다. 기존 저장 장면의 기본 Prim이 잘못됐다면 Script Editor에서 다음 코드로 복원하고 `Ctrl+S`로 저장한 뒤 New Action Graph를 다시 누릅니다. 기존 그래프는 삭제하지 않습니다.

```python
import omni.usd
stage = omni.usd.get_context().get_stage()
stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
```

USD 라이브러리로 기존 파일의 빈 경로를 재현했고 `/World` 복원 후 `/World/ActionGraph`로 계산되는 것을 확인했습니다. `test/check_graph_anchor.py`는 실제 USD Stage와 모의 importer로 5.x/6.x 코드 경로의 회귀를 검사합니다. 실행 시 Isaac Sim USD 라이브러리가 필요하며 GUI 재클릭 검증은 별도입니다.

## 검증과 제한

장면 로딩 중 `Aborted (core dumped)`와 USD `Corrupt asset` 오류가 발생하면 [USD 손상 복구 기록](../docs/usd-scene-recovery.md)을 참고하세요. 원본을 먼저 백업하고 정상 커밋의 후보를 검증한 뒤 복구합니다.

```bash
python3 ../ros2_ws/src/amr_description/test/check_model.py
python3 test/check_mecanum.py
python3 test/check_simulation.py
python3 test/check_lidar.py
python3 test/check_camera.py
```

위 검사는 표준 라이브러리만 사용하며 링크 트리·질량·관성·롤러 축·바퀴 속도 여유·명령 속도 제한을 검증합니다. 원통 롤러는 단순화된 형상이라 접촉 진동과 슬립 튜닝이 필요합니다. 실제 주행 시 지면 마찰, 접촉 안정성, 롤러 형상, 모터 게인을 검증해야 합니다.

`my_workspace/IsaacSim`의 standalone 5.1.0 설치를 기본 런타임으로 사용합니다. 별도의 Python 3.10/Isaac Sim 4.5 설치 흔적 및 시스템 `isaacsim` 명령과 구분합니다. 2026-11-02 RTX 4060 Laptop GPU에서 헤드리스 물리 실행 및 별도 ROS 2 프로세스의 IMU·휠·두 LiDAR 수신을 검증했습니다. GUI/RViz 화면 검증은 별도입니다.

참고: [URDF importer 예제](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/import_urdf.html), [6.x importer 변경점](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/urdf_mjcf_importer_exporter_pipeline.html).
