# 두 LiDAR의 Isaac Sim 5.1 / ROS 2 연동

날짜: 2026-11-02. 작업 브랜치: `feature/lidar-feedback`.

## 목표와 구현

대각선 장착 LiDAR 두 개가 실제 시뮬레이션 장애물의 거리를 측정하고, ROS 2 Jazzy에서 표준 `LaserScan`으로 수신되도록 연결한다.

- `isaac_sim/lidar.py`: PhysX 센서 생성, 기존 센서·그래프 재사용, 모델/설정 검증, 스캔과 정적 TF 발행.
- `isaac_sim/scene.py`: Stage 로딩 전에 PhysX 센서 확장 활성화.
- `isaac_sim/simulation.py`: 기존 IMU 구성 다음에 LiDAR 구성 호출.
- `ros2_ws/src/amr_description/config/compact_amr.json`: 각 센서에 `horizontal_resolution_deg: 0.5` 추가. 기존 270°·20 Hz·0.05–20 m 제원을 사용.
- `isaac_sim/check_topics.py`: IMU·휠·LiDAR 검사를 하나로 통합한 실행 파일. 시스템 Jazzy에서 실제 휠 상태·IMU 값과 두 스캔의 배열·프레임·각도·시간·거리·TF를 확인한다. 라이다마다 세 개 이상의 서로 다른 시각 표본을 요구한다. `--require-motion`과 `--fixture` 옵션을 함께 제공한다.

ROS 라이브러리는 기존 Isaac Sim 내장 Bridge를 사용하며 Python 패키지를 추가 설치하지 않는다. 센서별 `ReadLidarBeams → 유효성 분기 → ROS2Publisher(LaserScan)`을 연결한다. Context와 Simulation Time/Time Splitter는 그래프 안에서 공유한다. 범용 Publisher의 `ranges`에는 PhysX에서 읽은 실제 거리를 연결하고, 각도·거리 한계·시간 간격은 동일한 센서 제원에서 설정한다. `intensities`는 비워 둔다. LiDAR 그래프는 `/World/AMRLidar`이며 기존 Clock·IMU 그래프를 삭제하지 않는다.

스캔 프레임은 `lidar_front_left_scan`, `lidar_rear_right_scan`이고, `base_link` 기준 실제 Stage 변환을 `/tf_static`으로 발행한다. 기존 링크 TF와 중복되지 않도록 별도의 스캔 프레임을 사용한다. 장착 원통을 피하도록 원점은 링크 중심 위 25 mm에 둔다. 기본 모델의 스캔 평면은 지면에서 0.325 m다.

두 스캔을 `/scan` 하나로 병합하거나 `/odom`·`map` TF를 발행하지 않는다. SLAM 연결 시 스캔 병합과 오도메트리 구성이 별도로 필요하다. 카메라 구현은 포함하지 않는다.

## 검증

환경: Ubuntu 24.04, ROS 2 Jazzy, standalone Isaac Sim 5.1.0, RTX 4060 Laptop GPU 8 GB.

오프라인 검사:

```bash
# isaac_sim/에서 실행
python3 test/check_lidar.py
python3 test/check_simulation.py
python3 test/check_mecanum.py
python3 ../ros2_ws/src/amr_description/test/check_model.py
```

LiDAR 검사 3개, 재생 검사 11개, 메카넘·모델 검사를 통과했다. 기존 IMU/조인트 오프라인 검사 7개도 Isaac Sim USD 라이브러리로 실행해 통과했다. `colcon build --packages-select amr_description --symlink-install`, `colcon test --packages-select amr_description` 및 `colcon test-result --verbose`도 통과했다(패키지 검사 1개, 오류·실패 0개).

실제 GPU/DDS 통합 검증은 서로 다른 Python 환경의 두 프로세스로 수행한다. 아래 명령은 각각 별도 터미널에서 실행한다. 테스트는 기존 `omni_amr.usd`를 열지만 저장하지 않는다.

```bash
# 터미널 1: isaac_sim/에서 시스템 Python으로 먼저 수신 시작
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=42 python3 check_topics.py --fixture --seconds 90
```

```bash
# 터미널 2: isaac_sim/에서 standalone SDK의 Python으로 실행
env -u PYTHONPATH -u PYTHONHOME -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH \
    LD_LIBRARY_PATH=/home/buddha/my_workspace/IsaacSim/exts/isaacsim.ros2.bridge/jazzy/lib \
    ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=42 \
    /home/buddha/my_workspace/IsaacSim/python.sh test/run_lidar_fixture.py
```

테스트 장면은 각 라이다의 정면에 충돌 Cube를 추가하고, 시뮬레이션 4초 후 센서에서 표면까지 거리를 1.1 m에서 1.6 m로 이동한다. 각 센서가 양쪽 거리를 모두 측정해야 `--fixture` 검사가 통과한다. 구성을 두 번 호출해 센서·그래프 중복 생성도 확인한다. 실제 PhysX 데이터에서 수직 1행과 zenith 0을 확인한다.

실측 결과: 두 토픽 모두 540개 빔, 스캔 시간 약 0.05 s. 이동 후 전방 약 1.5999997 m, 후방 약 1.5999957 m를 수신했다. 일반 `run.sh --headless --steps 240` 경로에서는 장애물이 없는 평면에서 최대 거리 20 m를 확인했으며, simulation timestamp 기준 발행 간격은 약 20 Hz였다. 최종 범용 Publisher 구성으로 장애물 테스트와 기본 실행 모두 첫 스캔부터 시간 필드 오류 없이 통과했다. 기존 `/imu/data`, `/joint_states` 수신도 함께 확인했다. 최종 시뮬레이터와 각 ROS 수신 검사 프로세스는 종료 코드 0이다.

## 발생한 문제와 해결

1. PhysX LiDAR의 수직 FOV 0은 5.1에서 거부된다. 수직 FOV와 해상도를 각각 1°로 설정하면 실제로 수평 1행(zenith 0)이 생성된다.
2. 5.1 `IsaacReadLidarBeams`는 시작 직후 `numCols=0`, `numRows=0`, `rotationRate=0`인 초기 상태가 있다. `numCols > 1`을 확인하는 Compare와 Branch 노드를 추가해 데이터가 준비된 뒤에만 발행한다.
3. 빈 스캔 차단 후에도 전용 `ROS2PublishLaserScan`의 첫 메시지에서 `time_increment=inf`가 발생했다. 공식 5.1 소스에서 `writeData()`가 `generateBuffers()`보다 먼저 호출되고, 시간 계산이 아직 빈 `ranges.size`로 나누는 것을 확인했다. 범용 `ROS2Publisher`를 `sensor_msgs/msg/LaserScan`으로 설정해 우회했다. `time_increment = horizontal_resolution_deg / (360 × rate_hz)`를 사용하며, 헤더 시각은 Time Splitter의 초/나노초 출력에 연결한다. 설치된 SDK를 수정하거나 별도의 중계 노드를 추가하지 않는다. 수신 검사는 시작 시점을 포함해 무한대 시간 필드를 오류로 처리하며 최종 GPU/DDS 테스트에서 오류 없이 통과했다.
4. 시스템 Jazzy와 SDK Python/라이브러리 경로를 혼합하지 않는다. 기존 `run.sh`의 환경 분리 방식을 검증 스크립트 실행에도 적용한다.

## 제한 및 추적

- 거리 측정은 충돌 형상 기준의 이상적인 PhysX raycast다. 반사율·유리 투과·노이즈·센서 바이어스는 모델링하지 않는다.
- 낮은 장애물과 근접 사각을 모두 검출한다는 보장은 없다.
- 센서 TF는 설정 시점의 장착 위치 기준이며, 실행 중 장착 위치를 변경했다면 다시 실행한다.
- GUI/RViz 표시, SLAM 및 주행 중 모션 왜곡 검증은 후속 작업이다.
- 원본 USD의 빈 링크 visual 참조 경고는 기존 파일에 존재한다. 이번 센서 검증을 막지 않았으며 원본 파일은 수정하지 않았다.
- GitHub 커밋/PR과 Notion 외부 게시를 수행한 기록은 없다. 이 문서를 개발 기록으로 사용한다.

커밋 메시지 초안: `update(lidar): 대각 LiDAR 스캔과 ROS 2 센서 TF 연동`

공식 근거: [5.1 PhysX LiDAR](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physx_lidar.html), [범용 ROS 2 Publisher](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2Publisher.html), [전용 발행 노드 소스](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.ros2.bridge/nodes/OgnROS2PublishLaserScan.cpp), [시간 계산 구현](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.ros2.bridge/library/backend/Ros2Message.cpp).
