# 전방 RGB-D 카메라 연동

작업일: 2026-11-02. 브랜치: `feature/rgbd-camera`.

## 목표와 구성

전방 RGB·깊이 영상을 Isaac Sim 5.1에서 생성하고, ROS 2 Jazzy로 영상·CameraInfo·센서 TF를 발행한다. 기존 IMU·휠·LiDAR 검사를 유지하며 별도 검사 실행 파일을 늘리지 않는다.

`isaac_sim/camera.py`에서 기존 `camera_front_link` 아래에 `RgbdCamera`를 생성한다. `/World/AMRCamera`의 Render Product 노드가 640×480 offscreen 렌더링 자원을 구성하고, 두 `ROS2CameraHelper`가 RGB와 depth를 발행한다. 두 `ROS2CameraInfoHelper`는 각각의 CameraInfo 토픽을 발행한다. 두 영상은 같은 렌더 자원과 광학계를 공유하므로 정렬된 영상이다.

| 토픽 | 타입 | 설정 |
| --- | --- | --- |
| `/camera/color/image_raw` | Image | rgb8, 640×480 |
| `/camera/depth/image_raw` | Image | 32FC1, 깊이 단위 m |
| `/camera/color/camera_info` | CameraInfo | 같은 pinhole 광학계 |
| `/camera/depth/camera_info` | CameraInfo | 컬러에 정렬된 깊이 광학계 |
| `/tf_static` | TFMessage | base_link → camera_front_rgbd_optical_frame |

카메라는 +X 방향을 바라본다. USD 카메라의 -Z 전방/+Y 위 방향과 ROS optical의 +Z 전방/+Y 아래 방향을 구분해 변환한다. 기존 URDF의 optical 링크 TF와 중복되지 않도록 별도의 optical frame을 사용한다.

제원은 `ros2_ws/src/amr_description/config/compact_amr.json`의 `sensors.camera_front`에서 읽는다. 위치 `(0.225, 0, 0.235)` m, 수평 FOV 87°, clipping range 0.2–10 m, 목표 30 Hz다. 현재 Stage의 기준 60 Hz에 대해 helper의 `frameSkipCount=1`을 사용한다. 렌더 기준 주기를 정수로 나눌 수 없는 센서 주기는 오류로 처리한다. 수신 Hz는 초기화·GPU 부하·DDS 전송에 영향을 받으므로 정확한 실시간 30 Hz를 보장하지 않는다.

카메라의 초점 거리와 가로 aperture는 `fx = width / (2*tan(horizontal_fov/2))`를 만족하도록 설정한다. 세로 aperture는 해상도 비율에 맞춰 정사각 픽셀을 유지한다. 기본 `fx=fy`는 약 337.2096 px다.

## 실행과 검사

```bash
# isaac_sim/에서 실행
bash run.sh

# 별도 시스템 Jazzy 터미널
source /opt/ros/jazzy/setup.bash
python3 check_topics.py
```

통합 검사는 기존 센서와 함께 다음을 검사한다.

- RGB/depth 해상도·인코딩·프레임·바이트 길이·유효 timestamp.
- 두 영상에 공통된 timestamp 3개 이상.
- 두 CameraInfo의 해상도와 내부 파라미터.
- 광학 프레임의 위치·방향 TF.

`center_depth_m: null`은 영상 중앙에 유효한 깊이가 없는 경우다. 깊이를 무조건 0이나 최대 거리로 바꾸지 않는다. 원본 USD는 자동 저장하지 않는다.

## 검증 결과

Ubuntu 24.04 / ROS 2 Jazzy / standalone Isaac Sim 5.1.0 / RTX 4060 Laptop GPU 8 GB에서 헤드리스 GPU 렌더링과 별도 시스템 ROS 프로세스의 DDS 수신을 검증했다.

`test/run_camera_fixture.py`는 기존 USD를 메모리에 열고 카메라 정면에 시각·충돌 형상이 있는 표적을 추가한다. 표적은 처음 빨간색이며 표면 거리가 1.4 m이고, 시뮬레이션 4초 후 파란색·1.9 m로 변경한다. `check_topics.py --camera-fixture`는 두 색과 두 거리를 모두 받아야 통과한다.

- 빨간색·파란색 중앙 픽셀 변화와 깊이 near/far 단계 모두 수신.
- 이동 후 중앙 깊이 약 **1.899995 m** 수신.
- RGB 인코딩 rgb8, depth 인코딩 32FC1, 해상도 640×480 확인.
- 두 CameraInfo의 fx/fy 약 **337.209642 px** 확인.
- 카메라 optical TF와 기존 IMU·휠·두 LiDAR 수신 확인.
- 초기화 구간을 포함한 수신 표본의 simulation timestamp 기준 약 28.2 Hz 관측(목표 30 Hz).
- 시뮬레이터와 검사 프로세스 모두 종료 코드 0, 검사 오류 없음.
- 오프라인 카메라 검사 3개, 재생 검사 11개, LiDAR 검사 3개, 메카넘·모델 검사 통과.
- amr_description 패키지 colcon build/test 통과(패키지 검사 1개, 오류·실패 0).

재현 명령:

```bash
# 터미널 1, isaac_sim/에서 시스템 Jazzy
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=42 python3 check_topics.py --camera-fixture --seconds 180
```

```bash
# 터미널 2, isaac_sim/에서 SDK Python으로 실행
env -u PYTHONPATH -u PYTHONHOME -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH \
    LD_LIBRARY_PATH=/home/buddha/my_workspace/IsaacSim/exts/isaacsim.ros2.bridge/jazzy/lib \
    ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=42 \
    /home/buddha/my_workspace/IsaacSim/python.sh test/run_camera_fixture.py
```

기존 `test/run_lidar_fixture.py`에도 카메라 구성을 포함해 통합 검사와 함께 실행할 수 있다.

## 제한과 후속 작업

일반적인 이상적 pinhole RGB-D 센서이며 특정 RealSense 제품의 스테레오 알고리즘·시차·노이즈·왜곡을 재현하지 않는다. 깊이는 광선의 유클리드 거리가 아닌 영상 평면 기준 Z 거리다. RGB-D 점군 발행과 영상 인식 알고리즘은 포함하지 않는다. GUI/RViz 화면 검증과 장시간 주행 중 성능 측정은 별도로 수행한다.

추가 Python 의존성·RealSense 드라이버 설치는 없다. GitHub 커밋/PR 및 Notion 외부 게시를 수행하지 않았으며 이 문서를 개발 기록으로 제공한다.

커밋 메시지 초안: `update(camera): 전방 RGB-D 영상과 CameraInfo 및 ROS TF 연동`

공식 근거: [5.1 ROS 2 Cameras](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera.html), [5.1 발행 주기](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_publish_rate.html).
