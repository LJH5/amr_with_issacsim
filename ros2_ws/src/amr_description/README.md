# amr_description

소형 4륜 메카넘 AMR의 제원·URDF·RViz 표시 패키지입니다. Apache-2.0 라이선스를 사용합니다.

- 제원 원본: `config/compact_amr.json`
- 생성 모델: `urdf/compact_amr.urdf`
- 재생성: `python3 scripts/generate_robot.py` (이 패키지 경로에서 실행)
- 검사: `python3 test/check_model.py`

`config/compact_amr.json`의 `contact_materials`는 Isaac Sim용 롤러·바닥 접촉 설정입니다.
각각 정지 마찰 0.8, 운동 마찰 0.6, 반발 0, 결합 방식 average를 초기값으로 사용합니다.
시뮬레이터 시작 시 실제 Collider에 적용하며 ROS 제어 수식과는 분리합니다.
롤러 handedness는 +Y 속도의 기구학 계수이며 URDF의 실제 접촉 축과 일치해야 합니다.
수정 배경과 실행 검증은 [AMR 모델 수정 기록](../../../docs/amr-model-fix.md)을 참고하세요.

프로젝트 루트에서 Jazzy 환경을 로드한 뒤 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select amr_description --symlink-install
source install/local_setup.bash
ros2 launch amr_description display.launch.py
colcon test --packages-select amr_description
colcon test-result --verbose
```

표시에는 `ros-jazzy-joint-state-publisher-gui`와 RViz2가 필요합니다. 이 launch는 표시용 가상 joint state를 사용하므로 Isaac Sim과 동시에 실행하지 않습니다. 시뮬레이션에서는 실제 joint state와 TF 발행 책임을 정한 뒤 별도 launch로 연결합니다.

센서 형상과 좌표계가 포함되어 있지만 URDF만으로 측정 데이터가 생성되지는 않습니다. Isaac Sim 실행 방법·제원·구현 범위는 [isaac_sim/README.md](../../../isaac_sim/README.md)를 참고하세요. `config/compact_amr.json`의 IMU·LiDAR·RGB-D 카메라 제원과 토픽은 Isaac Sim 연동 코드가 사용합니다. command timeout은 후속 연동을 위한 설계 목표입니다.
