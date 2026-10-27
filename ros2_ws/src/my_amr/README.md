# my_amr

ROS 2 Jazzy용 `ament_cmake` / C++17 패키지입니다. 기본 실행 노드 `my_amr_node`를 제공하며, 주행·제어 기능은 아직 구현하지 않았습니다. 라이선스는 Apache-2.0입니다.

## 패키지 생성 옵션

요청한 arguments는 `ros2 pkg create`의 선택 옵션으로 적용했습니다.

```bash
ros2 pkg create my_amr \
  --destination-directory ros2_ws/src \
  --build-type ament_cmake \
  --package-format 3 \
  --license Apache-2.0 \
  --description "C++ entry point for the AMR project with ROS 2 Jazzy and Isaac Sim." \
  --maintainer-name "$(git config user.name)" \
  --maintainer-email "$(git config user.email)" \
  --node-name my_amr_node \
  --dependencies rclcpp
```

관리자 정보는 저장소 Git 설정을 사용했습니다. 의존성은 현재 노드에 필요한 `rclcpp`만 추가했습니다. 이후 메시지/TF 사용 시 해당 의존성을 추가합니다.

## 빌드와 실행

프로젝트 루트에서 시작하며, 기존 Humble/venv 환경을 섞지 않도록 새 서브셸을 사용합니다.

```bash
env -i HOME="$HOME" USER="$USER" TERM="${TERM:-xterm}" PATH=/usr/bin:/bin bash --noprofile --norc
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select my_amr --symlink-install
source install/setup.bash
ros2 run my_amr my_amr_node
```

시작 로그를 출력한 뒤 대기하며 `Ctrl+C`로 종료합니다. 표준 ROS 실행 인자도 지원합니다.

```bash
ros2 run my_amr my_amr_node --ros-args \
  -r __ns:=/amr \
  -r __node:=amr_node \
  -p use_sim_time:=true \
  --log-level info
```

`use_sim_time:=true`는 시뮬레이터의 `/clock`을 사용할 때 지정합니다. 현재 노드는 구동 명령이나 TF를 발행하지 않습니다.

## 검증

```bash
colcon test --packages-select my_amr
colcon test-result --verbose
```

기본 `ament_lint_auto` / `ament_lint_common` 검사를 활성화했습니다. GitHub/Notion 원격 반영은 수행하지 않았습니다.
