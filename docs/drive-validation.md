# 메카넘 주행·마찰 검증

기록일: 2026-11-05. 작업 브랜치: `feature/drive-validation`.

## 목적과 구현 경계

같은 명령에서 실제 휠 회전으로 추정한 이동량과 Isaac Sim 차체의 실제 이동량을 비교한다.
속도 추종 오차·이동 오차·횡방향 편향·회전 중심 이동·정지 거리를 CSV로 기록한다.
이 차이에는 슬립 외에 유효 휠 반지름, 롤러 접촉 형상, 관절·물리 모델 오차도 포함되므로
CSV를 마찰계수의 직접 측정값으로 해석하지 않는다.

- `isaac_sim/ground_truth.py`: 실제 `base_link`의 월드 pose를 발행한다.
- `ros2_ws/src/my_amr/include/my_amr/mecanum_odometry.hpp`와 `src/mecanum_odometry.cpp`:
  휠 증분의 전방 기구학·SE(2) 적분 계산.
- `drive_validation.*`: 동시각 관절·참값 수집, 자동 주행·정지, CSV와 상태 기록.
- `launch/validation.launch.py`: 기존 C++ 제어 노드와 검증기를 함께 시작하고,
  검증기가 종료되면 제어 노드도 종료한다. Isaac Sim은 별도 실행한다.

참값은 측정용 `/ground_truth/pose`이며 `/odom`이나 TF를 발행하지 않는다.
제어기는 참값을 사용해 경로를 보정하지 않는다. 기존 `omni_amr.usd`를 자동 저장하거나
마찰·질량·가속도·롤러를 변경하지 않는다.

## 빌드와 실행

기존 키보드와 다른 `/cmd_vel` 발행자, 별도의 `my_amr_node`를 종료한다.
시험은 장애물이 없는 평평한 장면에서 로봇을 실제로 움직인다.

```bash
cd /home/buddha/my_workspace/amr_with_issacsim/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select amr_description my_amr --symlink-install
source install/setup.bash
```

터미널 1 — Isaac Sim 실행:

```bash
cd /home/buddha/my_workspace/amr_with_issacsim/isaac_sim
ROS_DOMAIN_ID=42 bash run.sh --control ros
```

터미널 2 — 먼저 짧은 1회 시험:

```bash
cd /home/buddha/my_workspace/amr_with_issacsim/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch my_amr validation.launch.py \
  repetitions:=1 drive_seconds:=1.0 speed_m_s:=0.1 \
  output_prefix:=log/drive_validation/urethane_epoxy_short
```

위 launch에 제어 노드가 포함되므로 `drive.launch.py`나 키보드를 동시에 실행하지 않는다.
양쪽 ROS_DOMAIN_ID를 맞추고 시뮬레이션을 재생 상태로 둔다. `/clock` 구독 없이 메시지의
시뮬레이션 timestamp로 시험 시간을 계산한다.

정지 상태를 유지할 수 있는지 확인한 뒤 기본 18회 시험을 실행한다.

```bash
ROS_DOMAIN_ID=42 ros2 launch my_amr validation.launch.py \
  output_prefix:=log/drive_validation/urethane_epoxy_08_06
```

기본 동작은 전진·후진·좌측·우측·좌회전·우회전 순서로 각 5초 주행 후 2초 정지,
6방향 묶음을 3회 반복한다. 최초 준비 구간까지 약 128초의 시뮬레이션 시간이다.
실제 소요 시간은 GPU·센서 부하에 따라 다르다.

각 시험의 시작 pose를 기준으로 변위를 0으로 만들지만 로봇을 순간이동·자동 재배치하지는
않는다. 다른 마찰 조건과 비교할 때는 같은 장면·시작 pose에서 다시 실행하고
새 output_prefix를 사용한다. 기존 결과 파일은 덮어쓰지 않는다.

## 입력·출력과 계산

| 인터페이스 | 내용 |
| --- | --- |
| `/cmd_vel` | 검증기가 발행하는 Twist, x/y m/s, yaw rad/s |
| `/wheel_commands` | 기존 제어기의 목표 각속도, rad/s |
| `/joint_states` | 실제 휠 위치 rad·속도 rad/s |
| `/ground_truth/pose` | PoseStamped, `frame_id=world`, 실제 base_link 위치 m·쿼터니언 |

참값은 `IsaacReadWorldPose`의 Fabric 월드 변환을 읽고 native ROS2Publisher로 발행한다.
입력 Prim은 실제 rigid body `base_link`다. IJKR/xyzw 쿼터니언을 ROS x/y/z/w 필드로
연결하며 그래프 경로는 `/World/AMRGroundTruth`다. 기존 그래프를 확인·재사용한다.
`IsaacComputeOdometry`의 시작 기준 변위를 world pose로 표시하지 않는다.

동일한 나노초 timestamp의 관절·pose만 짝지어 처리한다. 짝이 맞지 않는 표본을 섞어
비교하지 않으며 대기 버퍼는 최대 100개다. 최종 표본보다 오래된 중복·역순 표본은
무시하고, 큰 시각 역행은 실행을 무효화한다.

공통 `compact_amr.json`의 휠 반지름·간격·롤러 방향으로 기구학 행렬의 최소제곱 역행렬을
구성한다. FL, FR, RL, RR의 실제 관절 위치 증분을 차체 증분으로 바꾸고 SE(2) 지수맵으로
적분한다. 단순 `명령 속도 × 시간`을 기대 거리로 사용하지 않는다.

참값의 월드 변위를 시험 시작 yaw로 회전해 같은 시작 차체 좌표계에서 비교한다.
누적 yaw는 각 표본 간 차이를 unwrap해 계산한다. `/wheel_commands`는 현재 제어기에서
wall clock timestamp를 사용하므로 추종 RMS는 최신 수신 목표와의 근사 비교다.
위치 오차 비교에 사용하는 관절·pose의 시각 정렬과 구분한다.
기존 제어기의 가속도 제한은 steady clock을 사용하므로 물리 시간과 wall time의 배율이
달라지면 명령 상승 구간도 달라질 수 있다. 마찰 비교 시 같은 실행 조건을 유지하고
실제로 수신한 목표·휠 회전량을 함께 확인한다.

## 파라미터

아래 값은 검증기의 초기 시험 조건이며 제조사 보증 기준은 아니다.

| launch 인자 | 기본값 | 의미 |
| --- | --- | --- |
| `repetitions` | 3 | 6방향 묶음 반복 수, 1~100 |
| `drive_seconds` | 5.0 | 방향별 명령 유지 시간, simulation s |
| `settle_seconds` | 2.0 | 시작 준비 및 방향별 정지 관찰 시간, simulation s |
| `speed_m_s` | 0.2 | 병진 명령, 최대 0.3 m/s |
| `yaw_rate_rad_s` | 0.3 | 회전 명령, 최대 0.5 rad/s |
| `travel_limit_m` | 1.5 | 최초 측정 위치에서 허용하는 수평 변위 |
| `stop_linear_tolerance_m_s` | 0.02 | 정지 판정의 실제 차체 병진 속도 한계 |
| `stop_yaw_tolerance_rad_s` | 0.03 | 정지 판정의 실제 차체 yaw 속도 한계 |
| `stop_wheel_tolerance_rad_s` | 0.2 | 정지 판정의 각 휠 속도 크기 한계 |
| `settled_window_s` | 0.5 | 관찰 종료 전 연속 정지 상태가 필요한 기간 |
| `output_prefix` | 시각 포함 이름 | CSV·상태 파일 공통 경로 |

세 속도 조건을 모두 만족해야 정지로 판정한다. settled_window_s는 settle_seconds보다
클 수 없다. 관찰 시간이 부족한 경우 `settle_seconds:=5.0`으로 늘릴 수 있다.
계속 움직이는 로봇을 통과시키려는 목적으로 허용 오차만 늘리지 않는다.

처음에는 모든 데이터가 준비될 때까지 최대 30초 기다리며 0 명령을 유지한다.
이후 관절·pose 동기 표본 또는 목표 메시지가 wall time 1초 이상 끊기면 중단한다.
동기 표본 간격이 simulation time 0.25초를 넘거나 시각이 0.5초 넘게 역행해도 중단한다.
일시정지·장면 변경 이후에는 새 시험으로 다시 시작한다.

다른 cmd_vel 발행자 또는 두 번째 wheel_commands 발행자를 발견하면 중단한다.
이는 발견 가능한 중복 발행자를 검사하는 기능이며 DDS 검색 이전의 순간적인 경합까지
차단하는 명령 중재기는 아니다.

## 결과 파일

- `*_samples.csv`: 시험·반복·drive/stop 구간, 시각, 명령, 휠 적분 pose,
  시작 차체 좌표계의 실제 변위, 원본 world pose, 휠 위치·속도·최신 목표.
- `*_summary.csv`: 방향·반복별 전체 주행 및 정지 관찰을 포함한 최종 지표.
- `*_status.txt`: waiting, running, completed 또는 aborted와 원인.

| 요약 열 | 해석 |
| --- | --- |
| `wheel_x_m`, `wheel_y_m`, `wheel_yaw_rad` | 실제 휠 회전에서 추정한 누적 변위 |
| `truth_x_m`, `truth_y_m`, `truth_yaw_rad` | 시작 차체 좌표계의 실제 누적 변위 |
| `position_error_m` | 위 두 평면 변위 사이 거리 |
| `yaw_error_rad` | 휠 적분 yaw - 실제 누적 yaw |
| `cross_track_m` | 전후진의 실제 Y 편향, 횡이동의 실제 X 편향 |
| `rotation_center_drift_m` | 회전 시험의 실제 평면 중심 이동 거리 |
| `wheel_tracking_rmse_rad_s` | 주행 구간의 실제 휠 속도·최신 목표 간 RMS |
| `stop_path_m` | 0 명령 이후 정지 관찰 구간에서 움직인 경로 길이, 흔들림 포함 |
| `stop_time_s` | 0 명령 후 마지막으로 정지 한계를 넘긴 시점까지의 시간 |
| `settled` | 정지 관찰 끝에서 연속 정지 조건 충족 여부 |

정지하지 않으면 그 시험의 `settled=0` 행을 남기고 다음 방향으로 진행하지 않는다.
통신 단절 등 중간 실패에서는 완료되지 않은 시험의 요약 행이 없을 수 있다.
`completed`는 모든 측정을 수집했다는 의미이며 마찰·주행 성능 합격 판정은 아니다.
ROS launch 자체의 종료 코드만 보지 말고 상태 파일·summary를 함께 확인한다.

## 검증 결과와 남은 문제

### 계산·ROS 동작

- C++ gtest: 축별 전방 기구학, 유한 회전 호, 비영점 시작 방향, 특이 형상·NaN 거부.
- 패키지 빌드 성공. colcon 결과: 80 tests, 0 errors, 0 failures, 13 skipped.
  skipped에는 배포 cppcheck의 알려진 문제 등 도구의 생략 결과가 포함된다.
  네트워크 제한 환경의 XML 검증은 공식 XSD를 로컬 Catalog로 연결했다.
- 기존 simulation 오프라인 검사 13개 통과.
- 실제 C++ 프로세스와 합성 DDS 입력으로 6방향 × 3회 CSV 생성 검증.
  비영점 월드 시작 위치·yaw, 뒤집힌 휠 이름 순서, 10% 이동 차이를 주입해 비교했다.
- 목표 스트림이 늦게 시작해도 정지로 기다리는 회귀 검사, Ctrl+C 중단·0 명령 확인.
- 피드백 단절·시간 리셋·중복 명령 발행을 각각 주입해 중단과 0 휠 목표를 확인.
- SDK/GPU fixture에서 비영점 월드 위치·방향을 포함한 160개 표본을 PhysX 원본과 대조.
  위치 차이 최대값은 출력 정밀도에서 0.000000 m였고 쿼터니언 내적은 0.999 이상이었다.
  그래프를 두 번 구성해 재사용도 확인했다. 이 검사는 10 mm 위치 차이를 실패 기준으로 삼는다.

### 현재 저장 장면의 실제 주행

현재 USD SHA-256은
`567c4eb7681745698ab999c31ee81f2dc28dbb14598baf061bb19d6c88ada43c`다.
저장된 바닥과 롤러 재질은 각각 static 0.8 / dynamic 0.6이다.

저속 전진 0.1 m/s, 1초 명령 + 2초 정지 관찰에서 다음을 측정했다.

| 항목 | 결과 |
| --- | --- |
| 동기 표본 | 180개 |
| 휠 적분 X | 약 0.08837 m |
| 실제 X | 약 0.09042 m |
| 평면 위치 오차 | 약 0.00218 m |
| 실제 누적 yaw | 약 0.08569 rad, 약 4.91° |
| 정지 관찰 중 경로 길이 | 약 0.00551 m |
| 관찰 마지막 구간 yaw 속도 | 약 0.03752 rad/s |
| 정지 판정 | `settled=0`, 시험 중단 |

마지막 yaw 속도가 기본 허용값 0.03 rad/s보다 컸다. 정상 정지 또는 마찰 튜닝 완료로
기록하지 않는다. 다음으로 물리 접촉·롤러 형상·하중·구동 설정을 비교하고, 같은 제어
조건의 반복 시험으로 원인을 확인해야 한다. 이번 작업은 그 물리 설정을 수정하지 않았다.

실제 장면에서는 첫 방향에서 정지 조건이 실패해 나머지 방향·18회 전체 시험을 완료하지
않았다. 합성 입력의 18회 통과와 실제 물리 시험을 혼동하지 않는다. 원시 CSV·로그는
`ros2_ws/log/drive_validation/2026-11-05-native/`에 보존했다(Git 제외).

## 재현 검사

```bash
# ros2_ws/, 시스템 Jazzy와 install/setup.bash를 source한 상태
colcon test --packages-select my_amr amr_description
colcon test-result --verbose
ROS_DOMAIN_ID=45 python3 src/my_amr/test/validation_integration.py
```

DDS fixture는 시뮬레이터 없이 격리된 domain에서 실행한다.
SDK fixture는 isaac_sim에서 기존 sensor fixture와 같은 SDK 환경 분리 방식으로 실행한다.

```bash
env -u PYTHONPATH -u PYTHONHOME -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH \
  LD_LIBRARY_PATH=/home/buddha/my_workspace/IsaacSim/exts/isaacsim.ros2.bridge/jazzy/lib \
  ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=44 \
  /home/buddha/my_workspace/IsaacSim/python.sh test/run_ground_truth_fixture.py
```

fixture는 저장 USD를 메모리에서만 변경한다. 종료 직전 명시적인 PASS와 종료 코드를
확인한다. SDK의 전체 확장 해제 종료 모드에서 종료 시 충돌이 관측돼 기본 fast shutdown을
사용한다. 시험 예외는 별도 비정상 종료로 처리해 SDK 종료가 실패를 숨기지 않게 했다.

공식 근거: [Isaac Sim 5.1 Read World Pose](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.nodes/docs/ogn/OgnIsaacReadWorldPose.html),
[범용 ROS 2 Publisher](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2Publisher.html).

Git commit/push는 수행하지 않았다. 커밋 메시지 초안:
`update(validation): 차체 참값과 자동 메카넘 주행 검증 및 CSV 기록 추가`.

Notion 진행 기록: [05. 자동 주행 검증](https://www.notion.so/40fe0bf3f929816eb4a4d3b6ef150d4f).

## 오차 원인 추가 분석

[롤러·기구학 정합성 분석](roller-drift-analysis.md)에서 실제 롤러 축과 기구학 수식의
좌우 부호 불일치를 확인했다. 별도 복사 모델에서 자유 회전을 유지하고 축 방향만 맞추면
전진 후 3초 yaw가 약 5.17°에서 0.026°로 줄었다. +vy 이동이 기존 모델에서는 반대로
나타나는 것도 확인했다. 원본 모델 수정·전체 18회 물리 재검증은 후속 작업이다.
