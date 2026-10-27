# Compact mecanum AMR — Isaac Sim

이 폴더가 시뮬레이터 실행 진입점입니다. ROS용 원본 모델과 제원은 `../ros2_ws/src/amr_description/`에 두고, 여기서 불러와 지면·조명·물리 장면 및 바퀴 구동을 구성합니다. USD와 실행 스크립트를 ROS 패키지로 옮길 필요는 없습니다.

## 소형 모델 제원

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

**현재 센서는 장착 형상과 TF 좌표계만 있습니다.** LiDAR 스캔, RGB-D 이미지, IMU 측정값 및 ROS 2 Bridge는 아직 구현하지 않았습니다. 두 LiDAR의 배치는 수평 시야를 넓히기 위한 것이며, 차체 가림·근접 사각·낮은 장애물 검출을 보장하지 않습니다.

## 여기서 실행

Isaac Sim 자체 Python 환경을 사용합니다. 프로젝트 `.venv`나 ROS 시스템 Python으로 실행하지 않습니다.

```bash
cd /home/buddha/my_workspace/amr_with_issacsim/isaac_sim

# 기본값: ../../IsaacSim/python.sh (my_workspace/IsaacSim 설치)
# 다른 경로에 설치했다면 실행 파일을 지정합니다.
# export ISAAC_SIM_PYTHON=/path/to/isaac-sim/python.sh

# pip 설치라면 Isaac Sim을 설치한 환경의 Python을 지정합니다.
# export ISAAC_SIM_PYTHON=/path/to/isaac-venv/bin/python

bash run.sh                              # 정지 상태로 모델 확인
bash run.sh --vx 0.3                     # 전진, 2초 후 명령 정지
bash run.sh --vy 0.3                     # 횡이동, 2초 후 명령 정지
bash run.sh --wz 0.4                     # 제자리 회전
bash run.sh --vx 0.2 --vy 0.2             # 대각 이동
bash run.sh --headless --steps 240        # 지정 업데이트 횟수 후 종료
bash run.sh --help
```

`ISAAC_SIM_PYTHON`을 생략하면 프로젝트와 같은 상위 폴더의 `IsaacSim/python.sh`를 사용합니다. 현재 `/home/buddha/my_workspace/IsaacSim`에서 Isaac Sim 5.1.0 설치를 확인했습니다. 다른 설치를 사용할 때만 환경변수를 지정합니다. 런처는 `PYTHONPATH`와 `PYTHONHOME`을 해제해 기존 ROS Python 경로 유입을 막습니다.

실행 시 제원에서 URDF를 재생성하고, 이동 가능한 로봇으로 가져오며, 모터 4개만 속도 제어합니다. 롤러 조인트는 구동하지 않습니다. 생성된 정지 장면은 `build/amr_scene.usda`에 저장하며 `build/`는 Git 제외 대상입니다. 4.5/5.x URDF 명령과 6.x 새 importer API를 구분하는 코드가 있지만, 실제 설치 버전별 실행 검증은 아직 하지 않았습니다.

속도 명령은 가속도를 제한하고 병진 속도 벡터를 1.5 m/s로 제한합니다. `--drive-seconds` 경과 후 속도 목표를 0으로 바꾸지만 물리적인 정지 거리가 0이라는 뜻은 아닙니다. 작은 바닥이므로 최고 속도·장시간 주행 전 바닥 크기를 확장해야 합니다. 현재 CLI 속도 명령은 ROS `/cmd_vel`과 연결되어 있지 않습니다.

## ROS launch와의 관계

GUI 실행 시 `omni.graph.window.action`(OmniGraph Action Graph Editor)을 명시적으로 활성화합니다. `Window → Graph Editors → Action Graph`에서 편집기를 엽니다. 메뉴가 없으면 `Window → Extensions`에서 해당 확장의 활성화 상태를 확인하세요. 이 설정은 그래프 편집기용이며 ROS 2 Bridge와 토픽 그래프를 자동 구성하지는 않습니다.

- `ros2 launch amr_description display.launch.py`: RViz에서 형상·TF 확인용. Isaac Sim을 실행하지 않습니다.
- `bash run.sh`: Isaac Sim 모델·물리 주행 확인용. ROS launch 없이 실행합니다.
- Isaac Sim 센서와 Bridge를 검증한 다음 `amr_bringup` launch에서 이 실행 파일과 ROS 노드를 묶습니다. 현재 통합 launch는 없습니다.

## 코드 구성

| 파일 | 담당 기능 |
| --- | --- |
| `run_amr.py` | 제원 로딩·URDF 재생성, SimulationApp 시작·종료 |
| `cli.py` | 실행 인자 파싱·검증 |
| `scene.py` | 편집기 확장, Stage·지면·조명·물리 환경 구성 |
| `robot.py` | Isaac Sim 버전에 맞는 URDF 가져오기 |
| `drive.py` | 휠 드라이브 설정, 수동 롤러 처리, 가속도 제한·속도 적용 |
| `mecanum.py` | 시뮬레이터에 의존하지 않는 메카넘 역기구학·속도 제한 |
| `simulation.py` | 장면 조립·저장, 재생 루프와 종료 시 정지 처리 |

Kit/USD 모듈은 SimulationApp 생성 후에만 불러옵니다. 실행 명령과 CLI 옵션은 그대로 유지합니다. 기존 메카넘·모델 검사 외에 재생 루프의 시간 제한·가속도·단위 변환·오류 시 정지 처리를 오프라인으로 검사합니다.

## Action Graph 생성 시 빈 경로 오류

`Failed to create ComputeGraph ""`와 `MakeAbsolutePath(): anchor is the empty path`가 함께 나오면 Stage의 `defaultPrim`을 확인합니다. 실제 저장된 장면에서 `defaultPrim = "compact_mecanum_amr"`였지만 로봇은 `/World/compact_mecanum_amr`에 있어 유효하지 않은 루트 Prim을 가리켰습니다. 편집기는 이 빈 기준 경로로 새 그래프 경로를 계산하다 실패했습니다.

`robot.py`에서 URDF 가져오기 후 기본 Prim을 `/World`로 복원하고, 기존 importer의 `make_default_prim`도 비활성화합니다. 수정 후 `bash run.sh`로 다시 실행하면 반영됩니다. 현재 열린 장면을 유지하려면 Script Editor에서 다음 코드로 복원한 뒤 New Action Graph를 다시 누릅니다. 기존 그래프는 삭제하지 않습니다.

```python
import omni.usd
stage = omni.usd.get_context().get_stage()
stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
```

USD 라이브러리로 기존 파일의 빈 경로를 재현했고 `/World` 복원 후 `/World/ActionGraph`로 계산되는 것을 확인했습니다. `test/check_graph_anchor.py`는 실제 USD Stage와 모의 importer로 5.x/6.x 코드 경로의 회귀를 검사합니다. 실행 시 Isaac Sim USD 라이브러리가 필요하며 GUI 재클릭 검증은 별도입니다.

## 검증과 제한

```bash
python3 ../ros2_ws/src/amr_description/test/check_model.py
python3 test/check_mecanum.py
python3 test/check_simulation.py
```

위 검사는 표준 라이브러리만 사용하며 링크 트리·질량·관성·롤러 축·바퀴 속도 여유·명령 속도 제한을 검증합니다. 원통 롤러는 단순화된 형상이라 접촉 진동과 슬립 튜닝이 필요합니다. 실제 주행 시 지면 마찰, 접촉 안정성, 롤러 형상, 모터 게인을 검증해야 합니다.

`my_workspace/IsaacSim`의 standalone 5.1.0 설치를 기본 런타임으로 사용합니다. 별도의 Python 3.10/Isaac Sim 4.5 설치 흔적 및 시스템 `isaacsim` 명령과 구분합니다. 이 작업 환경에서는 NVIDIA 드라이버 통신이 실패하여 Isaac Sim GUI/물리 실행 검증은 미완료입니다. EULA 동의와 설치·드라이버 설정은 사용자의 Isaac Sim 환경에서 완료해야 합니다.

참고: [URDF importer 예제](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/import_urdf.html), [6.x importer 변경점](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/urdf_mjcf_importer_exporter_pipeline.html).
