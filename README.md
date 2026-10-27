# amr_with_issacsim

**ROS 2 Jazzy + C++ + NVIDIA Isaac Sim 기반 자율주행 AMR 개발 프로젝트**

Ubuntu 24.04 LTS 환경에서 차동구동(Differential Drive) AMR의 시뮬레이션, 로봇 제어, 지도 작성(SLAM), 위치 추정, 경로 계획 및 자율주행을 단계적으로 구현합니다. 검증된 ROS 2 오픈소스를 활용하고, 이후 실제 로봇 하드웨어로 확장할 수 있도록 모듈형 아키텍처를 지향합니다.

> **프로젝트 상태:** 초기 설계 및 개발 환경 구성 단계. 아래 디렉터리와 실행 절차는 목표 구조이며, 구현 진행에 따라 업데이트합니다.
>
> **이름 안내:** 저장소명 `amr_with_issacsim`은 확정된 프로젝트 이름입니다. 시뮬레이터의 공식 명칭은 **NVIDIA Isaac Sim**입니다.

## 기술 스택

| 구분 | 기술 |
| --- | --- |
| 운영체제 | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| 주 개발 언어 | C++17 (필요 시 패키지별 상향) |
| 빌드 | CMake / ament_cmake / colcon |
| Python 도구 환경 | Python 3 `venv` (`.venv/`, 로컬 전용) |
| 시뮬레이션 | NVIDIA Isaac Sim + ROS 2 Bridge |
| 로봇 모델 | URDF / Xacro / USD(시뮬레이터) |
| 로봇 제어 | ros2_control, diff_drive_controller (검토/활용) |
| 지도 작성 | SLAM Toolbox |
| 위치 추정 | AMCL |
| 자율주행 | Nav2 (Navigation2) |
| 시각화 | RViz2 |
| 형상 관리 | Git / GitHub |
| 개발 문서 | Notion |

Isaac Sim과 ROS 2 Bridge의 상세 버전은 공식 호환성을 확인한 뒤 고정합니다.

> **가상환경 범위:** `.venv`는 Python 기반 보조 스크립트·개발 도구의 의존성을 격리합니다. ROS 2 Jazzy의 시스템 설치(`/opt/ros/jazzy`) 및 C++ `colcon` 빌드 환경까지 대체하지 않습니다. Isaac Sim은 자체 Python 런타임과 확장 기능을 사용하므로, 별도의 호환성 확인 없이 `.venv`의 패키지를 Isaac Sim에 주입하지 않습니다.

## 개발 목표

- 차동구동 AMR의 모델, 센서(2D LiDAR, IMU, 휠 엔코더) 및 주행 환경 구성
- Isaac Sim ↔ ROS 2 사이의 명령/센서/상태 데이터 통신
- C++ 기반의 모듈형 제어 및 상태 관리
- Odometry / TF 검증과 SLAM Toolbox 기반 2D 지도 생성
- AMCL 위치 추정 및 Nav2 경로 계획·장애물 회피
- Waypoint 기반 미션 관리와 안전·오류 처리
- GitHub 코드 및 테스트 관리, Notion 기술 문서 관리

## 시스템 구성

시스템은 다음과 같은 기능 단위로 구성합니다.

1. **Mission / State Logic (C++):** 목적지와 주행 미션을 관리하고 로봇 상태를 처리합니다.
2. **Nav2:** 경로 계획과 경로 추종을 담당하며, 주행 속도 명령(`/cmd_vel`)을 생성합니다.
3. **ROS 2 제어 계층:** 주행 명령을 로봇 구동 인터페이스로 전달하고 로봇의 상태를 관리합니다.
4. **Isaac Sim + ROS 2 Bridge:** 차동구동 AMR과 센서를 시뮬레이션하고 ROS 2와 데이터를 교환합니다.
5. **SLAM Toolbox / AMCL / RViz2:** 지도 생성, 위치 추정, 센서 및 주행 상태 시각화를 담당합니다.

**주요 데이터 흐름:** 미션 관리자가 Nav2에 목적지를 전달하면 Nav2가 주행 속도 명령(`/cmd_vel`)을 생성합니다. 제어 계층은 이 명령을 Isaac Sim의 로봇에 전달합니다. Isaac Sim에서 생성한 센서 데이터와 주행 상태(`/odom`, `/scan`, `/imu`, `/tf`)는 지도 작성, 위치 추정, 자율주행 모듈에서 활용합니다. TF와 Odometry의 발행 주체는 중복되지 않도록 구성합니다.

시뮬레이션용 인터페이스와 실제 로봇의 하드웨어 인터페이스를 분리하여, 상위 ROS 2 소프트웨어를 가능한 한 그대로 재사용하는 것을 원칙으로 합니다.

## 예정 디렉터리 구조

아래 구조는 텍스트와 표로 설명하며, 특수한 선 그리기 문자는 사용하지 않습니다. 저장소 루트에는 `AGENTS.md`, `README.md`, `.gitignore` 파일을 둡니다.

| 경로 | 역할 |
| --- | --- |
| `docs/` | 설계 문서, 테스트 기록 |
| `scripts/` | 개발 보조 스크립트 |
| `.venv/` | 로컬 Python 가상환경 (Git 추적 제외, 자동 생성) |
| `requirements-dev.txt` | Python 보조 도구 의존성이 생기면 버전 관리 |
| `isaac_sim/` | USD 에셋, 시뮬레이션 씬, 시뮬레이터 스크립트 |
| `ros2_ws/src/amr_description/` | URDF/Xacro 및 로봇 모델 |
| `ros2_ws/src/amr_bringup/` | 전체 시스템 실행용 Launch 및 설정 |
| `ros2_ws/src/amr_interfaces/` | 사용자 정의 메시지, 서비스, 액션 |
| `ros2_ws/src/amr_control/` | 로봇 제어 및 Odometry 관련 코드 |
| `ros2_ws/src/amr_simulation/` | Isaac Sim 연동 설정 |
| `ros2_ws/src/amr_mapping/` | SLAM 관련 설정 |
| `ros2_ws/src/amr_localization/` | 위치 추정 설정 |
| `ros2_ws/src/amr_navigation/` | Nav2 설정 및 지도 |
| `ros2_ws/src/amr_mission/` | 미션 및 상태 관리 |

각 ROS 2 C++ 패키지는 `package.xml`, `CMakeLists.txt`를 기본으로 사용하고, 필요에 따라 `include/<package>/`, `src/`, `launch/`, `config/`, `test/` 디렉터리를 분리합니다. 실제 필요한 디렉터리만 생성합니다.

## 개발 로드맵

- [ ] **01. 환경 구성:** Ubuntu / ROS 2 Jazzy / Python venv / 빌드 도구 / Isaac Sim 호환성 검증
- [ ] **02. 프로젝트 초기화:** GitHub, 패키지 구조, 개발 규칙 및 문서화 체계
- [ ] **03. 로봇 모델:** 차동구동 AMR, 센서, URDF/Xacro 및 USD 구성
- [ ] **04. 시뮬레이터 연동:** Isaac Sim ↔ ROS 2 Bridge 토픽 검증
- [ ] **05. 기본 제어:** `/cmd_vel`, 휠 제어, `/odom`, TF 검증
- [ ] **06. Mapping:** SLAM Toolbox로 맵 작성 및 저장
- [ ] **07. Localization:** AMCL 기반 지도 내 위치 추정
- [ ] **08. Navigation:** Nav2 경로 계획, 목적지 이동, 회피·복구 동작
- [ ] **09. Mission:** Waypoint, 상태 관리, 중단/재개 및 오류 처리
- [ ] **10. 통합 테스트:** 반복 주행·안정성·성능 측정 및 문서화

## 빠른 시작: Python venv 및 ROS 2 개발 환경

### 1. 사전 준비

Ubuntu 24.04에서 ROS 2 Jazzy와 `colcon`, `rosdep` 등을 먼저 설치합니다. `venv`는 Python 보조 도구에만 사용하며 ROS 2 자체를 `pip`로 설치하지 않습니다.

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
```

### 2. 저장소 루트에서 가상환경 생성·활성화

```bash
# 저장소 루트에서 실행
cd amr_with_issacsim
python3 -m venv .venv
source .venv/bin/activate
python -m pip --version
```

`.venv/`는 개인 개발 환경이므로 Git에 커밋하지 않습니다. Python 보조 스크립트에 추가 패키지가 필요한 경우 **검증된 패키지만** `requirements-dev.txt`에 명시하고, 파일이 있을 때 아래 명령으로 설치합니다.

```bash
python -m pip install -r requirements-dev.txt
```

현재 `requirements-dev.txt`는 아직 필수 의존성이 정해지지 않아 제공하지 않습니다. 파일 생성 전에는 위 설치 명령을 실행하지 마세요.

### 3. ROS 2 환경 로딩 및 C++ 패키지 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws

# rosdep 초기 설정과 업데이트가 완료된 환경에서
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

# 워크스페이스 패키지가 추가된 이후 실행
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

**참고:** `.venv`를 활성화하면 `PATH`의 Python 실행 파일이 바뀝니다. ROS 2 Jazzy의 시스템 패키지와 Python 버전/모듈 경로가 충돌할 수 있으므로, `colcon` 또는 ROS 2 Python 노드에서 임포트 오류가 발생하면 `deactivate`로 가상환경을 종료하고 ROS 2 설치에 맞는 시스템 환경에서 빌드·실행합니다. 필요한 Python 의존성은 ROS 2 패키지 관리 방식(`rosdep`/apt)과 역할을 구분합니다.

```bash
deactivate  # 가상환경을 사용 중일 때만
source /opt/ros/jazzy/setup.bash
```

Isaac Sim은 공식 실행 환경을 별도로 구성하고 ROS 2 Bridge로 연동합니다. 프로젝트 `.venv`를 Isaac Sim의 Python 환경으로 간주하지 않습니다.

ROS 2 Jazzy 설치 안내: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

## 개발 규칙

- **C++ 객체지향 설계:** 클래스별 단일 책임과 명확한 인터페이스를 유지합니다.
- **파일·패키지 분리:** 모든 기능을 하나의 소스 파일 또는 노드에 모으지 않습니다.
- **표준 인터페이스 우선:** ROS 2 표준 토픽·메시지·액션과 검증된 오픈소스를 우선 활용합니다.
- **설정 분리:** 제원, 토픽, TF 프레임 및 제어값은 파라미터/설정 파일로 관리합니다.
- **가상환경 분리:** Python 보조 도구는 `.venv/`로 격리하고 ROS 2 및 Isaac Sim 고유 런타임과 혼용하지 않습니다.
- **테스트·문서화:** 기능별 검증 후 GitHub에 변경 이력, Notion에 개발 기록을 남깁니다.

상세 규칙: [AGENTS.md](AGENTS.md)

## 프로젝트 관리

- **GitHub:** 소스코드, 브랜치, Pull Request, Issue, 테스트 이력
- **Notion:** 개발 로드맵, 아키텍처, 설계 의사결정, 개발 일지, 트러블슈팅

Notion 프로젝트 페이지 및 GitHub Issue 링크는 연결 후 추가합니다.

## 참고 자료

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [NVIDIA Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [Nav2 Documentation](https://docs.nav2.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [ros2_control Documentation](https://control.ros.org/)
