# amr_with_issacsim

**ROS 2 Jazzy + C++ + NVIDIA Isaac Sim 기반 자율주행 AMR 개발 프로젝트**

Ubuntu 24.04 LTS 환경에서 소형 4륜 메카넘 AMR의 시뮬레이션, 로봇 제어, 지도 작성(SLAM), 위치 추정, 경로 계획 및 자율주행을 단계적으로 구현합니다. 최고 병진 속도는 1.5 m/s이며, 초기 모델은 센서 포함 예약 크기 0.48 × 0.44 × 0.34 m로 설계했습니다. 검증된 ROS 2 오픈소스를 활용하고, 이후 실제 로봇 하드웨어로 확장할 수 있도록 모듈형 아키텍처를 지향합니다.

Isaac Sim 단독 실행은 [isaac_sim/README.md](isaac_sim/README.md)를 참고하세요. 모델·주행 코드는 준비했으며 실제 Isaac Sim 물리 실행과 센서/ROS Bridge 연동은 검증 전입니다.

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
| 로봇 제어 | ros2_control, mecanum_drive_controller (연동 검토) |
| 지도 작성 | SLAM Toolbox |
| 위치 추정 | AMCL |
| 자율주행 | Nav2 (Navigation2) |
| 시각화 | RViz2 |
| 형상 관리 | Git / GitHub |
| 개발 문서 | Notion |

Isaac Sim과 ROS 2 Bridge의 상세 버전은 공식 호환성을 확인한 뒤 고정합니다.

> **가상환경 범위:** `.venv`는 Python 기반 보조 스크립트·개발 도구의 의존성을 격리합니다. ROS 2 Jazzy의 시스템 설치(`/opt/ros/jazzy`) 및 C++ `colcon` 빌드 환경까지 대체하지 않습니다. Isaac Sim은 자체 Python 런타임과 확장 기능을 사용하므로, 별도의 호환성 확인 없이 `.venv`의 패키지를 Isaac Sim에 주입하지 않습니다.

## 개발 목표

- 4륜 메카넘 AMR의 모델, 센서(대각 LiDAR 2개, 전방 RGB-D, 중앙 IMU, 휠 피드백) 및 주행 환경 구성
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
4. **Isaac Sim + ROS 2 Bridge:** 메카넘 AMR과 센서를 시뮬레이션하고 ROS 2와 데이터를 교환합니다.
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
| `requirements-dev.txt` | `.venv`의 `pip freeze --all` 결과 |
| `requirements-apt.txt` | C++/ROS 2 시스템 패키지 설치 목록 |
| `system-packages.freeze.txt` | 지정한 apt 패키지의 실제 설치 버전 및 누락 현황 |
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
- [ ] **03. 로봇 모델:** 소형 메카넘 URDF 구성 완료, 센서·USD 물리 실행 검증 예정
- [ ] **04. 시뮬레이터 연동:** Isaac Sim ↔ ROS 2 Bridge 토픽 검증
- [ ] **05. 기본 제어:** `/cmd_vel`, 휠 제어, `/odom`, TF 검증
- [ ] **06. Mapping:** SLAM Toolbox로 맵 작성 및 저장
- [ ] **07. Localization:** AMCL 기반 지도 내 위치 추정
- [ ] **08. Navigation:** Nav2 경로 계획, 목적지 이동, 회피·복구 동작
- [ ] **09. Mission:** Waypoint, 상태 관리, 중단/재개 및 오류 처리
- [ ] **10. 통합 테스트:** 반복 주행·안정성·성능 측정 및 문서화

## 빠른 시작: Python venv 및 ROS 2 개발 환경

### 1. 사전 준비

Ubuntu 24.04 및 ROS 2 Jazzy apt 저장소가 필요합니다. `venv`는 Python 보조 도구에만 사용하며 ROS 2 자체를 `pip`로 설치하지 않습니다. 현재 호스트에는 Jazzy가 설치되어 있지만 Nav2, SLAM Toolbox 등 일부 패키지가 누락되어 있습니다. 설치·검증 현황은 [환경 구성 기록](docs/environment.md)을 참고하세요.

```bash
# 저장소 루트에서 실행. sudo 비밀번호는 터미널에서 직접 입력합니다.
bash scripts/setup_system.sh
```

이 스크립트는 `requirements-apt.txt`의 GCC/CMake, 포맷터, ROS 2 Jazzy, Nav2, SLAM Toolbox, ros2_control, Xacro 등을 설치하고 `.venv` 및 환경 기록을 갱신합니다. Isaac Sim과 GPU 드라이버는 설치하지 않습니다.

### 2. 저장소 루트에서 가상환경 생성·활성화

```bash
# 저장소 루트에서 실행
cd amr_with_issacsim
python3 -m venv .venv
source .venv/bin/activate
python -I -m pip --version
```

`.venv/`는 개인 개발 환경이므로 Git에 커밋하지 않습니다. 현재 Python 외부 의존성은 없으며, `requirements-dev.txt`에는 부트스트랩 도구 `pip==24.0`만 기록되어 있습니다. 환경 복원은 다음과 같습니다.

```bash
python -I -m pip install -r requirements-dev.txt
```

설치 후 freeze를 갱신하려면 `bash scripts/freeze_environment.sh`를 실행합니다. 내부적으로 `python -I -m pip freeze --all`과 `dpkg-query`를 사용합니다. `-I`는 기존 셸의 ROS `PYTHONPATH`가 Python 보조 환경에 유입되는 것을 막습니다. Python 보조 스크립트도 `python -I script.py`로 실행하면 같은 격리를 적용할 수 있습니다. `system-packages.freeze.txt`는 직접 지정한 apt 패키지의 현황이며, `# MISSING` 항목은 아직 설치되지 않았다는 뜻입니다. 전체 시스템 의존성을 고정하는 lock 파일은 아닙니다.

### 3. ROS 2 환경 로딩 및 C++ 패키지 빌드

```bash
# Humble 등이 이미 로드된 셸이라면 먼저 깨끗한 서브셸을 엽니다.
env -i HOME="$HOME" USER="$USER" TERM="${TERM:-xterm}" PATH=/usr/bin:/bin bash --noprofile --norc
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

현재 `ros2_ws/src/my_amr/`에 C++17 `ament_cmake` 패키지와 기본 노드가 있습니다. 생성 옵션·빌드·실행 인자는 [my_amr 사용법](ros2_ws/src/my_amr/README.md)을 참고하세요. rosdep은 최초 한 번 `sudo rosdep init`으로 초기화하고 사용자 계정에서 `rosdep update --rosdistro jazzy`를 실행합니다. 이미 초기화된 경우 `init`을 반복하지 않습니다.

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
