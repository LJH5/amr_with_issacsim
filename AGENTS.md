# AGENTS.md — ROS 2 AMR 프로젝트 개발 규칙

## 1. 프로젝트 목적 및 기술 스택

이 저장소는 자율주행 AMR(Autonomous Mobile Robot)을 개발하기 위한 프로젝트다. 개발은 시뮬레이션에서 시작하며, 추후 실제 로봇으로 확장할 수 있는 구조를 지향한다.

- **OS:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy Jalisco
- **주 개발 언어:** C++ (기본 C++17, 필요한 경우 패키지별 상향)
- **시뮬레이터:** NVIDIA Isaac Sim
- **자율주행:** Nav2 (Navigation2)
- **지도 작성:** SLAM Toolbox
- **위치 추정:** AMCL 등 ROS 2 표준 패키지
- **로봇 제어:** ros2_control / diff_drive_controller 우선 검토
- **시각화:** RViz2
- **Python 개발 환경:** Ubuntu 기본 Python 3 `venv`, 프로젝트 루트 `.venv/` (개발 보조 도구에만 사용)
- **소스 및 이슈 관리:** Git / GitHub
- **프로젝트 문서 및 진행 관리:** Notion

버전 및 의존성은 Ubuntu 24.04 + ROS 2 Jazzy와의 호환성을 우선한다. Isaac Sim, Nav2, SLAM Toolbox 등의 상세 버전은 공식 호환성을 확인한 후 확정한다.

## 2. 핵심 개발 원칙

1. **C++ 객체지향 설계:** 단일 책임 원칙(SRP), 캡슐화, 의존성 분리, 인터페이스 기반 설계를 우선한다.
2. **기능별 코드 분리:** 한 파일에 모든 기능을 구현하지 않는다. 주행 제어, 오도메트리, 센서 처리, 상태 관리, 미션 관리 등을 적절한 클래스와 파일로 구분한다.
3. **ROS 2 표준 구조:** `ament_cmake`, `colcon`, `package.xml`, `CMakeLists.txt`, ROS 2 표준 메시지·토픽·서비스·액션을 우선 사용한다.
4. **검증된 오픈소스 활용:** SLAM, Nav2, ros2_control 등 기존 기능을 불필요하게 재구현하지 않는다. 커스텀 코드는 AMR 고유 기능에 집중한다.
5. **시뮬레이터와 하드웨어 분리:** Isaac Sim에 종속된 구현을 상위 제어·미션 로직과 분리하여, 실제 AMR로 전환할 때 교체 범위를 최소화한다.
6. **설정과 코드 분리:** 로봇 제원, 토픽, 프레임, 제어 파라미터 등은 가능한 한 YAML/파라미터로 관리하며 하드코딩을 피한다.
7. **작은 단위로 개발 및 검증:** 패키지·노드·기능 단위로 빌드, 테스트, 문서화하고 이후 단계로 진행한다.
8. **변경 사항 추적:** 코드 변경은 GitHub에, 설계 결정·구현 과정·테스트 결과·트러블슈팅은 Notion에 기록한다.
9. **Python 가상환경 사용:** Python 보조 스크립트 및 개발 도구는 로컬 `.venv/`로 격리하되, 시스템 ROS 2와 Isaac Sim의 Python 실행 환경을 대체하거나 무단 혼합하지 않는다.

## 3. 권장 ROS 2 워크스페이스 구조

다음은 **초기 권장 구조**다. 실제 기능 경계에 맞춰 패키지를 추가·통합할 수 있지만, 별도 이유 없이 전체 기능을 하나의 패키지나 소스 파일에 몰아넣지 않는다.

```text
amr_with_issacsim/
├── AGENTS.md
├── README.md
├── .gitignore
├── .venv/                      # 로컬 생성, Git 추적 제외
├── requirements-dev.txt       # Python 보조 도구 의존성이 필요할 때 생성
├── docs/                       # GitHub에서 버전 관리할 설계·사용법·테스트 문서
├── scripts/                    # 개발·검증 보조 스크립트
├── isaac_sim/                  # Isaac Sim 장면·에셋·연동 스크립트
└── ros2_ws/
    └── src/
        ├── amr_description/    # URDF/Xacro, meshes, RViz 관련 설정
        ├── amr_bringup/        # 전체 시스템 launch, 공통 설정
        ├── amr_interfaces/     # 프로젝트 고유 msg/srv/action (필요할 때만)
        ├── amr_control/        # 로봇 고유 제어·상태/odometry 인터페이스
        ├── amr_simulation/     # Isaac Sim 연동에 필요한 ROS 2 구성
        ├── amr_mapping/        # SLAM Toolbox launch 및 설정
        ├── amr_localization/   # AMCL/지도 로딩 설정
        ├── amr_navigation/     # Nav2 파라미터, BT, launch
        └── amr_mission/        # 미션·웨이포인트·상위 상태 관리
```

C++ 코드가 들어가는 개별 `ament_cmake` 패키지는 ROS 2에서 일반적으로 쓰는 구조를 따른다.

```text
amr_control/
├── CMakeLists.txt
├── package.xml
├── include/amr_control/
│   ├── differential_drive.hpp
│   └── odometry.hpp
├── src/
│   ├── differential_drive.cpp
│   ├── odometry.cpp
│   └── control_node.cpp      # main/ROS 노드 진입점
├── config/
├── launch/
└── test/
```

> 위 파일명은 구조 설명을 위한 예시다. 실제 구현 방식(예: `ros2_control` 사용)에 따라 클래스를 변경한다. 구현이 없는 빈 패키지를 불필요하게 생성하지 않는다.

## 4. C++ 코딩 규칙

- 클래스는 역할이 명확해야 하며, **한 클래스가 통신·제어·경로 계획·UI를 모두 담당하지 않도록** 한다.
- 재사용 가능한 클래스는 `include/<package_name>/*.hpp`에 선언하고 `src/*.cpp`에 구현한다.
- 실행 파일의 `main()`은 노드 초기화 및 실행 위주로 최소화한다.
- 파일명·함수명·변수명은 `snake_case`, 클래스명은 `PascalCase`, 상수는 `kPascalCase`를 기본으로 한다. 외부 라이브러리의 기존 규칙과 통합할 때는 해당 코드베이스의 규칙을 존중한다.
- 매직 넘버를 피하고 상수 또는 ROS 2 파라미터로 명확하게 선언한다.
- 소유권은 스마트 포인터/RAII로 관리하고 불필요한 전역 변수·수동 메모리 관리를 피한다.
- 예외·오류·통신 실패·타임아웃을 고려하며, 로그는 `RCLCPP_*` 매크로를 사용한다.
- 스레드·타이머·콜백을 사용할 경우 데이터 경쟁과 콜백 실행 모델을 고려한다.
- 복잡한 알고리즘은 계산 로직과 ROS 입출력을 분리하여 단위 테스트를 가능하게 만든다.
- 공개 인터페이스와 비직관적인 제어 로직에는 간결한 주석/문서를 작성한다. 자명한 코드를 반복 설명하지 않는다.

## 5. ROS 2 인터페이스 및 TF 규칙

- 표준 메시지 타입을 우선 사용한다. 예: `geometry_msgs/msg/Twist`, `nav_msgs/msg/Odometry`, `sensor_msgs/msg/LaserScan`.
- 메시지·서비스·액션은 의미에 맞게 구분하고, 사용자 정의 인터페이스는 표준 타입으로 해결할 수 없을 때 추가한다.
- 노드·토픽·프레임 이름은 패키지 간 일관되게 유지하고 launch/파라미터를 통한 리매핑을 지원한다.
- 기본 TF 체인은 `map -> odom -> base_link -> sensor_frame`을 따른다.
- 동일 TF에 대해 복수 노드가 중복 발행하지 않도록 책임을 명확히 한다.
- 시뮬레이션과 실기체는 가능한 한 같은 상위 ROS 2 인터페이스를 사용한다.

## 6. 개발 순서 및 단계별 완료 기준

1. **환경 구축:** Ubuntu 24.04, ROS 2 Jazzy, Python `venv`, 빌드 도구, GPU/Isaac Sim 호환성 확인
2. **프로젝트 기반:** GitHub 저장소, ROS 2 워크스페이스, Notion 개발 문서 구성
3. **로봇 모델:** URDF/Xacro, 조인트, LiDAR/IMU, TF 구조 검증
4. **시뮬레이터 연동:** Isaac Sim ↔ ROS 2 Bridge, 센서 데이터 및 구동 명령 테스트
5. **주행 제어:** 차동구동, `/cmd_vel`, `/odom`, 제한값 및 타임아웃 검증
6. **매핑:** SLAM Toolbox를 사용한 지도 생성과 저장
7. **위치 추정:** AMCL과 지도 기반 로컬라이제이션 검증
8. **자율주행:** Nav2 경로 계획·추종·장애물 대응·복구 동작 확인
9. **상위 제어:** Mission Manager, 상태 머신, 웨이포인트 주행 및 오류 처리
10. **통합 테스트:** 목표 도착률, 위치 오차, 안전 정지, 장시간 구동 평가

각 단계에서는 다음을 충족해야 완료로 본다: **빌드 성공 → 기능 테스트 성공 → 실행 방법 기록 → GitHub 반영 → Notion 개발 기록 업데이트**.

## 7. GitHub 운영 규칙

### 7.1 브랜치 역할과 병합 순서

| 브랜치             | 역할
| ---               | ---
| `master`          | 빌드와 통합 테스트를 통과한 안정 버전
| `develop`         | 다음 버전의 기능과 수정 사항을 통합하는 개발 브랜치
| `test`            | `develop`에서 준비한 변경을 시뮬레이션·통합 테스트하는 검증 브랜치
| `feature/<topic>` | 개별 기능 추가·개선. 예: `feature/robot-description`
| `fix/<topic>`     | 버그 수정. 예: `fix/odom-frame`
| `docs/<topic>`    | 문서 변경. 예: `docs/git-workflow`
| `chore/<topic>`   | 환경·빌드 도구 등 유지보수. 예: `chore/dependencies`

- 브랜치 이름의 `<topic>`은 소문자 영문·숫자·하이픈을 사용한다.
- 작업 브랜치는 최신 `develop`에서 생성하며, 하나의 작업 목적에 집중한다.
- 기본 병합 순서는 **작업 브랜치 → `develop` → `test` → `master`**이며 Pull Request로 진행한다.
- `master`, `develop`, `test`에서 직접 기능을 개발하거나 직접 커밋하지 않는다.
- `test`에서 발견한 문제는 `develop` 기반 `fix/<topic>`에서 수정한 뒤 동일한 경로로 반영하고 재검증한다.
- `test` → `master`에는 검증한 변경만 포함한다. 검증 도중 `test`에 새 변경을 병합하면 영향을 받는 검증을 다시 수행한다.
- 위 브랜치명은 운영 규칙이다. 실제 로컬·원격 브랜치 생성 및 저장소 보호 설정 완료를 의미하지 않는다.

### 7.2 원격 변경 가져오기 (`git pull`)

- 먼저 `git status`로 현재 브랜치와 미커밋 변경을 확인하고 `git fetch origin`으로 원격 상태를 갱신한다.
- 미커밋 변경이 있으면 작업을 보존할 방법을 먼저 결정한다. 에이전트가 임의로 stash, 변경 폐기, `git reset --hard`, `git clean`을 실행하지 않는다.
- 현재 브랜치의 원격 변경은 **`git pull --ff-only origin <현재-브랜치>`**로 가져온다. 원격 브랜치가 없는 새 로컬 브랜치는 pull하지 않는다.
- `--ff-only`가 실패하면 로컬·원격 분기와 커밋 내용을 확인한다. 무조건적인 rebase, 강제 reset 또는 force push로 해결하지 않는다.
- 작업 브랜치에 최신 개발 내용을 반영할 때는 작업 트리가 깨끗한 상태에서 `git fetch origin` 후 `git merge origin/develop`을 사용한다. 충돌 해결 시 양쪽 변경 의도를 확인하고 관련 빌드·테스트를 다시 수행한다.
- 공유 브랜치의 이력을 다시 쓰지 않으며, 원격 변경을 확인하지 않고 덮어쓰지 않는다.

### 7.3 커밋 메시지

형식은 **`<type>: <변경 요약>`**을 기본으로 하며, 필요하면 **`<type>(<scope>): <변경 요약>`**을 사용한다. 요약은 한국어로 작성할 수 있으며 무엇을 바꿨는지 구체적으로 적는다.

| type      | 용도
| ----------| ---------------------------
| `update`  | 기능 추가 또는 기존 기능 개선
| `fix`     | 버그 수정
| `docs`    | 문서 변경
| `test`    | 테스트 추가·수정
| `refactor`| 외부 동작을 유지하는 코드 구조 개선
| `build`   | 빌드 설정·의존성 변경
| `ci`      | CI 자동화 설정 변경
| `chore`   | 위 항목에 속하지 않는 유지보수
| `revert`  | 기존 커밋 되돌리기

- 이 프로젝트는 기능 추가·개선에 `update`를 사용하며 `feat`와 혼용하지 않는다. `update`는 프로젝트 자체 규칙이다.
- 한 커밋에는 하나의 논리적 변경을 담는다. 서로 무관한 기능·버그·문서 변경은 분리한다.
- 설명이 필요하면 제목 다음에 빈 줄을 두고 변경 이유, 동작 영향, 검증 결과 및 관련 이슈를 본문에 기록한다.
- 호환성을 깨는 변경은 `update(interfaces)!: ...`처럼 `!`를 표시하고 본문에 `BREAKING CHANGE:`와 전환 방법을 적는다.

### 7.4 Pull Request와 추적 제외

- Pull Request에는 변경 목적, 주요 변경점, 재현/테스트 방법 및 결과, 관련 이슈를 작성한다.
- `develop` 병합 전 관련 패키지 빌드·테스트를 통과하고, `master` 반영 전 `test`에서 통합 검증 결과를 기록한다.
- `build/`, `install/`, `log/`, 캐시, 임시 파일, 대용량 생성물을 무분별하게 커밋하지 않는다.
- 비밀키·토큰·접속 정보는 코드와 저장소에 저장하지 않는다.

## 8. Notion 문서화 규칙

- 프로젝트 개요와 기술 스택, 시스템 아키텍처, 개발 로드맵을 유지한다.
- 개발 작업마다 **목표 / 설계 / 구현 / 테스트 결과 / 발생한 문제 / 해결 방법 / 관련 GitHub Issue·PR·Commit**을 기록한다.
- 기능 완료 시 실행 명령어, 주요 토픽·파라미터, 의존성을 재현 가능하게 정리한다.
- Notion은 진행 상황과 기술 결정의 기록장으로, GitHub는 소스코드와 변경 이력의 기준 저장소로 사용한다.
- 확정되지 않은 사항은 '검토 중'으로 표시하고 추측을 확정된 결정처럼 기록하지 않는다.

## 9. Python venv 운영 규칙

- 저장소 루트에서 `python3 -m venv .venv`로 가상환경을 생성하고 `source .venv/bin/activate`로 활성화한다. Ubuntu 24.04 기준 `python3-venv` 패키지가 필요할 수 있다.
- `.venv/`, `venv/`, Python 캐시 및 개인용 `.env`는 Git에 커밋하지 않는다. 현재 `.gitignore`에 `.venv/` 제외 규칙이 있다.
- Python 보조 도구에 외부 패키지가 필요해지면 `requirements-dev.txt`에 최소 의존성을 명시하고 사용 절차를 README에 기록한다. 파일이 없거나 의존성이 필요하지 않을 때 임의로 빈 의존성 파일을 만들거나 패키지를 설치하지 않는다.
- ROS 2 Jazzy는 Ubuntu/apt 기반 시스템 설치(`/opt/ros/jazzy`)를 기준으로 하며, ROS 2를 `pip`를 통해 `.venv`에 통째로 설치하려 하지 않는다.
- `colcon`·ROS 2 Python 실행 시 venv로 인해 `rclpy` 등 모듈 경로 충돌이 발생하면 `deactivate` 후 ROS 2 시스템 환경에서 작업한다. Python 버전, `PYTHONPATH`, 도구 경로를 점검한다.
- Isaac Sim의 내장 Python 환경은 프로젝트 `.venv`와 독립적으로 관리한다. Isaac Sim 자체 Python 패키지/확장에 대한 설치 지침을 우선한다.
- C++ 코드의 빌드는 여전히 `ament_cmake` / CMake / `colcon`으로 수행하며, `venv`가 C++ 라이브러리나 ROS 2 패키지 의존성을 격리한다고 가정하지 않는다.
- AI 에이전트는 Python 도구 추가 시 venv 필요성, 실행 환경, 의존성 기록 및 Git 추적 제외를 먼저 점검한다.

```bash
# 저장소 루트에서 한 번 생성
sudo apt install -y python3-venv
python3 -m venv .venv
source .venv/bin/activate

# Python 보조 도구 작업 이후
deactivate

# ROS 2 환경
source /opt/ros/jazzy/setup.bash
```

## 10. 빌드·테스트·품질 기준

- 변경된 ROS 2 패키지를 `colcon build --packages-select <package_name> --symlink-install`로 빌드한다.
- 필요한 환경을 source한 후 `colcon test --packages-select <package_name>` 및 `colcon test-result --verbose`를 활용한다.
- 계산 로직에는 `ament_cmake_gtest` 등 단위 테스트를 추가한다.
- `ament_lint_auto`, `clang-format` 등 정적 검사·포맷터를 적극 활용한다.
- 코드와 문서에 환경 의존 사항, 실행 절차, 예상 결과를 함께 기록한다.
- 실제 하드웨어 적용 시 비상정지, 속도 제한, 충돌 감시는 소프트웨어뿐 아니라 별도의 하드웨어 안전 요구사항을 검토한다.

## 11. AI 코딩 에이전트 작업 지침

- 코드 작성 전에 **기존 패키지와 파일 구조부터 확인**하고, 현재 설계에 맞춰 최소 범위로 변경한다.
- 새 기능을 작성할 때 적합한 패키지·클래스·헤더·구현 파일·테스트 파일을 먼저 구분한다.
- 하나의 큰 `.cpp` 파일로 모든 기능을 구현하지 않는다.
- 외부 API 및 ROS 2 배포판별 인터페이스는 **Jazzy 기준**으로 확인하고, 확인되지 않은 명령어나 패키지를 임의로 단정하지 않는다.
- 이미 존재하는 기능을 임의로 삭제하거나, 요청과 무관한 대규모 구조 변경을 하지 않는다.
- 가능한 경우 빌드·테스트를 실행하고, 실행하지 못했다면 그 사실과 이유를 명시한다.
- 결과 보고 시 **변경 파일 / 구현 내용 / 테스트 결과 / 남은 이슈 / 문서화할 내용**을 간결하게 요약한다.
- Notion 및 GitHub에 직접 반영할 권한·연결이 없는 상황에서는 반영했다고 주장하지 않고, 반영 가능한 문서·커밋 초안을 제공한다.

## 12. 프로젝트 설계 확장 원칙

초기에는 **단일 AMR의 시뮬레이션 자율주행**을 완성하는 데 집중한다. 이후 실제 로봇 하드웨어 인터페이스, 다중 로봇(Fleet) 관리, 외부 서비스 연동으로 확장할 수 있도록 경계를 분리한다. 확장 기능은 현재 단계의 안정성을 해치지 않는 범위에서 도입한다.
