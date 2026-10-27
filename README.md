# amr_with_issacsim

ROS 2 Jazzy + C++ + NVIDIA Isaac Sim 기반 자율주행 AMR 개발 프로젝트

Ubuntu 24.04 LTS 환경에서 차동구동(Differential Drive) AMR의 시뮬레이션, 로봇 제어, 지도 작성(SLAM), 위치 추정, 경로 계획 및 자율주행을 단계적으로 구현합니다. 검증된 ROS 2 오픈소스를 활용하고, 이후 실제 로봇 하드웨어로 확장할 수 있도록 모듈형 아키텍처를 지향합니다.
프로젝트 상태: 초기 설계 및 개발 환경 구성 단계. 아래 디렉터리와 실행 절차는 목표 구조이며, 구현 진행에 따라 업데이트합니다.

이름 안내: 저장소명 amr_with_issacsim은 확정된 프로젝트 이름입니다. 시뮬레이터의 공식 명칭은 NVIDIA Isaac Sim입니다.

기술 스택
구분	기술
운영체제	Ubuntu 24.04 LTS
ROS 2	Jazzy Jalisco
주 개발 언어	C++17 (필요 시 패키지별 상향)
빌드	CMake / ament_cmake / colcon
시뮬레이션	NVIDIA Isaac Sim + ROS 2 Bridge
로봇 모델	URDF / Xacro / USD(시뮬레이터)
로봇 제어	ros2_control, diff_drive_controller (검토/활용)
지도 작성	SLAM Toolbox
위치 추정	AMCL
자율주행	Nav2 (Navigation2)
시각화	RViz2
형상 관리	Git / GitHub
개발 문서	Notion


Isaac Sim과 ROS 2 Bridge의 상세 버전은 공식 호환성을 확인한 뒤 고정합니다.
개발 목표
- 차동구동 AMR의 모델, 센서(2D LiDAR, IMU, 휠 엔코더) 및 주행 환경 구성
- Isaac Sim ↔ ROS 2 사이의 명령/센서/상태 데이터 통신
- C++ 기반의 모듈형 제어 및 상태 관리
- Odometry / TF 검증과 SLAM Toolbox 기반 2D 지도 생성
- AMCL 위치 추정 및 Nav2 경로 계획·장애물 회피
- Waypoint 기반 미션 관리와 안전·오류 처리
- GitHub 코드 및 테스트 관리, Notion 기술 문서 관리
예정 아키텍처
                   ┌──────────────────────────┐
                   │   Mission / State Logic  │
                   │        (C++)             │
                   └────────────┬─────────────┘
                                │
                   ┌────────────▼─────────────┐
                   │           Nav2           │
                   │   Planner / Controller   │
                   └────────────┬─────────────┘
                                │ /cmd_vel
                   ┌────────────▼─────────────┐
                   │   ROS 2 Control Layer    │
                   └────────────┬─────────────┘
                                │
              ┌─────────────────▼────────────────┐
              │     Isaac Sim + ROS 2 Bridge     │
              │  Differential Drive / Sensors    │
              └─────────────────┬────────────────┘
                                │ /odom, /scan, /imu, /tf
                   ┌────────────▼─────────────┐
                   │ SLAM Toolbox / AMCL / RViz│
                   └──────────────────────────┘
시뮬레이터와 실기체의 하드웨어 인터페이스를 분리하는 것이 원칙입니다. 구체적인 제어·TF 발행 책임은 구현 과정에서 중복되지 않도록 설계합니다.

예정 디렉터리 구조
amr_with_issacsim/
├── AGENTS.md                  # 프로젝트 공통 개발 규칙
├── README.md                  # 프로젝트 소개
├── .gitignore
├── docs/                      # 설계 및 테스트 문서
├── scripts/                   # 개발 보조 스크립트
├── isaac_sim/                 # USD 에셋, 시뮬레이션 씬·스크립트
└── ros2_ws/
    └── src/
        ├── amr_description/   # URDF/Xacro, meshes
        ├── amr_bringup/       # 통합 launch 및 설정
        ├── amr_interfaces/    # 사용자 정의 msg/srv/action
        ├── amr_control/       # AMR 고유 주행 제어·Odometry
        ├── amr_simulation/    # Isaac Sim 연동 구성
        ├── amr_mapping/       # SLAM 관련 설정
        ├── amr_localization/  # 위치 추정 설정
        ├── amr_navigation/    # Nav2 구성 및 지도
        └── amr_mission/       # 미션 및 상태 관리
각 ROS 2 C++ 패키지는 표준 package.xml, CMakeLists.txt, include/<package>/, src/, launch/, config/, test/ 구조를 따릅니다. 실제 필요한 디렉터리만 생성합니다.
개발 로드맵
- [ ] 01. 환경 구성: Ubuntu / ROS 2 Jazzy / 빌드 도구 / Isaac Sim 호환성 검증
- [ ] 02. 프로젝트 초기화: GitHub, 패키지 구조, 개발 규칙 및 문서화 체계
- [ ] 03. 로봇 모델: 차동구동 AMR, 센서, URDF/Xacro 및 USD 구성
- [ ] 04. 시뮬레이터 연동: Isaac Sim ↔ ROS 2 Bridge 토픽 검증
- [ ] 05. 기본 제어: /cmd_vel, 휠 제어, /odom, TF 검증
- [ ] 06. Mapping: SLAM Toolbox로 맵 작성 및 저장
- [ ] 07. Localization: AMCL 기반 지도 내 위치 추정
- [ ] 08. Navigation: Nav2 경로 계획, 목적지 이동, 회피·복구 동작
- [ ] 09. Mission: Waypoint, 상태 관리, 중단/재개 및 오류 처리
- [ ] 10. 통합 테스트: 반복 주행·안정성·성능 측정 및 문서화
빠른 시작 (ROS 2 설치 후)
아래 명령어는 패키지 소스가 추가된 뒤 사용하는 기본 빌드 절차입니다. 현재 단계에서 실행 가능한 AMR 노드가 포함되어 있다는 의미는 아닙니다.
# 저장소를 클론했다면 루트 디렉터리에서 실행
source /opt/ros/jazzy/setup.bash
cd ros2_ws

# 의존성 설치 (rosdep 초기화·업데이트가 되어 있어야 함)
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

# 빌드
colcon build --symlink-install
source install/setup.bash

# 테스트
colcon test
colcon test-result --verbose
ROS 2 Jazzy 설치 안내: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
개발 규칙
- C++ 객체지향 설계: 클래스별 단일 책임과 명확한 인터페이스를 유지합니다.
- 파일·패키지 분리: 모든 기능을 하나의 소스 파일 또는 노드에 모으지 않습니다.
- 표준 인터페이스 우선: ROS 2 표준 토픽·메시지·액션과 검증된 오픈소스를 우선 활용합니다.
- 설정 분리: 제원, 토픽, TF 프레임 및 제어값은 파라미터/설정 파일로 관리합니다.
- 테스트·문서화: 기능별 검증 후 GitHub에 변경 이력, Notion에 개발 기록을 남깁니다.
상세 규칙: [AGENTS.md](AGENTS.md)
프로젝트 관리
- GitHub: 소스코드, 브랜치, Pull Request, Issue, 테스트 이력
- Notion: 개발 로드맵, 아키텍처, 설계 의사결정, 개발 일지, 트러블슈팅
Notion 프로젝트 페이지 및 GitHub Issue 링크는 연결 후 추가합니다.
참고 자료
- ROS 2 Jazzy Documentation
- NVIDIA Isaac Sim Documentation
- Nav2 Documentation
- SLAM Toolbox
- ros2_control Documentation
