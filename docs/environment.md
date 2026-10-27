# 개발 환경 구성 기록

기록일: 2026-10-27 (Asia/Seoul)

## 목표와 구성

`AGENTS.md`에 따라 Ubuntu 24.04 + ROS 2 Jazzy + C++17을 사용합니다. Python 보조 도구는 프로젝트 `.venv`에 격리하고, ROS 2/C++ 의존성은 apt, Isaac Sim은 별도 런타임으로 관리합니다.

| 항목 | 확인 결과 |
| --- | --- |
| OS | Ubuntu 24.04.5 LTS |
| 시스템 Python / `.venv` | Python 3.12.3 / 시스템 site-packages 비공유 |
| venv pip | 24.0 |
| GCC / CMake | 13.3.0 / 3.28.3 |
| ROS 2 | `/opt/ros/jazzy` 설치 확인, `rclpy` import 및 `rclcpp` 검색 성공 |
| colcon / rosdep | 설치 확인, rosdep sources 설정 존재 |
| Python 보조 패키지 | 아직 없음. `freeze --all`에 pip만 포함 |
| 시스템 패키지 | `system-packages.freeze.txt`에서 설치 버전과 `MISSING` 확인 |
| GPU / Isaac Sim | `nvidia-smi`가 드라이버 통신 실패. Isaac Sim 설치·실행 및 Bridge 호환성 검증 미완료 |

## 발생한 문제와 처리

- 시스템 `python3-venv`가 없어 기본 venv 생성 중 `ensurepip` 오류가 발생했습니다. `/usr/bin/python3 -m venv --without-pip .venv`로 생성한 뒤, 시스템 pip의 `--python .venv` 옵션으로 `.venv` 내부에 pip 24.0만 설치했습니다. 시스템 Python 패키지는 변경하지 않았습니다. 이후 `setup_system.sh`가 정상 완료되면 표준 `python3 -m venv` 방식으로 구성할 수 있습니다.
- 시스템 apt 설치는 권한 상승을 요청하여 시도했으나 sudo 비밀번호가 필요해 실행되지 않았습니다. 사용자가 터미널에서 `bash scripts/setup_system.sh`를 실행해야 누락된 패키지를 설치할 수 있습니다.
- 기존 셸의 `ROS_DISTRO=humble`, `PYTHONPATH=/opt/ros/humble/...` 설정을 발견했습니다. freeze 및 Python 도구 실행에 `-I`를 사용하고 ROS/C++ 검증은 초기화한 셸에서 Jazzy만 로드했습니다. `.bashrc` 등 개인 설정은 변경하지 않았습니다.

## 재현과 갱신

```bash
cd /home/buddha/my_workspace/amr_with_issacsim
bash scripts/setup_system.sh
source .venv/bin/activate
python -I -m pip check
bash scripts/freeze_environment.sh
deactivate
```

`requirements-dev.txt`는 venv의 실제 `pip freeze --all` 결과입니다. ROS 2와 Isaac Sim을 이 파일에 넣지 않습니다. `requirements-apt.txt`는 시스템 패키지 설치 목록이며, `system-packages.freeze.txt`는 지정한 패키지의 설치 현황입니다. 후자는 미설치 항목과 전이 의존성 한계 때문에 완전한 환경 복원 lock으로 사용하지 않습니다.

C++17은 개별 ROS 패키지의 `CMakeLists.txt`에서 `CMAKE_CXX_STANDARD 17`과 `CMAKE_CXX_STANDARD_REQUIRED ON`으로 지정합니다. 아직 실제 기능 구현이 없으므로 빈 ROS 패키지를 생성하지 않았습니다. `ros2_ws/src/`만 준비했습니다.

## 검증 및 남은 작업

- `.venv`의 격리 모드에서 `pip check` 통과 및 freeze에 `pip==24.0`만 포함되는지 확인했습니다.
- 설치/기록 스크립트의 `bash -n` 구문 검사 및 `.venv` Git 제외를 확인했습니다.
- 현재 apt 인덱스에서 설치 목록의 `apt-get --simulate install --no-upgrade`가 성공했습니다. 실제 다운로드·설치는 수행되지 않았습니다.
- Jazzy만 로드한 환경에서 임시 CMake 프로젝트로 C++17 + `ament_cmake` + `rclcpp` 컴파일·링크·실행이 모두 통과했습니다. `rclcpp::ok()` 호출로 링크를 확인했으며 DDS 토픽 통신 검증은 포함하지 않았습니다.
- 프로젝트 소스 패키지가 없어 프로젝트 `colcon build/test`는 실행하지 않았습니다.
- sudo를 통한 누락 시스템 패키지 설치, rosdep 사용자 캐시 갱신, NVIDIA 드라이버 정상화 및 Isaac Sim/Bridge 버전 확정·실행 검증이 남아 있습니다.
- GitHub 업로드 및 Notion 변경은 수행하지 않았습니다. 이 문서를 개발 기록으로 옮길 수 있습니다.

## 공식 근거

- [ROS 2 Jazzy Ubuntu 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Nav2 Jazzy 설치](https://docs.nav2.org/jazzy/getting_started/build_and_install/local_installation/)
- [ros2_control Jazzy 설치](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
- [pip: pip 없는 venv 관리](https://pip.pypa.io/en/stable/topics/python-option/)
- [Isaac Sim 시스템 요구 사항](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)

Isaac Sim 버전은 아직 고정하지 않았습니다. 설치할 버전의 OS/GPU/드라이버 및 ROS 2 Bridge 요구 사항을 확인한 뒤 별도 환경으로 구성해야 합니다.
