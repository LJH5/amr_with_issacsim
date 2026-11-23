# AMR 롤러 축·마찰·제자리 회전 수정

작업일: 2026-11-05. 브랜치: `fix/amr-model-drive`.
기준: `origin/develop`에서 생성하고 기존 검증 커밋 `e1f211a`까지 fast-forward 반영했다.

## 변경 내용

공통 URDF 생성기의 롤러 축 Z 성분을 `-hand*sin(angle)`로 수정했다.
이제 명목 바닥 접촉 롤러의 `axis_y/axis_x`가 제어 수식의 `hand`와 같다.
기존 모델은 횡이동 방향이 반대이고 yaw 저항의 기하학적 계수가 작아,
허브가 거의 정지해도 수동 롤러를 통해 차체가 돌아가기 쉬웠다.
제어·오도메트리 수식은 그대로 유지하고 물리 모델의 축을 맞췄다.

- `amr_description/scripts/generate_robot.py`와 생성된 `urdf/compact_amr.urdf` 수정.
- `isaac_sim/robot.py`가 저장 USD와 새 장면의 32개 롤러 조인트·링크를 URDF와 정합.
- PhysX 자세 기록과 맞도록 롤러에 translate/orient/scale 변환을 사용하고
  orientation 자료형을 `quatf`로 지정. matrix-only 변환 이후 발생한 quatd 경고 해결.
- 네 허브에 비충돌·무질량의 주황색 회전 표시 추가. 실제 허브 자식으로 함께 회전한다.
- 수동 롤러의 모터는 추가하지 않았다. 허브 4개만 기존 속도 드라이브로 구동한다.

차체·허브·센서의 기존 배치와 사용자 장면은 유지한다.
원본 USD를 해시별 `isaac_sim/build/model-fix-backup/`에 백업했고,
검증한 정지 장면을 별도 파일로 내보낸 뒤 재열기·배치 비교 후 원자적으로 교체했다.
교체한 USD의 SHA-256은
`b751ac4ff94a1b8028d407764a74301ecdda1cc85a0c0ff02980e77d360830c1`이다.

## 마찰 설정

원본은 `ros2_ws/src/amr_description/config/compact_amr.json`의 `contact_materials`다.

| 대상 | 정지 마찰 | 운동 마찰 | 반발 | 결합 방식 | USD Material |
| --- | --- | --- | --- | --- | --- |
| 32개 롤러 | 0.8 | 0.6 | 0 | average | `/World/PhysicsMaterials/UrethaneRollers` |
| 테스트 바닥 | 0.8 | 0.6 | 0 | average | `/World/PhysicsMaterials/EpoxyGround` |

이는 시뮬레이션 초기 설정이며 실제 우레탄·에폭시 조합의 측정값이 아니다.
재질 생성뿐 아니라 실제 Collider의 physics material binding까지 적용한다.
인스턴스 Collider는 원본 `/colliders`의 연결을 통해 적용한다.
기존 사용자 재질 Prim은 보존하지만 해당 접촉면의 활성 연결은 위 재질로 바꾼다.

실행마다 JSON을 적용하므로 값을 유지하려면 GUI와 함께 JSON도 수정해야 한다.
음수·비유한 마찰, 운동 마찰 > 정지 마찰, 잘못된 반발·결합 방식은 거부한다.

## 실행과 회전 확인

시뮬레이터, 제어 노드, 키보드를 각각 실행한다. 세 터미널의 `ROS_DOMAIN_ID`를 맞춘다.

```bash
# 터미널 1: 저장소 루트
cd isaac_sim
bash run.sh --control ros
```

```bash
# 터미널 2: 저장소 루트
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch my_amr drive.launch.py
```

```bash
# 터미널 3: 저장소 루트
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run my_amr keyboard_teleop --ros-args -p speed_m_s:=0.1
```

Q는 제자리 좌회전, E는 우회전, J는 정지다. Q의 0.3 rad/s 목표에서
휠 목표는 FL/FR/RL/RR 순서 `[-1.32, +1.32, -1.32, +1.32] rad/s`다.
E는 반대다. `/joint_states`의 실제 position·velocity와 허브의 주황색 표시로 확인한다.
대칭인 원통 허브의 외형만 보면 회전이 보이지 않을 수 있다.
`run.sh`만 실행하면 ROS 모드에서는 휠 명령이 없으므로 정지한다.

## 검증

- `colcon build --packages-select amr_description my_amr --symlink-install` 성공.
- `colcon test`와 `colcon test-result --verbose`: 80개, 오류·실패 0, skip 13.
- URDF 회귀 검사: 실제 접촉 축 방향의 속도 성분이 0인지 dot product로 검사.
  기존의 잘못된 cross product 검사를 교체했다.
- 오프라인 재생 루프 13개 검사 및 수정 Python 파일 flake8 통과.
- Isaac Sim 5.1 SDK: 저장 USD와 새 URDF 가져오기 모두 축 32개, 회전 표시 4개,
  템플릿·인스턴스 포함 롤러 Collider 연결 64개, 바닥 재질, 수동 롤러,
  중복 적용 시 불변성 검사 통과.
- 실제 C++ 키보드 → DDS → PhysX: WASD/Q/E의 네 휠 속도 방향, J 정지,
  키보드 종료 시 C++ timeout, 제어기 종료 시 시뮬레이터 watchdog 통과.
- 동일 원본·게인 200으로 롤러 축만 바꾼 SDK A/B에서 전진 후 정지 3초 yaw는
  5.16950° → 0.04287°였다. 초기 분석의 게인 조건 차이는
  [원인 분석 문서](roller-drift-analysis.md)에 정정했다.

별도 ROS 도메인에서 C++ 제어 노드에 순수 yaw 명령을 보내 실제 관절 위치와
차체 월드 자세를 측정했다. 각 명령은 wall time 3초 동안 유지했으며 시뮬레이션 시간과
구분한다. 아래는 목표값이 아닌 실제 시작·종료 위치 차이다.

| 명령 | FL / FR / RL / RR 실제 회전각 rad | 차체 yaw 변화 |
| --- | --- | --- |
| 좌회전 +0.3 rad/s | -1.7659 / +1.7728 / -1.7717 / +1.7799 | +0.40254 rad, 약 +23.06° |
| 우회전 -0.3 rad/s | +1.7982 / -1.7764 / +1.8065 / -1.7885 | -0.40314 rad, 약 -23.10° |

양 방향에서 네 휠의 누적 각도 부호와 차체 회전 방향을 통과했다.
일부 순간 속도에는 스파이크가 있어 속도 추종 정밀도까지 합격했다고 판단하지 않는다.
원시 각도·속도·yaw는 `ros2_ws/log/model_fix/rotation.json`에 보존했다.

SDK 검사는 Isaac Sim 전용 Python으로 `isaac_sim/test/run_model_fix_fixture.py`를
실행한다. 새 장면 검사는 `--new-scene`을 사용한다. `--output-scene`은 존재하지 않는
출력 경로만 허용하며 원본 USD에 직접 덮어쓰지 않는다.

원시 물리 검증 CSV는 `ros2_ws/log/model_fix/`, 로그는 `/tmp/amr_model_fix_*.log`에 있다.
자동 검증과 키보드는 동시에 실행하면 안 된다. 검사 중 발행자가 겹친
`six_direction_*` 측정은 무효 처리했다. 기존 2초 정지 관찰에서는 후진 후 휠 진동으로
정지 판정을 실패했다. 별도 도메인의 `final_*`에서 같은 허용값으로 5초를 관찰했지만
전진 후에도 정지 판정에 실패했다. 전진 전체 구간의 위치 오차는 2.04 mm,
차체 yaw 변화는 -0.070°였다. **6방향 자동 정지 검증 완료로 표시하지 않는다.**

최고 속도 1.5 m/s, 적재 상태, 전체 기본 18회 반복 검증은 이번 저속 확인과 별도다.
원통 롤러 근사에서 남는 접촉 진동·거리 오차를 마찰 적용만으로 해결됐다고 보지 않는다.
휠 속도 스파이크와 정지 진동은 접촉 형상·물리 solver·드라이브 게인의 후속 검증 대상이다.

Notion 개발 기록: [롤러 분석과 후속 모델 수정](https://www.notion.so/40fe0bf3f9298192aec0ca7b1a89bed6).
이번 변경의 commit/push는 수행하지 않았다.
