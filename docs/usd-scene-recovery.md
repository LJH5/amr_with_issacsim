# 저장 USD 손상으로 인한 실행 충돌 복구

기록일: 2026-11-04. 작업 브랜치: `fix/usd-scene-recovery`.

## 증상과 확인한 원인

`bash run.sh`가 장면을 읽는 중 `Aborted (core dumped)`로 종료됐다.
첨부 로그의 native stack에는 `Usd_CrateFile::_UnpackValue`,
`UsdStage::_ComposePrimTypeInfoImpl` 등이 있었다.

현재 `isaac_sim/omni_amr.usd`를 같은 SDK의 USD 라이브러리로 별도 열었을 때
`Corrupt asset ... exception thrown unpacking a value`를 재현했다.
현재 파일의 USD crate 데이터가 정상적으로 해석되지 않는 것이 확인된 원인이다.
손상이 발생한 정확한 시점과 저장·파일 교체 과정은 확인하지 못했다.
마찰계수 자체나 ROS C++ 제어 코드가 충돌 원인이라는 증거는 없다.

## 원본 보존과 복구

- 손상 파일: 84,484 bytes.
- 원본 SHA-256: `f991de179ee319521bacf67a9c428b6c14ec3eb237b1741127a8b6f4ba8787e0`.
- 보존 위치: `isaac_sim/build/recovery/amr-crash-original-f991de179ee3.usd`.
- 별도 임시 백업: `/tmp/amr-crash-original-f991de179ee3.usd`.
- 복구 원본: Git 커밋 `8a3ee00`의 `isaac_sim/omni_amr.usd`, 78,763 bytes.
- 복구 SHA-256: `8d99457980eecd25a95f01367df11fce22d00f6f6dfc66c2405c7bf36deca145`.

기본 브랜치의 오래된 USD 대신 마찰 재질이 추가된 커밋을 사용했다.
해당 파일을 별도 경로에서 먼저 열고 493개 Prim과 각 속성을 읽어 확인했다.
`/World/PhysicsMaterial_1`의 정지 마찰 1.0, 동마찰 약 0.8도 확인했다.
그 후 현재 파일의 해시가 백업 시점과 같은지 확인하고 임시 파일을 원자적으로
교체하여 복구했다. `git reset`, `git restore`, stash나 원본 삭제는 사용하지 않았다.

복구는 마지막 정상 커밋의 상태다. 손상 파일에만 있던 더 최근 편집까지 복구된
것으로 간주하지 않는다. 원본 백업은 Git 제외 대상이므로
`isaac_sim/build/` 정리 전에 별도 보관한다.

## 검증

Ubuntu 24.04 / Isaac Sim 5.1.0의 실제 GUI·GPU 환경에서 실행했다.

```bash
cd /home/buddha/my_workspace/amr_with_issacsim/isaac_sim
ROS_DOMAIN_ID=42 bash run.sh --control ros --steps 180
```

저장 장면·Physics 편집 확장·ROS Bridge·센서 구성을 거쳐 180회 앱 갱신 후
종료 코드 0으로 정상 종료했다. 실행 로그는 `/tmp/amr_usd_recovery_gui.log`다.
기존 두 빈 링크의 visual 참조 경고와 SDK deprecation 경고는 남아 있지만
이번 실행을 중단하지 않았다. ROS 수신기·키보드 주행·슬립 개선 시험은
이번 복구 검증에 포함하지 않았다. 소스 코드나 `run.sh` 변경은 필요하지 않았다.

일반 실행은 횟수 제한 없이 사용한다.

```bash
bash run.sh --control ros
```

## 재발 시 처리

1. 현재 USD를 먼저 별도 복사하고 크기·해시를 기록한다.
2. 셸 오류 마지막 줄보다 앞의 USD 오류·native stack을 확인한다.
3. 정상 커밋의 후보 파일을 별도 위치에서 검증한다.
4. 백업을 유지하며 정상 후보로 복구하고 실제 SDK에서 재실행한다.

USD를 바꾸는 Git 전환·복구 전에 해당 장면을 연 앱을 종료하고, 같은 파일을
여러 프로세스에서 동시에 저장하지 않는 방식으로 관리한다. 이것이 이번 손상의
확정 원인이라는 의미는 아니다. USD crate는 데이터를 지연 로드하고 기본적으로
메모리 매핑을 사용한다([OpenUSD 설명](https://openusd.org/25.11/glossary.html)).
