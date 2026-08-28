# F2 Safety: A3 시작 입력과 J7 limit 검토

## 연결 커밋

```text
[REVIEW] F2 Safety: prevent false A3 triggers and document J7 limits
```

## 기능 범위

이 변경은 서로 다른 두 F2 하드웨어 안전 항목을 한 검토 단위로 묶는다.

1. A3가 시작될 때 이미 닫혀 있던 gripper를 새 입력으로 오인하는 문제를 수정한다.
2. 잘못 계산된 것으로 확인된 J7 DYNAMIXEL limit의 PH42 후보값을 기록하되, 승인 전에는
   활성화하지 않는다.

관절별 follower limit 진입·해제 로그는 Motion Inspector가 사용하지 않고 실제 운용에서
작은 gripper endpoint 오차에도 반복 출력됐으므로 이 커밋에서 제외했다. 기존 follower
limit clamp 동작 자체는 변경하지 않는다.

## 1. A3 시작 시 gripper 오입력 방지

### 문제

2026년 8월 12일 현장 로그에서 Follower가 실행 중인 상태로 A3를 시작한 약 2초 뒤,
오른쪽 gripper 첫 값 `-3.52`가 threshold `-0.50` 아래로 들어왔다. 기존 구현은 내부
상태를 항상 “threshold 위”로 시작했기 때문에 이 첫 값을 새 falling edge로 판단했다.

그 결과 작업자가 gripper를 새로 누르지 않았는데도 다음 명령이 발생했다.

```text
enable=3
Start interpolation to save pose 3
```

### 수정

`GripperEdgeGate`가 첫 번째 finite sample을 현재 상태의 baseline으로만 저장한다. 첫
sample에서는 명령을 발행하지 않고, 이후 실제 `above → below` crossing만 새 입력으로
인정한다.

시작할 때 gripper가 이미 닫혀 있으면 다음 순서를 거쳐야 한다.

```text
첫 sample: below 상태를 baseline으로 저장, 명령 없음
gripper 해제: above 상태 확인
다시 누름: above → below crossing, enable=3 한 번 발행
```

`NaN`과 `Inf` sample은 baseline이나 edge 상태를 변경하지 않고 무시한다.

이 수정 후 UI와 s6가 사용하는 표준 A3 launch에서 `launch_gripper_trigger:=true`를 다시
사용한다.

### 변경 파일

- `ffw_joint_trajectory_command_broadcaster/include/.../gripper_edge_gate.hpp`
- `ffw_joint_trajectory_command_broadcaster/src/gripper_trigger.cpp`
- `ffw_joint_trajectory_command_broadcaster/test/test_gripper_edge_gate.cpp`
- `ffw_joint_trajectory_command_broadcaster/CMakeLists.txt`
- `docker/s6-services/avatar_bringup/run`
- `docs/f2_a3_leader_follower_control_architecture.md`

### 테스트 항목

- 시작 sample이 threshold 아래여도 trigger하지 않음
- 닫힌 상태로 시작하면 해제 후 다시 눌러야 trigger함
- 열린 상태로 시작한 뒤 실제로 누르면 한 번 trigger함
- `NaN` sample이 상태를 초기화하거나 trigger하지 않음

## 2. J7 DYNAMIXEL limit 후보 기록

### 확인된 오류

오른팔 J7(ID 7)과 왼팔 J7(ID 37)은 실제로 `PH42-020-S300-R`이며 분해능은
`607500 tick/rev`다. 기존 limit 값 `[-151875, 131875]`는 Y 계열의
`524288 tick/rev`로 왼팔 URDF를 환산한 값과 거의 정확히 일치한다.

또한 좌우 URDF는 mirror 관계지만 기존 F2 설정에는 양쪽 J7에 같은 raw limit이 들어가
있다. 그 결과 기존 DXL 범위는 실제 PH42 기준으로 양쪽 모두
`[-90.00°, +78.15°]`가 되고, 오른팔 upper는 URDF보다 약 `26.14°` 좁다.

원인과 영향은 [J7 위치 제한 설정 오류](./f2_j7_position_limit_error.md)에 시각 자료와
함께 정리했다.

### 현재 적용 상태

활성값은 기존 F2 값을 그대로 유지한다.

```text
오른팔 ID 7 : [-151875, 131875] tick
왼팔 ID 37  : [-151875, 131875] tick
```

PH42 분해능으로 URDF 범위를 포함하도록 계산한 다음 값은 기본 Xacro의 XML 주석으로만
남긴다.

```text
오른팔 ID 7 후보 : [-152804, 175980] tick
왼팔 ID 37 후보  : [-175980, 152804] tick
```

주석 후보는 적용된 설정이 아니다. 이 커밋은 DYNAMIXEL EEPROM을 변경하지 않으며,
후보값으로 로봇을 구동하거나 실기 안전성을 검증하지 않는다.

### Manager 검토 항목

- 설치된 ID 7·37의 EEPROM Min/Max/Homing Offset read-back
- torque-off 상태의 실제 기구 가동 범위
- 카메라, 케이블, cover, gripper 및 양팔 간섭 여부
- 기본 Xacro와 smooth Xacro를 같은 기준으로 변경할지 여부
- 새 범위가 기존 dataset/checkpoint 계약에 미치는 영향

## 검증 수준과 제한

- A3 edge gate는 단위 테스트와 소스 빌드 대상으로 검증한다.
- J7 계산은 model file, 실행 ping 기록, Git 이력 및 기존 데이터 분석에 근거한다.
- 이 커밋을 위해 로봇을 시작하거나 controller를 재시작하지 않는다.
- J7 후보 활성화와 powered validation은 별도의 manager 결정과 안전 시험이 필요하다.

## 검토자가 확인할 핵심

1. 첫 gripper sample을 무명령 baseline으로 처리하는 동작이 원하는 UX인지 확인한다.
2. J7 후보값이 활성값이 아니라 주석이라는 점을 확인한다.
3. 향후 J7 limit 변경을 별도 배포 계약 변경으로 처리할지 결정한다.
