# Intervention: F2 양팔 명령 소유권과 arbiter

## 연결 커밋

```text
[REVIEW] Intervention: route F2 arm commands through a single owner
```

## 기능 관계

이 커밋과 다음 `Intervention handoff` 커밋은 2026년 8월 14일 같은 Intervention
프로토타입을 구현하면서 함께 개발됐다.

- 이 커밋은 policy와 A3가 F2 최종 arm topic을 동시에 소유하지 못하게 하는 명령 기반을
  제공한다.
- 다음 커밋은 이 기반을 사용해 inference pause, A3 시작, 교정 녹화, 저장·폐기 및 policy
  resume 순서를 관리한다.

따라서 실제 Intervention 기능에는 두 커밋이 모두 필요하지만, 명령 경로와 운영 절차를
따로 검토하고 되돌릴 수 있도록 분리한다.

## 해결하려던 문제

기존 구조에서는 policy publisher와 A3 Leader가 최종 F2 arm controller topic에 직접
연결될 수 있었다. 두 source가 동시에 살아 있거나 전환 중 이전 publisher가 남으면 어떤
명령이 실제로 선택되는지 보장하기 어렵다.

Intervention은 실행 중인 policy를 멈추고 A3에 제어권을 넘긴 뒤 다시 policy로 돌아가야
한다. 단순히 한 publisher를 늦게 시작하는 방식으로는 다음 위험을 막을 수 없다.

- policy와 A3의 동시 publish
- 오래된 source의 마지막 trajectory 재사용
- 왼팔과 오른팔의 서로 다른 시점 명령 결합
- 잘못된 joint 순서나 크기의 메시지 전달
- `NaN`, `Inf` 또는 설치된 F2 limit 밖의 command 전달
- source가 끊긴 뒤 마지막 command를 계속 유지하는 문제

## 명령 구조

Policy와 A3는 최종 controller topic이 아니라 분리된 입력 topic으로 publish한다.

```text
Policy ──> /intervention/policy/arm_left  ──┐
Policy ──> /intervention/policy/arm_right ──┤
                                             ├─ command_source_arbiter
A3     ──> /intervention/teleop/arm_left  ──┤        │
A3     ──> /intervention/teleop/arm_right ──┘        ▼
                                      F2 최종 left/right arm controller topic
```

`command_source_arbiter`가 최종 F2 양팔 topic의 유일한 의도된 publisher가 된다.

OMX 오른손목 observer는 검토 대상에서 제외했으므로 `OBSERVER` mode와
`select_observer` 서비스는 포함하지 않는다.

## 소유권 mode

| Mode | 왼팔 source | 오른팔 source | 목적 |
| --- | --- | --- | --- |
| `NONE` | 없음 | 없음 | 시작, 전환, fault 상태에서 command 차단 |
| `POLICY` | policy | policy | inference 명령 전달 |
| `TELEOP` | A3 | A3 | Intervention 수동 교정 |

arbiter는 항상 `NONE`으로 시작한다. 준비되지 않은 source를 자동으로 선택하지 않으며,
fault가 latch된 상태에서는 입력이 다시 정상으로 돌아와도 명시적으로 clear하기 전까지
재선택하지 않는다.

## 입력 검증

Mode 선택과 활성 source 전달 전에 양팔 메시지에 다음 조건을 적용한다.

- joint 이름과 순서가 F2 계약과 정확히 일치
- trajectory point가 정확히 한 개
- position 개수가 joint 개수와 일치
- 모든 position이 finite 값
- 설치된 F2 effective limit 안의 값
- 왼팔과 오른팔 source가 모두 최근 메시지
- 좌우 message receipt time 차이가 설정된 pair skew 이내

기본값은 source 최대 age `0.25초`, 좌우 최대 skew `0.05초`다. 활성 source가 오래되거나
잘못된 message를 보내면 해당 sample만 clamp하여 계속하지 않는다. 전체 source를
fail-closed 처리하고 mode를 `NONE`으로 바꾼다.

## 최종 topic 충돌 방지

Mode를 선택할 때 최종 F2 arm topic의 publisher 정보를 조회한다. Arbiter 자신 외의
publisher가 남아 있으면 source 선택을 거부하고 publisher 이름을 fault에 기록한다.

이 검사는 “arbiter가 sole publisher여야 한다”는 설계 의도를 runtime에서 확인하기 위한
것이다. 다른 publisher를 강제로 종료하지는 않으며, 충돌이 해결되고 fault를 clear할
때까지 command를 전달하지 않는다.

## A3 연결

표준 `ffw_a3_ai.launch.py`에 왼팔과 오른팔 trajectory 출력 topic argument를 추가한다.
기본값은 기존 topic을 유지하므로 일반 A3 launch 동작은 바뀌지 않는다. Intervention에서만
두 argument를 `/intervention/teleop/arm_left|right`로 지정한다.

F2 follower launch는 `command_source_arbiter`를 함께 시작한다. A3와 policy가 격리
topic을 사용하기 전에는 `NONE`을 유지해야 하며, source readiness가 확인된 뒤에만
`POLICY` 또는 `TELEOP`을 선택한다.

### validation에서 보완한 실제 hand-off

추가 mock 통합 시험에서 A3 launch argument와 arbiter 서비스는 존재하지만,
Intervention controller가 이를 실제 hand-off 순서에 연결하지 않은 공백을 확인했다.
이 상태에서는 A3가 기존 최종 topic으로 직접 출력하거나 arbiter가 `NONE`에 남을 수
있었다.

`validation/f2-features`에서는 다음과 같이 연결했다.

1. Intervention 안전 gate 통과 후 arbiter의 `prepare_intervention`을 호출한다.
2. 유효한 일회성 s6 lease가 있을 때만 A3 출력을
   `/intervention/teleop/arm_left|right`로 remap한다.
3. A3 양팔 입력이 fresh pair가 된 뒤에만 `select_teleop`을 호출한다.
4. end·abort·watchdog에서는 먼저 `select_none`으로 출력을 차단하고 Leader를
   종료한 뒤 `disable_intervention`으로 상태를 정리한다.
5. policy resume은 Cyclo가 fresh policy pair를 확인한 뒤 `select_policy`를 호출하는
   기존 경로를 유지한다.

Leader s6 lifecycle과 one-shot lease는 Intervention controller가 단독으로 소유한다.
따라서 F2 launch에서는 arbiter의 `manage_leader_service`를 끄고, arbiter는 command
routing과 readiness 검증만 담당한다. `prepare_intervention`은 latched fault를 자동으로
지우지 않으며, fault가 있으면 명시적인 검토와 `clear_fault` 전까지 hand-off를
거부한다.

## 상태와 서비스

주요 서비스는 다음과 같다.

- `select_none`: 양팔 command 전달 중지
- `select_policy`: fresh policy pair 검증 후 policy 선택
- `select_teleop`: fresh A3 pair 검증 후 teleop 선택
- `clear_fault`: fault 해제 후 `NONE` 유지
- `get_status`: mode, fault, source readiness와 publisher 충돌 상태 조회
- `prepare_intervention`, `disable_intervention`: 다음 handoff 계층에서 사용하는 A3 준비·종료

상태는 `/command_source_arbiter/status`에 JSON으로 publish한다. A3 broadcaster에는
`/leader/intervention_mode`를 전달해 Intervention 중 save-pose 입력을 차단한다.

## 주요 변경 파일

- `ffw_bringup/ffw_bringup/command_source_arbiter.py`
- `ffw_bringup/test/test_command_source_arbiter.py`
- `ffw_bringup/launch/f2_follower_ai.launch.py`
- `ffw_bringup/launch/ffw_a3_ai.launch.py`
- `ffw_bringup/setup.py`
- `ffw_bringup/package.xml`

## 검증 항목

- 기본 mode가 `NONE`인지 확인
- fresh 양팔 pair가 없으면 선택을 거부하는지 확인
- joint 순서, position 크기 및 non-finite 값을 거부하는지 확인
- limit 밖의 한 관절이 있으면 whole source sample을 거부하는지 확인
- 활성 source 오류와 stale lease가 fault를 latch하고 `NONE`으로 만드는지 확인
- fault clear 전 자동 재개하지 않는지 확인
- 좌우 pair skew를 거부하는지 확인
- OMX `OBSERVER` mode나 관련 service가 남아 있지 않은지 확인
- 유효한 Intervention lease에서만 A3 출력이 teleop 입력으로 remap되는지 확인
- mock ROS graph에서 `NONE → TELEOP → NONE → POLICY → NONE` 전환과 실제 message
  차단·전달을 확인
- latched fault가 `prepare_intervention`으로 자동 해제되지 않는지 확인
- arbiter가 `rclpy.Node` 내부 subscription 상태를 덮어쓰지 않고 정상 종료되는지 확인

mock ROS graph 검사는 network가 차단된 일회성 컨테이너에서 다음 script로 수행한다.

```text
ffw_bringup/test/mock_intervention_handoff.py
```

이 검사는 실제 controller나 DYNAMIXEL에 연결하지 않고 mock topic만 사용한다.

## 안전 수준과 남은 제한

이 커밋은 명령 routing 기반을 구현하지만 powered Intervention 검증 완료를 의미하지
않는다. Policy와 A3의 실제 전환은 다음 handoff 커밋의 pause/resume, s6 lease, recording
절차와 함께 검증해야 한다.

또한 arbiter는 팔 명령만 다룬다. Head, lift와 mobile base는 이 routing에 포함되지 않으며,
별도 계약과 명시적 승인 없이 Intervention 대상으로 확장하면 안 된다.

## 검토자가 확인할 핵심

1. 최종 arm topic publisher를 arbiter 한 개로 제한하는 구조가 적절한지 확인한다.
2. `0.25초` source age와 `0.05초` pair skew 기본값을 검토한다.
3. effective limit 위반 시 clamp가 아니라 whole sample 거부가 맞는지 확인한다.
4. 다음 Intervention handoff 커밋과 함께 적용할지 결정한다.
