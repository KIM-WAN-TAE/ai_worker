# Motion Inspector 사용 안내

Motion Inspector는 F2 팔 명령과 실제 관절 응답을 실시간으로 비교하고, 필요한 구간을 rosbag과 정량 리포트로 남기는 비구동 진단 화면입니다. 이 검토 버전에는 로봇 명령 발행과 replay 기능이 없습니다.

## 1. 접속

로봇 PC와 노트북을 같은 격리된 로봇 LAN에 연결합니다. 로봇 PC 주소를 확인한 뒤 브라우저에서 다음 주소로 접속합니다.

```text
http://<robot-ip>:8765
```

접속이 되지 않으면 로봇 PC에서 컨테이너 상태와 API를 확인합니다.

```bash
docker ps --filter name=task_motion_inspector
curl http://127.0.0.1:8765/api/state
```

고정 IP를 문서에 전제로 두지 않습니다. 현재 네트워크 설정은 현장 담당자에게 확인하십시오.

## 2. 화면 읽기

상단 요약 카드에는 다음 정보가 표시됩니다.

| 항목 | 의미 |
| --- | --- |
| `Command source` | arbiter 또는 topic activity로 확인한 `TELEOP`, `POLICY`, `NONE` |
| `Max tracking error` | 전체 팔 관절 중 가장 큰 `Reference - Feedback` 절댓값 |
| `Active alerts` | 확인이 필요한 관절 수 |
| `DXL health` | Follower DYNAMIXEL 통신 상태 |

관절 표의 주요 열은 다음과 같습니다.

| 열 | 의미 |
| --- | --- |
| `Command` | Follower로 전달된 최종 `JointTrajectory` |
| `Reference` | controller 내부 목표 |
| `Actual` | 같은 controller state 메시지의 feedback |
| `Error` | `Reference - Actual` |
| `State` | 추종 오차, limit, 포화 또는 stale 진단 |

행을 선택하면 최근 60초 위치와 추종 오차 차트가 표시됩니다. `J7` 필터로 좌우 wrist 관절만 볼 수 있습니다.

## 3. 상태 해석

| 상태 | 의미 |
| --- | --- |
| `NORMAL` | 현재 확인된 이상 없음 |
| `CHECK` | source·command·controller 단계의 값 차이 확인 필요 |
| `TRACKING` | 기본 `2°` 이상 추종 오차가 지속됨 |
| `CRITICAL` | 기본 `8°` 이상 큰 추종 오차 |
| `LIMIT` | 목표가 설정된 DXL 범위 밖임 |
| `SATURATED` | DXL endpoint 부근에서 목표를 따라가지 못하는 상태 |
| `STALE` | 관련 신호가 최신이 아님 |

화면의 DXL range는 runtime `/robot_description`의 설정값입니다. 실제 actuator EEPROM을 읽은 결과로 해석하면 안 됩니다.

## 4. 세션 기록

1. 필요한 Follower·Leader 또는 inference를 기존 승인 절차로 별도 실행합니다.
2. Motion Inspector에서 command source와 topic 상태가 예상과 일치하는지 확인합니다.
3. `Session name`에 조건을 구분할 이름을 입력합니다.
4. `Record`를 누릅니다.
5. 조사할 동작 구간을 수행합니다.
6. `Stop`을 눌러 기록을 끝냅니다.
7. 분석이 끝나면 `Saved session`에서 세션을 선택하고 `Report`를 엽니다.

예시는 다음과 같습니다.

```text
j7_conservative_limit_teleop_01
policy_tracking_shadow_01
intervention_handoff_observation_01
```

Motion Inspector의 `Record`는 기존 로봇 동작을 시작하지 않습니다. 이미 ROS graph에 있는 진단 topic을 기록할 뿐입니다.

## 5. 결과 확인

기본 저장 경로는 다음과 같습니다.

```text
/home/robotis/ai_worker/docker/workspace/task_motion_inspector/sessions/
```

주요 결과는 다음과 같습니다.

- `motion_inspector.json`: source, 설정 digest, limit과 event
- `summary.json`: 프로그램용 요약
- `joint_metrics.csv`: 관절별 MAE, RMSE, P95, 최대 오차 등
- `report.md`: 검토용 Markdown 보고서
- `*.mcap`: 원본 rosbag

리포트가 없는 기존 세션은 `Analyze` 버튼으로 다시 분석할 수 있습니다.

## 6. J7 조사 시 확인할 항목

변경 전후를 비교할 때 같은 시작 자세와 비슷한 속도·경로를 사용하고 다음 항목을 확인합니다.

- 좌우 J7 command 범위
- controller reference와 feedback
- DXL present position과 current
- MAE, P95, 최대 추종 오차
- `OUTSIDE_DXL_RANGE`와 `HARD_LIMIT_SATURATION`
- 설정 digest와 runtime limit snapshot

현재 active J7 값과 PH42 환산 후보는 [F2 J7 위치 제한 오류 분석](../../docs/review/f2_j7_position_limit_error.md)에 구분되어 있습니다. 후보 값을 실기 설정으로 자동 적용하거나, Motion Inspector 결과만으로 최종 범위를 확정하지 마십시오.

## 7. 안전 범위

Motion Inspector 자체는 robot command를 publish하지 않습니다. 다만 화면을 관찰하는 동안 별도 Leader나 policy가 powered motion을 수행할 수 있으므로, 해당 동작은 기존의 현장 승인·E-stop·작업 공간 확인 절차를 그대로 따라야 합니다.

오류가 보이면 Motion Inspector 화면의 상태만 믿고 계속 운전하지 말고, 실제 명령 source를 중단한 뒤 controller·DXL 상태와 현장을 확인하십시오.
