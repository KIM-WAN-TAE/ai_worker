# Motion Inspector: 비구동 명령·관절 진단

## 연결 커밋

```text
[REVIEW] Motion Inspector: add read-only command and joint diagnostics
```

## 개발 배경

F2 J7 문제를 조사할 때 “상위 명령이 잘못됐는지”, “controller가 다른 값을 받았는지”,
“DXL limit에서 실제 관절이 멈췄는지”를 로그 한 줄만으로 구분하기 어려웠다. 서로 다른
시점의 최신 command와 `/joint_states`를 비교하면 지연까지 오차로 보일 수 있고, 특정
관절만 임시 로그를 추가하는 방식은 다른 관절이나 policy 경로에 재사용하기 어렵다.

Motion Inspector는 다음 신호를 같은 화면과 rosbag에 모아 이 문제를 재현 가능하게
조사하기 위해 만들었다.

```text
Leader target ─┐
Policy preview ├─> Final trajectory ─> Controller reference ─> Feedback ─> DXL state
               └─> Command source 상태
```

## 이번 검토 범위

이 커밋은 모니터링, 선택 구간 녹화와 오프라인 분석만 포함한다.

- ROS topic을 구독해 최근 60초의 팔 명령·응답을 표시한다.
- 명령 source, 추종 오차, 설정 limit과 DXL 상태를 진단한다.
- 선택 구간을 MCAP rosbag으로 저장한다.
- 세션별 JSON, CSV와 Markdown 리포트를 생성한다.
- robot command publisher, rosbag play와 replay API는 포함하지 않는다.

따라서 Motion Inspector 자체는 로봇을 움직이지 않는다. 다만 별도로 실행한 Leader나
policy를 관찰할 수 있으므로, powered motion의 승인과 현장 안전 절차를 대신하지 않는다.

## 신호 계약

| 단계 | Topic 또는 필드 | 용도 |
| --- | --- | --- |
| 변환된 Leader 목표 | `/robot/arm_left_leader/joint_states`, `/robot/arm_right_leader/joint_states` | reverse·offset·blend·limit이 반영된 broadcaster 목표 |
| Policy 미리보기 | `/inference/trajectory_preview` | policy가 생성한 trajectory preview |
| 최종 팔 명령 | `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory`, 오른팔 대응 topic | Follower로 전달되는 최종 command |
| Controller 상태 | `/arm_l_controller/controller_state`, `/arm_r_controller/controller_state` | 같은 메시지의 reference·output·feedback·error |
| 일반 관절 상태 | `/joint_states` | 독립 상태 확인 |
| DXL 상태 | `/dynamic_joint_states`, `/ffw_follower/dxl_state` | position, current, 통신·hardware error |
| 명령 소유권 | `/command_source_arbiter/status` | `NONE`, `POLICY`, `TELEOP`과 fault |
| runtime 설정 | `/robot_description` | URDF·ros2_control arm limit snapshot |

`JointTrajectoryControllerState` 한 메시지 안의 `reference`와 `feedback`을 우선 사용해
추종 오차를 계산한다. 이 방식은 서로 다른 topic의 마지막 표본 시각이 달라서 생기는
거짓 오차를 줄인다.

arbiter 상태가 없으면 최근 Leader·policy topic activity로 source를 추정하지만, 이 값은
확정 상태와 구분해 표시한다. source가 바뀌면 다른 source의 과거 값을 현재 비교값으로
재사용하지 않는다.

## limit 해석

F2 양팔 J1~J7의 actuator model과 해상도는 다음처럼 구분한다.

| 관절 | Model | Resolution |
| --- | --- | ---: |
| J1·J2 | YM080 | 524,288 tick/rev |
| J3~J6 | YM070 | 524,288 tick/rev |
| J7 | PH42-020-S300-R | 607,500 tick/rev |

runtime `/robot_description`에서 URDF lower·upper와 ros2_control의 Min·Max Position Limit을
읽어 비교한다. 이 값은 설정 snapshot이지 DXL EEPROM read-back이 아니다.

runtime description이 도착하기 전 fallback은 이번 검토 브랜치의 active 보수 값과
일치시켰다. 특히 좌우 J7 모두 active tick 범위 `[-151875, 131875]`를 사용한다. PH42
해상도로 계산한 확장 후보는 표시·분석 코드에 자동 적용하지 않는다.

## 진단 규칙

- source와 final command 차이
- final command와 controller reference 차이
- controller reference와 output 차이
- 기본 `2°` 이상 추종 오차가 `0.25초` 지속
- 기본 `8°` 이상 임계 추종 오차
- 설정된 DXL 범위 밖 목표
- DXL endpoint 부근에서 목표를 따라가지 못하는 포화 가능성
- stale topic, DXL 통신 오류와 hardware error

경보는 짧은 표본 흔들림으로 반복 열리지 않도록 debounce하며, 기본 `0.5초` 동안 정상
범위가 유지된 뒤 해제한다. 이 임계값은 조사 편의를 위한 진단 기준이며 controller의
공식 safety limit을 대체하지 않는다.

## 세션과 산출물

기본 저장 위치는 다음과 같다.

```text
docker/workspace/task_motion_inspector/sessions/<timestamp>_<session_name>/
```

각 세션에는 다음 산출물이 남는다.

- 원본 MCAP rosbag
- `motion_inspector.json`: source, 설정 digest, limit snapshot과 경보 event
- `summary.json`: 구조화된 결과
- `joint_metrics.csv`: 관절별 MAE, RMSE, P95·최대 오차 등
- `report.md`: 검토자가 읽을 수 있는 요약

오프라인 분석은 관절별 command 범위, 추종 오차 분포, 경고 지속시간, DXL 범위 밖 표본,
최대 current와 hardware error 표본 수를 계산한다.

## 주요 변경 파일

- `docker/task_motion_inspector/logic.py`
- `docker/task_motion_inspector/server.py`
- `docker/task_motion_inspector/analyze_session.py`
- `docker/task_motion_inspector/static/`
- `docker/task_motion_inspector/test_logic.py`
- `docker/task_motion_inspector/compose.yml`
- `docker/task_motion_inspector/run.sh`
- `docker/task_motion_inspector/README.md`
- `docker/task_motion_inspector/USAGE_KO.md`

## 이번 커밋에서 제외한 자료

`J7_ERROR_TO_MOTION_INSPECTOR_DEVELOPMENT_LOG_KO.md`와 `J7-1.png`, `J7-2.png`는 시험 중
mirror·확장 limit을 적용했던 시점의 값과 화면을 포함해 현재 active 설정과 충돌한다.
따라서 Git 검토 이력에는 포함하지 않는다. J7 오류의 현재 정리는
`docs/review/f2_j7_position_limit_error.md`와 그 시각 자료를 기준으로 한다.

물리 replay 프로토타입도 이 검토 커밋에서 제거했다. Motion Inspector의 목적을 진단으로
한정하고, rosbag이 로봇 명령을 재발행하는 별도 powered 기능이 검토 없이 섞이지 않게
하기 위해서다. 필요하다면 별도 기능·안전 계약·현장 승인 아래 다시 검토해야 한다.

## 검증 항목

- runtime XML에서 arm URDF·DXL 설정 범위를 읽는지 확인
- J1~J7 model별 tick resolution 확인
- 현재 J7 fallback 값 확인
- non-finite 값과 잘못된 session 이름 처리
- pipeline 단계별 값 차이와 tracking·limit·포화 분류
- 경보 open·close debounce
- Python·JavaScript·shell 문법 검사
- 실행 중 container에서 진단 logic 단위 테스트
- 기존 MCAP 세션을 `--no-write`로 재분석해 JSON summary 생성 확인

## 남은 제한

- EEPROM Min·Max Position Limit을 직접 읽지 않는다.
- topic timestamp와 QoS가 원본 node에서 부정확하면 화면 정렬에도 그 한계가 남는다.
- source 추정 모드는 arbiter status보다 신뢰도가 낮다.
- 화면과 리포트는 원인 조사 증거이며 collision, 기구 간섭 또는 powered 안전 검증을
  대신하지 않는다.
- 기존 세션의 sidecar에는 녹화 당시 설정이 보존되므로, 과거 limit snapshot을 현재 active
  설정으로 해석하면 안 된다.

## 검토자가 확인할 핵심

1. 비교하는 topic과 `reference - feedback` 기준이 실제 F2 controller 계약과 맞는지
   확인한다.
2. `2°/0.25초` 경고와 `8°` 임계 기준을 진단 기본값으로 유지할지 검토한다.
3. runtime 설정과 EEPROM read-back을 화면에서 충분히 구분했는지 확인한다.
4. 비구동 진단 기능으로 유지하고 replay는 별도 기능으로 분리하는 결정을 확인한다.
