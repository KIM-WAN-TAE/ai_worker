# Motion Inspector

Motion Inspector는 Leader 또는 policy에서 F2 Follower로 전달되는 팔 명령과 controller·DYNAMIXEL 응답을 같은 시간축에서 비교하는 비구동 진단 도구입니다.

이 검토 버전은 ROS topic을 구독하고 rosbag을 기록·분석할 뿐, 로봇 명령 publisher와 rosbag replay API를 제공하지 않습니다.

J7 오류의 원인과 현재 보수 설정, 승인 전 후보 값은 [F2 J7 위치 제한 오류 분석](../../docs/review/f2_j7_position_limit_error.md)을 참고하십시오.

## 실행

저장소의 `docker` 디렉터리에서 실행합니다.

```bash
docker compose -f task_motion_inspector/compose.yml up -d
```

기본 주소는 `http://localhost:8765`입니다. 같은 로봇 LAN에서는 `http://<robot-ip>:8765`로 접속합니다.

```bash
docker logs -f task_motion_inspector
docker compose -f task_motion_inspector/compose.yml down
```

## 비교하는 신호

| 단계 | ROS 신호 | 의미 |
| --- | --- | --- |
| Leader target | `/robot/arm_*_leader/joint_states` | broadcaster가 변환한 Leader 목표 |
| Policy preview | `/inference/trajectory_preview` | policy가 생성한 미리보기 명령 |
| Final command | `/leader/joint_trajectory_command_broadcaster_*/joint_trajectory` | Follower로 전달되는 최종 trajectory |
| Controller reference | `/arm_*_controller/controller_state.reference` | controller 내부 목표 |
| Controller feedback | `/arm_*_controller/controller_state.feedback` | controller가 사용하는 실제 관절값 |
| DXL position/current | `/dynamic_joint_states` | actuator 상태 |

추종 오차는 서로 다른 시점의 최신값을 단순 비교하지 않고, 같은 `JointTrajectoryControllerState` 메시지의 `reference`와 `feedback`을 우선 사용합니다.

`/command_source_arbiter/status`가 있으면 `TELEOP`, `POLICY`, `NONE`을 직접 표시합니다. 이 topic이 없을 때만 최근 Leader·policy topic activity로 source를 추정하며, 화면에 추정 상태임을 표시합니다.

## 경보와 limit

- 기본 추종 오차 경고: `2°`가 `0.25초` 이상 지속
- 기본 임계 추종 오차: `8°`
- 경보 해제: `0.5초` 동안 정상 범위 유지
- DXL limit 근접 또는 범위 밖 command
- DXL hard-limit 포화 가능성
- stale topic, DXL 통신 상태와 hardware error

J1·J2는 YM080, J3~J6은 YM070, J7은 PH42 해상도로 tick을 radian으로 환산합니다. 화면의 DXL 범위는 `/robot_description`의 ros2_control 설정 snapshot이며 EEPROM read-back이 아닙니다. `/robot_description`을 받기 전에는 현재 검토 브랜치의 보수 설정을 fallback으로 사용하고, 화면에 `fallback`으로 표시합니다.

## 녹화와 분석

세션은 다음 경로에 저장됩니다.

```text
/home/robotis/ai_worker/docker/workspace/task_motion_inspector/sessions/
```

`Record`와 `Stop`으로 선택 구간을 MCAP rosbag에 기록합니다. 녹화가 끝나면 다음 결과를 생성합니다.

| 파일 | 내용 |
| --- | --- |
| `motion_inspector.json` | command source, 설정 digest, limit snapshot, 경보 event |
| `summary.json` | 구조화된 분석 결과 |
| `joint_metrics.csv` | 관절별 정량 지표 |
| `report.md` | 사람이 읽는 요약 |
| `*.mcap` | 원본 ROS 기록 |

리포트에는 MAE, RMSE, P95·최대 추종 오차, 경고 지속시간, 설정된 DXL 범위 밖 command 표본, 최대 current와 hardware error 표본 수가 포함됩니다.

## 검증 범위

단위 테스트는 다음을 확인합니다.

- 안전한 session 이름 정규화
- runtime URDF·ros2_control limit 해석
- F2 arm motor별 resolution
- source→command→controller 단계 차이 분류
- tracking·limit·포화 경보
- 경보 debounce

이 도구의 화면과 리포트는 원인 조사 자료입니다. 표시가 정상이라는 이유만으로 powered motion의 안전성이나 J7 최종 limit을 확정하면 안 됩니다.
