# AI Worker F2 기능 검토 브랜치 안내

## 문서 목적

이 문서는 `review/f2-features` 브랜치에 모은 변경을 매니저가 기능 단위로 검토할 수
있도록 정리한 진입점이다. 각 기능의 개발 배경, 주요 코드, 검증 결과와 남은 제한은
아래의 상세 문서로 연결한다.

이 브랜치는 개인 저장소 `ukh0415/ai_worker_intern`에서 검토하기 위한 브랜치다. 원본
저장소의 `main`에 반영됐거나 production 배포가 승인됐다는 의미가 아니다.

## 브랜치와 기준

| 항목 | 값 |
| --- | --- |
| 개인 저장소 | `git@github.com:ukh0415/ai_worker_intern.git` |
| 검토 브랜치 | `review/f2-features` |
| 기능 분리 전 기준 | `78f3126` (`feature-intern`) |
| 커밋 접두어 | `[REVIEW]` |

## 기능별 커밋

| 커밋 | 큰 기능 | 포함 내용 | 상세 문서 |
| --- | --- | --- | --- |
| `508d078` | F2 Safety | A3 시작 시 첫 gripper 값을 기준값으로만 저장하고, J7 limit 문제와 PH42 후보를 문서화 | [F2 Safety](./f2_safety.md) |
| `d94d0d7` | Intervention | policy와 A3 명령을 한 arbiter로 모아 F2 양팔 최종 topic의 publisher를 하나로 제한 | [명령 소유권과 arbiter](./intervention_command_routing.md) |
| `08604d3` | Intervention | inference pause, A3 교정·녹화, 저장·폐기, policy resume 사이의 안전한 제어권 전환 | [추론과 A3 handoff](./intervention_handoff.md) |
| `f83dedc` | Motion Inspector | 명령·controller·DXL 신호의 실시간 비교, MCAP 녹화와 정량 리포트. 명령 publisher와 replay는 제외 | [Motion Inspector](./motion_inspector.md) |
| `9289b22` | Docker Workflow | AI Worker 셸 별칭을 단일 파일로 관리하고 개발 컨테이너에 read-only로 연결 | [Docker Workflow](./docker_workflow.md) |
| `abc5919` | F2 Safety | base·smooth Xacro의 오른쪽 J7 active endpoint를 mirror 관계로 복구하고, Motion Inspector·문서·시각자료를 같은 기준으로 정렬 | [J7 위치 제한 오류](./f2_j7_position_limit_error.md) |
| `e52cf80` | Motion Inspector | `SIGINT`·`SIGTERM` 종료 시 ROS context 중복 shutdown 오류를 남기지 않도록 정리 | [Motion Inspector](./motion_inspector.md) |

## 현재 기능 상태

### F2 Safety와 J7

- A3 시작 당시 gripper가 이미 닫혀 있어도 이를 새 누름으로 오인하지 않도록 첫 값을
  baseline으로 처리한다.
- 현재 검토 소스의 J7 active tick은 오른쪽 ID 7 `[-131875, 151875]`, 왼쪽 ID 37
  `[-151875, 131875]`다. 좌우 endpoint 배치는 mirror 관계다.
- PH42의 `607500 tick/rev`로 계산한 후보는 오른쪽 `[-152804, 175980]`, 왼쪽
  `[-175980, 152804]`이며, base·smooth Xacro의 XML 주석에만 있다.
- PH42 후보는 기구·카메라·케이블 안전 확인과 manager 승인 전에는 활성화하지 않는다.

### Intervention

- 최종 양팔 controller topic은 `command_source_arbiter` 하나만 publish하도록 구성한다.
- `NONE`, `POLICY`, `TELEOP`, `OBSERVER` source의 메시지 형식, freshness, 좌우 pair와
  설치된 limit을 검사하고, 이상이 있으면 `NONE`으로 닫는다.
- inference와 A3 사이의 전환 순서, 교정 구간 녹화와 episode 번호 동기화를 포함한다.
- 구현과 비구동 검증은 되어 있지만, powered Intervention 완료 기능으로 승인된 상태는
  아니다.

### Motion Inspector

- Leader·policy target, 최종 trajectory, controller reference·feedback, DXL position과
  current를 같은 시간축에서 비교한다.
- 선택 구간을 MCAP으로 기록하고 JSON, CSV와 Markdown 보고서를 만든다.
- `/robot_description`에서 runtime URDF와 ros2_control limit을 읽는다. 이 값은 EEPROM
  read-back이 아니다.
- robot command publisher, `ros2 bag play`와 replay API는 포함하지 않는다.

### Docker Workflow

- 반복 사용하는 build·bring-up·controller 명령 별칭을 `docker/bash_aliases`에서
  관리한다.
- source를 mount한 개발 컨테이너에서도 새 이미지 빌드 없이 같은 별칭을 읽도록 한다.

## 2026-08-31 fresh-clone 검증

GitHub의 `review/f2-features`를 `/tmp`에 새로 clone해, 기존 작업 폴더의 untracked 파일이나
빌드 잔여물을 사용하지 않고 다음을 확인했다.

- fresh clone HEAD `abc5919b40fb5054e07766e76ff9328463875e48`, clean worktree
- Motion Inspector 단위 테스트 `10/10` 통과
- Python·shell·Compose 설정 검사 통과
- Motion Inspector 경로에 command publisher, `ros2 bag play`, replay API가 없음
- fresh clone의 `ffw_description`을 일회성 overlay로 빌드한 뒤 base·smooth Xacro 확장
- 두 Xacro 모두 오른쪽 `[-131875, 151875]`, 왼쪽 `[-151875, 131875]`로 해석
- 일회성 subscriber-only 컨테이너에서 현재 F2의 14개 팔 관절 수신
- controller·joint·DXL topic age 약 `0.002~0.004 s`, command source `NONE`
- `/`, `/static/app.js`, `/api/state`, `/api/history/arm_r_joint7` 응답 확인
- `/api/replay/start`는 `404`
- `e52cf80` 적용 후 별도 인스턴스의 `SIGTERM` 종료가 중복 ROS shutdown 오류 없이 완료

검증 과정에서 fresh clone은 로봇 명령을 발행하지 않았다. F2 bring-up과 controller는
사용자가 미리 실행한 기존 runtime을 그대로 읽었다.

## live 확인에서 다시 관찰된 J7 상태

2026-08-31 live snapshot에서 오른쪽 J7은 reference `1.8201 rad`, feedback 약
`1.5696 rad`였고, 추종 오차 약 `0.2505 rad`와 `HARD_LIMIT_SATURATION`이 표시됐다.
runtime limit은 오른쪽 `[-1.363942, 1.570796] rad`, 왼쪽
`[-1.570796, 1.363942] rad`로 해석됐다.

이는 Motion Inspector가 알려진 J7 계약 불일치를 실제 신호에서 검출한다는 증거다.
동시에 현재 active limit이 URDF·초기 자세 범위와 완전히 정렬된 것은 아니라는 뜻이다.
이 검토 결과만으로 J7 limit 확대나 powered 동작을 승인하면 안 된다.

## 검토할 때 권장하는 순서

1. 이 문서에서 기능과 커밋의 대응 관계를 확인한다.
2. [F2 Safety](./f2_safety.md)와 [J7 위치 제한 오류](./f2_j7_position_limit_error.md)에서
   active 값과 미승인 후보를 구분한다.
3. Intervention은 [명령 routing](./intervention_command_routing.md)과
   [handoff 절차](./intervention_handoff.md)를 함께 검토한다.
4. [Motion Inspector](./motion_inspector.md)가 읽는 신호와 진단 한계를 확인한다.
5. 기능별 채택 여부를 결정한 뒤 원본 저장소로 필요한 커밋만 옮긴다.

## 저장소에서 제외한 원자료

이전 Motion Inspector 화면 캡처 2개와 2026-08-26 J7 작업 로그는 현재 설명용 assets와
동일한 파일이 아니며, A/B 시험과 접촉 관찰을 포함한다. GitHub 검토 자료와 현재 상태를
혼동하지 않도록 AI Worker 저장소에서는 제외하고, Git과 연결되지 않은 agents 작업
공간에 원본 해시와 함께 보존했다.
