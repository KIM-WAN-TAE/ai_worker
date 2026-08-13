# AI Worker Bring-up Control 개발 로그

## 문서 정보

- 대상: `docker/bringup_control_ui` 및 연동 s6 service
- 기록 기준일: 2026-08-12
- 목적: 구현 순서, 설계 결정, 현장 장애와 검증 근거를 후속 개발자가 재현할 수 있도록 기록
- 운영 절차: [USER_GUIDE.md](USER_GUIDE.md)
- 최종 구현 구조: [IMPLEMENTATION.md](IMPLEMENTATION.md)

## 1. 시작 배경

기존 `f2_bring_down.launch.py`는 독립적인 hardware bring-up이 아니다. 이미 실행 중인 F2 stack의 다음 자원을 사용한다.

- arm/head/lift `FollowJointTrajectory` Action Server
- 최신 `/joint_states`
- 활성 trajectory controller
- 정상 DXL 통신과 torque 상태

따라서 Follower가 완전히 꺼진 뒤에는 Bring-down을 실행할 수 없다. Bring-down launch 안에서 Action Server와 hardware stack을 다시 시작하는 방식도 검토했지만 채택하지 않았다. E-stop이나 fault 이후에 UI가 임의로 torque와 controller를 재가동하면 사고 원인을 확인하지 않은 채 로봇을 움직일 수 있기 때문이다.

최종 원칙은 다음과 같다.

```text
정상 종료
  Leader/Inference OFF
    → Bring down
    → Follower control 종료

E-stop/fault
  움직임보다 정지와 원인 확인 우선
    → 자동 Action Server/torque 재가동 금지
    → 정상 Bring-up 절차로 복귀
```

## 2. 1차 구현: 통합 제어 서버

FastAPI와 ROS 2 monitor node를 한 프로세스에 구성하고 다음 기능을 추가했다.

- Follower 모델 선택 및 s6 Bring-up/종료
- Leader 모델 선택 및 s6 ON/OFF
- F2 Bring-down
- F2 Init pose 복구
- ROS graph, publisher 수와 message freshness 감시
- DXL `comm_state`, hardware error 감시
- inference service와 RTC publisher 감시
- launch/pose 작업 로그 수집과 운영자용 오류 분류

초기 노출 모델:

- Follower: SG2, BG2, SH5, BH5, F2, F1
- Leader: A3, A2, LG2

SG5는 실제 launch가 없어 임의 매핑하지 않았다. SG2 Init pose 복구도 해당 profile이 아직 구현되지 않아 비활성 상태로 유지했다.

## 3. 안전 게이트

버튼의 disabled 상태만 신뢰하지 않고 모든 POST 요청에서 동일한 조건을 다시 검사하도록 구성했다.

주요 차단 조건:

- E-stop 활성 또는 설정된 E-stop heartbeat 단절
- DXL 통신/hardware fault
- 다른 pose/control 요청 처리 중
- inference service 또는 RTC runtime 존재
- Leader/외부 trajectory command publisher 존재
- controller 또는 `/joint_states` freshness 부족
- UI 밖에서 실행된 EXTERNAL process에 대한 중복 시작/잘못된 종료

Bring-down은 Leader를 자동으로 종료하지 않는다. command source를 자동 종료하면 누가 어떤 제어권을 해제했는지 불명확해지므로, 운영자가 Leader OFF를 명시적으로 수행하고 command quiet를 확인하도록 유지했다.

## 4. 다중 사용자 동기화

여러 노트북이 동시에 접속할 수 있도록 browser-local 상태 대신 서버의 실제 ROS/s6 상태를 단일 기준으로 사용했다.

- SSE로 0.5초마다 공통 snapshot 전달
- 첫 요청 승인 즉시 `STARTING`, `STOPPING`, `MOVING`, `RECOVERING` 전파
- 서버 `ControlCoordinator`가 요청을 직렬화
- 처리 중 충돌 요청은 HTTP `409`로 거절
- 모델, 실행 주체, 완료 결과를 모든 화면에 동일하게 표시

확인 문자열을 타이핑하는 방식은 제거하고 일반 확인 modal과 현장 작업 공간 checkbox만 유지했다.

브라우저 `fetch` header에 한글 client 이름을 사용했을 때 발생한 `ByteString` 변환 오류는 ASCII client ID를 생성해 `X-Control-Client`에 넣는 방식으로 해결했다.

## 5. 운영 명령과 Docker 통합

초기 물리 제어 환경 변수와 token 설정이 일반 운영에 혼란을 줘 다음 명령으로 통합했다.

```bash
ai_worker_control
ai_worker_control status
ai_worker_control stop
```

`ai_worker_control`은 컨테이너 안에서 UI를 공용 물리 제어 모드로 실행한다. 컨테이너가 재시작되면 운영자가 다시 명령을 실행해야 한다.

종료 과정에서 uvicorn의 ROS executor shutdown이 지연되는 사례가 있어 다음 보호를 추가했다.

- PID 파일의 프로세스 command가 실제 `uvicorn server:app`인지 확인
- 정상 종료 제한 시간 적용
- 검증된 UI 프로세스에만 제한 시간 이후 강제 종료 적용
- Follower/Leader process group은 UI 종료 대상에 포함하지 않음

## 6. F2 Init pose 복구

### 6.1 YAML 단일 원본

초기 recovery runner에는 F2 joint, pose와 Action topic이 Python에 중복 하드코딩돼 있었다. 이를 제거하고 다음 YAML을 단일 원본으로 변경했다.

```text
ffw_bringup/config/ffw_f2_follower/
ffw_f2_follower_initial_positions.yaml
```

YAML에서 읽는 항목:

- joint 이름과 순서
- `step_names`와 각 position
- Action topic
- duration
- position/velocity tolerance

ROS parameter YAML에 동일한 `/**:` 최상위 key가 반복되므로 일반 `yaml.safe_load()`만 사용하면 앞 section이 손실된다. loader가 각 `/**:` section을 임시 고유 key로 정규화한 뒤 네 executor를 검증하도록 구현했다.

현재 F2 plan:

| 그룹 | YAML 단계 | 시간 |
|---|---|---:|
| 왼팔 | `step0 → step1` | 단계당 5초 |
| 오른팔 | `step0 → step1` | 단계당 5초 |
| 헤드 | `home` 유지 | 단계당 5초 |
| 리프트 | `step0 → step1` | 단계당 10초 |

Swerve initial-position controller는 실행 중 recovery controller가 아니므로 대상에서 제외했다.

### 6.2 전역 barrier

각 executor가 독립적으로 다음 step으로 넘어가지 않도록 recovery coordinator가 phase barrier를 제공한다.

1. YAML과 모델 확인
2. DXL, joint feedback, Action Server 4개 preflight
3. 네 그룹의 현재 measured position 취득
4. 동일 phase trajectory 동시 전송
5. 모든 Action result와 measured stop 확인
6. 전 그룹 완료 후 다음 phase 실행

### 6.3 첫 실행 실패와 수정

첫 현장 실행은 ROS 2 `Node.clients` 읽기 전용 property와 runner의 `self.clients` 이름이 충돌해 Action Goal 전송 전에 실패했다.

```text
AttributeError: property 'clients' of 'RecoveryNode' object has no setter
```

저장 이름을 `self.action_clients`로 변경했다. 이후 RecoveryNode와 Action Client 4개의 무동작 생성 검사를 통과했고, 현장 로그에서 양팔·헤드·리프트의 두 phase Action Goal 수락과 controller 완료를 확인했다.

## 7. UI 개편

Cyclo Intelligence의 정보 구조를 참고해 단일 긴 화면을 다음 좌측 메뉴 구조로 개편했다.

- Home: Robot type, Leader type, Robot lifecycle
- Teleop: Leader ON/OFF와 Teleop 종료 후 Init pose
- Inference: Cyclo runtime과 Inference 종료 후 Init pose
- System: safety interlock, 진단, 이벤트와 원본 로그

Home의 Robot Lifecycle에는 Bring up, Bring down, Init pose를 한 블록에 배치했다.

공유 ROS 상태에서 mode를 결정한다.

- Leader 감지: 파란색 `TELEOP MODE`
- inference service/RTC 감지: 보라색 `INFERENCE MODE`
- 둘 다 감지: 적색 `MODE CONFLICT`
- 모두 분리: `STANDBY`

새 모드가 감지되면 모든 browser가 해당 화면으로 한 번 자동 이동한다. 사용자가 이후 다른 메뉴로 이동한 선택은 다음 mode 변화 전까지 유지한다.

## 8. Cyclo Inference 연동 판단

Cyclo orchestrator 코드를 확인한 결과 UI의 Stop은 session 완전 종료가 아니다.

```text
Stop  → PAUSED
Clear → STOP + UNLOAD + client disconnect
```

따라서 `PAUSED`도 Inference mode로 표시하며 Init pose를 허용하지 않는다. 다음 항목이 모두 사라져야 복구 버튼이 활성화된다.

- `/lerobot/inference_command` 또는 `/groot/inference_command`
- `/lerobot/rtc_status` 또는 `/groot/rtc_status` publisher
- active/recent trajectory command

AI Worker에 `interfaces/InferenceStatus` type support가 없으면 `PAUSED` 같은 세부 phase 대신 `CONNECTED`로 표시하지만, service/RTC 존재 여부로 동일하게 fail-closed한다.

## 9. Follower 정상 종료 UX

Follower가 실행 중이면 Bring up이 비활성화되는 이유와 다음 순서가 눈에 띄지 않는 문제가 있었다.

Home에 상태 기반 안내를 추가했다.

```text
F2 실행 중
  → Leader/Inference OFF
  → Bring down
  → Follower control 종료
  → Bring up 재활성화
```

Bring-down 성공이 확인되면 안내가 `이제 Follower control 종료`로 변경된다.

Follower stop을 Bring-down 성공 여부로 강제 차단하지는 않았다. hardware fault나 비정상 자세에서 Bring-down trajectory 자체를 실행할 수 없는 경우에도 control stack을 정지할 수 있어야 하기 때문이다. 정상 운영에서는 안내된 수순을 따르고, 이 예외는 fault 탈출 경로로만 사용한다.

## 10. A3 Leader 현장 사고와 대응

### 10.1 현상

2026-08-12 A3 Leader를 켠 직후 F2 오른팔이 갑자기 올라가 테이블을 건드리는 사고가 발생했다.

### 10.2 로그 근거

사고 당시 Follower는 이미 실행 중이었고 controller, joint feedback과 DXL도 정상 상태였다. 따라서 단순한 Leader-first가 직접 원인은 아니었다.

A3 시작 약 2초 뒤 다음 로그가 발생했다.

```text
[right] Gripper crossed threshold (-3.52 < -0.50), published enable=3
[right] Start interpolation to save pose 3
```

이 패턴은 이전 A3 시작 로그에서도 오른쪽 gripper 값 약 `-3.52~-3.53`으로 반복 확인됐다. `gripper_trigger`의 초기 `below_flag=false` 상태에서 이미 닫혀 있는 gripper의 첫 sample을 새로운 falling edge로 해석한 것이 원인이다.

### 10.3 조치

- UI/s6가 시작하는 A3 명령에 `launch_gripper_trigger:=false` 추가
- gripper save-pose 3 shortcut 비활성화
- Foot switch, joystick, 일반 trajectory broadcaster 유지
- Init pose 복구는 Leader OFF와 command quiet 후 UI에서 명시적으로 실행

추가 방어로 Leader ON 조건도 강화했다.

- Follower가 먼저 실행돼야 함
- arm/head/lift controller가 모두 준비돼야 함
- `/joint_states`가 최신이어야 함
- DXL 통신과 hardware 상태가 정상이어야 함
- web backend와 s6 preflight에서 이중 검사

## 11. 현장 확인 내역

2026-08-12 기준 다음 사항을 확인했다.

- Follower/Leader ON/OFF와 UI 상태 동기화
- F2 Bring-down controller 완료 및 exit code 0
- Bring-down 후 Follower control 정상 종료
- RecoveryNode Action Client 4개 생성
- F2 YAML recovery phase의 Action Goal 수락과 controller 완료
- Leader OFF 후 command publisher 제거와 Init pose gate 통과
- A3 startup gripper trigger의 반복 재현 로그와 원인 식별
- 임시 조회 전용 서버의 `/`, static file, `/api/state`, `/api/logs` 응답
- 다중 browser용 SSE shared state와 control serialization

실제 E-stop 신호 연동과 물리 E-stop 중 pose cancel은 아직 현장 시험이 필요하다.

## 12. 현재 검증 결과

- Python unit test 18개 통과
- Python compile 검사 통과
- JavaScript syntax 검사 통과
- HTML ID/DOM reference 검사 통과
- CSS brace 검사 통과
- shell `bash -n` 검사 통과
- `git diff --check` 통과
- F2 YAML 반복 section과 recovery plan 검사 통과
- RecoveryNode 무동작 생성 검사 통과

## 13. 남은 과제

1. 실제 공통 E-stop Bool/heartbeat 토픽 정의와 연결
2. E-stop 중 pose cancel 및 reset 이후 정상 시작 현장 시험
3. F2 recovery 가능한 joint/workspace envelope 정의
4. SG2 등 다른 모델의 recovery YAML/profile 구현
5. AI Worker image에 `interfaces` package를 포함해 inference 세부 phase 표시
6. A3 `gripper_trigger` 자체를 유지해야 한다면 첫 sample로 상태만 초기화하고 실제 threshold 재통과에만 edge를 발생시키도록 upstream 수정
7. A2/LG2를 포함한 모델별 Leader startup 저속 시험
8. F2 recovery 완료 후 전체 UI 결과 `DONE`, 최종 joint feedback과 measured tolerance 반복 확인

## 14. 변경하지 않은 범위

- E-stop 자동 해제
- torque 자동 재활성화
- Bring-down 내부 Leader 자동 종료
- collision planner 또는 collision-free 보장
- SG2 Init pose 복구
- UI 밖에서 시작된 EXTERNAL process의 강제 종료

## 15. 제어 화면 간소화

상시 노출되던 버튼별 차단 문구, 정상 종료 안내, launch 경로와 화면 소개 문장을 제거했다. Home, Teleop, Inference 화면에는 모델 선택, 핵심 상태와 제어 버튼만 남겼다.

안전 interlock 자체는 변경하지 않았다. 실행할 수 없는 버튼은 흐리게 표시하되 클릭할 수 있으며, 클릭하면 백엔드 gate가 제공한 실제 차단 사유를 알림 창으로 표시한다. 물리 제어 잠금, Leader/Inference 충돌, controller/feedback/DXL 미준비, 미지원 모델과 작업 공간 미확인도 같은 방식으로 안내한다.

E-stop/critical fault 배너, Home 상태 카드와 System의 상세 안전·로그 진단은 즉시 판단에 필요한 정보이므로 유지했다.

Inference 또는 Teleop runtime이 새로 감지되면 해당 mode 화면으로 이동하고, runtime이 완전히 분리되어 mode가 `STANDBY`로 바뀌면 Home으로 자동 복귀하도록 전환을 대칭으로 맞췄다. Cyclo Stop으로 `PAUSED` 상태만 된 경우에는 inference가 종료된 것이 아니므로 Inference 화면을 유지한다.

## 16. Cyclo Backend OFF 전역 종료

LeRobot 사용 후 GR00T으로 전환한 경우 현재 선택된 GR00T만 OFF되고 이전 `lerobot_server`가 남아 `/lerobot/inference_command`와 `/lerobot/rtc_status`를 계속 제공하는 현상을 확인했다. AI Worker는 이를 정상적으로 inference 가능 process로 판정해 mode와 Leader 안전 gate를 유지했다.

Cyclo의 Policy Backend OFF 동작을 전역 inference 종료 경계로 변경했다. 사용자가 어느 backend 화면에서 OFF를 누르더라도 LeRobot과 GR00T backend를 모두 종료한다. 따라서 현장 절차는 `Stop → Clear → OFF`만으로 끝나며 별도의 `docker stop`은 필요하지 않다. 현재 잔존 backend도 Cyclo supervisor API로 종료했고 AI Worker가 `STANDBY`, service 0개, RTC publisher 0개로 전환되는 것을 확인했다.
