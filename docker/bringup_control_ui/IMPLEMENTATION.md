# AI Worker Bring-up Control UI 작업 내역

## 1. 작업 목적

AI Worker의 Follower/Leader bring-up, F2 bring-down, inference 종료 후 자세 복구를 로봇 내부 웹에서 일관되게 관리하도록 종합 제어 UI를 추가했다.

기존 방식은 각 launch를 terminal에서 개별적으로 실행하기 때문에 다음 위험이 있었다.

- inference 또는 Leader가 command를 발행하는 동안 Follower를 다시 bring-up하면 첫 command가 크게 튈 수 있음
- `f2_bring_down.launch.py`가 기존 controller와 `/joint_states`에 의존하지만 준비 상태를 운영자가 직접 확인해야 함
- Leader가 살아 있는 상태에서 bring-down Action Goal과 Leader command가 충돌할 수 있음
- inference 이상 동작을 정지한 뒤 팔이 테이블 위에 남아도 init pose로 안전하게 복귀시키는 전용 절차가 없음
- 물리 E-stop, DXL 통신 오류, controller 실패와 launch 로그를 한 화면에서 확인하기 어려움
- s6 밖에서 수동 실행된 control stack을 서비스 상태만으로는 구분할 수 없음

## 2. 구현 범위

### 2.1 웹 UI

`bringup_control_ui/static/`에 별도 프레임워크 없이 동작하는 반응형 UI를 추가했다.

Cyclo Intelligence의 좌측 고정 메뉴, 밝은 배경, 흰색 panel 체계를 참고해 Home, Teleop, Inference, System의 네 화면으로 구성했다.

주요 화면 구성:

- Home에서 Follower/Leader 모델 선택
- Home의 단일 Robot Lifecycle 블록에서 Bring-up, Bring-down, Init pose와 Control 종료
- Teleop 화면에서 Leader ON/OFF 및 Teleop 종료 후 Init pose 복구
- Inference 화면에서 Cyclo runtime 상태 및 Inference 종료 후 Init pose 복구
- Follower, Leader/teleop, inference, DXL/feedback, E-stop 상태 요약
- 비활성 제어 버튼 클릭 시 안전 interlock 차단 사유 알림
- 실행 중 pose 작업과 실시간 출력
- launch 로그 원인 진단과 원본 로그 drawer
- E-stop/critical fault 적색 점멸 banner
- 작업 이력 audit trail

서버가 모든 브라우저에 공유하는 live state에서 동작 모드를 결정한다.

- Leader runtime 감지: `TELEOP MODE`, 파란색 화면 wash와 live indicator
- Inference service/RTC 감지: `INFERENCE MODE`, 보라색 화면 wash와 live indicator
- Leader와 Inference 동시 감지: `MODE CONFLICT`, 적색 critical banner
- 둘 다 분리: `STANDBY`

모드가 `STANDBY → TELEOP/INFERENCE`로 전환되면 모든 브라우저가 해당 모드 화면으로 한 번 자동 이동한다. 이후 사용자가 System/Home으로 이동한 것은 유지하고, 상태 SSE가 갱신될 때마다 강제로 되돌리지는 않는다.

Cyclo의 `STOP INFERENCE`는 orchestrator에서 `PAUSED`를 발행하고 policy service를 유지한다. 따라서 Stop만으로 복구를 허용하지 않으며, Clear/UNLOAD 후 service/RTC publisher가 분리돼야 기존 `recover_init` backend gate를 통과한다. `interfaces/InferenceStatus` type support가 AI Worker에 설치되지 않은 환경에서는 세부 phase 대신 `CONNECTED`로 표시하되 동일하게 fail-closed한다.

현재 실제 launch가 존재하는 모델만 노출한다.

| 역할 | UI 모델 | 실행 launch |
|---|---|---|
| Follower | SG2 | `ffw_sg2_follower_ai.launch.py` |
| Follower | BG2 | `ffw_bg2_follower_ai.launch.py` |
| Follower | SH5 | `ffw_sh5_follower_ai.launch.py` |
| Follower | BH5 | `ffw_bh5_follower_ai.launch.py` |
| Follower | F2 | `f2_follower_ai.launch.py` |
| Follower | F1 | `f1_follower_ai.launch.py` |
| Leader | A3 (기본) | `ffw_a3_ai.launch.py` |
| Leader | A2 | `ffw_a2_ai.launch.py` |
| Leader | LG2 | `ffw_lg2_leader_ai.launch.py` |

`SG5`라는 launch는 현재 저장소에 없으므로 임의 매핑하지 않았다.

### 2.2 ROS 상태 감시 백엔드

`server.py`에 FastAPI와 ROS 2 monitor node를 결합했다.

감시 대상:

- `/joint_states`
- `/leader/joint_states`
- arm/head/lift controller state
- Leader trajectory command 토픽 4개
- `/ffw_follower/dxl_state`
- `/lerobot/inference_command`, `/groot/inference_command` service
- `/lerobot/rtc_status`, `/groot/rtc_status` publisher
- `/task/inference_status` — `interfaces` type support가 설치된 경우
- `AI_WORKER_ESTOP_TOPIC`으로 지정한 `std_msgs/msg/Bool` 토픽

단순히 토픽 이름이 graph에 존재하는지만 보지 않고 다음 정보를 함께 사용한다.

- 실제 publisher 수
- 마지막 message 수신 시각
- s6 서비스 상태
- controller state의 freshness
- DXL `comm_state`
- DXL `dxl_hw_state`

### 2.3 외부 실행 프로세스 탐지

실환경 검증 중 `ai_worker_bringup` s6 서비스는 DOWN이지만 controller와 DXL publisher가 실제로 실행 중인 경우가 발견됐다.

이를 반영해 상태를 다음과 같이 분리했다.

- `managed`: UI가 s6를 통해 시작한 프로세스
- `runtime_active`: ROS publisher를 통해 실제로 감지된 실행 상태
- `EXTERNAL`: runtime은 살아 있지만 UI/s6가 소유하지 않는 상태

EXTERNAL 상태에서는:

- 중복 Follower/Leader 시작을 차단한다.
- UI에서 s6 종료를 시도하지 않는다.
- 자세 Action에 필요한 controller 상태로는 인정한다.
- 시작한 terminal 또는 외부 service에서 종료하도록 안내한다.

### 2.4 안전 게이트

모든 물리 동작은 UI button 상태뿐 아니라 POST API에서 동일한 조건을 다시 검사한다. 브라우저 조작이나 직접 API 호출로 UI 차단을 우회할 수 없다.

#### Follower Bring-up

다음 조건에서 차단한다.

- 물리 제어 기능 잠김
- E-stop 활성 또는 설정된 E-stop 신호가 최신이 아님
- critical hardware/communication fault
- Follower가 이미 실행 중이거나 EXTERNAL로 감지됨
- inference command service 또는 RTC runtime 존재
- Leader가 실행 중
- Leader/trajectory command publisher가 연결되어 있거나 최근 command가 수신됨

#### Leader 시작

다음 조건에서 차단한다.

- inference service/runtime 존재
- 다른 command stream 존재
- E-stop 또는 critical fault
- Leader가 이미 실행 중
- Follower가 실행되지 않음
- Follower arm/head/lift controller가 준비되지 않음
- Follower `/joint_states`가 최신이 아님
- Follower DXL 통신/hardware 상태가 정상이 아님

Follower가 켜져 있으면 Leader 시작 즉시 Follower가 움직일 수 있다는 추가 경고를 확인 모달에 표시한다. 별도 확인 문자열 입력은 요구하지 않는다.

동일 조건을 우회해 s6 service를 직접 시작하는 경우에도 `avatar_bringup` 공통 preflight가 Follower topic publisher 6개를 확인한다.

2026-08-12 현장 사고 로그에서는 Follower가 이미 정상 실행 중이었으나 A3 시작 약 2초 후 오른쪽 gripper 초기값 `-3.52`가 threshold `-0.50` 아래로 판정되어 `enable=3`과 `Start interpolation to save pose 3`가 자동 실행됐다. 같은 패턴이 여러 A3 시작 로그에서 반복 확인됐다. 즉 해당 사고는 Leader-first가 직접 원인이라기보다 gripper trigger의 startup edge 초기화 문제로 판단했다. 당시 UI/s6 A3 명령에서 shortcut을 차단했다. 2026-08-21에는 첫 유효 gripper sample을 무명령 baseline으로만 저장하고, 이후 above-to-below crossing만 인정하는 gate와 회귀 테스트를 추가한 뒤 `launch_gripper_trigger:=true`로 복원했다. 시작 시 이미 눌린 gripper는 한 번 풀었다가 다시 눌러야 동작한다.

#### Follower 정상 종료

F2의 권장 순서는 `Leader/Inference OFF → Bring down → Follower control 종료`다. Follower 실행 중 비활성 Bring-up을 누르면 이 재시작 순서를 알림으로 안내한다. 단, Bring-down이 불가능한 fault 상황에서 제어 종료까지 막는 것은 위험하므로 backend의 Follower stop을 Bring-down 성공 여부로 차단하지 않는다.

#### Bring-down

다음 조건이 모두 충족돼야 한다.

- 선택 Follower가 F2
- Follower control stack 실행 중
- arm/head/lift controller state 최신
- `/joint_states` 최신
- DXL 상태 정상
- Leader 종료
- Leader command publisher 제거 및 command quiet
- inference STOP/UNLOAD 및 policy container 종료
- E-stop 해제

Bring-down launch 내부에서 Leader를 자동 종료하지 않는다. Leader OFF는 별도의 명시적 사용자 작업으로 유지했다.

#### Init pose 복구

Bring-down 조건에 더해 현장 작업 공간 확인 checkbox를 요구한다.

복구 runner에는 pose 숫자를 별도로 하드코딩하지 않는다. UI가 선택 모델을 서버에 전달하고, 서버는 실행 중인 Follower 모델과 요청 모델의 일치를 확인한 뒤 해당 모델의 initial-position YAML 경로를 runner에 명시적으로 전달한다.

현재 지원 profile:

```text
F2
  → ffw_bringup/config/ffw_f2_follower/
    ffw_f2_follower_initial_positions.yaml
```

SG2는 복구 profile이 아직 없어 복구 기능을 노출하지 않는다.

### 2.5 F2 동기화 복구 coordinator

기존 `joint_trajectory_executor` 4개는 각자 step0에 도착하면 다른 그룹을 기다리지 않고 다음 step으로 진행한다.

새 `recovery_config.py`와 `recovery_runner.py`는 다음과 같은 전역 barrier를 제공한다.

1. 선택 모델과 YAML 경로 검증
2. 반복된 `/**:` ROS parameter section을 손실 없이 읽고 arm/head/lift 설정 검증
3. `/joint_states`, DXL 상태, YAML에 지정된 Action Server 4개 확인
4. 각 YAML phase의 목표를 네 controller에 동시에 전송
5. 모든 Action result 성공 및 YAML tolerance 이내의 실제 정지 확인
6. 전 그룹이 도달한 뒤 다음 YAML phase 실행

joint 이름, step 이름/position, duration, position/velocity tolerance와 Action topic은 모두 YAML을 단일 원본으로 사용한다. Trajectory는 phase마다 100 point quintic interpolation으로 생성한다. 그룹의 YAML 단계 수가 더 적으면 마지막 단계를 유지한다. 따라서 현재 F2 YAML에서 헤드는 `home`으로 이동한 뒤 다음 phase에도 그 자세를 유지하고, 양팔과 리프트는 `step0 → step1`을 수행한다. Swerve initial-position controller는 실행 중 복구용 controller가 아니므로 복구 대상에서 제외한다.

작업 도중 다음 상태가 발생하면 자동 중단한다.

- E-stop `true`
- DXL 통신 오류
- DXL hardware error
- DXL 상태 수신 단절
- `/joint_states` 수신 단절
- 외부 Leader/inference trajectory command 재개
- 전체 pose 작업 60초 timeout
- Action Goal 거부 또는 controller failure

이 coordinator는 다음 동작을 의도적으로 하지 않는다.

- `ros2_control_node` 자동 시작
- Action Server 자동 생성
- torque 자동 활성화
- E-stop 자동 해제
- collision-free 경로 보장

### 2.6 E-stop 처리

현재 저장소에는 모든 로봇이 공통으로 사용하는 물리 E-stop ROS 토픽이 없다.

따라서 환경 변수로 실제 `std_msgs/msg/Bool` 토픽을 연결하도록 구현했다.

```bash
AI_WORKER_ESTOP_TOPIC=/safety/e_stop
```

동작:

- `true`: 전체 화면 적색 점멸, pose 작업 중단, 모든 시작/이동 차단
- 최신 `false`: E-stop 해제 신호로 표시
- configured 상태에서 message가 없거나 오래됨: 시작/이동 차단
- 미설정: UI에 직접 신호 미설정 경고 표시
- Follower 실행 중 feedback/DXL 단절: 별도 critical fault로 적색 표시

E-stop을 UI에서 해제하거나 torque를 다시 켜지는 않는다.

### 2.7 Launch 로그 진단

s6 로그와 일회성 pose 작업 출력을 읽어 다음 유형을 운영자용 문구로 분류한다.

- inference 안전 차단
- `/dev/follower`, `/dev/leader` 포트 연결 실패
- DXL 통신/hardware error
- controller/spawner 실패
- Action Server timeout
- camera/Realsense/ZED 문제
- LiDAR/Lakibeam 문제
- ROS package/executable 누락
- 웹 포트 중복

진단 카드에는 다음을 함께 표시한다.

- severity
- 이해하기 쉬운 제목
- 권장 조치
- 실제 로그 evidence

원본 로그도 별도 drawer에서 확인할 수 있다.

### 2.8 인증과 네트워크

일반 운영자는 AI Worker 컨테이너 안에서 다음 명령을 실행한다.

```bash
ai_worker_control
```

이 명령이 서버 실행과 공용 물리 제어 설정을 함께 적용하므로 운영자가 환경 변수나 token을 직접 다루지 않는다. 명령을 실행하지 않았거나 `Ctrl+C`로 종료한 상태에서는 웹 제어 서버 자체가 열리지 않는다.

단일 서버의 `ControlCoordinator`가 모든 브라우저 요청을 직렬화하고, 처리 중 상태를 SSE snapshot에 포함한다. 따라서 첫 요청이 `STARTING/STOPPING/MOVING/RECOVERING`으로 전환되는 동안 다른 요청은 HTTP `409`로 차단된다.

진행 중인 일회성 pose 작업의 cancel은 fail-safe 목적으로 물리 제어 잠금과 무관하게 허용한다. Token이 설정된 배포에서는 cancel에도 같은 인증을 적용한다.

### 2.9 s6 및 Docker 통합

추가 사항:

- `bringup_control_ui` s6 longrun 정의 추가
- `ai_worker_control [start|stop|status]` 운영 명령 추가
- UI는 컨테이너 시작 시 자동 실행하지 않고 운영 명령으로 전면 실행
- Follower/Leader hardware service는 계속 수동 시작
- Follower s6 모델에 F2/F1 추가
- `/run/leader_type` 기반 Leader 선택 구조 추가
- Follower와 Leader 시작 스크립트 모두 inference service/RTC/command publisher 사전 검사
- Follower 시작 시 `/leader/joint_states` publisher 추가 검사
- Compose 환경 변수와 기본 포트 `8780` 추가

## 3. 생성 및 변경 파일

### 신규

- `docker/bringup_control_ui/__init__.py`
- `docker/bringup_control_ui/logic.py`
- `docker/bringup_control_ui/server.py`
- `docker/bringup_control_ui/recovery_config.py`
- `docker/bringup_control_ui/recovery_runner.py`
- `docker/bringup_control_ui/ai_worker_control`
- `docker/bringup_control_ui/run.sh`
- `docker/bringup_control_ui/static/index.html`
- `docker/bringup_control_ui/static/app.js`
- `docker/bringup_control_ui/static/styles.css`
- `docker/bringup_control_ui/test_logic.py`
- `docker/bringup_control_ui/README.md`
- `docker/bringup_control_ui/IMPLEMENTATION.md`
- `docker/bringup_control_ui/DEVELOPMENT_LOG.md`
- `docker/bringup_control_ui/USER_GUIDE.md`
- `docker/s6-services/bringup_control_ui/run`
- `docker/s6-services/bringup_control_ui/type`

### 변경

- `docker/docker-compose.yml`
- `docker/Dockerfile.arm64`
- `docker/Dockerfile.amd64`
- `docker/s6-services/ai_worker_bringup/run`
- `docker/s6-services/avatar_bringup/run`
- `docker/s6-services/common/ros2_service_run.sh`

작업 전부터 존재한 `ffw_joint_trajectory_command_broadcaster` 변경과 `task_motion_inspector` 파일은 수정하거나 되돌리지 않았다.

## 4. 검증 결과

### 자동 검사

- 안전/복구 설정/운영 모드 unit test 18개 통과
- Python AST/compile 검사 통과
- JavaScript syntax 검사 통과
- shell `bash -n` 검사 통과
- `docker compose config --quiet` 통과
- `git diff --check` 통과
- F2 YAML의 반복 `/**:` section 5개 중 복구 대상 4개 파싱 확인
- F2 arm `step0 → step1`, head `home`, lift `step0 → step1` plan 검사 확인
- `--check-config`로 ROS Goal 전송 없이 컨테이너 내 YAML 설정 검사 통과

### 실행 환경 검사

실행 중인 `ai_worker` 컨테이너에서 임시 read-only 포트로 UI를 기동했다.

확인 항목:

- `/` 응답 200
- `/static/app.js` 응답 200
- `/static/styles.css` 응답 200
- `/api/state` 응답 200
- `/api/logs` 응답 200
- s6 DOWN이지만 실제 F2 publisher가 살아 있는 상태를 `EXTERNAL`로 탐지
- EXTERNAL Follower 중복 시작 차단
- EXTERNAL 프로세스에 대한 s6 종료 시도 차단
- 물리 제어 잠금 상태에서 start POST 거부
- RecoveryNode와 Action Client 4개 무동작 생성 확인
- 실제 F2 Bring-down Action 완료 및 exit code 0 확인
- Bring-down 후 Follower control 정상 종료 확인
- F2 recovery phase별 Action Goal 수락과 controller 완료 확인
- A3 시작 시 반복되던 오른쪽 gripper startup trigger 로그 재현 및 원인 식별

검증용 임시 웹 프로세스는 검사 후 종료했다.

## 5. 현장 검증 범위

실제 F2에서 Bring-down, Follower 종료와 Init pose recovery trajectory의 controller 수행을 확인했다. A3 Leader 시작 사고는 log timeline으로 startup gripper trigger까지 추적하고 UI/s6 시작 경로에서 차단했다.

다음 항목은 아직 수행하지 않았다.

- 실제 E-stop Bool 신호 연결 시험
- pose 작업 중 물리 E-stop cancel 시험
- F2 recovery의 전체 UI 결과 `DONE`과 최종 measured tolerance 반복 확인
- A3 trigger 차단 적용 후 저속 Leader 재시험
- A2/LG2 전체 물리 시험
- SG2 recovery 시험

## 6. 배포 전 남은 현장 작업

1. 실제 물리 E-stop Bool 토픽 이름 확인
2. 토픽이 없다면 hardware/safety bridge에서 ROS heartbeat 제공
3. F2 recovery의 `현재 자세 → step0` 경로를 실제 작업 공간에서 저속 검증
4. 검증된 recovery 가능 joint/workspace 범위 정의
5. A3 trigger 차단 적용 후 E-stop을 준비한 저속 Leader 시험
6. A2/LG2 모델별 시작/종료 시험
7. 유지보수 시간에 image rebuild가 필요한 배포 경로 확인
8. mock 또는 모터 분리 환경에서 fault별 버튼 시험
9. 현장 E-stop을 손에 둔 상태로 cancel 시험
10. F2 recovery 완료 후 UI `DONE`, 최종 joint feedback과 tolerance를 반복 확인

## 7. 설계상 제약

- F2 recovery는 collision planner가 아니라 검증된 waypoint 기반 joint-space trajectory다.
- 물리 E-stop 신호가 ROS로 제공되지 않으면 UI가 버튼 상태를 직접 알 수 없다.
- `interfaces` message type이 AI Worker 이미지에 없으면 inference phase 상세값 대신 service/RTC publisher/command stream 연결 상태로 보수적으로 판정한다.
- UI 밖에서 시작된 프로세스는 감지할 수 있지만 안전하게 종료할 소유권은 없다.
- Bring-down은 기존 `f2_bring_down.launch.py`와 executor 구현을 사용하므로 전역 step barrier는 init recovery에만 적용된다.
- A3 gripper save-pose shortcut은 첫 유효 sample 무명령 baseline gate를 거쳐 UI/s6 운영 경로에서 다시 활성화됐다. 시작 시 이미 threshold 아래인 입력은 명령을 만들지 않으며, release 후 fresh press에서만 save pose 3을 발행한다.
- 개발 과정과 현장 로그 근거는 `DEVELOPMENT_LOG.md`에 기록한다.

## 8. Inference Intervention

Cyclo inference 화면에 사람의 실패 교정 데이터를 안전하게 취득하는 intervention 흐름을 추가했다. 정책 컨테이너는 종료하지 않고 모델을 메모리에 유지한다.

```text
INFERENCING
  → STOP_INFERENCE / PAUSED 확인
  → 실제 trajectory command quiet 확인
  → A3 Leader 시작 및 joint feedback 확인
  → START_INFERENCE_RECORD
  → 사람 교정 조작
  → 저장 또는 폐기
  → Leader 종료 및 command quiet 확인
  → RESUME_INFERENCE
```

AI Worker가 intervention lease와 단계를 보관하므로 여러 브라우저에서 동일한 상태를 본다. Cyclo supervisor의 `/api/intervention/*`는 host-network의 AI Worker `:8780` API를 프록시하며, AI Worker token이 설정된 경우 브라우저에 노출하지 않고 서버 환경 변수로 전달한다.

Cyclo Inference 설정에는 Intervention 전용 Task Num/Name을 추가했다. 선택값은 기존 `TaskInfo.tags`에 URL encoding하여 ROS 인터페이스 변경 없이 브라우저와 orchestrator 사이에 동기화한다. `START_INFERENCE_RECORD`에서만 이를 `task_num/task_name`으로 승격하며, cyclo_data는 명시적 identity가 있으면 timestamp 강제 치환을 하지 않는다. 같은 identity는 기존 DataManager를 재사용해 episode를 누적하고, 선택값이 모두 비어 있으면 기존 timestamp 기반 inference dataset 동작을 유지한다.

중간 오류, E-stop, DXL/feedback 단절 또는 PAUSED 상태 이탈이 발생하면 watchdog은 Leader를 종료하고 `PAUSED_SAFE`로 전환한다. 이 경로에서는 inference를 자동 재개하지 않는다.

Intervention 시작 gate는 현재 검증 대상인 F2+A3 조합, Real Robot inference의 `PAUSED`, 정상 Follower/controller/feedback/DXL, E-stop 해제, 최근 policy trajectory command 없음과 Leader OFF를 모두 요구한다. Warm pause 동안 policy publisher 연결은 남을 수 있으므로 publisher 존재가 아니라 실제 최근 command 수신 여부로 hand-off를 판정한다.

일반 `avatar_bringup` s6 preflight는 inference service, RTC publisher 또는 trajectory publisher 연결이 하나라도 남아 있으면 계속 종료 코드 64로 차단한다. Intervention에서만 AI Worker가 위 gate를 통과한 직후 `/run/ai_worker_intervention_preflight`에 20초짜리 일회용 hand-off lease를 만든다. s6는 version, 만료 시각과 선택 Leader 모델을 확인하고 파일을 즉시 소비한다. 이 lease는 PAUSED policy의 연결만 허용하며 Follower-first 검사는 우회하지 않는다. 따라서 UI의 일반 Leader ON이나 s6 직접 실행은 기존의 엄격한 차단을 그대로 적용받는다.

### 8.1 Cyclo ROS interface 연결

AI Worker Compose는 Cyclo의 `interfaces` 소스를 `/opt/cyclo_interfaces_src`에 읽기 전용으로 마운트한다. `bringup_control_ui/run.sh`는 이를 `/var/lib/ai_worker_cyclo_interfaces` 전용 overlay에 빌드하고 source한다.

- Cyclo의 IDL이 단일 원본이다.
- AI Worker controller의 `/root/ros2_ws/install`은 수정하지 않는다.
- 최초 실행 또는 IDL 변경 시에만 overlay를 다시 빌드한다.
- 공식 `interfaces/msg/InferenceStatus`, `RecordingStatus`, `RtcStatus` type support를 사용한다.
- source 또는 빌드가 없으면 intervention은 `PAUSED`를 확인할 수 없어 fail-closed된다.
