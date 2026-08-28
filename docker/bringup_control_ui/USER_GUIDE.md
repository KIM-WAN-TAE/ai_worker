# AI Worker Bring-up Control UI 사용설명서

## 1. 문서 목적

이 문서는 현장 운영자가 AI Worker의 Follower, Leader, inference 연동 상태를 확인하고 Bring-up, Bring-down, F2 init pose 복구를 안전하게 수행하기 위한 절차를 설명한다.

개발 과정과 현장 장애 분석은 [DEVELOPMENT_LOG.md](DEVELOPMENT_LOG.md), 구현 상세는 [IMPLEMENTATION.md](IMPLEMENTATION.md)를 참고한다.

UI는 물리 E-stop을 대체하지 않는다. 로봇을 움직이는 동안에는 항상 E-stop에 즉시 접근할 수 있어야 한다.

## 2. 접속 주소

기본 주소:

```text
http://<로봇 PC IP>:8780
```

예시:

```text
http://192.168.0.20:8780
```

페이지가 열리지 않으면 다음을 확인한다.

1. `ai_worker` 컨테이너가 실행 중인지 확인한다.
2. 이미지가 UI 추가 이후 버전으로 rebuild됐는지 확인한다.
3. `BRINGUP_CONTROL_PORT`가 `8780`이 아닌 값으로 변경됐는지 확인한다.
4. 같은 포트를 사용하는 다른 프로세스가 있는지 확인한다.
5. 로봇 PC 방화벽과 접속 PC의 네트워크를 확인한다.

## 3. 시작 및 접속

### 3.1 Docker와 컨테이너 시작

AI Worker 컨테이너가 꺼져 있다면 호스트에서 시작하고 들어간다.

```bash
cd /home/robotis/ai_worker/docker
./container.sh start
./container.sh enter
```

이미 컨테이너가 실행 중이면 `./container.sh enter`만 실행한다.

### 3.2 Control UI 시작

컨테이너 안에서 다음 명령을 실행한다.

```bash
ai_worker_control
```

별도의 환경 변수 설정이나 token 입력은 필요하지 않다. 터미널에 표시되는 `http://로봇-IP:8780` 주소로 접속한 뒤 다음 상태가 실제 로봇과 일치하는지 확인한다.

- Follower ON/OFF
- Leader ON/OFF
- teleop 상태
- inference 연결 상태
- controller state
- `/joint_states`
- DXL 상태

### 3.3 종료와 다음 사용

- 현재 터미널에서 `Ctrl+C`: UI 종료
- 다른 터미널에서 `ai_worker_control stop`: UI 종료
- `ai_worker_control status`: 실행 상태 확인

Worker를 종료하면 UI도 종료된다. 다음에 Worker를 켠 뒤 컨테이너에서 `ai_worker_control`을 다시 실행한다.

## 4. 화면과 모드 읽는 법

왼쪽 메뉴는 다음과 같이 구성된다.

- `Home`: Robot type과 Leader type 선택, Bring-up/Bring-down/Init pose
- `Teleop`: Leader ON/OFF, command 상태, Teleop 종료 후 Init pose
- `Inference`: Cyclo inference 상태, Inference 종료 후 Init pose
- `System`: 안전 조건, 원인 진단, 최근 이벤트와 원본 로그

Leader가 연결되면 화면 배경과 Teleop 블록이 파란색으로 바뀌며 `TELEOP MODE`가 표시된다. Inference service 또는 RTC publisher가 연결되면 보라색으로 바뀌며 `INFERENCE MODE`가 표시된다. 두 runtime이 동시에 감지되면 적색 `MODE CONFLICT`로 표시되며 움직임 작업이 차단된다.

Leader 또는 Inference가 새로 감지되면 UI는 각각 Teleop 또는 Inference 화면으로 자동 이동한다. 여러 브라우저가 같은 backend 상태를 사용하므로 다른 노트북에서도 동일한 모드와 배경색이 표시된다.

### 4.1 상단 상태 표시

#### FOLLOWER

- `DOWN`: Follower control stack이 감지되지 않음
- `UP`: UI/s6가 시작한 Follower 실행 중
- `EXTERNAL`: terminal이나 다른 service에서 시작된 Follower가 감지됨

EXTERNAL 상태에서는 중복 Bring-up이 차단된다. UI의 Control 종료도 사용할 수 없으며, 원래 시작한 terminal 또는 service에서 종료해야 한다.

#### LEADER / TELEOP

- `OFF`: Leader publisher 미감지
- `ON · COMMAND QUIET`: Leader는 켜져 있지만 최근 command 없음
- `TELEOP ACTIVE`: Leader command 또는 실제 teleop activity 감지
- `EXTERNAL`: UI 밖에서 실행된 Leader 감지

#### INFERENCE

- `DISCONNECTED`: policy command service와 RTC runtime 미감지
- `READY`: orchestrator가 준비 상태
- `LOADING`: 모델 loading 중
- `INFERENCING`: inference 동작 중
- `PAUSED`: inference pause 상태
- `UNAVAILABLE · CONNECTED`: 상세 message type은 없지만 policy service/RTC publisher가 감지됨

`PAUSED` 또는 `READY`로 보이더라도 policy container가 살아 있으면 Bring-up과 pose recovery가 차단된다. 반드시 STOP, UNLOAD 후 container까지 종료한다.

Cyclo Intelligence의 **Stop**은 현재 inference를 `PAUSED`로 전환할 뿐 model/service를 제거하지 않는다. 따라서 Stop 직후 Inference 화면은 계속 활성 색상으로 표시되고 Init pose는 비활성 상태다. Cyclo에서 Clear/UNLOAD를 완료하고 service와 RTC publisher가 사라진 뒤 복구 버튼이 활성화되는지 확인한다.

Cyclo UI에서는 `Stop → Clear → Policy Backend OFF` 순서로 종료한다. Backend OFF는 LeRobot과 GR00T을 모두 종료하므로 이전에 선택했던 backend가 남아 AI Worker의 Inference Mode를 유지하지 않는다. 별도의 terminal 명령은 필요하지 않다.

#### DXL / FEEDBACK

- `HEALTHY`: 최신 `/joint_states`, controller state, DXL 정상
- `CHECK REQUIRED`: feedback 지연, DXL 상태 미확인 또는 오류
- `OFFLINE`: Follower가 실행되지 않은 상태

#### E-STOP SIGNAL

- `ACTIVE`: E-stop `true`
- `RELEASED`: 최신 `false` heartbeat 수신
- `WAITING`: 설정된 토픽에서 message 대기 또는 지연
- `NOT CONFIGURED`: 직접 E-stop ROS 토픽이 설정되지 않음

## 5. Follower Bring-up

### 5.1 실행 전 확인

1. 로봇 주변에 사람과 장애물이 없는지 확인한다.
2. 로봇이 해당 모델의 정상 시작 자세인지 확인한다.
3. 물리 E-stop 위치와 작동 가능 상태를 확인한다.
4. inference를 STOP하고 UNLOAD한다.
5. Groot/LeRobot policy container를 종료한다.
6. Leader를 종료한다.
7. 화면에서 `Trajectory source 분리`가 통과됐는지 확인한다.
8. 기존 terminal에서 실행한 Follower가 없는지 확인한다.

### 5.2 실행

1. Follower 모델을 선택한다.
2. 표시되는 launch 이름이 실제 로봇과 일치하는지 확인한다.
3. **BRING UP**을 누른다.
4. 확인 창에서 경고를 읽는다.
5. 실행 버튼을 누른다.
6. Follower 상태가 `UP`으로 바뀌는지 확인한다.
7. controller, joint feedback, DXL 항목이 정상으로 바뀔 때까지 기다린다.

### 5.3 실행 실패 시

오른쪽 **원인 진단**을 확인한다. 더 자세한 출력은 **전체 로그 → Follower**에서 확인한다.

다음 항목을 우선 확인한다.

- `/dev/follower` 존재 여부
- U2D2/USB 전원과 케이블
- inference service 또는 RTC publisher
- Leader publisher
- 기존 외부 Follower launch
- controller spawner 실패
- DXL Hardware Error Status

## 6. Leader와 Teleoperation

### 6.1 Leader 시작 전 확인

1. inference가 완전히 종료됐는지 확인한다.
2. 다른 trajectory publisher가 없는지 확인한다.
3. Follower가 켜져 있다면 Leader 시작 직후 움직일 수 있음을 인지한다.
4. Leader와 Follower의 시작 자세가 크게 다르지 않은지 확인한다.
5. 주변을 비우고 E-stop을 준비한다.

### 6.2 Leader 시작

1. Follower를 먼저 Bring up한다.
2. Follower controller, `/joint_states`, DXL이 모두 정상인지 확인한다.
3. Leader 모델을 선택한다. 지원 모델은 A3, A2, LG2이며 기본값은 A3다.
4. **LEADER ON**을 누른다.
5. 경고를 확인하고 **실행**을 누른다.
6. UI가 Teleop 화면으로 이동하고 파란색 `TELEOP MODE`가 표시되는지 확인한다.

Follower가 꺼져 있거나 상태 확인이 끝나지 않으면 Leader ON은 UI와 backend 양쪽에서 차단된다.

현재 UI가 시작하는 A3는 startup 오작동을 막는 baseline gate가 적용된 gripper save-pose trigger를 실행한다. 첫 유효 gripper 값은 상태 기준으로만 사용되어 아무 명령도 내보내지 않는다. 시작할 때 gripper가 이미 눌려 있었다면 먼저 완전히 풀고 다시 눌러야 해당 side가 save pose 3으로 이동한다. 전체 F2를 동기화해 복구해야 할 때는 Leader OFF 후 UI의 **Init pose**를 사용한다.

### 6.3 Leader 종료

1. 로봇을 안정된 자세에서 멈춘다.
2. **LEADER OFF**를 누른다.
3. Leader 상태가 `OFF`가 될 때까지 기다린다.
4. `Trajectory source 분리`가 통과될 때까지 기다린다.

Bring-down은 Leader를 자동으로 종료하지 않는다. 반드시 사용자가 Leader OFF를 명시적으로 수행한다.

Teleop 도중 원하는 위치에서 정지한 경우에도 Leader OFF 후 command quiet가 확인되면 Teleop 화면의 **Init pose**를 사용할 수 있다. Leader가 켜져 있거나 최근 command가 남아 있으면 버튼은 비활성화된다.

## 7. F2 Bring-down

Bring-down은 F2를 정상 작업 자세에서 공통 안전 자세를 거쳐 arms-down 자세로 이동한다. 프로세스 종료 동작이 아니다.

### 7.1 실행 조건

- 선택 모델 F2
- Follower control stack 실행 중
- Leader OFF
- Leader command publisher 없음
- inference service/RTC runtime 없음
- command stream quiet
- arm/head/lift controller 준비
- 최신 `/joint_states`
- DXL 정상
- E-stop 해제

### 7.2 실행 순서

1. inference를 STOP/UNLOAD하고 container를 종료한다.
2. Leader를 OFF한다.
3. 안전 조건이 모두 통과될 때까지 기다린다.
4. **BRING DOWN**을 누른다.
5. 경고를 확인하고 **실행**을 누른다.
6. 작업 진행 panel에서 step 로그를 확인한다.
7. 완료 badge가 `DONE`인지 확인한다.
8. 로봇이 완전히 멈춘 뒤 필요한 경우 **CONTROL 종료**를 누른다.

### 7.3 실행되지 않을 때

흐리게 표시된 버튼을 누르면 나타나는 알림에서 차단 이유를 확인한다.

주요 메시지:

- `Leader를 종료하고...`: Leader OFF 후 publisher가 사라질 때까지 기다린다.
- `Inference를 STOP/UNLOAD...`: policy container까지 종료한다.
- `trajectory controller가...`: controller 상태와 launch 로그를 확인한다.
- `/joint_states feedback...`: hardware/control loop를 확인한다.
- `DXL 상태...`: 통신과 hardware error를 확인한다.

## 8. Inference 이상 동작 후 F2 Init Pose 복구

이 기능은 inference가 이상 동작해 STOP한 뒤 팔이 테이블 위에 남은 경우를 위한 제한된 복구 절차다.

복구 자세의 단일 원본은 다음 F2 YAML이다.

```text
ffw_bringup/config/ffw_f2_follower/ffw_f2_follower_initial_positions.yaml
```

복구 runner는 이 파일에서 joint 이름, 단계별 position, Action topic, duration과 tolerance를 읽는다. 현재 YAML 기준으로 양팔과 리프트는 `step0 → step1`, 헤드는 `home`으로 이동한 뒤 그 자세를 유지한다. SG2 복구는 아직 구현되지 않아 SG2 선택 시 버튼이 비활성화된다.

모든 임의 자세에서 collision-free임을 보장하지 않는다.

### 8.1 실행하면 안 되는 경우

- 물리 E-stop 원인이 확인되지 않음
- 팔 또는 물체가 테이블/주변 구조물과 접촉 중
- 케이블이 걸려 있음
- 현재 자세에서 step0까지의 joint-space 경로가 검증되지 않음
- DXL hardware error 존재
- encoder/current joint 상태를 신뢰할 수 없음
- Leader 또는 inference command publisher가 살아 있음

이 경우 자동 복구 대신 로봇 전원을 안전하게 관리하고 수동 recovery 절차를 사용한다.

### 8.2 실행 조건

1. inference STOP
2. inference UNLOAD
3. Groot/LeRobot policy container 종료
4. Leader OFF
5. Follower control stack 유지
6. controller 및 DXL 정상
7. 작업 공간 확인

### 8.3 실행 순서

1. F2가 선택됐는지 확인한다.
2. 상단 inference가 `DISCONNECTED`인지 확인한다.
3. Leader가 `OFF`인지 확인한다.
4. 모든 안전 조건을 확인한다.
5. `팔 이동 경로와 테이블 주변에 사람·물체가 없음` checkbox를 선택한다.
6. **INIT POSE로 복구**를 누른다.
7. 경고를 확인하고 **실행**을 누른다.
8. YAML의 첫 phase 완료 로그를 확인한다.
9. 이후 YAML의 다음 phase 완료 로그를 확인한다.
10. 완료 badge가 `DONE`인지 확인한다.
11. 필요하면 inference를 다시 load하고 시작한다.

### 8.4 즉시 중단

복구 중 이상 움직임이 보이면:

1. 가능하면 UI의 **진행 중 동작 즉시 중단**을 누른다.
2. 위험이 임박하면 UI보다 물리 E-stop을 우선한다.
3. 원인을 확인하기 전 복구를 반복하지 않는다.

UI는 다음 상태에서도 pose 작업을 자동 중단한다.

- 외부 trajectory command 재개
- joint feedback 단절
- DXL 상태 단절
- DXL 통신/hardware error
- E-stop 감지
- controller Action 실패
- 전체 timeout

## 9. E-stop 발생 시

### 9.1 즉시 대응

1. 로봇에서 안전거리를 확보한다.
2. E-stop을 유지한다.
3. 사람, 로봇, 테이블과 주변 물체 상태를 확인한다.
4. 이상 동작 원인을 확인한다.
5. inference와 Leader를 종료한다.

E-stop 토픽이 연결돼 있으면 화면 전체가 적색 점멸하고 다음 문구가 표시된다.

```text
E-STOP / CRITICAL FAULT 감지
자동 재기동되지 않습니다.
로봇을 안전한 원래 위치로 정리한 뒤 정상 Bring-up 절차를 다시 수행하십시오.
```

### 9.2 재기동 절차

1. E-stop 원인을 제거한다.
2. 로봇과 주변 작업 공간을 정리한다.
3. 로봇을 제조사/현장 절차에 따라 안전한 원래 위치로 돌려놓는다.
4. inference와 Leader가 종료됐는지 확인한다.
5. E-stop을 수동 해제한다.
6. 최신 E-stop `false` 신호를 확인한다.
7. hardware 전원, DXL 통신, encoder, error 상태를 확인한다.
8. 정상 Follower Bring-up 절차를 처음부터 수행한다.

E-stop 직후 UI에서 Action Server나 torque를 자동으로 켜고 바로 복구하지 않는다.

## 10. Follower Control 종료

F2는 정상 운영에서는 먼저 Bring-down을 완료한 뒤 종료한다.

1. inference 종료
2. Leader OFF
3. F2의 경우 Bring-down 완료
4. 로봇 정지 확인
5. **CONTROL 종료** 선택
6. Follower가 `DOWN`이 되는지 확인

Follower가 이미 실행 중일 때 흐리게 표시된 **Bring up**을 누르면 `Bring down → Follower control 종료 → Bring up` 순서를 알림으로 안내한다.

Bring-down을 완료하지 않아도 Control 종료 기능 자체는 남아 있다. 이는 hardware fault나 비정상 동작 중 Bring-down trajectory를 실행할 수 없는 상황을 위한 탈출 경로다. 정상 상황에서 편의를 위해 건너뛰어서는 안 된다.

EXTERNAL 상태에서는 UI가 종료할 수 없다. 해당 launch를 시작한 terminal에서 `Ctrl+C`를 사용하거나 소유 service를 정상 종료한다.

## 11. 로그와 오류 해결

### 11.1 원인 진단 카드

UI는 최근 로그를 다음 유형으로 요약한다.

| 진단 | 확인할 항목 |
|---|---|
| Inference 안전 차단 | STOP, UNLOAD, policy container 종료 |
| 로봇 포트 연결 실패 | `/dev/follower`, `/dev/leader`, USB, U2D2, udev |
| Dynamixel 오류 | `comm_state`, Hardware Error Status, 전원 |
| Controller 기동 실패 | controller manager, interface claim, spawner 로그 |
| Action Server 대기 | arm/head/lift controller active 여부 |
| 카메라 경고 | USB 연결, serial, Realsense/ZED 설정 |
| LiDAR 경고 | 전원, USB/network, Lakibeam 설정 |
| ROS 설치/빌드 누락 | package build, install setup source |
| 포트 중복 | 기존 웹 프로세스, `BRINGUP_CONTROL_PORT` |

### 11.2 원본 로그 확인

1. 우측 **전체 로그**를 누른다.
2. `Follower`, `Leader`, `Pose 작업` 중 하나를 선택한다.
3. **새로고침**을 누른다.
4. 가장 마지막 error 이전의 warning부터 확인한다.

### 11.3 Terminal에서 확인

```bash
docker logs -f ai_worker
```

s6 서비스 상태:

```bash
docker exec ai_worker sh -c '
S6_SVSTAT=$(ls /package/admin/s6-*/command/s6-svstat | head -1)
"$S6_SVSTAT" /run/service/ai_worker_bringup
"$S6_SVSTAT" /run/service/avatar_bringup
'
```

ROS publisher 확인:

```bash
docker exec ai_worker bash -lc '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 topic info /leader/joint_states
ros2 topic info /groot/rtc_status
ros2 topic info /lerobot/rtc_status
'
```

## 12. 권장 운영 순서

### 정상 시작

```text
Inference/Leader 완전 종료
  → Follower 모델 선택
  → Follower Bring-up
  → DXL/controller 정상 확인
  → 필요 시 Leader ON 또는 inference 시작
```

### 정상 종료

```text
Inference STOP/UNLOAD/container 종료
  → Leader OFF
  → command quiet 확인
  → F2 Bring-down
  → Follower Control 종료
```

### Software stop 후 복구

```text
Inference STOP/UNLOAD/container 종료
  → Leader OFF
  → 현장 작업 공간 확인
  → F2 init pose 복구
  → 상태 확인
  → inference 재시작
```

### E-stop 후 복구

```text
E-stop 유지
  → 원인 및 현장 확인
  → inference/Leader 종료
  → 로봇을 안전한 원래 위치로 정리
  → E-stop 수동 해제
  → hardware/DXL 확인
  → 정상 Follower Bring-up을 처음부터 수행
```

## 13. 절대 하지 말아야 할 작업

- inference publisher가 살아 있는 상태에서 Follower 재시작
- Leader가 켜진 상태에서 Bring-down 실행
- E-stop 직후 원인 확인 없이 자동 pose recovery
- 임의의 비정상 자세에서 경로 확인 없이 init recovery
- UI의 경고를 우회해 직접 Action Goal 전송
- 같은 `/dev/follower`를 두 controller manager에서 중복 점유
- EXTERNAL 프로세스를 무시하고 UI에서 다시 Bring-up
- Token을 Git 또는 공유 문서에 기록

## 14. 현장 시험 체크리스트

### 조회 기능

- [ ] Follower 실제 상태와 UI 상태 일치
- [ ] Leader 실제 상태와 UI 상태 일치
- [ ] inference service/runtime 상태 일치
- [ ] teleop command 발생 시 `TELEOP ACTIVE` 표시
- [ ] DXL 오류를 올바르게 표시
- [ ] EXTERNAL process 탐지

### 안전 차단

- [ ] inference container가 있으면 Bring-up 차단
- [ ] Leader publisher가 있으면 Follower Bring-up 차단
- [ ] Follower가 꺼져 있으면 Leader ON 차단
- [ ] Follower controller/feedback/DXL 비정상 시 Leader ON 차단
- [ ] Leader ON이면 Bring-down 차단
- [ ] command publisher가 있으면 recovery 차단
- [ ] controller feedback이 없으면 pose 작업 차단
- [ ] DXL error 시 pose 작업 차단
- [ ] E-stop true 시 적색 점멸 및 pose 작업 중단

### 물리 동작

- [x] Follower Bring-up/종료 정상
- [ ] A3 trigger 차단 적용 후 Leader ON/OFF 저속 재시험
- [x] F2 Bring-down 실행 및 controller 완료
- [ ] F2 step0 전역 barrier 확인
- [x] F2 init recovery Action Goal과 controller phase 완료
- [ ] pose 작업 중 UI cancel 확인
- [ ] pose 작업 중 물리 E-stop 확인

물리 시험은 반드시 주변을 비우고 저속으로 시작하며 E-stop에 손이 닿는 상태에서 진행한다.

## 15. Intervention 사용법

Intervention은 현재 F2 Follower와 A3 Leader 조합에서 Cyclo Intelligence의 Inference 화면으로 사용한다. 별도로 policy Docker를 종료하거나 다시 켜지 않는다.

Inference 설정의 `Intervention Dataset`에서 Task Num과 Task Name을 함께 입력할 수 있다. 예를 들어 `000389`와 `OMX_Insert_F2_Intervention`을 입력하면 저장 위치는 `Task_000389_OMX_Insert_F2_Intervention_MCAP`이며, 이후 같은 선택으로 `Save`할 때 episode 0, 1, 2 순서로 누적된다. 두 항목은 모든 Cyclo 브라우저에 동기화된다. 한 항목만 입력했거나 경로 구분자가 포함되면 Intervention 시작이 차단된다. 둘 다 비워두면 기존 호환 방식대로 Save마다 `Task_<UTC시각>_inference_MCAP`을 만든다.

1. Real Robot inference 중 실패 시점에 `Intervene`를 누른다.
2. 화면이 `RECORDING`이 된 것을 확인하고 A3 Leader로 실패 구간을 교정한다.
3. 데이터를 쓸 경우 `Save`, 버릴 경우 `Discard`를 누른다.
4. UI가 Leader를 종료하고 command quiet를 확인한 뒤 inference를 자동 재개한다.

`Abort`는 녹화를 폐기하고 Leader를 종료하지만 inference는 `PAUSED`로 유지한다. `PAUSED_SAFE`가 표시되면 현장과 로봇 상태를 확인한 후 `Resume`을 명시적으로 눌러야 한다.

`Switching` 뒤 `PAUSED_SAFE` 또는 `Resume`이 표시되고 Leader와 Recording이 시작되지 않았다면 hand-off가 실패한 것이다. 이때 일반 Teleop 화면이나 터미널에서 Leader를 직접 켜지 않는다. Inference는 안전하게 `PAUSED`로 유지되므로 로그의 실패 원인을 확인한 다음, 계속 운전하려면 작업 공간과 로봇 자세를 확인하고 `Resume`을 누른다. Intervention을 다시 시도할 때는 inference가 정상 재개된 뒤 물리 E-stop을 준비하고 `Intervene`부터 다시 시작한다.

Intervention 도중 다른 노트북에서도 같은 단계와 버튼이 보이며 중복 시작은 차단된다. Follower controller/DXL 이상, E-stop 또는 policy command가 아직 흐르는 상태에서는 시작되지 않는다.

AI Worker 상태의 Inference와 Recording 항목에서 `typed_status_available=true`여야 물리 Intervention을 시험할 수 있다. 최초 AI Worker Control 시작에서는 Cyclo interface overlay 빌드 때문에 웹 접속까지 수 분이 걸릴 수 있고, 이후에는 IDL이 바뀌지 않는 한 바로 시작한다.

현재 ROS E-stop 토픽이 설정되지 않았다면 UI의 적색 감시는 실제 E-stop 상태를 알 수 없다. 첫 물리 시험은 반드시 주변을 비우고 물리 E-stop을 손에 둔 상태에서 짧고 낮은 위험의 교정 동작으로 진행한다.
