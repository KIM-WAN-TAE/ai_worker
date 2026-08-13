# AI Worker Bring-up Control

로봇 내부에서 Follower/Leader bring-up, F2 bring-down과 init-pose 복구를 관리하는 ROS 상태 기반 웹 콘솔입니다.

관련 문서:

- [USER_GUIDE.md](USER_GUIDE.md): 현장 사용설명서와 안전 절차
- [IMPLEMENTATION.md](IMPLEMENTATION.md): 현재 구현 구조와 설계
- [DEVELOPMENT_LOG.md](DEVELOPMENT_LOG.md): 개발 과정, 현장 장애와 결정 근거

## 제공 기능

- Cyclo Intelligence와 같은 좌측 메뉴 구조: Home, Teleop, Inference, System
- Home에서 Robot type과 Leader type을 먼저 선택하고 Robot lifecycle을 한 블록에서 관리
- Leader 연결 시 파란색 Teleop mode, inference runtime 연결 시 보라색 Inference mode로 전체 화면 상태 전환
- Teleop/Inference runtime이 새로 감지되면 해당 좌측 메뉴 화면으로 자동 이동
- 실제 설치된 launch에 맞춘 Follower 선택: SG2, BG2, SH5, BH5, F2, F1
- A3, A2, LG2 Leader 시작/종료와 teleoperation command 감시
- inference service, `/task/inference_status`(interface 설치 시), RTC 상태 및 실제 trajectory command 감시
- F2 bring-down: Leader/inference/외부 command가 남아 있으면 백엔드에서 실행 차단
- F2 복구: 선택 모델의 initial-position YAML을 단일 원본으로 사용해 단계별 전역 barrier 실행
- DXL 통신/hardware error, controller state, `/joint_states`, 설정형 물리 E-stop 감시
- s6 launch 로그를 운영자용 원인과 조치 문구로 분류

UI는 기본 `8780` 포트를 사용하며 A3가 기본 Leader입니다. 컨테이너 안에서 운영 명령을 실행했을 때만 웹 서버가 열립니다.

## 시작과 종료

AI Worker 컨테이너에 들어가 다음 명령 하나로 시작합니다.

```bash
ai_worker_control
```

브라우저 주소는 `http://로봇-IP:8780`입니다. 이 명령은 격리된 로봇 LAN의 공용 물리 제어 모드로 서버를 열며, 별도의 환경 변수나 token 입력은 필요하지 않습니다. 터미널에서 `Ctrl+C`를 누르거나 Worker를 종료하면 UI도 종료됩니다.

별도 터미널에서 상태 확인과 종료도 할 수 있습니다.

```bash
ai_worker_control status
ai_worker_control stop
```

Worker를 다시 켠 다음에는 `ai_worker_control`을 다시 실행해야 합니다. 명령 실행 후에는 같은 LAN의 모든 접속자가 로봇 제어를 요청할 수 있습니다.

## 다중 사용자 동기화

- 실제 ROS/s6 상태가 모든 브라우저의 공통 기준입니다.
- 요청 승인 즉시 `STARTING`, `STOPPING`, `MOVING`, `RECOVERING` 상태를 전체 UI에 전파합니다.
- 처리 중인 요청이 있으면 다른 브라우저의 충돌 요청은 HTTP `409`로 거절됩니다.
- 실행 모델과 완료 상태는 다음 SSE 갱신에서 모든 브라우저에 동일하게 표시됩니다.

## E-stop 연동

저장소에는 모든 기종에 공통인 물리 E-stop ROS 토픽이 정의되어 있지 않습니다. 실제 시스템이 `std_msgs/msg/Bool` 신호를 제공한다면 다음처럼 연결합니다.

```bash
export AI_WORKER_ESTOP_TOPIC=/safety/e_stop
```

`true`가 수신되면 전체 UI가 적색 점멸 상태가 되고 pose 작업을 즉시 중단합니다. 토픽이 설정되지 않으면 UI는 **E-stop 직접 신호 미설정**을 계속 표시하며 이를 자동 안전 확인으로 간주하지 않습니다. 다만 저장소에 공통 신호가 아직 없으므로 미설정 사실 하나만으로 모든 제어를 차단하지는 않으며, 현장 확인이 계속 필요합니다. Follower 실행 중 `/joint_states` 또는 DXL 상태가 끊기는 경우도 별도 critical fault로 표시합니다.

## 안전 동작 원칙

- Bring-down은 Leader를 자동 종료하지 않습니다. Leader OFF는 사용자가 명시적으로 요청하고 command quiet 상태를 확인해야 합니다.
- inference service가 존재하면 session이 `PAUSED` 또는 `READY`여도 Follower bring-up과 pose recovery를 차단합니다. STOP, UNLOAD 후 policy container까지 종료해야 합니다.
- E-stop은 자동으로 해제하거나 torque를 켜지 않습니다. 원인 확인, 물리 reset, 로봇 자세 정리 후 정상 bring-up 절차로 복귀합니다.
- F2 복구 경로는 임의의 모든 자세에서 collision-free임을 보장하지 않습니다. 현장 작업 공간 확인을 요구하며, 검증된 recovery envelope는 별도로 구축해야 합니다.
- 현재 F2 복구 설정은 `ffw_bringup/config/ffw_f2_follower/ffw_f2_follower_initial_positions.yaml`에서 읽습니다. joint, 자세값, 단계명, Action topic, duration과 tolerance를 복구 runner에 중복 정의하지 않습니다.
- SG2 복구 profile은 아직 구현하지 않았으므로 SG2 선택 시 복구 버튼은 비활성 상태를 유지합니다.
- Cyclo의 `STOP INFERENCE`는 `PAUSED`이며 완전 종료가 아닙니다. Inference service/RTC가 남아 있는 동안 Init pose는 비활성화되고, Clear/UNLOAD 및 runtime 분리 후에만 다시 활성화됩니다.
- Leader ON은 Follower가 먼저 실행되고 arm/head/lift controller, `/joint_states`, DXL이 모두 정상일 때만 허용됩니다.
- F2 정상 종료 수순은 `Bring down → Follower control 종료`로 안내합니다. 비정상 동작 시 제어 종료 경로를 막지 않기 위해 Bring down은 강제 interlock이 아니라 강한 운영 안내로 적용합니다.
- A3의 gripper save-pose trigger는 시작 당시 이미 닫힌 오른쪽 gripper를 새 trigger로 오인하는 문제가 확인되어 UI/s6가 시작하는 A3 launch에서는 비활성화됩니다.

## 개발 실행 및 테스트

저수준 개발 실행은 다음 스크립트를 직접 사용합니다. 일반 운영자는 `ai_worker_control`을 사용합니다.

```bash
/root/ros2_ws/src/ai_worker/docker/bringup_control_ui/run.sh
```

순수 안전 로직 테스트는 ROS 없이 실행됩니다.

```bash
python3 -m unittest bringup_control_ui.test_logic
```

현재 자동 검사 기준은 18개 unit test입니다. 실제 물리 시험 결과와 남은 항목은 [DEVELOPMENT_LOG.md](DEVELOPMENT_LOG.md)를 참고하십시오.
