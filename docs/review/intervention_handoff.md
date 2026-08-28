# Intervention: 추론과 A3 사이의 안전한 제어권 전환

## 연결 커밋

```text
[REVIEW] Intervention: add safe inference-to-A3 handoff
```

## 기능 관계

이 커밋은 `Intervention` 기능의 운영·안전 절차를 담당한다. 앞선
`Intervention: route F2 arm commands through a single owner` 커밋이 policy와 A3의
팔 명령을 하나의 arbiter로 모았다면, 이 커밋은 그 명령 소유권을 언제 넘기고 회수할지를
관리한다. 실제 Intervention을 사용하려면 두 커밋이 모두 필요하다.

## 해결하려던 문제

실행 중인 policy가 실패한 구간에서 사람이 A3로 교정 동작을 추가하려면 다음 순서를
모호함 없이 지켜야 한다.

1. inference가 실제로 `PAUSED`인지 확인한다.
2. policy command가 충분히 조용해졌는지 확인한다.
3. F2 Follower와 안전 신호가 정상일 때만 A3 Leader를 시작한다.
4. 사람의 교정 구간을 Cyclo recording 상태와 맞춰 기록한다.
5. A3와 남은 command가 모두 사라진 뒤에만 inference 재개를 허용한다.

브라우저 한 탭에만 상태를 저장하거나 단순히 프로세스 시작 순서에 의존하면 중복 요청,
UI 종료, s6 시작 지연, 안전 상태 변화에 대응할 수 없다. 그래서 Intervention 상태와
watchdog은 AI Worker의 상주 controller가 소유하고, 웹 UI는 이 controller에 요청만
전달하도록 분리했다.

## 런타임 구조

```text
Cyclo/UI 또는 AI Worker UI
           │
           ▼
선택형 UI proxy :8780
           │ loopback
           ▼
상주 intervention_controller :8779
           ├─ ROS 상태 감시
           ├─ 안전 gate와 watchdog
           ├─ s6 Leader 시작·종료
           └─ Intervention 상태 보관
```

- `intervention_controller`는 s6 `longrun` 서비스로 컨테이너 시작 시 등록된다.
- controller 기본 주소는 `127.0.0.1:8779`다. 외부에 직접 노출하지 않는다.
- 운영자가 `ai_worker_control start`를 실행하면 선택형 UI proxy가 기본 `8780` 포트에서
  열린다.
- 여러 브라우저가 접속해도 Intervention의 단계와 제어 lock은 controller 하나가
  공유한다.
- Cyclo의 inference·recording typed status를 읽기 위해 compose에서 Cyclo interfaces
  source를 읽기 전용으로 마운트하고 별도 overlay에 빌드한다. type support가 없으면
  Intervention 시작을 허용하지 않는 fail-closed 동작을 유지한다.

## 시작 gate

현재 구현은 검증 대상인 `F2 Follower + A3 Leader` 조합만 허용한다. A3를 시작하기 전에
다음 조건을 모두 확인한다.

- 물리 제어 기능이 명시적으로 활성화됨
- inference typed status가 `PAUSED`
- 최근 policy trajectory command가 없음
- 선택된 Follower가 F2이며 실행 중임
- F2 controller, joint feedback, DXL 상태가 정상임
- E-stop이 해제됐고 감시 신호가 최신임
- 다른 Leader와 로봇 제어 작업이 없음

연결된 publisher가 있다는 이유만으로 pause 실패로 판단하지 않는다. warm pause에서는
policy publisher 연결이 남을 수 있으므로 실제 최근 command 수신 여부를 별도로 확인한다.

## s6 일회용 hand-off lease

일반 Leader s6 preflight는 inference service나 RTC·trajectory publisher 연결이 남아 있으면
계속 시작을 차단한다. Intervention만 다음 절차로 제한적인 예외를 사용한다.

1. controller가 모든 시작 gate를 통과한다.
2. `/run/ai_worker_intervention_preflight`에 version, 20초 만료 시각, Leader model과
   128-bit nonce를 기록한다.
3. s6 preflight가 내용을 검증하고 파일을 즉시 소비한다.
4. s6가 같은 nonce를 acknowledgement 파일에 기록한다.
5. controller가 acknowledgement를 확인한 경우에만 Leader feedback 대기로 넘어간다.

이 lease는 `PAUSED` policy 연결만 일시적으로 허용한다. Follower-first 검사, hardware
상태 검사와 일반 Leader 시작 차단은 우회하지 않는다. 만료됐거나 model·nonce가 다른
lease는 거부한다.

## 상태 전이

```text
PREPARING → READY → RECORDING → RETURNING → READY_TO_RESUME
     └──────── 오류·Abort·watchdog ───────────────→ PAUSED_SAFE
```

- `PREPARING`: pause·command quiet 확인 후 s6 lease를 발급하고 A3 준비를 기다린다.
- `READY`: A3 runtime과 joint feedback이 확인되어 사람이 조작할 수 있다.
- `RECORDING`: Cyclo의 typed recording status가 `RECORDING`임을 확인했다.
- `RETURNING`: A3를 종료하고 Leader runtime과 최근 command가 사라지기를 기다린다.
- `READY_TO_RESUME`: 명시적인 inference resume을 기다린다.
- `PAUSED_SAFE`: 오류 또는 사용자 중단 뒤 inference를 자동 재개하지 않는 안전 정지다.

`complete`는 inference가 다시 `INFERENCING`이고 Leader가 남아 있지 않을 때만 상태를
초기화한다. 녹화 파일의 저장·폐기와 inference pause/resume 요청 자체는 Cyclo가
담당하며, AI Worker는 그 typed status와 로봇 제어권 전환을 검증한다.

## watchdog과 실패 처리

`PREPARING`, `READY`, `RECORDING` 동안 다음 변화가 발생하면 watchdog이 A3를 종료하고
`PAUSED_SAFE`로 전환한다.

- E-stop 활성화
- inference가 `PAUSED` 상태를 벗어남
- Follower joint feedback 단절
- Follower DXL 상태 이상

중간 예외, 명시적 Abort 또는 watchdog 차단 뒤에는 inference를 자동 재개하지 않는다.
현장 상태를 확인한 다음 사용자가 별도로 Resume 또는 reset을 선택해야 한다.

## 주요 변경 파일

- `docker/bringup_control_ui/server.py`
- `docker/bringup_control_ui/logic.py`
- `docker/bringup_control_ui/ui_proxy.py`
- `docker/bringup_control_ui/static/app.js`
- `docker/bringup_control_ui/static/styles.css`
- `docker/bringup_control_ui/test_logic.py`
- `docker/bringup_control_ui/test_lifecycle.py`
- `docker/s6-services/intervention_controller/`
- `docker/s6-services/common/ros2_service_run.sh`
- `docker/docker-compose.yml`
- `docker/Dockerfile.amd64`
- `docker/Dockerfile.arm64`

## 검증 범위

이 커밋에서는 다음을 오프라인으로 확인한다.

- Intervention gate와 reset gate 단위 테스트
- 상주 controller의 Dockerfile·compose·s6 등록 계약 테스트
- Python 모듈 문법 검사
- shell script 문법 검사

## 안전 수준과 남은 검증

이 커밋은 powered hardware 검증 완료를 뜻하지 않는다. 과거 첫 현장 시도에서는
inference `PAUSED` 확인 후 A3 joint feedback timeout으로 `PAUSED_SAFE`가 됐으며, 그 뒤
일회용 lease와 acknowledgement 절차가 추가됐다. 다음 항목은 별도 현장 검증이 필요하다.

- 실제 F2+A3에서 pause→A3→recording→A3 종료→resume 전체 순서
- Intervention 중 E-stop과 Follower 통신 단절 시 fail-closed 동작
- UI 종료 또는 여러 브라우저의 동시 요청 시 상태 일관성
- Cyclo에 저장된 교정 episode의 내용과 동기화 결과

Head, lift와 mobile base는 이 기능의 제어 대상이 아니다. 별도 계약과 명시적 승인 없이
확장하면 안 된다.

## 검토자가 확인할 핵심

1. 상주 controller와 선택형 UI proxy의 책임 분리가 적절한지 확인한다.
2. 20초 일회용 lease와 nonce acknowledgement가 s6 시작 경쟁 조건을 충분히 막는지
   확인한다.
3. 오류 시 inference를 자동 재개하지 않는 정책을 확인한다.
4. Cyclo 측 recording·resume 구현과 API 계약을 함께 검토한다.
