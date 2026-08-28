"""Pure safety and diagnostic logic for the AI Worker bring-up console."""

from __future__ import annotations

import re
from dataclasses import asdict, dataclass
from typing import Iterable

try:
    from .recovery_config import RECOVERY_PROFILES
except ImportError:
    from recovery_config import RECOVERY_PROFILES


FOLLOWER_MODELS = {
    "sg2": {
        "label": "SG2",
        "launch": "ffw_sg2_follower_ai.launch.py",
        "bring_down": False,
        "recover_init": False,
    },
    "bg2": {
        "label": "BG2",
        "launch": "ffw_bg2_follower_ai.launch.py",
        "bring_down": False,
        "recover_init": False,
    },
    "sh5": {
        "label": "SH5",
        "launch": "ffw_sh5_follower_ai.launch.py",
        "bring_down": False,
        "recover_init": False,
    },
    "bh5": {
        "label": "BH5",
        "launch": "ffw_bh5_follower_ai.launch.py",
        "bring_down": False,
        "recover_init": False,
    },
    "f2": {
        "label": "F2",
        "launch": "f2_follower_ai.launch.py",
        "bring_down": True,
        "recover_init": True,
        "recovery_yaml": RECOVERY_PROFILES["f2"]["config"],
    },
    "f1": {
        "label": "F1",
        "launch": "f1_follower_ai.launch.py",
        "bring_down": False,
        "recover_init": False,
    },
}

LEADER_MODELS = {
    "a3": {"label": "A3", "launch": "ffw_a3_ai.launch.py"},
    "a2": {"label": "A2", "launch": "ffw_a2_ai.launch.py"},
    "lg2": {"label": "LG2", "launch": "ffw_lg2_leader_ai.launch.py"},
}


def operating_mode(
    *,
    leader_up: bool,
    inference_present: bool,
    inference_active: bool,
    inference_phase: str = "UNAVAILABLE",
    intervention_active: bool = False,
) -> dict[str, str | bool]:
    """Return the shared UI mode derived from live ROS/process state."""
    if intervention_active:
        return {
            "name": "intervention",
            "label": "INTERVENTION",
            "detail": "Inference가 일시정지되고 사람에게 조작권이 전달되었습니다.",
            "conflict": False,
        }
    inference_session = inference_present or inference_active
    if leader_up and inference_session:
        return {
            "name": "conflict",
            "label": "MODE CONFLICT",
            "detail": "Leader와 Inference runtime이 동시에 감지되었습니다.",
            "conflict": True,
        }
    if inference_session:
        phase = inference_phase if inference_phase and inference_phase != "UNAVAILABLE" else "CONNECTED"
        return {
            "name": "inference",
            "label": "INFERENCE MODE",
            "detail": f"Inference session · {phase}",
            "conflict": False,
        }
    if leader_up:
        return {
            "name": "teleop",
            "label": "TELEOP MODE",
            "detail": "Leader가 연결되어 Teleoperation 상태입니다.",
            "conflict": False,
        }
    return {
        "name": "idle",
        "label": "STANDBY",
        "detail": "Teleop과 Inference가 모두 분리되어 있습니다.",
        "conflict": False,
    }

@dataclass(frozen=True)
class SafetyState:
    physical_control_enabled: bool = False
    estop_active: bool = False
    estop_signal_ready: bool = True
    critical_fault: bool = False
    inference_present: bool = False
    inference_active: bool = False
    leader_up: bool = False
    leader_stream_active: bool = False
    command_stream_active: bool = False
    follower_up: bool = False
    follower_managed: bool = True
    leader_managed: bool = True
    controllers_ready: bool = False
    feedback_fresh: bool = False
    dxl_ok: bool = False
    operation_running: bool = False
    control_busy: bool = False
    selected_follower: str = ""


@dataclass(frozen=True)
class GateResult:
    ok: bool
    failures: tuple[str, ...]
    warnings: tuple[str, ...] = ()

    def to_dict(self) -> dict:
        return asdict(self)


def gate_intervention_start(state: SafetyState, inference_phase: str) -> GateResult:
    """Allow teleop takeover only from a confirmed, quiet inference pause."""
    failures: list[str] = []
    warnings: list[str] = []
    if not state.physical_control_enabled:
        failures.append("서버의 물리 제어 기능이 잠겨 있습니다.")
    if state.estop_active:
        failures.append("E-stop이 감지되어 Intervention을 시작할 수 없습니다.")
    if not state.estop_signal_ready:
        failures.append("설정된 E-stop 감시 토픽의 최신 해제 신호를 확인할 수 없습니다.")
    if state.critical_fault:
        failures.append("로봇 통신 또는 하드웨어 fault가 있습니다.")
    if not state.follower_up:
        failures.append("Follower가 실행 중이어야 합니다.")
    else:
        if not state.controllers_ready:
            failures.append("Follower arm/head/lift controller가 모두 준비되지 않았습니다.")
        if not state.feedback_fresh:
            failures.append("Follower /joint_states feedback이 최신 상태가 아닙니다.")
        if not state.dxl_ok:
            failures.append("Follower DXL 통신과 hardware 상태가 정상으로 확인되지 않았습니다.")
    if state.selected_follower != "f2":
        failures.append("Intervention은 현재 F2 + A3 조합에서만 지원됩니다.")
    if not state.inference_present:
        failures.append("실행 중인 Inference session이 없습니다.")
    if inference_phase != "PAUSED":
        failures.append("Inference가 PAUSED로 확인된 뒤 Intervention을 시작할 수 있습니다.")
    if state.leader_up or state.leader_stream_active:
        failures.append("Leader가 이미 켜져 있거나 command stream이 남아 있습니다.")
    # Policy publishers may remain connected while the warm model is paused.
    # Only a recent message is unsafe for this hand-off.
    if state.command_stream_active:
        failures.append("최근 trajectory command가 감지되었습니다. 정책 command가 멈출 때까지 기다리십시오.")
    if state.operation_running or state.control_busy:
        failures.append("다른 로봇 제어 작업이 진행 중입니다.")
    if not failures:
        warnings.append("Leader 시작 직후 사람의 조작 명령이 Follower에 전달됩니다.")
    return GateResult(not failures, tuple(failures), tuple(warnings))


def gate_intervention_reset(
    phase: str,
    *,
    leader_up: bool,
    command_recent: bool,
    recording_phase: str,
) -> GateResult:
    """Allow a faulted hand-off to return to a fresh inference start safely."""
    failures: list[str] = []
    if phase not in {"PAUSED_SAFE", "READY_TO_RESUME"}:
        failures.append("안전 중단 또는 Resume 대기 상태에서만 Intervention을 초기화할 수 있습니다.")
    if leader_up:
        failures.append("Leader가 실행 중이므로 Intervention을 초기화할 수 없습니다.")
    if command_recent:
        failures.append("최근 trajectory command가 남아 있어 Intervention을 초기화할 수 없습니다.")
    if recording_phase == "RECORDING":
        failures.append("Intervention recording을 먼저 취소하거나 저장해야 합니다.")
    return GateResult(not failures, tuple(failures))


def gate_action(action: str, state: SafetyState) -> GateResult:
    """Apply fail-closed physical-control rules to a requested operation."""
    failures: list[str] = []
    warnings: list[str] = []

    stopping_action = action in {"follower_stop", "leader_stop", "operation_cancel"}
    if action != "operation_cancel" and not state.physical_control_enabled:
        failures.append("서버의 물리 제어 기능이 잠겨 있습니다.")
    if not stopping_action and state.estop_active:
        failures.append("E-stop이 감지되어 움직임 명령이 차단되었습니다.")
    if not stopping_action and not state.estop_signal_ready:
        failures.append("설정된 E-stop 감시 토픽의 최신 해제 신호를 확인할 수 없습니다.")
    if not stopping_action and state.critical_fault:
        failures.append("로봇 통신 또는 하드웨어 fault가 있어 움직임 명령이 차단되었습니다.")
    if action != "operation_cancel" and state.operation_running:
        failures.append("다른 자세 이동 작업이 진행 중입니다.")
    if action != "operation_cancel" and state.control_busy:
        failures.append("다른 사용자의 제어 요청이 처리 중입니다.")

    if action == "follower_start":
        if state.follower_up:
            failures.append("Follower bring-up이 이미 실행 중입니다.")
        if state.inference_present:
            failures.append("Inference service/container가 살아 있습니다. STOP과 UNLOAD 후 container를 종료하십시오.")
        if state.leader_up or state.leader_stream_active:
            failures.append("Leader가 켜져 있거나 command stream이 남아 있습니다. Leader를 먼저 종료하십시오.")
        if state.command_stream_active:
            failures.append("최근 trajectory command가 감지되었습니다. command publisher를 완전히 정지하십시오.")

    elif action == "follower_stop":
        if not state.follower_up:
            failures.append("Follower bring-up이 실행 중이 아닙니다.")
        elif not state.follower_managed:
            failures.append("Follower가 UI 밖에서 실행되어 s6로 종료할 수 없습니다. 시작한 terminal/service에서 종료하십시오.")
        if state.inference_present or state.inference_active:
            failures.append("Inference가 연결된 상태에서는 control stack을 종료할 수 없습니다.")
        if state.leader_up or state.leader_stream_active:
            failures.append("Follower보다 Leader를 먼저 종료해야 합니다.")

    elif action == "leader_start":
        if state.leader_up:
            failures.append("Leader bring-up이 이미 실행 중입니다.")
        if not state.follower_up:
            failures.append("Follower를 먼저 Bring up해야 Leader를 시작할 수 있습니다.")
        else:
            if not state.controllers_ready:
                failures.append("Follower arm/head/lift trajectory controller가 모두 준비된 뒤 Leader를 시작하십시오.")
            if not state.feedback_fresh:
                failures.append("Follower /joint_states feedback이 최신 상태가 아닙니다.")
            if not state.dxl_ok:
                failures.append("Follower DXL 통신과 hardware 상태가 정상으로 확인되지 않았습니다.")
        if state.inference_present or state.inference_active:
            failures.append("Inference가 연결된 상태에서는 Leader를 시작할 수 없습니다.")
        if state.command_stream_active:
            failures.append("다른 trajectory command stream이 아직 활성 상태입니다.")
        if state.follower_up and state.controllers_ready and state.feedback_fresh and state.dxl_ok:
            warnings.append("Follower가 켜져 있으므로 Leader 시작 직후 teleoperation 명령이 전달될 수 있습니다.")

    elif action == "leader_stop":
        if not state.leader_up:
            failures.append("Leader bring-up이 실행 중이 아닙니다.")
        elif not state.leader_managed:
            failures.append("Leader가 UI 밖에서 실행되어 s6로 종료할 수 없습니다. 시작한 terminal/service에서 종료하십시오.")

    elif action in {"bring_down", "recover_init"}:
        label = "Bring-down" if action == "bring_down" else "Init pose 복구"
        if state.selected_follower != "f2":
            failures.append(f"{label}는 현재 F2에서만 지원됩니다.")
        if not state.follower_up:
            failures.append("Follower control stack이 실행 중이어야 합니다.")
        if state.leader_up or state.leader_stream_active:
            failures.append("Leader를 종료하고 command stream이 멈춘 뒤 실행하십시오.")
        if state.inference_present or state.inference_active:
            failures.append("Inference를 STOP/UNLOAD하고 container까지 종료한 뒤 실행하십시오.")
        if state.command_stream_active:
            failures.append("최근 trajectory command가 감지되어 자세 이동을 시작할 수 없습니다.")
        if not state.controllers_ready:
            failures.append("arm/head/lift trajectory controller가 모두 준비되지 않았습니다.")
        if not state.feedback_fresh:
            failures.append("/joint_states feedback이 최신 상태가 아닙니다.")
        if not state.dxl_ok:
            failures.append("DXL 통신 또는 hardware 상태가 정상으로 확인되지 않았습니다.")
        if action == "recover_init":
            warnings.append("현재 자세에서 공통 step0까지의 경로가 작업 공간과 충돌하지 않는지 현장에서 확인해야 합니다.")

    elif action != "operation_cancel":
        failures.append(f"알 수 없는 작업입니다: {action}")

    return GateResult(not failures, tuple(failures), tuple(warnings))


_DIAGNOSTIC_RULES: tuple[tuple[re.Pattern[str], str, str, str], ...] = (
    (re.compile(r"SAFETY BLOCK|inference service is still active", re.I), "danger", "Inference 안전 차단", "Inference를 STOP/UNLOAD하고 policy container를 종료하십시오."),
    (re.compile(r"/dev/(follower|leader).*?(not found|no such|cannot|failed)|failed to open.*port|could not open.*port", re.I), "danger", "로봇 포트 연결 실패", "USB/U2D2 전원, udev symlink와 /dev/follower 또는 /dev/leader를 확인하십시오."),
    (re.compile(r"hardware error|dxl.*error|comm(_state)?.*error", re.I), "danger", "Dynamixel 오류", "DXL 통신 상태와 Hardware Error Status를 확인하고 필요하면 전원을 재인가하십시오."),
    (re.compile(r"controller.*(failed|error)|failed.*controller|spawner.*(failed|error)", re.I), "danger", "Controller 기동 실패", "controller_manager 로그에서 실패한 controller와 command/state interface 충돌을 확인하십시오."),
    (re.compile(r"waiting for action server|action server.*not available|action.*timeout", re.I), "warning", "Action Server 대기", "arm/head/lift controller가 active인지 확인하십시오."),
    (re.compile(r"realsense|zed|camera", re.I), "warning", "카메라 경고", "카메라 USB 연결과 serial 설정을 확인하십시오."),
    (re.compile(r"lidar|lakibeam|scan", re.I), "warning", "LiDAR 경고", "LiDAR 전원, 포트와 네트워크 설정을 확인하십시오."),
    (re.compile(r"package .* not found|executable .* not found|no executable", re.I), "danger", "ROS 설치/빌드 누락", "workspace가 빌드되었고 install/setup.bash가 source되었는지 확인하십시오."),
    (re.compile(r"address already in use", re.I), "danger", "포트 중복", "같은 웹 또는 ROS bridge 프로세스가 이미 실행 중인지 확인하십시오."),
)


def diagnose_lines(lines: Iterable[str], limit: int = 12) -> list[dict[str, str]]:
    """Turn noisy launch output into deduplicated operator-facing diagnostics."""
    found: list[dict[str, str]] = []
    seen: set[str] = set()
    for raw in reversed(list(lines)):
        line = raw.strip()
        if not line:
            continue
        for pattern, severity, title, guidance in _DIAGNOSTIC_RULES:
            if not pattern.search(line) or title in seen:
                continue
            seen.add(title)
            found.append({
                "severity": severity,
                "title": title,
                "guidance": guidance,
                "evidence": line[-500:],
            })
            break
        if len(found) >= limit:
            break
    return found


def parse_s6_status(raw: str) -> dict[str, object]:
    text = raw.strip()
    lowered = text.lower()
    state = "up" if lowered.startswith("up") else "down" if lowered.startswith("down") else "unknown"
    pid_match = re.search(r"\(pid\s+(\d+)\)", text)
    uptime_match = re.search(r"up\s*\(pid\s+\d+\)\s*(\d+)\s+seconds", text, re.I)
    return {
        "state": state,
        "pid": int(pid_match.group(1)) if pid_match else None,
        "uptime_s": int(uptime_match.group(1)) if uptime_match else None,
        "raw": text,
    }
