#!/usr/bin/env python3
"""ROS-aware FastAPI control plane for AI Worker bring-up operations."""

from __future__ import annotations

import asyncio
import glob
import json
import os
import re
import secrets
import signal
import subprocess
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from dynamixel_interfaces.msg import DynamixelState
from fastapi import FastAPI, Header, HTTPException, Query, Request
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

try:
    from interfaces.msg import InferenceStatus, RecordingStatus, RtcStatus
except ImportError:  # The policy interfaces package may live only in another container.
    InferenceStatus = None
    RecordingStatus = None
    RtcStatus = None

from logic import (
    FOLLOWER_MODELS,
    LEADER_MODELS,
    SafetyState,
    diagnose_lines,
    gate_action,
    gate_intervention_reset,
    gate_intervention_start,
    operating_mode,
    parse_s6_status,
)


ROOT = Path(__file__).resolve().parent
AI_WORKER_ROOT = ROOT.parents[1]
STATIC = ROOT / "static"
PHYSICAL_CONTROL_ENABLED = os.environ.get("ENABLE_BRINGUP_CONTROL", "false").lower() in {"1", "true", "yes"}
CONTROL_TOKEN = os.environ.get("BRINGUP_CONTROL_TOKEN", "").strip()
ESTOP_TOPIC = os.environ.get("AI_WORKER_ESTOP_TOPIC", "").strip()
FRESH_SECONDS = float(os.environ.get("BRINGUP_FRESH_SECONDS", "2.0"))
COMMAND_QUIET_SECONDS = float(os.environ.get("BRINGUP_COMMAND_QUIET_SECONDS", "2.0"))
STARTUP_GRACE_SECONDS = float(os.environ.get("BRINGUP_STARTUP_GRACE_SECONDS", "20.0"))
LOG_ROOT = Path(os.environ.get("BRINGUP_LOG_ROOT", "/var/log"))
ROBOT_TYPE_FILE = Path("/run/robot_type")
LEADER_TYPE_FILE = Path("/run/leader_type")
INTERVENTION_PREFLIGHT_LEASE = Path("/run/ai_worker_intervention_preflight")
INTERVENTION_PREFLIGHT_ACK = Path("/run/ai_worker_intervention_preflight.ack")

SERVICES = {"follower": "ai_worker_bringup", "leader": "avatar_bringup"}
COMMAND_TOPICS = (
    "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
    "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
    "/leader/joystick_controller_left/joint_trajectory",
    "/leader/joystick_controller_right/joint_trajectory",
)
CONTROLLER_TOPICS = (
    "/arm_l_controller/controller_state",
    "/arm_r_controller/controller_state",
    "/head_controller/controller_state",
    "/lift_controller/controller_state",
)
INFERENCE_SERVICES = ("/lerobot/inference_command", "/groot/inference_command")
RTC_TOPICS = ("/groot/rtc_status", "/lerobot/rtc_status")
INFERENCE_PHASES = {0: "READY", 1: "LOADING", 2: "INFERENCING", 3: "PAUSED"}
RTC_PHASES = {0: "UNCONFIGURED", 1: "COLD_WARMUP", 2: "RUNNING", 3: "PAUSED", 4: "STOPPED", 5: "FAULT_LATCHED", 6: "INFEASIBLE"}
RECORD_PHASES = {0: "READY", 1: "RECORDING", 2: "SAVING", 3: "PAUSED"}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def monotonic() -> float:
    return time.monotonic()


def age(seen: float | None) -> float | None:
    return None if seen is None else max(0.0, monotonic() - seen)


def read_selected(path: Path, default: str) -> str:
    try:
        return path.read_text(encoding="utf-8").strip().lower() or default
    except OSError:
        return default


class OperationManager:
    """Own the one allowed one-shot pose operation and its bounded logs."""

    def __init__(self, event_callback: Callable[[str, str], None]) -> None:
        self.lock = threading.RLock()
        self.process: subprocess.Popen[str] | None = None
        self.kind: str | None = None
        self.started_at: float | None = None
        self.logs: deque[str] = deque(maxlen=600)
        self.last_result: dict[str, Any] | None = None
        self._event = event_callback

    def start(self, kind: str, command: list[str]) -> None:
        with self.lock:
            if self.process and self.process.poll() is None:
                raise RuntimeError("다른 자세 이동 작업이 이미 실행 중입니다.")
            self.logs.clear()
            self.kind = kind
            self.started_at = monotonic()
            self.last_result = None
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            process = self.process
        self._event("warning", f"{kind} 시작")
        threading.Thread(target=self._drain, args=(process, kind), daemon=True).start()

    def _drain(self, process: subprocess.Popen[str], kind: str) -> None:
        if process.stdout:
            for line in process.stdout:
                with self.lock:
                    self.logs.append(line.rstrip())
        code = process.wait()
        with self.lock:
            if self.process is process:
                self.process = None
                self.last_result = {
                    "kind": kind,
                    "ok": code == 0,
                    "exit_code": code,
                    "finished_at": utc_now(),
                }
        level = "info" if code == 0 else "danger"
        self._event(level, f"{kind} {'완료' if code == 0 else f'실패(exit {code})'}")

    def cancel(self, reason: str = "사용자 중단") -> bool:
        with self.lock:
            process = self.process
            kind = self.kind
        if not process or process.poll() is not None:
            return False
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        self._event("warning", f"{kind} 중단: {reason}")
        return True

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            running = bool(self.process and self.process.poll() is None)
            return {
                "running": running,
                "kind": self.kind if running else None,
                "started_age": age(self.started_at) if running else None,
                "logs": list(self.logs)[-100:],
                "last_result": self.last_result,
            }


class ControlCoordinator:
    """Serialize commands from every browser and expose their shared transition state."""

    PHASES = {
        "follower_start": "STARTING",
        "follower_stop": "STOPPING",
        "leader_start": "STARTING",
        "leader_stop": "STOPPING",
        "bring_down": "MOVING",
        "recover_init": "RECOVERING",
        "intervention_start": "HANDING_OVER",
        "intervention_end": "RETURNING_CONTROL",
        "intervention_reset": "RESETTING",
    }

    def __init__(self) -> None:
        self._command_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._current: dict[str, Any] | None = None
        self._last: dict[str, Any] | None = None

    def try_acquire(self) -> bool:
        return self._command_lock.acquire(blocking=False)

    def announce(self, action: str, actor: str, model: str = "") -> None:
        with self._state_lock:
            self._current = {
                "action": action,
                "phase": self.PHASES.get(action, "RUNNING"),
                "actor": actor[:80],
                "model": model,
                "started_at": utc_now(),
                "started_monotonic": monotonic(),
            }

    def finish(self, ok: bool, error: str = "") -> None:
        with self._state_lock:
            current = self._current
            if current:
                self._last = {
                    key: value for key, value in current.items() if key != "started_monotonic"
                }
                self._last.update({
                    "ok": ok,
                    "error": error[:500],
                    "finished_at": utc_now(),
                    "duration_s": round(monotonic() - current["started_monotonic"], 3),
                })
            self._current = None
        self._command_lock.release()

    def release(self) -> None:
        self._command_lock.release()

    def snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            current = None
            if self._current:
                current = {key: value for key, value in self._current.items() if key != "started_monotonic"}
                current["age_s"] = round(monotonic() - self._current["started_monotonic"], 1)
            return {"running": current is not None, "current": current, "last": dict(self._last) if self._last else None}


class InterventionManager:
    """Shared, server-owned intervention lease visible to every browser."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._current: dict[str, Any] | None = None
        self._last: dict[str, Any] | None = None

    def begin(self, actor: str, model: str) -> None:
        with self._lock:
            if self._current is not None:
                raise RuntimeError("다른 사용자의 Intervention이 이미 진행 중입니다.")
            self._current = {
                "phase": "PREPARING",
                "actor": actor[:80],
                "leader_model": model,
                "started_at": utc_now(),
                "updated_at": utc_now(),
                "error": "",
            }

    def transition(self, phase: str, error: str = "") -> None:
        with self._lock:
            if self._current is None:
                raise RuntimeError("진행 중인 Intervention이 없습니다.")
            self._current["phase"] = phase
            self._current["updated_at"] = utc_now()
            self._current["error"] = error[:500]

    def clear(self) -> None:
        with self._lock:
            if self._current is None:
                return
            self._last = {**self._current, "finished_at": utc_now()}
            self._current = None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            current = dict(self._current) if self._current else None
            return {
                "active": current is not None,
                "current": current,
                "last": dict(self._last) if self._last else None,
            }


class BringupMonitor(Node):
    def __init__(self) -> None:
        super().__init__("ai_worker_intervention_controller")
        self.lock = threading.RLock()
        self.topic_seen: dict[str, float] = {}
        self.graph_topics: set[str] = set()
        self.graph_services: set[str] = set()
        self.publisher_counts: dict[str, int] = {}
        self.events: deque[dict[str, str]] = deque(maxlen=120)
        self.dxl = {"seen": None, "comm_state": None, "errors": {}}
        self.estop_active = False
        self.estop_seen: float | None = None
        self.inference = {"seen": None, "phase": None, "robot_type": "", "error": ""}
        self.recording = {"seen": None, "phase": None, "task_name": ""}
        self.rtc: dict[str, dict[str, Any]] = {}
        self.operation: OperationManager | None = None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, "/leader/joint_states", lambda _m: self._touch("/leader/joint_states"), qos)
        self.create_subscription(JointState, "/joint_states", lambda _m: self._touch("/joint_states"), qos)
        for topic in COMMAND_TOPICS:
            self.create_subscription(JointTrajectory, topic, lambda _m, t=topic: self._touch(t), reliable)
        for topic in CONTROLLER_TOPICS:
            self.create_subscription(JointTrajectoryControllerState, topic, lambda _m, t=topic: self._touch(t), qos)
        self.create_subscription(DynamixelState, "/ffw_follower/dxl_state", self._dxl, qos)
        if ESTOP_TOPIC:
            self.create_subscription(Bool, ESTOP_TOPIC, self._estop, 10)
        if InferenceStatus is not None:
            self.create_subscription(InferenceStatus, "/task/inference_status", self._inference, reliable)
        if RecordingStatus is not None:
            self.create_subscription(RecordingStatus, "/data/recording/status", self._recording, reliable)
        if RtcStatus is not None:
            for backend in ("groot", "lerobot"):
                self.create_subscription(RtcStatus, f"/{backend}/rtc_status", lambda m, b=backend: self._rtc(b, m), reliable)
        self.arbiter_clients = {
            action: self.create_client(Trigger, f"/command_source_arbiter/{action}")
            for action in (
                "prepare_intervention",
                "select_none",
                "select_teleop",
                "disable_intervention",
            )
        }
        self.create_timer(1.0, self._refresh_graph)
        self.create_timer(0.2, self._operation_watchdog)

    def set_operation_manager(self, operation: OperationManager) -> None:
        self.operation = operation

    def _touch(self, topic: str) -> None:
        with self.lock:
            self.topic_seen[topic] = monotonic()

    def _dxl(self, msg: DynamixelState) -> None:
        with self.lock:
            self.topic_seen["/ffw_follower/dxl_state"] = monotonic()
            self.dxl = {
                "seen": monotonic(),
                "comm_state": int(msg.comm_state),
                "errors": {str(index): int(value) for index, value in zip(msg.id, msg.dxl_hw_state)},
            }

    def _estop(self, msg: Bool) -> None:
        with self.lock:
            previous = self.estop_active
            self.estop_active = bool(msg.data)
            self.estop_seen = monotonic()
        if self.estop_active and not previous:
            self.event("danger", "물리 E-stop 신호가 감지되었습니다.")
        elif previous and not self.estop_active:
            self.event("warning", "E-stop 신호가 해제되었습니다. 원인 확인 후 정상 절차로 재기동하십시오.")

    def _inference(self, msg) -> None:
        with self.lock:
            self.inference = {
                "seen": monotonic(),
                "phase": int(msg.inference_phase),
                "robot_type": str(msg.robot_type),
                "error": str(msg.error),
            }

    def _rtc(self, backend: str, msg) -> None:
        with self.lock:
            self.rtc[backend] = {
                "seen": monotonic(),
                "state": int(msg.rtc_state),
                "publish_to_robot": bool(msg.publish_to_robot),
                "fault": str(msg.fault),
            }

    def _recording(self, msg) -> None:
        with self.lock:
            self.recording = {
                "seen": monotonic(),
                "phase": int(msg.record_phase),
                "task_name": str(msg.task_info.task_name),
            }

    def _refresh_graph(self) -> None:
        try:
            topics = self.get_topic_names_and_types()
            services = self.get_service_names_and_types()
            monitored = {
                "/leader/joint_states",
                "/joint_states",
                "/ffw_follower/dxl_state",
                "/task/inference_status",
                "/data/recording/status",
                *COMMAND_TOPICS,
                *CONTROLLER_TOPICS,
                *RTC_TOPICS,
            }
            publisher_counts = {
                topic: len(self.get_publishers_info_by_topic(topic))
                for topic in monitored
            }
            with self.lock:
                self.graph_topics = {name for name, _types in topics}
                self.graph_services = {name for name, _types in services}
                self.publisher_counts = publisher_counts
        except Exception as exc:
            self.get_logger().warning(f"ROS graph discovery failed: {exc}")

    def event(self, level: str, message: str) -> None:
        with self.lock:
            self.events.appendleft({"time": utc_now(), "level": level, "message": message})

    def call_arbiter(self, action: str, timeout: float = 2.0) -> tuple[bool, str]:
        """Call one arbiter Trigger service while the ROS executor keeps spinning."""
        client = self.arbiter_clients[action]
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f"command-source arbiter service unavailable: {action}"
        completed = threading.Event()
        result: dict[str, Any] = {}
        future = client.call_async(Trigger.Request())

        def done_callback(done_future) -> None:
            try:
                result["response"] = done_future.result()
            except Exception as exc:
                result["error"] = str(exc)
            finally:
                completed.set()

        future.add_done_callback(done_callback)
        if not completed.wait(timeout):
            future.cancel()
            return False, f"command-source arbiter call timeout: {action}"
        if "error" in result:
            return False, f"command-source arbiter call failed: {result['error']}"
        response = result.get("response")
        if response is None:
            return False, f"command-source arbiter returned no response: {action}"
        return bool(response.success), str(response.message)

    def ros_snapshot(self) -> dict[str, Any]:
        with self.lock:
            seen = dict(self.topic_seen)
            services = set(self.graph_services)
            dxl = dict(self.dxl)
            dxl["errors"] = dict(self.dxl.get("errors", {}))
            inference = dict(self.inference)
            recording = dict(self.recording)
            rtc = {key: dict(value) for key, value in self.rtc.items()}
            publisher_counts = dict(self.publisher_counts)
            estop_active = self.estop_active
            estop_seen = self.estop_seen
            events = list(self.events)
        command_ages = {topic: age(seen.get(topic)) for topic in COMMAND_TOPICS}
        controller_ages = {topic: age(seen.get(topic)) for topic in CONTROLLER_TOPICS}
        inference_services = sorted(name for name in INFERENCE_SERVICES if name in services)
        rtc_publishers = sorted(topic for topic in RTC_TOPICS if publisher_counts.get(topic, 0) > 0)
        rtc_active = any(
            bool(item.get("publish_to_robot")) and (age(item.get("seen")) or 1e9) < FRESH_SECONDS
            for item in rtc.values()
        )
        phase_active = inference.get("phase") in {1, 2, 3} and (age(inference.get("seen")) or 1e9) < 5.0
        if inference.get("phase") is not None:
            inference_phase = INFERENCE_PHASES.get(inference.get("phase"), "UNKNOWN")
        elif inference_services or rtc_publishers:
            inference_phase = "CONNECTED"
        else:
            inference_phase = "UNAVAILABLE"
        return {
            "topic_seen": seen,
            "publisher_counts": publisher_counts,
            "command_ages": command_ages,
            "controller_ages": controller_ages,
            "joint_state_age": age(seen.get("/joint_states")),
            "leader_joint_state_age": age(seen.get("/leader/joint_states")),
            "dxl": {
                **dxl,
                "age": age(dxl.get("seen")),
                "hardware_errors": {key: value for key, value in dxl.get("errors", {}).items() if value},
            },
            "estop": {
                "configured": bool(ESTOP_TOPIC),
                "topic": ESTOP_TOPIC or None,
                "signal_seen": estop_seen is not None,
                "age": age(estop_seen),
                "active": estop_active,
            },
            "inference": {
                "services": inference_services,
                "present": bool(inference_services or rtc_publishers),
                "rtc_publishers": rtc_publishers,
                "active": bool(rtc_active or phase_active),
                "phase": inference_phase,
                "phase_age": age(inference.get("seen")),
                "robot_type": inference.get("robot_type", ""),
                "error": inference.get("error", ""),
                "rtc": {
                    backend: {**item, "state_label": RTC_PHASES.get(item.get("state"), "UNKNOWN"), "age": age(item.get("seen"))}
                    for backend, item in rtc.items()
                },
                "typed_status_available": InferenceStatus is not None,
            },
            "recording": {
                "phase": RECORD_PHASES.get(recording.get("phase"), "UNAVAILABLE"),
                "phase_age": age(recording.get("seen")),
                "task_name": recording.get("task_name", ""),
                "typed_status_available": RecordingStatus is not None,
            },
            "events": events,
        }

    def _operation_watchdog(self) -> None:
        operation = self.operation
        if not operation:
            return
        op = operation.snapshot()
        if not op["running"]:
            return
        ros = self.ros_snapshot()
        if ros["estop"]["active"]:
            operation.cancel("E-stop 감지")
            return
        if ros["joint_state_age"] is None or ros["joint_state_age"] >= FRESH_SECONDS:
            operation.cancel("joint feedback 단절")
            return
        dxl = ros["dxl"]
        if dxl.get("age") is None or dxl["age"] >= FRESH_SECONDS:
            operation.cancel("DXL 상태 단절")
            return
        if dxl.get("comm_state") not in {None, 0} or dxl.get("hardware_errors"):
            operation.cancel("DXL 통신/hardware fault")
            return
        if (op.get("started_age") or 0.0) > 60.0:
            operation.cancel("자세 이동 전체 timeout")
            return
        started = operation.started_at or monotonic()
        if any((self.topic_seen.get(topic) or 0.0) > started + 0.2 for topic in COMMAND_TOPICS):
            operation.cancel("외부 trajectory command stream 재개")


class ActionRequest(BaseModel):
    model: str = ""


class InterventionRequest(BaseModel):
    model: str = "a3"
    outcome: str = ""


def run_command(command: list[str], timeout: float = 15.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout, check=False)


def s6_tool(name: str) -> str:
    """Resolve s6-overlay tools, which are not on PATH in every shell."""
    matches = sorted(glob.glob(f"/package/admin/s6-*/command/{name}"))
    return matches[0] if matches else name


_service_cache_lock = threading.Lock()
_service_cache: tuple[float, dict[str, dict[str, Any]]] = (0.0, {})


def service_statuses(force: bool = False) -> dict[str, dict[str, Any]]:
    global _service_cache
    with _service_cache_lock:
        if not force and monotonic() - _service_cache[0] < 0.8:
            return {key: dict(value) for key, value in _service_cache[1].items()}
        statuses: dict[str, dict[str, Any]] = {}
        for role, name in SERVICES.items():
            service_dir = Path("/run/service") / name
            if not service_dir.exists():
                statuses[role] = {"name": name, "state": "unknown", "pid": None, "uptime_s": None, "raw": "not registered"}
                continue
            result = run_command([s6_tool("s6-svstat"), str(service_dir)], timeout=3.0)
            statuses[role] = {"name": name, **parse_s6_status((result.stdout or result.stderr).strip())}
        _service_cache = (monotonic(), statuses)
        return {key: dict(value) for key, value in statuses.items()}


def log_tail(service_name: str, lines: int = 180) -> list[str]:
    path = LOG_ROOT / service_name / "current"
    try:
        raw = path.read_bytes()[-200_000:].decode(errors="replace").splitlines()
    except OSError:
        return []
    cleaned = [re.sub(r"^@[0-9a-f]{24}\s*", "", line) for line in raw]
    return cleaned[-lines:]


rclpy.init()
node = BringupMonitor()
operation = OperationManager(node.event)
coordinator = ControlCoordinator()
intervention = InterventionManager()
node.set_operation_manager(operation)
executor = SingleThreadedExecutor()
executor.add_node(node)
ros_thread = threading.Thread(target=executor.spin, name="bringup-control-ros", daemon=True)
ros_thread.start()

app = FastAPI(title="AI Worker Intervention Controller")


def selected_models() -> tuple[str, str]:
    return read_selected(ROBOT_TYPE_FILE, "f2"), read_selected(LEADER_TYPE_FILE, "a3")


def build_snapshot() -> dict[str, Any]:
    follower_model, leader_model = selected_models()
    services = service_statuses()
    ros = node.ros_snapshot()
    follower_managed = services["follower"]["state"] == "up"
    leader_managed = services["leader"]["state"] == "up"
    controller_ready = all(value is not None and value < FRESH_SECONDS for value in ros["controller_ages"].values())
    feedback_fresh = ros["joint_state_age"] is not None and ros["joint_state_age"] < FRESH_SECONDS
    controller_publishers = all(ros["publisher_counts"].get(topic, 0) > 0 for topic in CONTROLLER_TOPICS)
    follower_runtime = follower_managed or controller_publishers or ros["publisher_counts"].get("/ffw_follower/dxl_state", 0) > 0
    command_recent = any(value is not None and value < COMMAND_QUIET_SECONDS for value in ros["command_ages"].values())
    command_publishers = any(ros["publisher_counts"].get(topic, 0) > 0 for topic in COMMAND_TOPICS)
    command_active = command_recent or command_publishers
    leader_runtime = leader_managed or ros["publisher_counts"].get("/leader/joint_states", 0) > 0
    leader_stream = leader_runtime and command_active
    dxl = ros["dxl"]
    dxl_ok = (
        dxl.get("age") is not None
        and dxl["age"] < FRESH_SECONDS
        and dxl.get("comm_state") == 0
        and not dxl.get("hardware_errors")
    )
    critical_reasons: list[str] = []
    follower_uptime = services["follower"].get("uptime_s") or (STARTUP_GRACE_SECONDS + 1 if follower_runtime else 0)
    if ros["estop"]["active"]:
        critical_reasons.append("물리 E-stop 신호가 활성화되었습니다.")
    estop_signal_ready = (
        not ros["estop"]["configured"]
        or (
            ros["estop"]["signal_seen"]
            and ros["estop"]["age"] is not None
            and ros["estop"]["age"] < FRESH_SECONDS
            and not ros["estop"]["active"]
        )
    )
    if follower_runtime and follower_uptime > STARTUP_GRACE_SECONDS:
        if not feedback_fresh:
            critical_reasons.append("Follower 제어 중 /joint_states feedback이 끊겼습니다.")
        if dxl.get("age") is None or dxl["age"] >= FRESH_SECONDS:
            critical_reasons.append("Follower 제어 중 DXL 상태가 끊겼습니다.")
    dxl_is_current = dxl.get("age") is not None and dxl["age"] < FRESH_SECONDS
    if dxl_is_current and dxl.get("comm_state") not in {None, 0}:
        critical_reasons.append(f"DXL 통신 오류: comm_state={dxl.get('comm_state')}")
    if dxl_is_current and dxl.get("hardware_errors"):
        critical_reasons.append(f"DXL Hardware Error: {dxl['hardware_errors']}")
    op = operation.snapshot()
    control = coordinator.snapshot()
    last_pose_result = op.get("last_result") or {}
    last_control_result = control.get("last") or {}
    intervention_state = intervention.snapshot()
    bring_down_complete = bool(
        follower_runtime
        and follower_model == "f2"
        and last_pose_result.get("kind") == "F2 bring-down"
        and last_pose_result.get("ok")
        and (
            not last_control_result.get("finished_at")
            or last_control_result["finished_at"] <= last_pose_result.get("finished_at", "")
        )
    )
    mode = operating_mode(
        leader_up=leader_runtime,
        inference_present=bool(ros["inference"]["present"]),
        inference_active=bool(ros["inference"]["active"]),
        inference_phase=str(ros["inference"]["phase"]),
        intervention_active=bool(intervention_state["active"]),
    )
    safety = SafetyState(
        physical_control_enabled=PHYSICAL_CONTROL_ENABLED,
        estop_active=bool(ros["estop"]["active"]),
        estop_signal_ready=estop_signal_ready,
        critical_fault=bool(critical_reasons),
        inference_present=bool(ros["inference"]["present"]),
        inference_active=bool(ros["inference"]["active"]),
        leader_up=leader_runtime,
        leader_managed=leader_managed,
        leader_stream_active=leader_stream,
        command_stream_active=command_active,
        follower_up=follower_runtime,
        follower_managed=follower_managed,
        controllers_ready=controller_ready,
        feedback_fresh=feedback_fresh,
        dxl_ok=dxl_ok,
        operation_running=bool(op["running"]),
        control_busy=bool(control["running"]),
        selected_follower=follower_model,
    )
    gates = {action: gate_action(action, safety).to_dict() for action in (
        "follower_start", "follower_stop", "leader_start", "leader_stop", "bring_down", "recover_init", "operation_cancel"
    )}
    combined_logs = log_tail(SERVICES["follower"]) + log_tail(SERVICES["leader"]) + op["logs"]
    return {
        "time": utc_now(),
        "models": {"followers": FOLLOWER_MODELS, "leaders": LEADER_MODELS},
        "selected": {"follower": follower_model, "leader": leader_model},
        "services": {
            "follower": {**services["follower"], "runtime_active": follower_runtime, "managed": follower_managed},
            "leader": {**services["leader"], "runtime_active": leader_runtime, "managed": leader_managed},
        },
        "ros": ros,
        "operation": op,
        "control": control,
        "intervention": intervention_state,
        "mode": mode,
        "shutdown": {
            "bring_down_recommended": bool(follower_runtime and follower_model == "f2"),
            "bring_down_complete": bring_down_complete,
            "emergency_stop_available": True,
        },
        "safety": {
            "physical_control_enabled": PHYSICAL_CONTROL_ENABLED,
            "token_configured": bool(CONTROL_TOKEN),
            "critical": bool(critical_reasons),
            "critical_reasons": critical_reasons,
            "controllers_ready": controller_ready,
            "feedback_fresh": feedback_fresh,
            "dxl_ok": dxl_ok,
            "command_stream_active": command_active,
            "command_recent": command_recent,
            "command_publishers_present": command_publishers,
            "leader_stream_active": leader_stream,
            "follower_runtime_active": follower_runtime,
            "leader_runtime_active": leader_runtime,
            "teleop_active": leader_runtime and (leader_stream or (ros["leader_joint_state_age"] is not None and ros["leader_joint_state_age"] < FRESH_SECONDS)),
        },
        "gates": gates,
        "diagnostics": diagnose_lines(combined_logs),
    }


def authorize(action: str, token: str | None) -> None:
    if CONTROL_TOKEN and token != CONTROL_TOKEN:
        raise HTTPException(401, "제어 token이 일치하지 않습니다.")


def require_gate(action: str) -> dict[str, Any]:
    snapshot = build_snapshot()
    gate = snapshot["gates"][action]
    if not gate["ok"]:
        raise HTTPException(409, {"message": "안전 조건을 통과하지 못했습니다.", **gate})
    return snapshot


def change_service(role: str, up: bool) -> None:
    service = SERVICES[role]
    result = run_command([s6_tool("s6-rc"), "-u" if up else "-d", "change", service], timeout=20.0)
    if result.returncode != 0:
        evidence = (result.stderr or result.stdout).strip()
        raise HTTPException(500, f"{service} {'시작' if up else '종료'} 요청 실패: {evidence}")
    global _service_cache
    with _service_cache_lock:
        _service_cache = (0.0, {})


def require_arbiter_action(
    action: str,
    *,
    timeout: float = 2.0,
    retry_readiness: bool = False,
) -> str:
    """Require one command-routing transition or stop the hand-off."""
    deadline = monotonic() + timeout
    last_message = "not attempted"
    while True:
        remaining = max(0.05, deadline - monotonic())
        success, message = node.call_arbiter(action, timeout=min(1.0, remaining))
        if success:
            return message
        last_message = message
        if not retry_readiness or monotonic() >= deadline:
            break
        if "fault latched" in message or "publisher conflict" in message:
            break
        time.sleep(0.05)
    raise RuntimeError(f"command-source arbiter {action} 실패: {last_message}")


def best_effort_arbiter_action(action: str) -> None:
    success, message = node.call_arbiter(action, timeout=1.0)
    if not success:
        node.event("danger", f"command-source arbiter {action} 확인 실패: {message}")


def create_intervention_preflight_lease(model: str) -> str:
    """Create a short-lived, one-shot exception for the s6 Leader preflight."""
    clear_intervention_preflight_lease()
    expires_epoch = int(time.time()) + 20
    nonce = secrets.token_hex(16)
    temporary = INTERVENTION_PREFLIGHT_LEASE.with_name(
        f"{INTERVENTION_PREFLIGHT_LEASE.name}.{os.getpid()}.tmp"
    )
    temporary.write_text(f"v1 {expires_epoch} {model} {nonce}\n", encoding="utf-8")
    temporary.chmod(0o600)
    os.replace(temporary, INTERVENTION_PREFLIGHT_LEASE)
    return nonce


def clear_intervention_preflight_lease() -> None:
    for path in (INTERVENTION_PREFLIGHT_LEASE, INTERVENTION_PREFLIGHT_ACK):
        try:
            path.unlink()
        except FileNotFoundError:
            pass


def wait_for_intervention_preflight_ack(nonce: str, timeout: float = 8.0) -> bool:
    """Wait until s6 proves that it consumed and validated this exact lease."""
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        try:
            acknowledged = INTERVENTION_PREFLIGHT_ACK.read_text(encoding="utf-8").strip()
        except FileNotFoundError:
            time.sleep(0.05)
            continue
        try:
            INTERVENTION_PREFLIGHT_ACK.unlink()
        except FileNotFoundError:
            pass
        return secrets.compare_digest(acknowledged, nonce)
    return False


def wait_for_snapshot(predicate: Callable[[dict[str, Any]], bool], timeout: float) -> dict[str, Any]:
    deadline = monotonic() + timeout
    latest = build_snapshot()
    while monotonic() < deadline:
        latest = build_snapshot()
        if predicate(latest):
            return latest
        time.sleep(0.2)
    return latest


def intervention_start_gate(snapshot: dict[str, Any]) -> dict[str, Any]:
    safety = snapshot["safety"]
    ros = snapshot["ros"]
    gate = gate_intervention_start(
        SafetyState(
            physical_control_enabled=bool(safety["physical_control_enabled"]),
            estop_active=bool(ros["estop"]["active"]),
            estop_signal_ready=(
                not ros["estop"]["configured"]
                or (
                    ros["estop"]["signal_seen"]
                    and ros["estop"]["age"] is not None
                    and ros["estop"]["age"] < FRESH_SECONDS
                    and not ros["estop"]["active"]
                )
            ),
            critical_fault=bool(safety["critical"]),
            inference_present=bool(ros["inference"]["present"]),
            inference_active=bool(ros["inference"]["active"]),
            leader_up=bool(safety["leader_runtime_active"]),
            leader_stream_active=bool(safety["leader_stream_active"]),
            # A paused policy keeps publishers connected. Intervention only
            # requires the actual command stream to have gone quiet.
            command_stream_active=bool(safety["command_recent"]),
            follower_up=bool(safety["follower_runtime_active"]),
            controllers_ready=bool(safety["controllers_ready"]),
            feedback_fresh=bool(safety["feedback_fresh"]),
            dxl_ok=bool(safety["dxl_ok"]),
            operation_running=bool(snapshot["operation"]["running"]),
            # The intervention endpoint holds the coordinator lock while it
            # evaluates this gate, so its own transition is not a conflict.
            control_busy=False,
            selected_follower=str(snapshot["selected"]["follower"]),
        ),
        str(ros["inference"]["phase"]),
    )
    return gate.to_dict()


def intervention_watchdog_loop() -> None:
    """Fail closed if safety or inference ownership changes mid-intervention."""
    while True:
        time.sleep(0.25)
        current = intervention.snapshot()["current"]
        if not current or current.get("phase") not in {"PREPARING", "READY", "RECORDING"}:
            continue
        snapshot = build_snapshot()
        reasons: list[str] = []
        if snapshot["ros"]["estop"]["active"]:
            reasons.append("E-stop 감지")
        if snapshot["ros"]["inference"]["phase"] != "PAUSED":
            reasons.append("Inference PAUSED 상태 이탈")
        if current.get("phase") in {"READY", "RECORDING"}:
            if not snapshot["safety"]["feedback_fresh"]:
                reasons.append("Follower joint feedback 단절")
            if not snapshot["safety"]["dxl_ok"]:
                reasons.append("Follower DXL 상태 이상")
        if not reasons or not coordinator.try_acquire():
            continue
        error = " · ".join(reasons)
        announced = False
        try:
            coordinator.announce(
                "intervention_end",
                "safety-watchdog",
                str(current.get("leader_model", "")),
            )
            announced = True
            best_effort_arbiter_action("select_none")
            if snapshot["services"]["leader"]["managed"]:
                change_service("leader", False)
            best_effort_arbiter_action("disable_intervention")
            intervention.transition("PAUSED_SAFE", error)
            node.event("danger", f"Intervention watchdog 안전 차단: {error}")
        except Exception as exc:
            intervention.transition("PAUSED_SAFE", f"{error} · Leader 종료 실패: {exc}")
            node.event("danger", f"Intervention watchdog Leader 종료 실패: {exc}")
        finally:
            if announced:
                coordinator.finish(False, error)
            else:
                coordinator.release()


intervention_watchdog_thread = threading.Thread(
    target=intervention_watchdog_loop,
    name="intervention-safety-watchdog",
    daemon=True,
)
intervention_watchdog_thread.start()


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC / "index.html", headers={"Cache-Control": "no-store"})


@app.get("/static/{filename}")
def static_file(filename: str) -> FileResponse:
    if filename not in {"app.js", "styles.css"}:
        raise HTTPException(404)
    return FileResponse(STATIC / filename, headers={"Cache-Control": "no-store"})


@app.get("/api/state")
def state() -> dict[str, Any]:
    return build_snapshot()


@app.get("/api/events")
async def events() -> StreamingResponse:
    async def stream():
        while True:
            yield "data: " + json.dumps(build_snapshot(), separators=(",", ":")) + "\n\n"
            await asyncio.sleep(0.5)
    return StreamingResponse(stream(), media_type="text/event-stream", headers={"Cache-Control": "no-cache"})


@app.get("/api/logs")
def logs(source: str = Query("follower", pattern="^(follower|leader|operation)$")) -> dict[str, Any]:
    if source == "operation":
        lines = operation.snapshot()["logs"]
    else:
        lines = log_tail(SERVICES[source], 400)
    return {"source": source, "lines": lines, "diagnostics": diagnose_lines(lines)}


@app.get("/api/intervention/state")
def intervention_state() -> dict[str, Any]:
    snapshot = build_snapshot()
    return {
        **snapshot["intervention"],
        "inference_phase": snapshot["ros"]["inference"]["phase"],
        "record_phase": snapshot["ros"]["recording"]["phase"],
        "leader_up": snapshot["safety"]["leader_runtime_active"],
        "command_recent": snapshot["safety"]["command_recent"],
    }


@app.post("/api/intervention/{action}")
def intervention_action(
    action: str,
    body: InterventionRequest,
    http_request: Request,
    x_control_token: str | None = Header(default=None),
    x_control_client: str | None = Header(default=None),
) -> dict[str, Any]:
    if action not in {"start", "recording", "end", "complete", "abort", "reset"}:
        raise HTTPException(404, "지원하지 않는 Intervention 작업입니다.")
    authorize("intervention", x_control_token)
    actor = (x_control_client or (http_request.client.host if http_request.client else "browser")).strip() or "browser"

    if action == "recording":
        current = intervention.snapshot()["current"]
        if not current or current.get("phase") not in {"READY", "RECORDING"}:
            raise HTTPException(409, "Leader hand-off가 완료된 Intervention이 없습니다.")
        observed = wait_for_snapshot(
            lambda item: item["ros"]["recording"]["phase"] == "RECORDING",
            4.0,
        )
        if observed["ros"]["recording"]["typed_status_available"] and observed["ros"]["recording"]["phase"] != "RECORDING":
            raise HTTPException(409, "Cyclo recording이 RECORDING 상태로 확인되지 않았습니다.")
        intervention.transition("RECORDING")
        node.event("warning", f"Intervention 실패 구간 녹화 시작 · {actor}")
        return {"ok": True, **intervention.snapshot()}

    if action == "complete":
        current = intervention.snapshot()["current"]
        if not current or current.get("phase") != "READY_TO_RESUME":
            raise HTTPException(409, "Resume 완료를 확인할 Intervention이 없습니다.")
        snapshot = wait_for_snapshot(
            lambda item: item["ros"]["inference"]["phase"] == "INFERENCING",
            5.0,
        )
        if snapshot["ros"]["inference"]["phase"] != "INFERENCING":
            raise HTTPException(409, "Inference가 INFERENCING으로 복귀한 것이 확인되지 않았습니다.")
        if snapshot["safety"]["leader_runtime_active"]:
            raise HTTPException(409, "Leader가 남아 있어 Intervention을 완료할 수 없습니다.")
        intervention.clear()
        node.event("info", f"Intervention 종료, Inference 복귀 완료 · {actor}")
        return {"ok": True, **intervention.snapshot()}

    if not coordinator.try_acquire():
        raise HTTPException(409, "다른 사용자의 로봇 제어 요청이 처리 중입니다.")

    announced = False
    ok = False
    error = ""
    try:
        if action == "start":
            model = body.model.strip().lower() or "a3"
            if model not in LEADER_MODELS:
                raise HTTPException(400, "지원하지 않는 Leader 모델입니다.")
            if intervention.snapshot()["active"]:
                raise HTTPException(409, "Intervention이 이미 진행 중입니다.")
            snapshot = wait_for_snapshot(
                lambda item: (
                    item["ros"]["inference"]["phase"] == "PAUSED"
                    and not item["safety"]["command_recent"]
                ),
                COMMAND_QUIET_SECONDS + 3.0,
            )
            gate = intervention_start_gate(snapshot)
            if not gate["ok"]:
                raise HTTPException(409, {"message": "Intervention 안전 조건을 통과하지 못했습니다.", **gate})
            coordinator.announce("intervention_start", actor, model)
            announced = True
            intervention.begin(actor, model)
            require_arbiter_action("prepare_intervention")
            LEADER_TYPE_FILE.write_text(model + "\n", encoding="utf-8")
            lease_nonce = create_intervention_preflight_lease(model)
            try:
                change_service("leader", True)
                if not wait_for_intervention_preflight_ack(lease_nonce):
                    raise RuntimeError(
                        "Leader s6 preflight가 Intervention hand-off 허가를 확인하지 못했습니다."
                    )
            except Exception:
                clear_intervention_preflight_lease()
                raise
            ready = wait_for_snapshot(
                lambda item: (
                    item["safety"]["leader_runtime_active"]
                    and item["ros"]["leader_joint_state_age"] is not None
                    and item["ros"]["leader_joint_state_age"] < FRESH_SECONDS
                ),
                20.0,
            )
            if not (
                ready["safety"]["leader_runtime_active"]
                and ready["ros"]["leader_joint_state_age"] is not None
                and ready["ros"]["leader_joint_state_age"] < FRESH_SECONDS
            ):
                raise RuntimeError("Leader joint feedback이 제한 시간 안에 준비되지 않았습니다.")
            if (
                ready["ros"]["estop"]["active"]
                or ready["ros"]["inference"]["phase"] != "PAUSED"
                or not ready["safety"]["feedback_fresh"]
                or not ready["safety"]["dxl_ok"]
            ):
                raise RuntimeError("Leader 준비 중 안전 상태가 변경되어 조작권 전환을 중단했습니다.")
            require_arbiter_action(
                "select_teleop",
                timeout=5.0,
                retry_readiness=True,
            )
            intervention.transition("READY")
            node.event("warning", f"Intervention 조작권을 {LEADER_MODELS[model]['label']} Leader로 전환 · {actor}")

        elif action == "end":
            current = intervention.snapshot()["current"]
            if not current or current.get("phase") not in {"READY", "RECORDING"}:
                raise HTTPException(409, "종료할 Intervention이 없습니다.")
            coordinator.announce("intervention_end", actor, str(current.get("leader_model", "")))
            announced = True
            intervention.transition("RETURNING")
            require_arbiter_action("select_none")
            snapshot = build_snapshot()
            if snapshot["safety"]["leader_runtime_active"] and not snapshot["services"]["leader"]["managed"]:
                raise RuntimeError("Leader가 UI 밖에서 실행되어 안전하게 종료할 수 없습니다.")
            if snapshot["services"]["leader"]["managed"]:
                change_service("leader", False)
            quiet = wait_for_snapshot(
                lambda item: not item["safety"]["leader_runtime_active"] and not item["safety"]["command_recent"],
                10.0,
            )
            if quiet["safety"]["leader_runtime_active"] or quiet["safety"]["command_recent"]:
                raise RuntimeError("Leader 종료 후 command quiet 상태를 확인하지 못했습니다.")
            require_arbiter_action("disable_intervention")
            intervention.transition("READY_TO_RESUME")
            node.event("warning", f"Intervention Leader 종료, Inference Resume 대기 · {actor}")

        elif action == "reset":
            current = intervention.snapshot()["current"]
            if not current:
                return {"ok": True, **intervention.snapshot()}
            snapshot = build_snapshot()
            gate = gate_intervention_reset(
                str(current.get("phase", "")),
                leader_up=bool(snapshot["safety"]["leader_runtime_active"]),
                command_recent=bool(snapshot["safety"]["command_recent"]),
                recording_phase=str(snapshot["ros"]["recording"]["phase"]),
            )
            if not gate.ok:
                raise HTTPException(
                    409,
                    {
                        "message": "Intervention 초기화 안전 조건을 통과하지 못했습니다.",
                        **gate.to_dict(),
                    },
                )
            coordinator.announce(
                "intervention_reset",
                actor,
                str(current.get("leader_model", "")),
            )
            announced = True
            require_arbiter_action("disable_intervention")
            intervention.clear()
            node.event("warning", f"Intervention 종료, 새 Inference 시작 대기 · {actor}")

        else:  # abort: fail closed; never resume inference here.
            current = intervention.snapshot()["current"]
            if not current:
                return {"ok": True, **intervention.snapshot()}
            coordinator.announce("intervention_end", actor, str(current.get("leader_model", "")))
            announced = True
            best_effort_arbiter_action("select_none")
            snapshot = build_snapshot()
            if snapshot["services"]["leader"]["managed"]:
                change_service("leader", False)
            wait_for_snapshot(
                lambda item: not item["safety"]["leader_runtime_active"] and not item["safety"]["command_recent"],
                10.0,
            )
            best_effort_arbiter_action("disable_intervention")
            intervention.transition("PAUSED_SAFE", "사용자 중단 — Inference는 자동 재개하지 않았습니다.")
            node.event("danger", f"Intervention 안전 중단, Inference PAUSED 유지 · {actor}")

        ok = True
        return {"ok": True, **intervention.snapshot()}
    except Exception as exc:
        error = str(exc.detail) if isinstance(exc, HTTPException) else str(exc)
        current = intervention.snapshot()["current"]
        if current:
            best_effort_arbiter_action("select_none")
            try:
                snapshot = build_snapshot()
                if snapshot["services"]["leader"]["managed"]:
                    change_service("leader", False)
            except Exception:
                pass
            best_effort_arbiter_action("disable_intervention")
            intervention.transition("PAUSED_SAFE", error)
        node.event("danger", f"Intervention {action} 실패 · {actor}: {error}")
        if isinstance(exc, HTTPException):
            raise
        raise HTTPException(500, error) from exc
    finally:
        if announced:
            coordinator.finish(ok, error)
        else:
            coordinator.release()


@app.post("/api/actions/{action}")
def action_endpoint(
    action: str,
    body: ActionRequest,
    http_request: Request,
    x_control_token: str | None = Header(default=None),
    x_control_client: str | None = Header(default=None),
) -> dict[str, Any]:
    if action not in {"follower_start", "follower_stop", "leader_start", "leader_stop", "bring_down", "recover_init", "operation_cancel"}:
        raise HTTPException(404, "지원하지 않는 작업입니다.")
    authorize(action, x_control_token)
    if action == "operation_cancel":
        operation.cancel("사용자 즉시 중단")
        return {"ok": True, "action": action, "warnings": []}

    model = body.model.strip().lower()
    if action == "follower_start" and model not in FOLLOWER_MODELS:
        raise HTTPException(400, "지원하지 않는 Follower 모델입니다.")
    if action == "leader_start" and model not in LEADER_MODELS:
        raise HTTPException(400, "지원하지 않는 Leader 모델입니다.")
    if action == "recover_init" and (
        model not in FOLLOWER_MODELS or not FOLLOWER_MODELS[model].get("recover_init")
    ):
        raise HTTPException(400, "선택한 Follower 모델은 init pose 복구를 지원하지 않습니다.")

    if not coordinator.try_acquire():
        current = coordinator.snapshot()["current"] or {}
        actor = current.get("actor", "다른 사용자")
        phase = current.get("phase", "RUNNING")
        raise HTTPException(409, {
            "message": f"{actor}의 제어 요청이 처리 중입니다.",
            "failures": [f"다른 제어 요청이 {phase} 상태입니다. 완료 후 다시 시도하십시오."],
        })

    announced = False
    ok = False
    error = ""
    actor = (x_control_client or (http_request.client.host if http_request.client else "browser")).strip() or "browser"
    try:
        snapshot = require_gate(action)
        if action == "recover_init" and model != snapshot["selected"]["follower"]:
            raise HTTPException(
                409,
                f"복구 요청 모델({model.upper()})과 실행 중인 Follower 모델"
                f"({snapshot['selected']['follower'].upper()})이 일치하지 않습니다.",
            )
        coordinator.announce(action, actor, model)
        announced = True

        if action == "follower_start":
            ROBOT_TYPE_FILE.write_text(model + "\n", encoding="utf-8")
            change_service("follower", True)
            node.event("warning", f"{FOLLOWER_MODELS[model]['label']} Follower bring-up 요청 · {actor}")
        elif action == "follower_stop":
            change_service("follower", False)
            node.event("warning", f"Follower control stack 종료 요청 · {actor}")
        elif action == "leader_start":
            LEADER_TYPE_FILE.write_text(model + "\n", encoding="utf-8")
            change_service("leader", True)
            node.event("warning", f"{LEADER_MODELS[model]['label']} Leader bring-up 요청 · {actor}")
        elif action == "leader_stop":
            change_service("leader", False)
            node.event("warning", f"Leader 종료 요청 · {actor} — command stream quiet 상태를 기다립니다.")
        elif action == "bring_down":
            operation.start("F2 bring-down", ["ros2", "launch", "ffw_bringup", "f2_bring_down.launch.py"])
        else:
            recovery_yaml = AI_WORKER_ROOT / FOLLOWER_MODELS[model]["recovery_yaml"]
            if not recovery_yaml.is_file():
                raise HTTPException(500, f"복구 YAML을 찾을 수 없습니다: {recovery_yaml}")
            operation.start(
                f"{FOLLOWER_MODELS[model]['label']} init pose 복구",
                [
                    "python3",
                    str(ROOT / "recovery_runner.py"),
                    "--model",
                    model,
                    "--config",
                    str(recovery_yaml),
                ],
            )
        ok = True
        return {"ok": True, "action": action, "warnings": snapshot["gates"][action]["warnings"]}
    except Exception as exc:
        error = str(exc.detail) if isinstance(exc, HTTPException) else str(exc)
        if announced:
            node.event("danger", f"{action} 요청 실패 · {actor}: {error}")
        raise
    finally:
        if announced:
            coordinator.finish(ok, error)
        else:
            coordinator.release()


@app.on_event("shutdown")
def shutdown() -> None:
    operation.cancel("웹 supervisor 종료")
    if rclpy.ok():
        rclpy.shutdown()
    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
