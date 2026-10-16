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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

# The Cyclo interface set evolves independently from AI Worker.  Import each
# status type separately so removal of an optional telemetry message (for
# example ``RtcStatus``) cannot disable the safety-critical inference and
# recording subscriptions as collateral damage.
try:
    from interfaces.msg import InferenceStatus
except ImportError:  # The policy interfaces package may live only in another container.
    InferenceStatus = None

try:
    from interfaces.msg import RecordingStatus
except ImportError:  # The policy interfaces package may live only in another container.
    RecordingStatus = None

try:
    from interfaces.msg import RtcStatus
except ImportError:  # Optional RTC telemetry is absent on non-RTC Cyclo branches.
    RtcStatus = None

from logic import (
    FOLLOWER_MODELS,
    LEADER_MODELS,
    SafetyState,
    diagnose_lines,
    gate_action,
    gate_arbiter_fault_reset,
    gate_intervention_preflight,
    gate_intervention_reset,
    gate_intervention_start,
    gate_leader_standby_start,
    gate_paused_pose_move,
    intervention_resume_confirmed,
    operating_mode,
    parse_s6_status,
    sustained_failure_elapsed,
)
from recovery_config import load_custom_recovery_groups


ROOT = Path(__file__).resolve().parent
AI_WORKER_ROOT = ROOT.parents[1]
STATIC = ROOT / "static"
PHYSICAL_CONTROL_ENABLED = os.environ.get("ENABLE_BRINGUP_CONTROL", "false").lower() in {"1", "true", "yes"}
CONTROL_TOKEN = os.environ.get("BRINGUP_CONTROL_TOKEN", "").strip()
ESTOP_TOPIC = os.environ.get("AI_WORKER_ESTOP_TOPIC", "").strip()
FRESH_SECONDS = float(os.environ.get("BRINGUP_FRESH_SECONDS", "2.0"))
STANDBY_UNHEALTHY_GRACE_SECONDS = float(
    os.environ.get("BRINGUP_STANDBY_UNHEALTHY_GRACE_SECONDS", "3.0")
)
COMMAND_QUIET_SECONDS = float(os.environ.get("BRINGUP_COMMAND_QUIET_SECONDS", "2.0"))
STARTUP_GRACE_SECONDS = float(os.environ.get("BRINGUP_STARTUP_GRACE_SECONDS", "20.0"))
LOG_ROOT = Path(os.environ.get("BRINGUP_LOG_ROOT", "/var/log"))
ROBOT_TYPE_FILE = Path("/run/robot_type")
LEADER_TYPE_FILE = Path("/run/leader_type")
INTERVENTION_PREFLIGHT_LEASE = Path("/run/ai_worker_intervention_preflight")
INTERVENTION_PREFLIGHT_ACK = Path("/run/ai_worker_intervention_preflight.ack")
LEADER_STANDBY_STATE_FILE = Path("/run/ai_worker_leader_standby.json")
INFERENCE_STATUS_STATE_FILE = Path("/run/ai_worker_inference_status.json")
INFERENCE_STATUS_RESTORE_SECONDS = float(
    os.environ.get("BRINGUP_INFERENCE_STATUS_RESTORE_SECONDS", "60.0")
)

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
INTERVENTION_LEADERS = {"a3", "mini"}
INFERENCE_PHASES = {
    0: "READY",
    1: "LOADING",
    2: "INFERENCING",
    3: "PAUSED",
    4: "SYNCING",
}
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
        "custom_pose_move": "MOVING_TO_USER_POSE",
        "intervention_start": "HANDING_OVER",
        "intervention_end": "RETURNING_CONTROL",
        "intervention_reset": "RESETTING",
        "intervention_reset_fault": "RESETTING_FAULT",
        "leader_standby_start": "WARMING_LEADER",
        "leader_standby_stop": "STOPPING_STANDBY",
        "manual_teleop_start": "STARTING_MANUAL_TELEOP",
        "manual_teleop_stop": "STOPPING_MANUAL_TELEOP",
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

    def begin(
        self,
        actor: str,
        model: str,
        *,
        prewarmed: bool = False,
        recording_continued: bool = False,
    ) -> None:
        with self._lock:
            if self._current is not None:
                raise RuntimeError("다른 사용자의 Intervention이 이미 진행 중입니다.")
            self._current = {
                "phase": "PREPARING",
                "actor": actor[:80],
                "leader_model": model,
                "prewarmed": bool(prewarmed),
                "recording_continued": bool(recording_continued),
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


class LeaderStandbyManager:
    """Persist ownership of one isolated, warm Leader process."""

    def __init__(self, state_file: Path) -> None:
        self._lock = threading.RLock()
        self._state_file = state_file
        self._current: dict[str, Any] | None = None
        self._last: dict[str, Any] | None = None
        try:
            payload = json.loads(state_file.read_text(encoding="utf-8"))
            if (
                isinstance(payload, dict)
                and payload.get("model") == "mini"
                and payload.get("phase") in {"WARMING", "READY", "IN_USE"}
            ):
                self._current = payload
        except (OSError, json.JSONDecodeError):
            pass

    def _persist(self) -> None:
        if self._current is None:
            try:
                self._state_file.unlink()
            except FileNotFoundError:
                pass
            return
        temporary = self._state_file.with_name(
            f"{self._state_file.name}.{os.getpid()}.tmp"
        )
        temporary.write_text(
            json.dumps(self._current, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.chmod(0o600)
        os.replace(temporary, self._state_file)

    def begin(self, actor: str, model: str) -> None:
        with self._lock:
            if self._current is not None:
                raise RuntimeError("Leader standby가 이미 준비 중이거나 실행 중입니다.")
            self._current = {
                "phase": "WARMING",
                "actor": actor[:80],
                "model": model,
                "started_at": utc_now(),
                "updated_at": utc_now(),
                "error": "",
            }
            self._persist()

    def transition(self, phase: str, error: str = "") -> None:
        with self._lock:
            if self._current is None:
                raise RuntimeError("진행 중인 Leader standby가 없습니다.")
            self._current["phase"] = phase
            self._current["updated_at"] = utc_now()
            self._current["error"] = error[:500]
            self._persist()

    def clear(self, error: str = "") -> None:
        with self._lock:
            if self._current is not None:
                self._last = {
                    **self._current,
                    "finished_at": utc_now(),
                    "error": error[:500] or self._current.get("error", ""),
                }
            self._current = None
            self._persist()

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
        try:
            restored = json.loads(
                INFERENCE_STATUS_STATE_FILE.read_text(encoding="utf-8")
            )
            restored_age = max(
                0.0,
                time.time() - float(restored.get("stored_at", 0.0)),
            )
            restored_phase = int(restored.get("phase"))
            if (
                restored_phase in INFERENCE_PHASES
                and restored_age <= INFERENCE_STATUS_RESTORE_SECONDS
            ):
                self.inference = {
                    "seen": monotonic(),
                    "phase": restored_phase,
                    "robot_type": str(restored.get("robot_type", "")),
                    "error": str(restored.get("error", "")),
                }
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            pass
        self.recording = {
            "seen": None,
            "phase": None,
            "task_name": "",
            "scope": "",
        }
        self.rtc: dict[str, dict[str, Any]] = {}
        self.arbiter = {
            "seen": None,
            "mode": "UNAVAILABLE",
            "fault": "",
            "publisher_conflict": "",
            "intervention_enabled": False,
            "sources": {},
        }
        self.operation: OperationManager | None = None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        retained_status = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
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
            self.create_subscription(
                InferenceStatus,
                "/task/inference_status",
                self._inference,
                retained_status,
            )
        if RecordingStatus is not None:
            self.create_subscription(RecordingStatus, "/data/recording/status", self._recording, reliable)
        if RtcStatus is not None:
            for backend in ("groot", "lerobot"):
                self.create_subscription(RtcStatus, f"/{backend}/rtc_status", lambda m, b=backend: self._rtc(b, m), reliable)
        self.create_subscription(
            String,
            "/command_source_arbiter/status",
            self._arbiter_status,
            retained_status,
        )
        self.arbiter_clients = {
            action: self.create_client(Trigger, f"/command_source_arbiter/{action}")
            for action in (
                "prepare_intervention",
                "select_none",
                "select_teleop",
                "disable_intervention",
                "release_teleop",
                "clear_fault",
                "get_status",
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
        status = {
            "seen": monotonic(),
            "phase": int(msg.inference_phase),
            "robot_type": str(msg.robot_type),
            "error": str(msg.error),
        }
        with self.lock:
            self.inference = status
        temporary = INFERENCE_STATUS_STATE_FILE.with_name(
            f"{INFERENCE_STATUS_STATE_FILE.name}.{os.getpid()}.tmp"
        )
        try:
            temporary.write_text(
                json.dumps(
                    {
                        "stored_at": time.time(),
                        "phase": status["phase"],
                        "robot_type": status["robot_type"],
                        "error": status["error"],
                    },
                    sort_keys=True,
                ) + "\n",
                encoding="utf-8",
            )
            temporary.chmod(0o600)
            os.replace(temporary, INFERENCE_STATUS_STATE_FILE)
        except OSError as exc:
            self.get_logger().warning(
                f"Failed to persist inference status snapshot: {exc}"
            )

    def _rtc(self, backend: str, msg) -> None:
        with self.lock:
            self.rtc[backend] = {
                "seen": monotonic(),
                "state": int(msg.rtc_state),
                "publish_to_robot": bool(msg.publish_to_robot),
                "fault": str(msg.fault),
            }

    def _recording(self, msg) -> None:
        tags = [str(tag or "") for tag in (msg.task_info.tags or [])]
        scope = next(
            (
                tag.split(":", 1)[1]
                for tag in tags
                if tag.startswith("recording_source:")
            ),
            "",
        )
        with self.lock:
            self.recording = {
                "seen": monotonic(),
                "phase": int(msg.record_phase),
                "task_name": str(msg.task_info.task_name),
                "scope": scope,
            }

    def _arbiter_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warning("Ignoring malformed command-source arbiter status")
            return
        if not isinstance(payload, dict):
            return
        with self.lock:
            self.arbiter = {
                "seen": monotonic(),
                "mode": str(payload.get("mode", "UNAVAILABLE")),
                "fault": str(payload.get("fault", "")),
                "publisher_conflict": str(payload.get("publisher_conflict", "")),
                "intervention_enabled": bool(payload.get("intervention_enabled", False)),
                "sources": payload.get("sources", {})
                if isinstance(payload.get("sources", {}), dict)
                else {},
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
            arbiter = dict(self.arbiter)
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
        phase_active = inference.get("phase") in {1, 2, 3, 4} and (age(inference.get("seen")) or 1e9) < 5.0
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
                "typed_status_available": (
                    InferenceStatus is not None
                    and inference.get("phase") is not None
                ),
            },
            "recording": {
                "phase": RECORD_PHASES.get(recording.get("phase"), "UNAVAILABLE"),
                "phase_age": age(recording.get("seen")),
                "task_name": recording.get("task_name", ""),
                "scope": recording.get("scope", ""),
                "typed_status_available": RecordingStatus is not None,
            },
            "arbiter": {
                **arbiter,
                "age": age(arbiter.get("seen")),
                "available": (
                    arbiter.get("seen") is not None
                    and (age(arbiter.get("seen")) or 1e9) < FRESH_SECONDS
                ),
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
        if op.get("kind") == "F2 사용자 초기 위치 이동":
            inference = ros["inference"]
            arbiter = ros["arbiter"]
            recording = ros["recording"]
            if inference["phase"] not in {"PAUSED", "READY"}:
                operation.cancel("Inference 정지 상태 이탈")
                return
            if recording["phase"] not in {"READY", "SAVING"}:
                operation.cancel("Recording 종료 상태 이탈")
                return
            if (
                not arbiter["available"]
                or arbiter["mode"] != "NONE"
                or arbiter["fault"]
                or arbiter["publisher_conflict"]
                or arbiter["intervention_enabled"]
            ):
                operation.cancel("Command arbiter 안전 상태 이탈")
                return
        started = operation.started_at or monotonic()
        if any((self.topic_seen.get(topic) or 0.0) > started + 0.2 for topic in COMMAND_TOPICS):
            operation.cancel("외부 trajectory command stream 재개")


class ActionRequest(BaseModel):
    model: str = ""


class InterventionRequest(BaseModel):
    model: str = "a3"
    outcome: str = ""


class PoseMoveRequest(BaseModel):
    model: str = "f2"
    left_arm: list[float]
    right_arm: list[float]
    head: list[float]
    lift: list[float]
    duration: float = 8.0


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
shutdown_event = threading.Event()
node = BringupMonitor()
operation = OperationManager(node.event)
coordinator = ControlCoordinator()
intervention = InterventionManager()
leader_standby = LeaderStandbyManager(LEADER_STANDBY_STATE_FILE)
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
    standby_state = leader_standby.snapshot()
    standby_current = standby_state.get("current") or {}
    standby_claimed = bool(
        standby_current.get("model") == leader_model
        and standby_current.get("phase") in {"WARMING", "READY", "IN_USE"}
    )
    standby_ready = bool(
        standby_claimed
        and standby_current.get("phase") == "READY"
        and leader_runtime
        and ros["leader_joint_state_age"] is not None
        and ros["leader_joint_state_age"] < FRESH_SECONDS
    )
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
        leader_standby=standby_claimed,
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
        leader_standby=standby_ready,
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
        "leader_standby": {
            **standby_state,
            "ready": standby_ready,
            "claimed": standby_claimed,
        },
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
            "leader_standby_ready": standby_ready,
            "teleop_active": (
                leader_runtime
                and not standby_claimed
                and (
                    leader_stream
                    or (
                        ros["leader_joint_state_age"] is not None
                        and ros["leader_joint_state_age"] < FRESH_SECONDS
                    )
                )
            ),
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


def create_leader_preflight_lease(model: str, purpose: str) -> str:
    """Create a short-lived lease for one isolated Leader startup."""
    if purpose not in {"intervention", "standby"}:
        raise ValueError(f"unsupported isolated Leader startup purpose: {purpose}")
    clear_intervention_preflight_lease()
    expires_epoch = int(time.time()) + 20
    nonce = secrets.token_hex(16)
    temporary = INTERVENTION_PREFLIGHT_LEASE.with_name(
        f"{INTERVENTION_PREFLIGHT_LEASE.name}.{os.getpid()}.tmp"
    )
    temporary.write_text(
        f"v2 {expires_epoch} {purpose} {model} {nonce}\n",
        encoding="utf-8",
    )
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


def isolated_leader_failures(snapshot: dict[str, Any], model: str) -> list[str]:
    """Describe why a warm Leader is not fresh, isolated, and ready."""
    ros = snapshot["ros"]
    safety = snapshot["safety"]
    teleop_source = ros["arbiter"].get("sources", {}).get("teleop", {})
    failures: list[str] = []
    if model != "mini":
        failures.append(f"지원하지 않는 standby 모델({model or 'unknown'})")
    if snapshot["selected"]["leader"] != model:
        failures.append("선택된 Leader 모델 불일치")
    if not snapshot["services"]["leader"]["managed"]:
        failures.append("Leader service 비관리 상태")
    if not safety["leader_runtime_active"]:
        failures.append("Leader runtime 미감지")
    leader_age = ros["leader_joint_state_age"]
    if leader_age is None:
        failures.append("Leader joint feedback 미수신")
    elif leader_age >= FRESH_SECONDS:
        failures.append(f"Leader joint feedback 지연({leader_age:.3f}s)")
    if not ros["arbiter"]["available"]:
        failures.append("Command arbiter 상태 미수신")
    if ros["arbiter"]["mode"] not in {"NONE", "POLICY", "WAYPOINT"}:
        failures.append(f"Command arbiter 격리 모드 이탈({ros['arbiter']['mode']})")
    if ros["arbiter"]["fault"]:
        failures.append(f"Command arbiter fault({ros['arbiter']['fault']})")
    if ros["arbiter"]["publisher_conflict"]:
        failures.append("최종 command publisher 충돌")
    if ros["arbiter"]["intervention_enabled"]:
        failures.append("Intervention routing lease 잔류")
    if teleop_source.get("ready") is not True:
        failures.append(
            "Teleop source 미준비"
            + (f"({teleop_source.get('reason')})" if teleop_source.get("reason") else "")
        )
    return failures


def isolated_leader_ready(snapshot: dict[str, Any], model: str) -> bool:
    """Return whether a Leader is warm, fresh, isolated, and not in control."""
    return not isolated_leader_failures(snapshot, model)


def manual_teleop_failures(snapshot: dict[str, Any], model: str) -> list[str]:
    """Describe why a controller-owned manual TELEOP session is unhealthy."""
    ros = snapshot["ros"]
    safety = snapshot["safety"]
    arbiter = ros["arbiter"]
    teleop_source = arbiter.get("sources", {}).get("teleop", {})
    failures: list[str] = []
    if model != "mini" or snapshot["selected"]["leader"] != "mini":
        failures.append("Mini Leader 선택 상태 불일치")
    if not snapshot["services"]["leader"]["managed"]:
        failures.append("Leader service 비관리 상태")
    if not safety["leader_runtime_active"]:
        failures.append("Leader runtime 미감지")
    leader_age = ros["leader_joint_state_age"]
    if leader_age is None:
        failures.append("Leader joint feedback 미수신")
    elif leader_age >= FRESH_SECONDS:
        failures.append(f"Leader joint feedback 지연({leader_age:.3f}s)")
    if not safety["controllers_ready"] or not safety["feedback_fresh"]:
        failures.append("Follower controller 또는 feedback 미준비")
    if not safety["dxl_ok"]:
        failures.append("Follower DXL 상태 이상")
    if ros["estop"]["active"]:
        failures.append("E-stop 감지")
    if ros["inference"]["phase"] != "READY":
        failures.append(f"Inference 상태 이탈({ros['inference']['phase']})")
    if ros["recording"]["phase"] not in {"READY", "RECORDING"}:
        failures.append(f"Recording 상태 이탈({ros['recording']['phase']})")
    if not arbiter["available"]:
        failures.append("Command arbiter 상태 미수신")
    if arbiter["mode"] not in {"TELEOP", "WAYPOINT"}:
        failures.append(f"Command arbiter 제어 모드 이탈({arbiter['mode']})")
    if not arbiter["intervention_enabled"]:
        failures.append("Intervention routing lease 해제")
    if arbiter["fault"]:
        failures.append(f"Command arbiter fault({arbiter['fault']})")
    if arbiter["publisher_conflict"]:
        failures.append("최종 command publisher 충돌")
    if teleop_source.get("ready") is not True:
        failures.append(
            "Teleop source 미준비"
            + (f"({teleop_source.get('reason')})" if teleop_source.get("reason") else "")
        )
    return failures


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
            leader_standby=bool(safety.get("leader_standby_ready", False)),
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


def intervention_preflight_gate(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Expose the same fail-closed preflight used by the start endpoint."""
    safety = snapshot["safety"]
    ros = snapshot["ros"]
    gate = gate_intervention_preflight(
        state=SafetyState(
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
            leader_standby=bool(safety.get("leader_standby_ready", False)),
            leader_stream_active=bool(safety["leader_stream_active"]),
            follower_up=bool(safety["follower_runtime_active"]),
            controllers_ready=bool(safety["controllers_ready"]),
            feedback_fresh=bool(safety["feedback_fresh"]),
            dxl_ok=bool(safety["dxl_ok"]),
            operation_running=bool(snapshot["operation"]["running"]),
            control_busy=bool(snapshot["control"]["running"]),
            selected_follower=str(snapshot["selected"]["follower"]),
        ),
        inference_phase=str(ros["inference"]["phase"]),
        inference_status_ready=bool(ros["inference"]["typed_status_available"]),
        recording_phase=str(ros["recording"]["phase"]),
        recording_status_ready=bool(ros["recording"]["typed_status_available"]),
        arbiter_available=bool(ros["arbiter"]["available"]),
        arbiter_mode=str(ros["arbiter"]["mode"]),
        arbiter_fault=str(ros["arbiter"]["fault"]),
        publisher_conflict=str(ros["arbiter"]["publisher_conflict"]),
        intervention_enabled=bool(ros["arbiter"]["intervention_enabled"]),
    )
    return gate.to_dict()


def leader_standby_start_gate(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Validate a warm Leader startup without changing command ownership."""
    safety = snapshot["safety"]
    ros = snapshot["ros"]
    return gate_leader_standby_start(
        state=SafetyState(
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
            leader_standby=False,
            leader_stream_active=bool(safety["leader_stream_active"]),
            follower_up=bool(safety["follower_runtime_active"]),
            controllers_ready=bool(safety["controllers_ready"]),
            feedback_fresh=bool(safety["feedback_fresh"]),
            dxl_ok=bool(safety["dxl_ok"]),
            operation_running=bool(snapshot["operation"]["running"]),
            control_busy=False,
            selected_follower=str(snapshot["selected"]["follower"]),
        ),
        inference_phase=str(ros["inference"]["phase"]),
        inference_status_ready=bool(ros["inference"]["typed_status_available"]),
        recording_phase=str(ros["recording"]["phase"]),
        recording_status_ready=bool(ros["recording"]["typed_status_available"]),
        arbiter_available=bool(ros["arbiter"]["available"]),
        arbiter_mode=str(ros["arbiter"]["mode"]),
        arbiter_fault=str(ros["arbiter"]["fault"]),
        publisher_conflict=str(ros["arbiter"]["publisher_conflict"]),
        intervention_enabled=bool(ros["arbiter"]["intervention_enabled"]),
    ).to_dict()


def arbiter_fault_reset_gate(snapshot: dict[str, Any]) -> dict[str, Any]:
    ros = snapshot["ros"]
    safety = snapshot["safety"]
    return gate_arbiter_fault_reset(
        inference_phase=str(ros["inference"]["phase"]),
        inference_status_ready=bool(ros["inference"]["typed_status_available"]),
        leader_up=bool(safety["leader_runtime_active"]),
        command_recent=bool(safety["command_recent"]),
        recording_phase=str(ros["recording"]["phase"]),
        arbiter_available=bool(ros["arbiter"]["available"]),
        arbiter_mode=str(ros["arbiter"]["mode"]),
        publisher_conflict=str(ros["arbiter"]["publisher_conflict"]),
        leader_standby=bool(safety.get("leader_standby_ready", False)),
    ).to_dict()


def paused_pose_move_gate(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Validate an operator-entered pose move while policy stays paused."""
    safety = snapshot["safety"]
    ros = snapshot["ros"]
    return gate_paused_pose_move(
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
            leader_standby=bool(safety.get("leader_standby_ready", False)),
            leader_stream_active=bool(safety["leader_stream_active"]),
            command_stream_active=bool(safety["command_recent"]),
            follower_up=bool(safety["follower_runtime_active"]),
            controllers_ready=bool(safety["controllers_ready"]),
            feedback_fresh=bool(safety["feedback_fresh"]),
            dxl_ok=bool(safety["dxl_ok"]),
            operation_running=bool(snapshot["operation"]["running"]),
            control_busy=bool(snapshot["control"]["running"]),
            selected_follower=str(snapshot["selected"]["follower"]),
        ),
        inference_phase=str(ros["inference"]["phase"]),
        inference_status_ready=bool(ros["inference"]["typed_status_available"]),
        recording_phase=str(ros["recording"]["phase"]),
        recording_status_ready=bool(ros["recording"]["typed_status_available"]),
        intervention_active=bool(snapshot["intervention"]["active"]),
        arbiter_available=bool(ros["arbiter"]["available"]),
        arbiter_mode=str(ros["arbiter"]["mode"]),
        arbiter_fault=str(ros["arbiter"]["fault"]),
        publisher_conflict=str(ros["arbiter"]["publisher_conflict"]),
        intervention_enabled=bool(ros["arbiter"]["intervention_enabled"]),
    ).to_dict()


def wait_for_arbiter_safe(timeout: float = 3.0) -> dict[str, Any]:
    snapshot = wait_for_snapshot(
        lambda item: (
            item["ros"]["arbiter"]["available"]
            and item["ros"]["arbiter"]["mode"] == "NONE"
            and not item["ros"]["arbiter"]["fault"]
            and not item["ros"]["arbiter"]["publisher_conflict"]
            and not item["ros"]["arbiter"]["intervention_enabled"]
        ),
        timeout,
    )
    arbiter = snapshot["ros"]["arbiter"]
    if not (
        arbiter["available"]
        and arbiter["mode"] == "NONE"
        and not arbiter["fault"]
        and not arbiter["publisher_conflict"]
        and not arbiter["intervention_enabled"]
    ):
        raise RuntimeError("Command arbiter의 NONE/fault-clear 상태를 확인하지 못했습니다.")
    return snapshot


def intervention_watchdog_loop() -> None:
    """Fail closed if safety or inference ownership changes mid-intervention."""
    while not shutdown_event.wait(0.25):
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
            leader_standby.clear(error)
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


def leader_standby_watchdog_loop() -> None:
    """Remove a warm Leader if its isolation or feedback guarantee is lost."""
    unhealthy_since: float | None = None
    unhealthy_identity: tuple[str, str] | None = None
    while not shutdown_event.wait(0.25):
        current = leader_standby.snapshot()["current"]
        if not current:
            unhealthy_since = None
            unhealthy_identity = None
            continue
        phase = str(current.get("phase", ""))
        intervention_active = intervention.snapshot()["active"]
        if intervention_active:
            unhealthy_since = None
            unhealthy_identity = None
            continue
        snapshot = build_snapshot()
        reasons: list[str] = []
        orphaned_transition = phase in {"WARMING", "STOPPING"}
        if orphaned_transition:
            reasons.append(f"소유자 없는 standby 전환 상태({phase})")
        elif phase == "READY":
            if snapshot["ros"]["estop"]["active"]:
                reasons.append("E-stop 감지")
            reasons.extend(
                isolated_leader_failures(snapshot, str(current.get("model", "")))
            )
        elif phase == "IN_USE":
            arbiter = snapshot["ros"]["arbiter"]
            if arbiter["mode"] == "NONE" and not arbiter["intervention_enabled"]:
                try:
                    leader_standby.transition("READY")
                except RuntimeError:
                    pass
                unhealthy_since = None
                unhealthy_identity = None
                continue
            reasons.extend(
                manual_teleop_failures(snapshot, str(current.get("model", "")))
            )
        else:
            reasons.append(f"알 수 없는 standby 상태({phase})")
        if not reasons:
            if current.get("error"):
                try:
                    leader_standby.transition("READY")
                except RuntimeError:
                    pass
            unhealthy_since = None
            unhealthy_identity = None
            continue
        identity = (
            str(current.get("started_at", "")),
            str(current.get("model", "")),
        )
        now = monotonic()
        if unhealthy_identity != identity:
            unhealthy_identity = identity
            unhealthy_since = now
            continue
        if not sustained_failure_elapsed(
            unhealthy_since,
            now,
            STANDBY_UNHEALTHY_GRACE_SECONDS,
        ):
            continue
        if not coordinator.try_acquire():
            continue
        # Re-check after acquiring ownership. A single delayed ROS graph or
        # feedback sample must not tear down an otherwise isolated standby.
        current = leader_standby.snapshot()["current"]
        if not current or intervention.snapshot()["active"]:
            unhealthy_since = None
            unhealthy_identity = None
            coordinator.release()
            continue
        snapshot = build_snapshot()
        phase = str(current.get("phase", ""))
        if phase == "READY":
            reasons = []
            if snapshot["ros"]["estop"]["active"]:
                reasons.append("E-stop 감지")
            reasons.extend(
                isolated_leader_failures(snapshot, str(current.get("model", "")))
            )
        elif phase == "IN_USE":
            reasons = manual_teleop_failures(
                snapshot, str(current.get("model", "")))
        else:
            reasons = [f"소유자 없는 standby 전환 상태({phase})"]
        if not reasons:
            unhealthy_since = None
            unhealthy_identity = None
            coordinator.release()
            continue
        error = " · ".join(reasons)
        announced = False
        try:
            coordinator.announce(
                "leader_standby_stop",
                "standby-watchdog",
                str(current.get("model", "")),
            )
            announced = True
            if (
                snapshot["ros"]["arbiter"]["mode"] == "TELEOP"
                or snapshot["ros"]["arbiter"]["intervention_enabled"]
            ):
                best_effort_arbiter_action("select_none")
                best_effort_arbiter_action("disable_intervention")
            best_effort_arbiter_action("release_teleop")
            if snapshot["services"]["leader"]["managed"]:
                change_service("leader", False)
            leader_standby.clear(error)
            node.event("danger", f"Leader standby watchdog 안전 종료: {error}")
        except Exception as exc:
            try:
                leader_standby.transition(
                    "READY",
                    f"{error} · Leader 종료 실패: {exc}",
                )
            except RuntimeError:
                pass
            node.event("danger", f"Leader standby watchdog 종료 실패: {exc}")
        finally:
            if announced:
                coordinator.finish(False, error)
            else:
                coordinator.release()
        unhealthy_since = None
        unhealthy_identity = None


leader_standby_watchdog_thread = threading.Thread(
    target=leader_standby_watchdog_loop,
    name="leader-standby-watchdog",
    daemon=True,
)
leader_standby_watchdog_thread.start()


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
async def events(http_request: Request) -> StreamingResponse:
    async def stream():
        while (
            not shutdown_event.is_set()
            and not await http_request.is_disconnected()
        ):
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
    preflight = intervention_preflight_gate(snapshot)
    fault_reset = arbiter_fault_reset_gate(snapshot)
    pose_move = paused_pose_move_gate(snapshot)
    return {
        **snapshot["intervention"],
        "standby": snapshot["leader_standby"],
        "inference_phase": snapshot["ros"]["inference"]["phase"],
        "record_phase": snapshot["ros"]["recording"]["phase"],
        "leader_up": snapshot["safety"]["leader_runtime_active"],
        "command_recent": snapshot["safety"]["command_recent"],
        "inference_status_ready": snapshot["ros"]["inference"]["typed_status_available"],
        "recording_status_ready": snapshot["ros"]["recording"]["typed_status_available"],
        "arbiter": snapshot["ros"]["arbiter"],
        "operation": snapshot["operation"],
        "preflight": preflight,
        "fault_reset": fault_reset,
        "pose_move": pose_move,
    }


@app.post("/api/intervention/pose")
def move_to_operator_pose(
    body: PoseMoveRequest,
    http_request: Request,
    x_control_token: str | None = Header(default=None),
    x_control_client: str | None = Header(default=None),
) -> dict[str, Any]:
    """Move F2 to validated operator-entered whole-body joint lists."""
    authorize("recover_init", x_control_token)
    model = body.model.strip().lower()
    if model != "f2":
        raise HTTPException(400, "사용자 초기 위치 이동은 현재 F2에서만 지원됩니다.")

    targets = {
        "left_arm": body.left_arm,
        "right_arm": body.right_arm,
        "head": body.head,
        "lift": body.lift,
    }
    recovery_yaml = AI_WORKER_ROOT / FOLLOWER_MODELS[model]["recovery_yaml"]
    try:
        load_custom_recovery_groups(
            model,
            targets,
            duration=body.duration,
            config_path=recovery_yaml,
        )
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from exc

    if not coordinator.try_acquire():
        raise HTTPException(409, "다른 사용자의 로봇 제어 요청이 처리 중입니다.")

    actor = (
        x_control_client
        or (http_request.client.host if http_request.client else "browser")
    ).strip() or "browser"
    announced = False
    ok = False
    error = ""
    try:
        snapshot = build_snapshot()
        gate = paused_pose_move_gate(snapshot)
        if not gate["ok"]:
            raise HTTPException(409, {
                "message": "사용자 초기 위치 이동 안전 조건을 통과하지 못했습니다.",
                **gate,
            })
        coordinator.announce("custom_pose_move", actor, model)
        announced = True
        operation.start(
            "F2 사용자 초기 위치 이동",
            [
                "python3",
                str(ROOT / "recovery_runner.py"),
                "--model",
                model,
                "--config",
                str(recovery_yaml),
                "--targets-json",
                json.dumps(targets, separators=(",", ":")),
                "--duration",
                str(body.duration),
            ],
        )
        ok = True
        return {
            "ok": True,
            "action": "custom_pose_move",
            "warnings": gate["warnings"],
        }
    except Exception as exc:
        error = str(exc.detail) if isinstance(exc, HTTPException) else str(exc)
        if announced:
            node.event("danger", f"사용자 초기 위치 이동 실패 · {actor}: {error}")
        if isinstance(exc, HTTPException):
            raise
        raise HTTPException(500, error) from exc
    finally:
        if announced:
            coordinator.finish(ok, error)
        else:
            coordinator.release()


@app.post("/api/intervention/{action}")
def intervention_action(
    action: str,
    body: InterventionRequest,
    http_request: Request,
    x_control_token: str | None = Header(default=None),
    x_control_client: str | None = Header(default=None),
) -> dict[str, Any]:
    if action not in {
        "standby",
        "standby_stop",
        "teleop_start",
        "teleop_stop",
        "start",
        "recording",
        "end",
        "complete",
        "abort",
        "reset",
        "reset_fault",
    }:
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
        intervention_snapshot = intervention.snapshot()
        current = intervention_snapshot["current"]
        if current is None:
            # COMPLETE is the final acknowledgement in a browser-driven
            # transaction.  Treat a duplicate acknowledgement as idempotent:
            # the first response may have reached the server even when the
            # browser lost it during a mode switch.
            return {
                "ok": True,
                "already_complete": True,
                **intervention_snapshot,
            }
        if current.get("phase") != "READY_TO_RESUME":
            raise HTTPException(409, "Resume 완료를 확인할 Intervention이 없습니다.")
        snapshot = wait_for_snapshot(
            lambda item: intervention_resume_confirmed(
                inference_phase=item["ros"]["inference"]["phase"],
                arbiter_available=item["ros"]["arbiter"]["available"],
                arbiter_mode=item["ros"]["arbiter"]["mode"],
                arbiter_fault=item["ros"]["arbiter"]["fault"],
                publisher_conflict=item["ros"]["arbiter"]["publisher_conflict"],
                intervention_enabled=item["ros"]["arbiter"]["intervention_enabled"],
            ),
            5.0,
        )
        inference_phase = snapshot["ros"]["inference"]["phase"]
        arbiter = snapshot["ros"]["arbiter"]
        if not intervention_resume_confirmed(
            inference_phase=inference_phase,
            arbiter_available=arbiter["available"],
            arbiter_mode=arbiter["mode"],
            arbiter_fault=arbiter["fault"],
            publisher_conflict=arbiter["publisher_conflict"],
            intervention_enabled=arbiter["intervention_enabled"],
        ):
            raise HTTPException(
                409,
                "Inference의 SYNCING/INFERENCING 상태와 Command arbiter의 "
                "POLICY 제어권이 함께 확인되지 않았습니다.",
            )
        standby_preserved = bool(
            current.get("prewarmed")
            and isolated_leader_ready(
                snapshot,
                str(current.get("leader_model", "")),
            )
            and snapshot["leader_standby"]["ready"]
        )
        if snapshot["safety"]["leader_runtime_active"] and not standby_preserved:
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
        if action == "standby":
            model = body.model.strip().lower() or "mini"
            if model != "mini":
                raise HTTPException(400, "Warm standby는 현재 Mini Leader만 지원합니다.")
            if intervention.snapshot()["active"]:
                raise HTTPException(409, "Intervention 중에는 Leader standby를 시작할 수 없습니다.")
            current_standby = leader_standby.snapshot()["current"]
            if current_standby:
                snapshot = wait_for_snapshot(
                    lambda item: isolated_leader_ready(item, model),
                    STANDBY_UNHEALTHY_GRACE_SECONDS,
                )
                current_standby = leader_standby.snapshot()["current"]
                if (
                    current_standby
                    and current_standby.get("model") == model
                    and current_standby.get("phase") == "READY"
                    and isolated_leader_ready(snapshot, model)
                ):
                    return {"ok": True, "standby": snapshot["leader_standby"]}
                raise HTTPException(409, "Leader standby가 이미 준비 중이거나 점검이 필요합니다.")
            snapshot = build_snapshot()
            gate = leader_standby_start_gate(snapshot)
            if not gate["ok"]:
                raise HTTPException(
                    409,
                    {
                        "message": "Leader standby 안전 조건을 통과하지 못했습니다.",
                        **gate,
                    },
                )
            coordinator.announce("leader_standby_start", actor, model)
            announced = True
            leader_standby.begin(actor, model)
            LEADER_TYPE_FILE.write_text(model + "\n", encoding="utf-8")
            lease_nonce = create_leader_preflight_lease(model, "standby")
            try:
                change_service("leader", True)
                if not wait_for_intervention_preflight_ack(lease_nonce):
                    raise RuntimeError(
                        "Leader s6 preflight가 standby 격리 허가를 확인하지 못했습니다."
                    )
            except Exception:
                clear_intervention_preflight_lease()
                raise
            ready = wait_for_snapshot(
                lambda item: isolated_leader_ready(item, model),
                20.0,
            )
            if not isolated_leader_ready(ready, model):
                raise RuntimeError(
                    "Mini Leader standby가 제한 시간 안에 격리·준비 상태가 되지 않았습니다."
                )
            require_arbiter_action("release_teleop")
            leader_standby.transition("READY")
            node.event("info", f"Mini Leader warm standby 준비 완료 · {actor}")

        elif action == "standby_stop":
            current_standby = leader_standby.snapshot()["current"]
            if not current_standby:
                return {"ok": True, "standby": leader_standby.snapshot()}
            if intervention.snapshot()["active"]:
                raise HTTPException(409, "Intervention 중에는 Leader standby를 종료할 수 없습니다.")
            snapshot = build_snapshot()
            if (
                snapshot["ros"]["arbiter"]["mode"] == "TELEOP"
                or snapshot["ros"]["arbiter"]["intervention_enabled"]
            ):
                raise HTTPException(409, "TELEOP routing을 먼저 안전하게 종료해야 합니다.")
            coordinator.announce(
                "leader_standby_stop",
                actor,
                str(current_standby.get("model", "")),
            )
            announced = True
            leader_standby.transition("STOPPING")
            require_arbiter_action("release_teleop")
            if snapshot["services"]["leader"]["managed"]:
                change_service("leader", False)
            stopped = wait_for_snapshot(
                lambda item: not item["safety"]["leader_runtime_active"],
                10.0,
            )
            if stopped["safety"]["leader_runtime_active"]:
                raise RuntimeError("Mini Leader standby가 제한 시간 안에 종료되지 않았습니다.")
            leader_standby.clear()
            node.event("info", f"Mini Leader warm standby 종료 · {actor}")

        elif action == "teleop_start":
            if intervention.snapshot()["active"]:
                raise HTTPException(
                    409,
                    "Intervention 중에는 수동 상대 TELEOP을 시작할 수 없습니다.",
                )
            current_standby = leader_standby.snapshot()["current"]
            if not current_standby:
                raise HTTPException(
                    409,
                    "Mini Leader가 미리 연결되지 않았습니다. 먼저 standby를 준비해 주세요.",
                )
            standby_phase = str(current_standby.get("phase", ""))
            if current_standby.get("model") != "mini" or standby_phase not in {
                "READY",
                "IN_USE",
            }:
                raise HTTPException(409, "Mini Leader standby가 준비 상태가 아닙니다.")
            if standby_phase == "READY":
                snapshot = wait_for_snapshot(
                    lambda item: isolated_leader_ready(item, "mini"),
                    STANDBY_UNHEALTHY_GRACE_SECONDS,
                )
                if not isolated_leader_ready(snapshot, "mini"):
                    raise HTTPException(
                        409,
                        "Mini Leader standby 상태가 불안정합니다: "
                        + " · ".join(isolated_leader_failures(snapshot, "mini")),
                    )
            else:
                snapshot = build_snapshot()
                failures = manual_teleop_failures(snapshot, "mini")
                if failures:
                    raise HTTPException(
                        409,
                        "기존 수동 TELEOP 상태가 불안정합니다: "
                        + " · ".join(failures),
                    )
            if snapshot["ros"]["inference"]["phase"] != "READY":
                raise HTTPException(409, "Inference가 READY일 때만 수동 TELEOP을 시작할 수 있습니다.")
            if snapshot["ros"]["recording"]["phase"] not in {"READY", "RECORDING"}:
                raise HTTPException(
                    409,
                    "Recording이 READY 또는 RECORDING일 때만 수동 TELEOP을 시작할 수 있습니다.",
                )
            coordinator.announce("manual_teleop_start", actor, "mini")
            announced = True
            if standby_phase == "READY":
                require_arbiter_action("prepare_intervention")
                leader_standby.transition("IN_USE")
            require_arbiter_action(
                "select_teleop",
                timeout=5.0,
                retry_readiness=True,
            )
            ready = wait_for_snapshot(
                lambda item: not manual_teleop_failures(item, "mini"),
                5.0,
            )
            failures = manual_teleop_failures(ready, "mini")
            if failures:
                raise RuntimeError(
                    "수동 상대 TELEOP 연결 상태를 확인하지 못했습니다: "
                    + " · ".join(failures)
                )
            node.event("info", f"수동 상대 TELEOP 연결 성공 · {actor}")

        elif action == "teleop_stop":
            if intervention.snapshot()["active"]:
                raise HTTPException(
                    409,
                    "Intervention 중에는 수동 TELEOP 종료 API를 사용할 수 없습니다.",
                )
            current_standby = leader_standby.snapshot()["current"]
            coordinator.announce("manual_teleop_stop", actor, "mini")
            announced = True
            require_arbiter_action("select_none")
            require_arbiter_action("disable_intervention")
            require_arbiter_action("release_teleop")
            if current_standby:
                ready = wait_for_snapshot(
                    lambda item: isolated_leader_ready(item, "mini"),
                    STANDBY_UNHEALTHY_GRACE_SECONDS,
                )
                if not isolated_leader_ready(ready, "mini"):
                    raise RuntimeError(
                        "수동 TELEOP 종료 후 standby 격리 상태를 복구하지 못했습니다: "
                        + " · ".join(isolated_leader_failures(ready, "mini"))
                    )
                leader_standby.transition("READY")
            node.event("info", f"수동 상대 TELEOP 종료, 제어권 NONE · {actor}")

        elif action == "reset_fault":
            if intervention.snapshot()["active"]:
                raise HTTPException(409, "진행 중인 Intervention을 먼저 안전 중단해야 합니다.")
            snapshot = build_snapshot()
            gate = arbiter_fault_reset_gate(snapshot)
            if not gate["ok"]:
                raise HTTPException(
                    409,
                    {
                        "message": "Arbiter fault 초기화 안전 조건을 통과하지 못했습니다.",
                        **gate,
                    },
                )
            coordinator.announce("intervention_reset_fault", actor, "")
            announced = True
            require_arbiter_action("clear_fault")
            wait_for_arbiter_safe()
            node.event("warning", f"Command arbiter fault 초기화, mode NONE 유지 · {actor}")

        elif action == "start":
            model = body.model.strip().lower() or "a3"
            if model not in INTERVENTION_LEADERS:
                raise HTTPException(400, "지원하지 않는 Leader 모델입니다.")
            if intervention.snapshot()["active"]:
                raise HTTPException(409, "Intervention이 이미 진행 중입니다.")
            standby_current = leader_standby.snapshot().get("current") or {}
            warm_standby_selected = bool(
                model == "mini"
                and standby_current.get("model") == model
                and standby_current.get("phase") == "READY"
            )
            snapshot = wait_for_snapshot(
                lambda item: (
                    item["ros"]["inference"]["phase"] == "PAUSED"
                    and not item["safety"]["command_recent"]
                    and (
                        not warm_standby_selected
                        or isolated_leader_ready(item, model)
                    )
                ),
                max(
                    COMMAND_QUIET_SECONDS + 3.0,
                    STANDBY_UNHEALTHY_GRACE_SECONDS,
                ),
            )
            if warm_standby_selected and not isolated_leader_ready(snapshot, model):
                raise HTTPException(
                    409,
                    "Mini standby 준비 상태가 안정화되지 않았습니다: "
                    + " · ".join(isolated_leader_failures(snapshot, model)),
                )
            gate = intervention_start_gate(snapshot)
            if not gate["ok"]:
                raise HTTPException(409, {"message": "Intervention 안전 조건을 통과하지 못했습니다.", **gate})
            preflight = intervention_preflight_gate(snapshot)
            if not preflight["ok"]:
                raise HTTPException(409, {"message": "Intervention preflight를 통과하지 못했습니다.", **preflight})
            coordinator.announce("intervention_start", actor, model)
            announced = True
            standby_snapshot = leader_standby.snapshot()
            standby_current = standby_snapshot.get("current") or {}
            prewarmed = bool(
                model == "mini"
                and standby_current.get("model") == model
                and standby_current.get("phase") == "READY"
                and isolated_leader_ready(snapshot, model)
            )
            intervention.begin(
                actor,
                model,
                prewarmed=prewarmed,
                recording_continued=(
                    snapshot["ros"]["recording"]["phase"] == "RECORDING"
                ),
            )
            require_arbiter_action("prepare_intervention")
            if prewarmed:
                leader_standby.transition("IN_USE")
                require_arbiter_action("release_teleop")
            else:
                LEADER_TYPE_FILE.write_text(model + "\n", encoding="utf-8")
                lease_nonce = create_leader_preflight_lease(model, "intervention")
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
            keep_warm = bool(current.get("prewarmed"))
            if snapshot["safety"]["leader_runtime_active"] and not snapshot["services"]["leader"]["managed"]:
                raise RuntimeError("Leader가 UI 밖에서 실행되어 안전하게 종료할 수 없습니다.")
            if snapshot["services"]["leader"]["managed"] and not keep_warm:
                change_service("leader", False)
            quiet = wait_for_snapshot(
                lambda item: (
                    (keep_warm or not item["safety"]["leader_runtime_active"])
                    and not item["safety"]["command_recent"]
                ),
                10.0,
            )
            if (
                (not keep_warm and quiet["safety"]["leader_runtime_active"])
                or quiet["safety"]["command_recent"]
            ):
                raise RuntimeError("Leader 종료 후 command quiet 상태를 확인하지 못했습니다.")
            require_arbiter_action("disable_intervention")
            if keep_warm:
                standby_ready_snapshot = wait_for_snapshot(
                    lambda item: isolated_leader_ready(item, "mini"),
                    3.0,
                )
                if not isolated_leader_ready(standby_ready_snapshot, "mini"):
                    raise RuntimeError("Intervention 종료 후 Mini standby 상태를 복구하지 못했습니다.")
                leader_standby.transition("READY")
            intervention.transition("READY_TO_RESUME")
            node.event(
                "warning",
                (
                    f"Intervention 종료, Mini standby 유지, Inference Resume 대기 · {actor}"
                    if keep_warm
                    else f"Intervention Leader 종료, Inference Resume 대기 · {actor}"
                ),
            )

        elif action == "reset":
            current = intervention.snapshot()["current"]
            if not current:
                return {"ok": True, **intervention.snapshot()}
            # The recorder publishes SAVING only after capture has stopped and
            # metadata has been committed.  Full-episode archival may continue
            # in the background, so do not hold Intervention reset until READY.
            # Only bridge the short typed-status race after the stop response.
            if current.get("phase") == "READY_TO_RESUME":
                snapshot = wait_for_snapshot(
                    lambda item: item["ros"]["recording"]["phase"] in {"SAVING", "READY"},
                    5.0,
                )
            else:
                snapshot = build_snapshot()
            gate = gate_intervention_reset(
                str(current.get("phase", "")),
                leader_up=bool(snapshot["safety"]["leader_runtime_active"]),
                command_recent=bool(snapshot["safety"]["command_recent"]),
                recording_phase=str(snapshot["ros"]["recording"]["phase"]),
                leader_standby=bool(snapshot["safety"].get("leader_standby_ready", False)),
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
            require_arbiter_action("select_none")
            require_arbiter_action("disable_intervention")
            reset_snapshot = build_snapshot()
            reset_gate = arbiter_fault_reset_gate(reset_snapshot)
            if not reset_gate["ok"]:
                raise HTTPException(
                    409,
                    {
                        "message": "Arbiter fault 초기화 안전 조건을 통과하지 못했습니다.",
                        **reset_gate,
                    },
                )
            require_arbiter_action("clear_fault")
            wait_for_arbiter_safe()
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
            leader_standby.clear("Intervention 안전 중단")
            wait_for_snapshot(
                lambda item: not item["safety"]["leader_runtime_active"] and not item["safety"]["command_recent"],
                10.0,
            )
            best_effort_arbiter_action("disable_intervention")
            intervention.transition("PAUSED_SAFE", "사용자 중단 — Inference는 자동 재개하지 않았습니다.")
            node.event("danger", f"Intervention 안전 중단, Inference PAUSED 유지 · {actor}")

        ok = True
        payload = {"ok": True, **intervention.snapshot()}
        if action in {"standby", "standby_stop", "teleop_start", "teleop_stop"}:
            payload["standby"] = leader_standby.snapshot()
        return payload
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
            leader_standby.clear(error)
        elif action == "standby" and leader_standby.snapshot()["current"]:
            try:
                snapshot = build_snapshot()
                if snapshot["services"]["leader"]["managed"]:
                    change_service("leader", False)
            except Exception:
                pass
            leader_standby.clear(error)
        elif action == "standby_stop":
            try:
                leader_standby.transition("READY", error)
            except RuntimeError:
                pass
        elif action in {"teleop_start", "teleop_stop"}:
            best_effort_arbiter_action("select_none")
            best_effort_arbiter_action("disable_intervention")
            best_effort_arbiter_action("release_teleop")
            try:
                snapshot = wait_for_snapshot(
                    lambda item: isolated_leader_ready(item, "mini"),
                    STANDBY_UNHEALTHY_GRACE_SECONDS,
                )
                if isolated_leader_ready(snapshot, "mini"):
                    leader_standby.transition("READY", error)
                elif snapshot["services"]["leader"]["managed"]:
                    change_service("leader", False)
                    leader_standby.clear(error)
            except Exception:
                leader_standby.clear(error)
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
    shutdown_event.set()
    operation.cancel("웹 supervisor 종료")
    if rclpy.ok():
        rclpy.shutdown()
    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
