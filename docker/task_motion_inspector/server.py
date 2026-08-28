#!/usr/bin/env python3
"""Read-only ROS 2 web tool for recording and diagnosing task motion."""

from __future__ import annotations

import asyncio
import hashlib
import json
import math
import os
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
import rosbag2_py
from control_msgs.msg import DynamicJointState, JointTrajectoryControllerState
from dynamixel_interfaces.msg import DynamixelState
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from analyze_session import analyze_session
from logic import (
    ARM_DXL_MAP,
    DEFAULT_LIMITS,
    ConditionLatch,
    classify_joint,
    finite,
    parse_f2_limits,
    safe_session_name,
)


ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
SESSION_ROOT = Path(os.environ.get("MOTION_SESSION_ROOT", "/workspace/task_motion_inspector/sessions"))

COMMAND_TOPICS = {
    "left": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
    "right": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
}
SOURCE_TOPICS = {
    "left": "/robot/arm_left_leader/joint_states",
    "right": "/robot/arm_right_leader/joint_states",
}
CONTROLLER_TOPICS = {
    "left": "/arm_l_controller/controller_state",
    "right": "/arm_r_controller/controller_state",
}
POLICY_PREVIEW_TOPIC = "/inference/trajectory_preview"
SOURCE_STATUS_TOPIC = "/command_source_arbiter/status"
ROBOT_DESCRIPTION_TOPIC = "/robot_description"
MONITOR_TOPICS = [
    "/leader/joint_states",
    *SOURCE_TOPICS.values(),
    *COMMAND_TOPICS.values(),
    *CONTROLLER_TOPICS.values(),
    POLICY_PREVIEW_TOPIC,
    SOURCE_STATUS_TOPIC,
    ROBOT_DESCRIPTION_TOPIC,
    "/joint_states",
    "/dynamic_joint_states",
    "/ffw_follower/dxl_state",
]
# F2 arm motor/ID mapping is stable; its configured limits are refreshed from
# runtime /robot_description instead of being treated as EEPROM read-back.
DXL_TO_JOINT = {dxl: spec["joint"] for dxl, spec in ARM_DXL_MAP.items()}

TRACKING_WARN_RAD = math.radians(float(os.environ.get("TRACKING_WARN_DEG", "2.0")))
TRACKING_CRITICAL_RAD = math.radians(float(os.environ.get("TRACKING_CRITICAL_DEG", "8.0")))
FRESH_SECONDS = 2.0
HISTORY_SECONDS = float(os.environ.get("MOTION_HISTORY_SECONDS", "60"))
HISTORY_MAX_SAMPLES = int(os.environ.get("MOTION_HISTORY_MAX_SAMPLES", "12000"))
TRACKING_WARN_DURATION_S = float(os.environ.get("TRACKING_WARN_DURATION_S", "0.25"))
TRACKING_CLEAR_DURATION_S = float(os.environ.get("TRACKING_CLEAR_DURATION_S", "0.5"))
SATURATION_DURATION_S = float(os.environ.get("SATURATION_DURATION_S", "0.25"))
def now_s() -> float:
    return time.monotonic()


def point_positions(names: list[str], positions: Any) -> dict[str, float]:
    return {
        name: value
        for name, raw in zip(names, positions)
        if (value := finite(raw)) is not None
    }


def message_stamp_s(msg: Any, received_s: float) -> float:
    """Use a non-zero ROS header stamp, otherwise the local receive time."""
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = int(getattr(stamp, "sec", 0) or 0)
    nanosec = int(getattr(stamp, "nanosec", 0) or 0)
    return sec + nanosec / 1e9 if sec or nanosec else received_s


class StartRecording(BaseModel):
    name: str = "motion"


class MotionInspector(Node):
    def __init__(self) -> None:
        super().__init__("task_motion_inspector")
        self.lock = threading.RLock()
        self.values: dict[str, dict[str, Any]] = {}
        self.histories: dict[str, dict[str, deque[tuple[float, float]]]] = defaultdict(
            lambda: defaultdict(lambda: deque(maxlen=HISTORY_MAX_SAMPLES))
        )
        self.limits = json.loads(json.dumps(DEFAULT_LIMITS))
        self.config_digest = "fallback"
        self.command_source: dict[str, Any] = {
            "mode": "UNKNOWN",
            "fault": "",
            "seen": None,
            "inferred": True,
        }
        self.alert_latches: dict[tuple[str, str], ConditionLatch] = {}
        self.topic_seen: dict[str, float] = {}
        self.events: deque[dict[str, Any]] = deque(maxlen=80)
        self.session_events: list[dict[str, Any]] = []
        self.session_started_at: str | None = None
        self.record_process: subprocess.Popen[bytes] | None = None
        self.active_session: str | None = None
        self.dxl: dict[str, Any] = {"seen": None, "comm_state": None, "torque": {}, "errors": {}}
        self.graph_topics: set[str] = set()

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(JointState, "/leader/joint_states", self._leader, qos)
        self.create_subscription(JointState, SOURCE_TOPICS["left"], lambda m: self._source("left", m), qos)
        self.create_subscription(JointState, SOURCE_TOPICS["right"], lambda m: self._source("right", m), qos)
        self.create_subscription(JointState, "/joint_states", self._joint_states, qos)
        self.create_subscription(JointTrajectory, COMMAND_TOPICS["left"], lambda m: self._trajectory("left", m), reliable)
        self.create_subscription(JointTrajectory, COMMAND_TOPICS["right"], lambda m: self._trajectory("right", m), reliable)
        self.create_subscription(JointTrajectory, POLICY_PREVIEW_TOPIC, self._policy_preview, reliable)
        self.create_subscription(JointTrajectoryControllerState, CONTROLLER_TOPICS["left"], lambda m: self._controller("left", m), qos)
        self.create_subscription(JointTrajectoryControllerState, CONTROLLER_TOPICS["right"], lambda m: self._controller("right", m), qos)
        self.create_subscription(DynamicJointState, "/dynamic_joint_states", self._dynamic, qos)
        self.create_subscription(DynamixelState, "/ffw_follower/dxl_state", self._dxl_state, qos)
        self.create_subscription(String, SOURCE_STATUS_TOPIC, self._source_status, transient)
        self.create_subscription(String, ROBOT_DESCRIPTION_TOPIC, self._robot_description, transient)
        self.create_timer(1.0, self._refresh_graph)

    def _touch(self, topic: str) -> None:
        self.topic_seen[topic] = now_s()

    def _set_many(self, values: dict[str, float], key: str, stamp_s: float | None = None) -> None:
        stamp = time.time() if stamp_s is None else float(stamp_s)
        for name, value in values.items():
            row = self.values.setdefault(name, {})
            row[key] = value
            row[f"{key}_stamp"] = stamp
            history = self.histories[name][key]
            history.append((stamp, value))
            cutoff = stamp - HISTORY_SECONDS
            while history and history[0][0] < cutoff:
                history.popleft()

    def _leader(self, msg: JointState) -> None:
        received = time.time()
        with self.lock:
            self._touch("/leader/joint_states")
            self._set_many(
                point_positions(list(msg.name), msg.position),
                "leader_raw",
                message_stamp_s(msg, received),
            )

    def _source(self, side: str, msg: JointState) -> None:
        received = time.time()
        topic = SOURCE_TOPICS[side]
        with self.lock:
            self._touch(topic)
            self._set_many(
                point_positions(list(msg.name), msg.position),
                "source",
                message_stamp_s(msg, received),
            )

    def _joint_states(self, msg: JointState) -> None:
        received = time.time()
        with self.lock:
            self._touch("/joint_states")
            self._set_many(
                point_positions(list(msg.name), msg.position),
                "joint_state",
                message_stamp_s(msg, received),
            )

    def _trajectory(self, side: str, msg: JointTrajectory) -> None:
        topic = COMMAND_TOPICS[side]
        if not msg.points:
            return
        received = time.time()
        with self.lock:
            self._touch(topic)
            positions = point_positions(list(msg.joint_names), msg.points[-1].positions)
            stamp = message_stamp_s(msg, received)
            self._set_many(positions, "command", stamp)
            # Keep the trajectory alias used by existing recorded sessions.
            self._set_many(positions, "trajectory", stamp)

    def _policy_preview(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        received = time.time()
        with self.lock:
            self._touch(POLICY_PREVIEW_TOPIC)
            self._set_many(
                point_positions(list(msg.joint_names), msg.points[-1].positions),
                "policy",
                message_stamp_s(msg, received),
            )

    def _controller(self, side: str, msg: JointTrajectoryControllerState) -> None:
        topic = CONTROLLER_TOPICS[side]
        names = list(msg.joint_names)
        received = time.time()
        stamp = message_stamp_s(msg, received)
        with self.lock:
            self._touch(topic)
            self._set_many(point_positions(names, msg.reference.positions), "reference", stamp)
            self._set_many(point_positions(names, msg.output.positions), "output", stamp)
            self._set_many(point_positions(names, msg.feedback.positions), "feedback", stamp)
            errors = point_positions(names, msg.error.positions)
            self._set_many(errors, "tracking_error", stamp)
            for name in names:
                self._evaluate_joint(name, now_s())

    def _dynamic(self, msg: DynamicJointState) -> None:
        received = time.time()
        stamp = message_stamp_s(msg, received)
        with self.lock:
            self._touch("/dynamic_joint_states")
            for dxl_name, interface in zip(msg.joint_names, msg.interface_values):
                joint = DXL_TO_JOINT.get(dxl_name)
                if not joint:
                    continue
                raw = dict(zip(interface.interface_names, interface.values))
                present = finite(raw.get("Present Position"))
                if present is not None:
                    # The installed driver currently publishes radians. Keep a
                    # raw-tick fallback for bags/mocks that expose control-table counts.
                    rad_per_tick = self.limits.get(joint, {}).get("rad_per_tick", 1.0)
                    position_rad = present * rad_per_tick if abs(present) > 100.0 else present
                    self._set_many({joint: position_rad}, "dxl_present", stamp)
                    self.values[joint]["dxl_present_raw"] = present
                current = finite(raw.get("Present Current"))
                if current is not None:
                    self._set_many({joint: current}, "dxl_current", stamp)
                hw_error = finite(raw.get("Hardware Error Status", raw.get("Error Code")))
                if hw_error is not None:
                    self.values.setdefault(joint, {})["dxl_hw_error"] = int(hw_error)
                self._evaluate_joint(joint, now_s())

    def _source_status(self, msg: String) -> None:
        with self.lock:
            self._touch(SOURCE_STATUS_TOPIC)
            try:
                payload = json.loads(msg.data)
                self.command_source = {
                    "mode": str(payload.get("mode") or "UNKNOWN"),
                    "fault": str(payload.get("fault") or ""),
                    "seen": now_s(),
                    "inferred": False,
                    "publisher_conflict": str(payload.get("publisher_conflict") or ""),
                }
            except (TypeError, ValueError, json.JSONDecodeError):
                self.command_source = {
                    "mode": "UNKNOWN",
                    "fault": "invalid arbiter status",
                    "seen": now_s(),
                    "inferred": False,
                }

    def _robot_description(self, msg: String) -> None:
        with self.lock:
            self._touch(ROBOT_DESCRIPTION_TOPIC)
            digest = hashlib.sha256(msg.data.encode("utf-8")).hexdigest()[:12]
            if digest == self.config_digest:
                return
            try:
                self.limits = parse_f2_limits(msg.data)
            except (ValueError, ET.ParseError) as exc:
                self._event("warning", f"robot_description limit 해석 실패: {exc}", event_type="CONFIG")
                return
            self.config_digest = digest
            self._event(
                "info",
                f"런타임 arm limit 갱신 · config {digest}",
                event_type="CONFIG",
            )

    def _latch(self, joint: str, kind: str, on_delay_s: float, off_delay_s: float) -> ConditionLatch:
        key = (joint, kind)
        if key not in self.alert_latches:
            self.alert_latches[key] = ConditionLatch(on_delay_s, off_delay_s)
        return self.alert_latches[key]

    def _evaluate_joint(self, joint: str, stamp: float) -> None:
        row = self.values.get(joint)
        if not row:
            return
        reference = finite(row.get("reference"))
        feedback = finite(row.get("feedback"))
        error = finite(row.get("tracking_error"))
        if error is None and reference is not None and feedback is not None:
            error = reference - feedback
            row["tracking_error"] = error
        warn_condition = error is not None and abs(error) > TRACKING_WARN_RAD
        critical_condition = error is not None and abs(error) > TRACKING_CRITICAL_RAD
        warn_latch = self._latch(
            joint,
            "tracking_warning",
            TRACKING_WARN_DURATION_S,
            TRACKING_CLEAR_DURATION_S,
        )
        critical_latch = self._latch(joint, "tracking_critical", 0.1, 0.3)
        warn_transition = warn_latch.update(warn_condition, stamp)
        critical_transition = critical_latch.update(critical_condition, stamp)
        row["tracking_warning"] = warn_latch.active
        row["tracking_critical"] = critical_latch.active
        if warn_transition:
            verb = "발생" if warn_transition == "opened" else "해제"
            self._event(
                "warning" if warn_transition == "opened" else "info",
                f"{joint} 추종 오차 {verb}"
                + (f" · {math.degrees(abs(error)):.2f}°" if error is not None else ""),
                event_type="TRACKING_WARNING",
                joint=joint,
                value=error,
            )
        if critical_transition:
            verb = "발생" if critical_transition == "opened" else "해제"
            self._event(
                "critical" if critical_transition == "opened" else "info",
                f"{joint} 임계 추종 오차 {verb}"
                + (f" · {math.degrees(abs(error)):.2f}°" if error is not None else ""),
                event_type="TRACKING_CRITICAL",
                joint=joint,
                value=error,
            )

        limits = self.limits.get(joint)
        dxl_present = finite(row.get("dxl_present"))
        desired = reference if reference is not None else finite(row.get("command"))
        saturated = False
        if limits and desired is not None and dxl_present is not None:
            low, high = limits["hardware"]
            outside = desired < low - math.radians(0.15) or desired > high + math.radians(0.15)
            at_endpoint = min(abs(dxl_present - low), abs(dxl_present - high)) <= math.radians(0.15)
            saturated = outside and at_endpoint
        saturation_latch = self._latch(
            joint,
            "hard_limit_saturation",
            SATURATION_DURATION_S,
            TRACKING_CLEAR_DURATION_S,
        )
        saturation_transition = saturation_latch.update(saturated, stamp)
        row["hard_limit_saturation"] = saturation_latch.active
        if saturation_transition:
            verb = "감지" if saturation_transition == "opened" else "해제"
            self._event(
                "critical" if saturation_transition == "opened" else "info",
                f"{joint} DXL hard-limit 포화 {verb}",
                event_type="HARD_LIMIT_SATURATION",
                joint=joint,
                value=error,
            )

    def _dxl_state(self, msg: DynamixelState) -> None:
        with self.lock:
            self._touch("/ffw_follower/dxl_state")
            self.dxl = {
                "seen": now_s(),
                "comm_state": int(msg.comm_state),
                "torque": {str(i): bool(v) for i, v in zip(msg.id, msg.torque_state)},
                "errors": {str(i): int(v) for i, v in zip(msg.id, msg.dxl_hw_state)},
            }

    def _refresh_graph(self) -> None:
        try:
            names = self.get_topic_names_and_types()
            with self.lock:
                self.graph_topics = {name for name, _ in names}
        except Exception as exc:  # graph discovery must not kill monitoring
            self.get_logger().warning(f"topic discovery failed: {exc}")

    def _event(
        self,
        level: str,
        message: str,
        *,
        event_type: str = "SYSTEM",
        joint: str | None = None,
        value: float | None = None,
    ) -> None:
        with self.lock:
            event = {
                "time": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                "level": level,
                "message": message,
                "type": event_type,
                "joint": joint,
                "value": value,
                "source_mode": self.command_source.get("mode", "UNKNOWN"),
            }
            self.events.appendleft(event)
            if self.record_process:
                self.session_events.append(dict(event))

    def _command_source_snapshot(self, current: float) -> dict[str, Any]:
        result = dict(self.command_source)
        seen = result.get("seen")
        if seen is not None and current - float(seen) <= FRESH_SECONDS:
            result["age"] = round(current - float(seen), 3)
            return result
        policy_age = current - self.topic_seen.get(POLICY_PREVIEW_TOPIC, -1e9)
        leader_age = max(
            current - self.topic_seen.get(SOURCE_TOPICS["left"], -1e9),
            current - self.topic_seen.get(SOURCE_TOPICS["right"], -1e9),
        )
        mode = "POLICY" if policy_age < 0.5 else "TELEOP" if leader_age < 0.5 else "UNKNOWN"
        return {"mode": mode, "fault": "", "seen": None, "age": None, "inferred": True}

    def snapshot(self) -> dict[str, Any]:
        current = now_s()
        wall = time.time()
        with self.lock:
            source_info = self._command_source_snapshot(current)
            source_mode = str(source_info.get("mode") or "UNKNOWN").upper()
            joints: dict[str, Any] = {}
            max_error = 0.0
            alert_count = 0
            for name, raw in sorted(self.values.items()):
                if not (name.startswith("arm_l_joint") or name.startswith("arm_r_joint")):
                    continue
                row = dict(raw)
                latest_stamp = max(
                    (float(value) for key, value in row.items() if key.endswith("_stamp")),
                    default=0.0,
                )
                row["stale"] = latest_stamp > 0.0 and wall - latest_stamp > FRESH_SECONDS
                # Use only the producer selected by the command arbiter. This
                # prevents an old teleop sample from becoming the comparison
                # source while POLICY is active, and vice versa.
                source_key = "policy" if source_mode == "POLICY" else "source"
                if source_mode == "UNKNOWN":
                    source_key = max(
                        ("source", "policy"),
                        key=lambda key: float(row.get(f"{key}_stamp", 0.0)),
                    )
                source_stamp = float(row.get(f"{source_key}_stamp", 0.0))
                source_value = finite(row.get(source_key))
                row["source"] = (
                    source_value
                    if source_value is not None
                    and source_stamp > 0.0
                    and wall - source_stamp <= FRESH_SECONDS
                    else None
                )
                row["source_kind"] = (
                    "policy" if source_key == "policy" else "transformed_leader"
                )
                row["flags"] = classify_joint(row, self.limits.get(name), TRACKING_WARN_RAD)
                row["limits"] = self.limits.get(name)
                error = finite(row.get("tracking_error"))
                if error is not None:
                    max_error = max(max_error, abs(error))
                if row["flags"]:
                    alert_count += 1
                joints[name] = row
            topic_status = {
                topic: {
                    "available": topic in self.graph_topics,
                    "age": round(current - self.topic_seen[topic], 3) if topic in self.topic_seen else None,
                }
                for topic in MONITOR_TOPICS
            }
            return {
                "mode": "recording" if self.record_process else "idle",
                "active_session": self.active_session,
                "joints": joints,
                "summary": {
                    "alert_joints": alert_count,
                    "max_tracking_error_deg": math.degrees(max_error),
                    "config_digest": self.config_digest,
                },
                "command_source": source_info,
                "topics": topic_status,
                "dxl": self.dxl,
                "events": list(self.events),
                "thresholds": {
                    "tracking_warn_deg": math.degrees(TRACKING_WARN_RAD),
                    "tracking_warn_duration_s": TRACKING_WARN_DURATION_S,
                    "tracking_critical_deg": math.degrees(TRACKING_CRITICAL_RAD),
                },
            }

    def history(self, joint: str, seconds: float = 20.0, max_points: int = 1000) -> dict[str, Any]:
        if not (joint.startswith("arm_l_joint") or joint.startswith("arm_r_joint")):
            raise ValueError("지원하지 않는 joint 이름입니다.")
        seconds = min(max(float(seconds), 1.0), HISTORY_SECONDS)
        max_points = min(max(int(max_points), 100), 3000)
        cutoff = time.time() - seconds
        with self.lock:
            streams = {}
            for key, samples in self.histories.get(joint, {}).items():
                points = [[stamp, value] for stamp, value in samples if stamp >= cutoff]
                if len(points) > max_points:
                    stride = math.ceil(len(points) / max_points)
                    points = points[::stride]
                streams[key] = points
            return {
                "joint": joint,
                "seconds": seconds,
                "series": streams,
                "limits": self.limits.get(joint),
            }

    def sessions(self) -> list[dict[str, Any]]:
        SESSION_ROOT.mkdir(parents=True, exist_ok=True)
        result = []
        for directory in SESSION_ROOT.iterdir():
            metadata = directory / "metadata.yaml"
            if not directory.is_dir() or not metadata.exists():
                continue
            size = sum(p.stat().st_size for p in directory.rglob("*") if p.is_file())
            result.append({
                "name": directory.name,
                "modified": datetime.fromtimestamp(directory.stat().st_mtime, timezone.utc).isoformat(timespec="seconds"),
                "size_mb": round(size / 1024 / 1024, 2),
                "has_report": (directory / "summary.json").exists(),
            })
        return sorted(result, key=lambda item: item["modified"], reverse=True)

    def start_recording(self, name: str) -> str:
        with self.lock:
            if self.record_process:
                raise RuntimeError("녹화가 이미 실행 중입니다.")
            SESSION_ROOT.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session = f"{stamp}_{safe_session_name(name)}"
            output = SESSION_ROOT / session
            topics = list(MONITOR_TOPICS)
            missing_commands = [topic for topic in COMMAND_TOPICS.values() if topic not in self.graph_topics]
            if missing_commands:
                raise RuntimeError("명령 토픽을 찾을 수 없습니다: " + ", ".join(missing_commands))
            args = ["ros2", "bag", "record", "-s", "mcap", "-o", str(output), *topics]
            self.record_process = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
            self.active_session = session
            self.session_events = []
            self.session_started_at = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            self._event("info", f"녹화 시작: {session}")
            # A condition may already be latched before Record is pressed. Log
            # that initial state so the session report does not depend on a
            # fresh threshold crossing during the selected interval.
            for joint, row in sorted(self.values.items()):
                error = finite(row.get("tracking_error"))
                if row.get("tracking_critical"):
                    self._event(
                        "critical",
                        f"{joint} 녹화 시작 시 임계 추종 오차 활성"
                        + (f" · {math.degrees(abs(error)):.2f}°" if error is not None else ""),
                        event_type="TRACKING_CRITICAL",
                        joint=joint,
                        value=error,
                    )
                elif row.get("tracking_warning"):
                    self._event(
                        "warning",
                        f"{joint} 녹화 시작 시 추종 오차 경고 활성"
                        + (f" · {math.degrees(abs(error)):.2f}°" if error is not None else ""),
                        event_type="TRACKING_WARNING",
                        joint=joint,
                        value=error,
                    )
                if row.get("hard_limit_saturation"):
                    self._event(
                        "critical",
                        f"{joint} 녹화 시작 시 DXL hard-limit 포화 활성",
                        event_type="HARD_LIMIT_SATURATION",
                        joint=joint,
                        value=error,
                    )
            return session

    def stop_recording(self) -> str:
        with self.lock:
            if not self.record_process:
                raise RuntimeError("실행 중인 녹화가 없습니다.")
            process = self.record_process
            session = self.active_session or "unknown"
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        with self.lock:
            self._event("info", f"녹화 완료: {session}")
            self._write_session_sidecar(session)
            self.record_process = None
            self.active_session = None
            self.session_started_at = None
        threading.Thread(
            target=self._analyze_session_safely,
            args=(SESSION_ROOT / safe_session_name(session),),
            name=f"motion-analysis-{safe_session_name(session)}",
            daemon=True,
        ).start()
        return session

    def _analyze_session_safely(self, path: Path) -> None:
        try:
            analyze_session(path, write_files=True)
            self._event("info", f"세션 분석 완료: {path.name}", event_type="REPORT")
        except Exception as exc:
            self.get_logger().warning(f"session analysis failed for {path}: {exc}")
            self._event("warning", f"세션 분석 실패: {path.name}", event_type="REPORT")

    def analyze_saved_session(self, session: str) -> dict[str, Any]:
        safe_name = safe_session_name(session)
        path = SESSION_ROOT / safe_name
        with self.lock:
            if self.record_process and self.active_session == safe_name:
                raise RuntimeError("녹화 중인 세션은 분석할 수 없습니다.")
        if not path.is_dir() or not (path / "metadata.yaml").is_file():
            raise RuntimeError("완성된 rosbag 세션을 찾을 수 없습니다.")
        result = analyze_session(path, write_files=True)
        self._event("info", f"세션 분석 완료: {safe_name}", event_type="REPORT")
        return result

    def _write_session_sidecar(self, session: str) -> None:
        path = SESSION_ROOT / safe_session_name(session)
        if not path.is_dir():
            self.get_logger().warning(f"session sidecar directory missing: {path}")
            return
        payload = {
            "schema_version": 1,
            "session": session,
            "started_at": self.session_started_at,
            "finished_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "command_source": self._command_source_snapshot(now_s()),
            "config_digest": self.config_digest,
            "limits": self.limits,
            "thresholds": {
                "tracking_warn_deg": math.degrees(TRACKING_WARN_RAD),
                "tracking_warn_duration_s": TRACKING_WARN_DURATION_S,
                "tracking_critical_deg": math.degrees(TRACKING_CRITICAL_RAD),
                "saturation_duration_s": SATURATION_DURATION_S,
            },
            "events": list(self.session_events),
        }
        try:
            (path / "motion_inspector.json").write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except OSError as exc:
            self.get_logger().warning(f"failed to write session sidecar: {exc}")


SESSION_ROOT.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = MotionInspector()
executor = SingleThreadedExecutor()
executor.add_node(node)
ros_thread = threading.Thread(target=executor.spin, name="ros2-executor", daemon=True)
ros_thread.start()

app = FastAPI(title="Task Motion Inspector")


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
    return node.snapshot()


@app.get("/api/history/{joint}")
def history(joint: str, seconds: float = 20.0, max_points: int = 1000) -> dict[str, Any]:
    try:
        return node.history(joint, seconds, max_points)
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from exc


@app.get("/api/events")
async def events() -> StreamingResponse:
    async def stream():
        while True:
            yield "data: " + json.dumps(node.snapshot(), separators=(",", ":")) + "\n\n"
            await asyncio.sleep(0.1)
    return StreamingResponse(stream(), media_type="text/event-stream", headers={"Cache-Control": "no-cache"})


@app.get("/api/sessions")
def sessions() -> list[dict[str, Any]]:
    return node.sessions()


@app.get("/api/sessions/{session}/summary")
def session_summary(session: str) -> dict[str, Any]:
    path = SESSION_ROOT / safe_session_name(session) / "summary.json"
    if not path.is_file():
        raise HTTPException(404, "분석 결과가 아직 없습니다.")
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise HTTPException(500, f"분석 결과를 읽지 못했습니다: {exc}") from exc


@app.get("/api/sessions/{session}/report")
def session_report(session: str) -> FileResponse:
    path = SESSION_ROOT / safe_session_name(session) / "report.md"
    if not path.is_file():
        raise HTTPException(404, "보고서가 아직 없습니다.")
    return FileResponse(path, media_type="text/markdown", filename=f"{safe_session_name(session)}_report.md")


@app.post("/api/sessions/{session}/analyze")
def analyze_saved_session(session: str) -> dict[str, Any]:
    try:
        return node.analyze_saved_session(session)
    except (RuntimeError, ValueError, OSError) as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/api/record/start")
def record_start(request: StartRecording) -> dict[str, Any]:
    try:
        return {"ok": True, "session": node.start_recording(request.name)}
    except RuntimeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/api/record/stop")
def record_stop() -> dict[str, Any]:
    try:
        return {"ok": True, "session": node.stop_recording()}
    except RuntimeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.on_event("shutdown")
def shutdown() -> None:
    if node.record_process:
        node.stop_recording()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
