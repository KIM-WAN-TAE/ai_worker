#!/usr/bin/env python3
"""Fail-closed arm command-source arbiter for F2 policy/teleoperation handoff."""

from __future__ import annotations

import copy
import glob
import json
import math
import subprocess
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory


MODE_NONE = "NONE"
MODE_POLICY = "POLICY"
MODE_TELEOP = "TELEOP"
ARMS = ("left", "right")


@dataclass
class SourceSample:
    message: Optional[JointTrajectory] = None
    seen_s: Optional[float] = None
    error: str = "not received"


class CommandSourceArbiterCore:
    """ROS-independent validation and lease state used by the node and tests."""

    def __init__(
        self,
        joint_names: dict[str, list[str]],
        *,
        max_age_s: float = 0.25,
        max_pair_skew_s: float = 0.05,
        position_limits: Optional[dict[str, list[tuple[float, float]]]] = None,
    ) -> None:
        self.joint_names = {arm: list(joint_names[arm]) for arm in ARMS}
        self.position_limits = None
        if position_limits is not None:
            normalized_limits = {
                arm: [
                    (float(lower), float(upper))
                    for lower, upper in position_limits[arm]
                ]
                for arm in ARMS
            }
            for arm in ARMS:
                if len(normalized_limits[arm]) != len(self.joint_names[arm]):
                    raise ValueError(
                        f"position limit length mismatch for {arm}: "
                        f"{len(normalized_limits[arm])} != {len(self.joint_names[arm])}"
                    )
                for lower, upper in normalized_limits[arm]:
                    if (
                        not math.isfinite(lower)
                        or not math.isfinite(upper)
                        or lower > upper
                    ):
                        raise ValueError(
                            f"invalid position limits for {arm}: {(lower, upper)!r}"
                        )
            self.position_limits = normalized_limits
        self.max_age_s = max(0.01, float(max_age_s))
        self.max_pair_skew_s = max(0.0, float(max_pair_skew_s))
        self.mode = MODE_NONE
        self.fault = ""
        self.samples = {
            source: {arm: SourceSample() for arm in ARMS}
            for source in (MODE_POLICY, MODE_TELEOP)
        }

    @staticmethod
    def source_for_arm(mode: str, arm: str) -> str:
        """Map one ownership mode and arm to its physical input source."""
        if arm not in ARMS:
            raise ValueError(f"unknown arm: {arm}")
        if mode in (MODE_POLICY, MODE_TELEOP):
            return mode
        raise ValueError(f"unknown ownership mode: {mode}")

    def update(self, source: str, arm: str, msg: JointTrajectory, now_s: float) -> bool:
        if source not in self.samples or arm not in ARMS:
            raise ValueError(f"unknown source/arm: {source}/{arm}")
        error = self._validate(arm, msg)
        sample = self.samples[source][arm]
        sample.message = copy.deepcopy(msg) if not error else None
        sample.seen_s = float(now_s)
        sample.error = error
        active_source = None
        if self.mode != MODE_NONE:
            active_source = self.source_for_arm(self.mode, arm)
        if error and active_source == source:
            self.latch_fault(f"{source.lower()} {arm}: {error}")
        return not error

    def _validate(self, arm: str, msg: JointTrajectory) -> str:
        expected = self.joint_names[arm]
        names = list(getattr(msg, "joint_names", []) or [])
        if names != expected:
            return f"joint order mismatch: {names!r} != {expected!r}"
        points = list(getattr(msg, "points", []) or [])
        if len(points) != 1:
            return f"expected exactly one trajectory point, got {len(points)}"
        positions = list(getattr(points[0], "positions", []) or [])
        if len(positions) != len(expected):
            return f"position length mismatch: {len(positions)} != {len(expected)}"
        if not all(math.isfinite(float(value)) for value in positions):
            return "positions contain NaN or Inf"
        if self.position_limits is not None:
            for name, value, (lower, upper) in zip(
                expected,
                positions,
                self.position_limits[arm],
            ):
                numeric = float(value)
                if numeric < lower or numeric > upper:
                    return (
                        f"position limit violation: {name}={numeric:.9f} "
                        f"outside [{lower:.9f}, {upper:.9f}]"
                    )
        return ""

    def readiness(self, mode: str, now_s: float) -> tuple[bool, str]:
        if mode not in (MODE_POLICY, MODE_TELEOP):
            return False, f"unknown ownership mode {mode}"
        seen = []
        for arm in ARMS:
            source = self.source_for_arm(mode, arm)
            sample = self.samples[source][arm]
            if sample.error:
                return False, f"{arm}/{source.lower()}: {sample.error}"
            if sample.seen_s is None:
                return False, f"{arm}/{source.lower()}: not received"
            age = max(0.0, float(now_s) - sample.seen_s)
            if age > self.max_age_s:
                return False, f"{arm}/{source.lower()}: stale ({age:.3f}s)"
            seen.append(sample.seen_s)
        skew = max(seen) - min(seen)
        if skew > self.max_pair_skew_s:
            return False, f"arm pair skew {skew:.3f}s exceeds {self.max_pair_skew_s:.3f}s"
        return True, "ready"

    def select(self, mode: str, now_s: float) -> tuple[bool, str]:
        if self.fault:
            return False, f"fault latched: {self.fault}"
        ready, reason = self.readiness(mode, now_s)
        if not ready:
            self.mode = MODE_NONE
            return False, reason
        self.mode = mode
        return True, f"selected {mode}"

    def select_none(self) -> None:
        self.mode = MODE_NONE

    def clear_fault(self) -> None:
        self.fault = ""
        self.mode = MODE_NONE

    def latch_fault(self, reason: str) -> None:
        self.fault = str(reason)
        self.mode = MODE_NONE

    def watchdog(self, now_s: float) -> tuple[bool, str]:
        if self.mode == MODE_NONE:
            return True, "idle"
        active = self.mode
        ready, reason = self.readiness(active, now_s)
        if not ready:
            self.latch_fault(f"{active.lower()} lease lost: {reason}")
            return False, self.fault
        return True, "ready"

    def status(self, now_s: float) -> dict:
        sources = {}
        for mode in (MODE_POLICY, MODE_TELEOP):
            ready, reason = self.readiness(mode, now_s)
            sources[mode.lower()] = {"ready": ready, "reason": reason}
        return {"mode": self.mode, "fault": self.fault, "sources": sources}


class CommandSourceArbiter(Node):
    """Sole publisher to the F2 arm controller input topics."""

    DEFAULT_JOINTS = {
        "left": [
            "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
            "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1",
        ],
        "right": [
            "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
            "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1",
        ],
    }
    # Effective F2 limits. J7 uses the installed PH42 tick ranges rather than
    # the wider nominal URDF limits; an out-of-range chunk is rejected whole.
    F2_POSITION_LIMITS = {
        "left": [
            (-math.pi, math.pi),
            (0.0, math.pi),
            (-math.pi, math.pi),
            (-2.93613510181236, 1.0785802414820915),
            (-math.pi, math.pi),
            (-math.pi / 2.0, math.pi / 2.0),
            (-math.pi / 2.0, 1.3639424895214947),
            (0.0, 1.1),
        ],
        "right": [
            (-math.pi, math.pi),
            (-math.pi, 0.0),
            (-math.pi, math.pi),
            (-2.93613510181236, 1.0785802414820915),
            (-math.pi, math.pi),
            (-math.pi / 2.0, math.pi / 2.0),
            (-1.3639424895214947, math.pi / 2.0),
            (0.0, 1.1),
        ],
    }

    def __init__(self) -> None:
        super().__init__("command_source_arbiter")
        self.declare_parameter("max_source_age_s", 0.25)
        self.declare_parameter("max_pair_skew_s", 0.05)
        self.declare_parameter("manage_leader_service", True)
        self.declare_parameter("leader_service_name", "avatar_bringup")
        self.declare_parameter("policy_left_topic", "/intervention/policy/arm_left")
        self.declare_parameter("policy_right_topic", "/intervention/policy/arm_right")
        self.declare_parameter("teleop_left_topic", "/intervention/teleop/arm_left")
        self.declare_parameter("teleop_right_topic", "/intervention/teleop/arm_right")
        self.declare_parameter(
            "final_left_topic",
            "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        )
        self.declare_parameter(
            "final_right_topic",
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        )
        self.declare_parameter("left_joint_names", self.DEFAULT_JOINTS["left"])
        self.declare_parameter("right_joint_names", self.DEFAULT_JOINTS["right"])

        joints = {
            arm: list(self.get_parameter(f"{arm}_joint_names").value)
            for arm in ARMS
        }
        self.core = CommandSourceArbiterCore(
            joints,
            max_age_s=float(self.get_parameter("max_source_age_s").value),
            max_pair_skew_s=float(self.get_parameter("max_pair_skew_s").value),
            position_limits=self.F2_POSITION_LIMITS,
        )
        self._intervention_enabled = False
        self._source_topics = {
            MODE_POLICY: {
                arm: str(self.get_parameter(f"policy_{arm}_topic").value)
                for arm in ARMS
            },
            MODE_TELEOP: {
                arm: str(self.get_parameter(f"teleop_{arm}_topic").value)
                for arm in ARMS
            },
        }
        self._final_topics = {
            arm: str(self.get_parameter(f"final_{arm}_topic").value)
            for arm in ARMS
        }

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._final_publishers = {
            arm: self.create_publisher(JointTrajectory, self._final_topics[arm], reliable)
            for arm in ARMS
        }
        self._source_subscriptions = []
        for source in (MODE_POLICY, MODE_TELEOP):
            for arm in ARMS:
                self._source_subscriptions.append(self.create_subscription(
                    JointTrajectory,
                    self._source_topics[source][arm],
                    lambda msg, src=source, side=arm: self._source_callback(src, side, msg),
                    reliable,
                ))

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, "/command_source_arbiter/status", status_qos)
        self._intervention_publisher = self.create_publisher(
            Bool, "/leader/intervention_mode", status_qos)
        self._leader_command_publishers = {
            arm: self.create_publisher(UInt8, f"/leader/{arm}_command", 10)
            for arm in ARMS
        }

        self.create_service(Trigger, "~/select_none", self._select_none)
        self.create_service(Trigger, "~/select_policy", self._select_policy)
        self.create_service(Trigger, "~/select_teleop", self._select_teleop)
        self.create_service(Trigger, "~/clear_fault", self._clear_fault)
        self.create_service(Trigger, "~/prepare_intervention", self._prepare_intervention)
        self.create_service(Trigger, "~/disable_intervention", self._disable_intervention)
        self.create_service(Trigger, "~/get_status", self._get_status)
        self.create_timer(0.05, self._watchdog)
        self.create_timer(0.2, self._publish_status)
        self._publish_intervention_mode()
        self._publish_status()
        self.get_logger().info("F2 command-source arbiter ready in fail-closed NONE mode")

    @staticmethod
    def _now_s() -> float:
        return time.monotonic()

    def _source_callback(self, source: str, arm: str, msg: JointTrajectory) -> None:
        valid = self.core.update(source, arm, msg, self._now_s())
        active_source = None
        if self.core.mode != MODE_NONE:
            active_source = self.core.source_for_arm(self.core.mode, arm)
        if valid and active_source == source:
            self._final_publishers[arm].publish(msg)
        self._publish_status()

    def _publisher_conflict(self) -> str:
        conflicts = []
        for arm in ARMS:
            infos = self.get_publishers_info_by_topic(self._final_topics[arm])
            others = [info for info in infos if info.node_name != self.get_name()]
            if others:
                names = sorted({f"{item.node_namespace}/{item.node_name}" for item in others})
                conflicts.append(f"{arm}: {', '.join(names)}")
        return "; ".join(conflicts)

    def _select(self, source: str, response: Trigger.Response) -> Trigger.Response:
        conflict = self._publisher_conflict()
        if conflict:
            self.core.latch_fault(f"final-topic publisher conflict: {conflict}")
            response.success = False
            response.message = self.core.fault
            self._publish_status()
            return response
        if source == MODE_POLICY:
            self._release_leader_arms()
        success, message = self.core.select(source, self._now_s())
        response.success = success
        response.message = message
        if success:
            for arm in ARMS:
                arm_source = self.core.source_for_arm(source, arm)
                sample = self.core.samples[arm_source][arm]
                if sample.message is not None:
                    self._final_publishers[arm].publish(sample.message)
        self._publish_status()
        return response

    def _select_none(self, _request, response):
        self.core.select_none()
        self._release_leader_arms()
        response.success = True
        response.message = "selected NONE; both A3 arms released"
        self._publish_status()
        return response

    def _select_policy(self, _request, response):
        return self._select(MODE_POLICY, response)

    def _select_teleop(self, _request, response):
        return self._select(MODE_TELEOP, response)

    def _clear_fault(self, _request, response):
        self.core.clear_fault()
        response.success = True
        response.message = "fault cleared; mode remains NONE"
        self._publish_status()
        return response

    def _prepare_intervention(self, _request, response):
        if self.core.fault:
            response.success = False
            response.message = f"fault latched: {self.core.fault}"
            self._publish_status()
            return response
        self.core.select_none()
        self._intervention_enabled = True
        self._publish_intervention_mode()
        self._release_leader_arms()
        ok, message = self._change_leader_service(up=True)
        if not ok:
            self._intervention_enabled = False
            self._publish_intervention_mode()
        response.success = ok
        response.message = message
        self._publish_status()
        return response

    def _disable_intervention(self, _request, response):
        self.core.select_none()
        self._release_leader_arms()
        self._intervention_enabled = False
        self._publish_intervention_mode()
        ok, message = self._change_leader_service(up=False)
        if not ok:
            # The leader may still be running. Keep save-pose commands blocked
            # and expose intervention=true so the caller can retry disable.
            self._intervention_enabled = True
            self._publish_intervention_mode()
        response.success = ok
        response.message = message
        self._publish_status()
        return response

    def _get_status(self, _request, response):
        response.success = True
        response.message = json.dumps(self._status_payload(), sort_keys=True)
        return response

    def _release_leader_arms(self, arms=ARMS) -> None:
        message = UInt8()
        message.data = 0
        for arm in arms:
            self._leader_command_publishers[arm].publish(message)

    def _publish_intervention_mode(self) -> None:
        message = Bool()
        message.data = self._intervention_enabled
        self._intervention_publisher.publish(message)

    def _watchdog(self) -> None:
        previous = self.core.mode
        ok, reason = self.core.watchdog(self._now_s())
        if not ok and previous != MODE_NONE:
            self.get_logger().error(f"Command source fail-closed: {reason}")
            self._release_leader_arms()
            self._publish_status()

    def _status_payload(self) -> dict:
        payload = self.core.status(self._now_s())
        payload["intervention_enabled"] = self._intervention_enabled
        payload["final_topics"] = dict(self._final_topics)
        payload["publisher_conflict"] = self._publisher_conflict()
        return payload

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(self._status_payload(), sort_keys=True)
        self._status_publisher.publish(message)

    def _change_leader_service(self, *, up: bool) -> tuple[bool, str]:
        if not bool(self.get_parameter("manage_leader_service").value):
            return True, "leader service management disabled; readiness will be monitored"
        service = str(self.get_parameter("leader_service_name").value)
        matches = sorted(glob.glob("/package/admin/s6-*/command/s6-rc"))
        if not matches:
            return False, "s6-rc not found; cannot manage A3 leader service"
        command = [matches[-1], "-u" if up else "-d", "change", service]
        try:
            result = subprocess.run(
                command,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=20.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, f"leader service request failed: {exc}"
        if result.returncode != 0:
            detail = (result.stderr or result.stdout or "unknown s6 error").strip()
            return False, f"leader service request failed: {detail}"
        state = "started" if up else "stopped"
        return True, f"A3 leader service {state}; waiting for source readiness"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandSourceArbiter()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.core.select_none()
        if rclpy.ok():
            node._release_leader_arms()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
