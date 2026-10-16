#!/usr/bin/env python3
"""Non-actuating ROS graph smoke test for F2 policy/Intervention hand-off."""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ffw_bringup.command_source_arbiter import CommandSourceArbiter


def trajectory(names: list[str], value: float) -> JointTrajectory:
    message = JointTrajectory()
    message.joint_names = list(names)
    point = JointTrajectoryPoint()
    point.positions = [value] * len(names)
    message.points = [point]
    return message


class HandoffProbe(Node):
    def __init__(self) -> None:
        super().__init__("mock_intervention_handoff_probe")
        self.received = {"left": [], "right": []}
        self.source_publishers = {
            source: {
                arm: self.create_publisher(
                    JointTrajectory,
                    f"/intervention/{source}/arm_{arm}",
                    10,
                )
                for arm in ("left", "right")
            }
            for source in ("policy", "teleop", "waypoint")
        }
        final_topics = {
            "left": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
            "right": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        }
        self.final_subscriptions = [
            self.create_subscription(
                JointTrajectory,
                topic,
                lambda msg, side=arm: self.received[side].append(msg),
                10,
            )
            for arm, topic in final_topics.items()
        ]
        self.arbiter_clients = {
            action: self.create_client(
                Trigger,
                f"/command_source_arbiter/{action}",
            )
            for action in (
                "prepare_intervention",
                "select_none",
                "select_teleop",
                "select_waypoint",
                "select_policy",
                "disable_intervention",
            )
        }

    def call(self, action: str, timeout: float = 2.0):
        client = self.arbiter_clients[action]
        if not client.wait_for_service(timeout_sec=timeout):
            raise AssertionError(f"service unavailable: {action}")
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise AssertionError(f"service timeout: {action}")
        response = future.result()
        if response is None or not response.success:
            detail = "no response" if response is None else response.message
            raise AssertionError(f"service failed: {action}: {detail}")
        return response

    def publish_pair(self, source: str, value: float) -> None:
        for arm in ("left", "right"):
            names = CommandSourceArbiter.DEFAULT_JOINTS[arm]
            self.source_publishers[source][arm].publish(trajectory(names, value))


def wait_until(predicate, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition timed out")


def main() -> int:
    rclpy.init(args=["--ros-args", "-p", "manage_leader_service:=false"])
    arbiter = CommandSourceArbiter()
    probe = HandoffProbe()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(arbiter)
    executor.add_node(probe)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        time.sleep(0.5)
        probe.call("prepare_intervention")
        for _ in range(3):
            probe.publish_pair("teleop", 0.0)
            time.sleep(0.03)
        probe.call("select_teleop")
        wait_until(lambda: all(probe.received[arm] for arm in ("left", "right")))

        probe.call("select_none")
        time.sleep(0.1)
        blocked_counts = {arm: len(items) for arm, items in probe.received.items()}
        for _ in range(3):
            probe.publish_pair("teleop", 0.0)
            time.sleep(0.03)
        time.sleep(0.1)
        assert blocked_counts == {
            arm: len(items) for arm, items in probe.received.items()
        }, "TELEOP messages leaked while the arbiter was in NONE"

        for _ in range(3):
            probe.publish_pair("waypoint", 0.0)
            time.sleep(0.03)
        probe.call("select_waypoint")
        wait_until(
            lambda: all(
                len(probe.received[arm]) > blocked_counts[arm]
                for arm in ("left", "right")
            )
        )
        probe.call("select_none")
        waypoint_counts = {arm: len(items) for arm, items in probe.received.items()}

        for _ in range(3):
            probe.publish_pair("policy", 0.0)
            time.sleep(0.03)
        probe.call("select_policy")
        wait_until(
            lambda: all(
                len(probe.received[arm]) > blocked_counts[arm]
                and len(probe.received[arm]) > waypoint_counts[arm]
                for arm in ("left", "right")
            )
        )
        probe.call("select_none")
        probe.call("disable_intervention")
        print(
            "mock hand-off passed: NONE -> TELEOP -> NONE -> "
            "WAYPOINT -> NONE -> POLICY -> NONE"
        )
        return 0
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        probe.destroy_node()
        arbiter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
