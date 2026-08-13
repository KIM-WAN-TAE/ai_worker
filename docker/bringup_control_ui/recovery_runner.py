#!/usr/bin/env python3
"""Move an active follower stack through its model-specific recovery YAML."""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time
from pathlib import Path

from control_msgs.action import FollowJointTrajectory
from dynamixel_interfaces.msg import DynamixelState
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from recovery_config import RECOVERY_PROFILES, RecoveryGroup, load_recovery_groups


class RecoveryNode(Node):
    def __init__(self, model: str, groups: tuple[RecoveryGroup, ...]) -> None:
        super().__init__(f"{model}_pose_recovery_coordinator")
        self.groups = groups
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        self.feedback_seen = 0.0
        self.dxl_seen = 0.0
        self.dxl_ok = False
        self.estop_active = False
        self.cancelled = False
        self.goal_handles = []
        self.action_clients = {
            group.executor_name: ActionClient(self, FollowJointTrajectory, group.action)
            for group in groups
        }
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/joint_states", self._feedback, qos)
        self.create_subscription(DynamixelState, "/ffw_follower/dxl_state", self._dxl, qos)
        estop_topic = os.environ.get("AI_WORKER_ESTOP_TOPIC", "").strip()
        if estop_topic:
            self.create_subscription(Bool, estop_topic, self._estop, 10)

    def _feedback(self, msg: JointState) -> None:
        self.feedback_seen = time.monotonic()
        for index, name in enumerate(msg.name):
            if index < len(msg.position) and math.isfinite(msg.position[index]):
                self.positions[name] = float(msg.position[index])
            if index < len(msg.velocity) and math.isfinite(msg.velocity[index]):
                self.velocities[name] = float(msg.velocity[index])

    def _dxl(self, msg: DynamixelState) -> None:
        self.dxl_seen = time.monotonic()
        self.dxl_ok = int(msg.comm_state) == 0 and not any(int(value) for value in msg.dxl_hw_state)

    def _estop(self, msg: Bool) -> None:
        self.estop_active = bool(msg.data)

    def abort_reason(self) -> str | None:
        if self.cancelled:
            return "사용자 또는 supervisor가 복구를 중단했습니다."
        if self.estop_active:
            return "E-stop이 감지되었습니다."
        if self.dxl_seen and not self.dxl_ok:
            return "DXL 통신 또는 hardware error가 감지되었습니다."
        if self.dxl_seen and time.monotonic() - self.dxl_seen >= 2.0:
            return "DXL 상태 수신이 끊겼습니다."
        if self.feedback_seen and time.monotonic() - self.feedback_seen >= 1.0:
            return "/joint_states feedback 수신이 끊겼습니다."
        return None

    def cancel(self) -> None:
        self.cancelled = True
        for handle in self.goal_handles:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    def spin_until(self, predicate, timeout: float, description: str) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            reason = self.abort_reason()
            if reason:
                raise RuntimeError(reason)
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f"{description} timeout ({timeout:.1f}s)")

    def preflight(self) -> None:
        required = {joint for group in self.groups for joint in group.joints}
        print("[PREFLIGHT] 최신 joint feedback과 DXL 상태를 기다립니다.", flush=True)
        self.spin_until(
            lambda: required.issubset(self.positions)
            and time.monotonic() - self.feedback_seen < 1.0
            and self.dxl_seen > 0
            and time.monotonic() - self.dxl_seen < 2.0
            and self.dxl_ok,
            8.0,
            "feedback/DXL 준비",
        )
        for group in self.groups:
            print(f"[PREFLIGHT] {group.label} Action Server 확인: {group.action}", flush=True)
            if not self.action_clients[group.executor_name].wait_for_server(timeout_sec=5.0):
                raise RuntimeError(f"Action Server를 찾을 수 없습니다: {group.action}")

    @staticmethod
    def trajectory(joints, start, target, duration: float) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joints)
        points = 100
        for index in range(points):
            elapsed = duration * index / (points - 1)
            normalized = elapsed / duration
            n2 = normalized * normalized
            n3 = n2 * normalized
            n4 = n3 * normalized
            n5 = n4 * normalized
            position_gain = 10 * n3 - 15 * n4 + 6 * n5
            velocity_gain = (30 * n2 - 60 * n3 + 30 * n4) / duration
            acceleration_gain = (60 * normalized - 180 * n2 + 120 * n3) / (duration * duration)
            point = JointTrajectoryPoint()
            point.positions = [a + (b - a) * position_gain for a, b in zip(start, target)]
            point.velocities = [(b - a) * velocity_gain for a, b in zip(start, target)]
            point.accelerations = [(b - a) * acceleration_gain for a, b in zip(start, target)]
            point.time_from_start.sec = int(elapsed)
            point.time_from_start.nanosec = int((elapsed % 1.0) * 1e9)
            trajectory.points.append(point)
        return trajectory

    def run_phase(self, phase_index: int, phase_count: int) -> None:
        selected_steps = {
            group.executor_name: group.step_for_phase(phase_index)
            for group in self.groups
        }
        detail = ", ".join(
            f"{group.label}:{selected_steps[group.executor_name].name}"
            for group in self.groups
        )
        label = f"복구 phase {phase_index + 1}/{phase_count} ({detail})"
        print(f"[PHASE] {label} 시작 — 모든 그룹을 동시에 전송합니다.", flush=True)
        send_futures = {}
        for group in self.groups:
            start = tuple(self.positions[joint] for joint in group.joints)
            goal = FollowJointTrajectory.Goal()
            target = selected_steps[group.executor_name].positions
            goal.trajectory = self.trajectory(group.joints, start, target, group.duration)
            send_futures[group.executor_name] = self.action_clients[group.executor_name].send_goal_async(goal)

        self.spin_until(
            lambda: all(future.done() for future in send_futures.values()),
            5.0,
            f"{label} goal 응답",
        )
        handles = {}
        for executor_name, future in send_futures.items():
            handle = future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError(f"{executor_name} controller가 {label} goal을 거부했습니다.")
            handles[executor_name] = handle
        self.goal_handles = list(handles.values())

        result_futures = {name: handle.get_result_async() for name, handle in handles.items()}
        max_duration = max(group.duration for group in self.groups)
        self.spin_until(
            lambda: all(future.done() for future in result_futures.values()),
            max_duration + 8.0,
            f"{label} trajectory 결과",
        )
        for executor_name, future in result_futures.items():
            wrapped = future.result()
            if wrapped is None or wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                code = None if wrapped is None else wrapped.result.error_code
                detail = "" if wrapped is None else wrapped.result.error_string
                raise RuntimeError(f"{executor_name} {label} 실패: code={code} {detail}")

        def measured_complete() -> bool:
            if time.monotonic() - self.feedback_seen > 1.0:
                return False
            for group in self.groups:
                target = selected_steps[group.executor_name].positions
                for joint, expected in zip(group.joints, target):
                    if abs(self.positions.get(joint, math.inf) - expected) >= group.position_tolerance:
                        return False
                    velocity = self.velocities.get(joint)
                    if velocity is not None and abs(velocity) >= group.velocity_tolerance:
                        return False
            return True

        self.spin_until(measured_complete, 3.0, f"{label} measured pose 확인")
        self.goal_handles = []
        print(f"[PHASE] {label} 완료 — 모든 그룹 도달 및 정지 확인.", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Model-specific init pose recovery runner")
    parser.add_argument("--model", required=True, choices=sorted(RECOVERY_PROFILES))
    parser.add_argument("--config", type=Path, help="model initial-position YAML path")
    parser.add_argument(
        "--check-config",
        action="store_true",
        help="validate and print the YAML-derived plan without connecting to ROS or moving the robot",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        groups = load_recovery_groups(args.model, args.config)
    except ValueError as exc:
        print(f"[FAILED] {exc}", file=sys.stderr, flush=True)
        return 2

    config_path = args.config or Path(__file__).resolve().parents[2] / str(
        RECOVERY_PROFILES[args.model]["config"]
    )
    print(f"[CONFIG] model={args.model} yaml={config_path.resolve()}", flush=True)
    for group in groups:
        steps = " -> ".join(step.name for step in group.steps)
        print(
            f"[CONFIG] {group.label}: {steps} · {group.duration:.3g}s · {group.action}",
            flush=True,
        )
    if args.check_config:
        print("[CHECK] YAML 복구 설정 검증 완료. ROS goal은 전송하지 않았습니다.", flush=True)
        return 0

    rclpy.init()
    node = RecoveryNode(args.model, groups)

    def stop_handler(_signum, _frame):
        node.cancel()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        node.preflight()
        phase_count = max(len(group.steps) for group in groups)
        for phase_index in range(phase_count):
            node.run_phase(phase_index, phase_count)
        print(f"[COMPLETE] {args.model.upper()} init pose 복구가 완료되었습니다.", flush=True)
        return 0
    except Exception as exc:
        print(f"[FAILED] {exc}", file=sys.stderr, flush=True)
        node.cancel()
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
