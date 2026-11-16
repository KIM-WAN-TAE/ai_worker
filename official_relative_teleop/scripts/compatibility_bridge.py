#!/usr/bin/env python3
"""Translate existing Mini controls and retain the current arbiter as output owner.

Only upstream Cyclo generates active arm trajectories. Standby samples follow
measured joints so selecting TELEOP cannot replay a pre-intervention target.
"""
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String, UInt8
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotis_interfaces.msg import TeleoperationCommand, ControlModeStatus
from robotis_interfaces.srv import SetPreset

JOINTS = {side: [f'arm_{letter}_joint{i}' for i in range(1, 8)] + [f'gripper_{letter}_joint1']
          for side, letter in [('left', 'l'), ('right', 'r')]}


class CompatibilityBridge(Node):
    def __init__(self):
        super().__init__('official_relative_bridge')
        self.declare_parameter('auto_select_teleop', True)
        self.arbiter = {}
        self.arbiter_seen = 0.
        self.feedback = {}
        self.feedback_seen = 0.
        self.source = ''
        self.source_seen = 0.
        self.source_future = None
        self.cancel_future = None
        self.source_cancelled = False
        self.cancel_request_time = 0.
        self.source_request_time = 0.
        self.route_future = None
        self.route_attempted = False
        self.sequence = 0
        self.requires_model_reset = True
        self.route_changed = 0.
        self.control = ControlModeStatus()
        self.control_seen = 0.
        self.gripper_anchor = {side: None for side in JOINTS}
        self.gripper_target = {}
        self.executed_gripper = {}
        self.preset_queue = deque()
        self.preset_future = None
        self.preset_inflight_side = None
        self.preset_wait = None
        self.preset_generation = 0
        self.output_seen = {side: 0. for side in JOINTS}
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.outputs = {side: self.create_publisher(JointTrajectory, f'/intervention/teleop/arm_{side}', 10)
                        for side in JOINTS}
        self.buttons = self.create_publisher(TeleoperationCommand, '/leader/teleoperation/joystick_command', 10)
        self.grippers = {side: self.create_publisher(JointTrajectory, f'/official_relative/action/{side}/gripper', 1)
                         for side in JOINTS}
        self.recorded_commands = {side: self.create_publisher(
            JointState, f'/robot/arm_{side}_leader/joint_states', 10) for side in JOINTS}
        self.source_client = self.create_client(SetBool, '/official_relative/set_source')
        self.cancel_client = self.create_client(Trigger, '/official_relative_controller/cancel_poses')
        self.route_client = self.create_client(Trigger, '/command_source_arbiter/select_teleop')
        self.preset_client = self.create_client(SetPreset, '/leader/teleoperation/set_preset')
        self.create_subscription(String, '/command_source_arbiter/status', self.on_arbiter, latched)
        self.create_subscription(String, '/official_relative/source', self.on_source, latched)
        self.create_subscription(JointState, '/joint_states', self.on_feedback, qos_profile_sensor_data)
        self.create_subscription(ControlModeStatus, '/official_relative/control/status', self.on_control, latched)
        for side in JOINTS:
            self.create_subscription(JointTrajectory, f'/official_relative/raw_gripper/{side}',
                                     lambda msg, s=side: self.on_gripper(s, msg), reliable)
            self.create_subscription(JointTrajectory, f'/command_source_arbiter/executed/arm_{side}',
                                     lambda msg, s=side: self.on_executed(s, msg), latched)
            self.create_subscription(UInt8, f'/leader/{side}_command',
                                     lambda msg, s=side: self.on_button(s, msg.data), 10)
            self.create_subscription(JointTrajectory, f'/official_relative/output/{side}',
                                     lambda msg, s=side: self.on_output(s, msg), reliable)
        self.create_timer(.02, self.tick)

    def prepared(self):
        return (time.monotonic()-self.arbiter_seen < .6
                and self.arbiter.get('intervention_enabled') is True
                and not self.arbiter.get('fault') and not self.arbiter.get('publisher_conflict'))

    def teleop_selected(self):
        return self.prepared() and self.arbiter.get('mode') == 'TELEOP'

    def source_fresh(self):
        return time.monotonic()-self.source_seen < .5

    def on_arbiter(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
        except (ValueError, TypeError):
            return
        previous = self.arbiter.get('mode')
        self.arbiter, self.arbiter_seen = value, time.monotonic()
        if previous != value.get('mode'):
            self.preset_generation += 1
            self.preset_queue.clear()
            self.preset_wait = None
            self.requires_model_reset = True
            self.route_changed = time.monotonic()
            self.disable_both()
            self.output_seen = {side: 0. for side in JOINTS}

    def on_source(self, message):
        if self.source != message.data:
            self.source_cancelled = False
        self.source, self.source_seen = message.data, time.monotonic()

    def on_feedback(self, message):
        if len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, message.position))
        if not all(name in values and math.isfinite(values[name]) for names in JOINTS.values() for name in names):
            return
        self.feedback, self.feedback_seen = values, time.monotonic()

    def disable_both(self):
        self.gripper_anchor = {side: None for side in JOINTS}
        self.sequence += 1
        self.buttons.publish(TeleoperationCommand(request_id=self.sequence, target_arm='both',
                                                   command=TeleoperationCommand.COMMAND_DISABLE))

    def on_control(self, message):
        for side in JOINTS:
            was = self.control.requested_arms in (side, 'both')
            active = message.requested_arms in (side, 'both')
            if was != active or message.state in (message.STATE_ERROR, message.STATE_LOADING):
                self.gripper_anchor[side] = None
        self.control, self.control_seen = message, time.monotonic()

    def on_executed(self, side, message):
        if message.points and len(message.joint_names) == len(message.points[0].positions):
            recorded = JointState(name=message.joint_names, position=message.points[0].positions)
            recorded.header.stamp = self.get_clock().now().to_msg()
            self.recorded_commands[side].publish(recorded)
        name = JOINTS[side][-1]
        if name in message.joint_names and message.points:
            index = message.joint_names.index(name)
            positions = message.points[0].positions
            if index < len(positions) and math.isfinite(positions[index]):
                self.executed_gripper[side] = positions[index]

    def on_gripper(self, side, message):
        now = time.monotonic()
        if (not self.teleop_selected() or self.requires_model_reset or not self.source_fresh()
                or self.source != 'teleop' or now-self.feedback_seen >= .25
                or not self.control_seen or self.control.state not in (
                    self.control.STATE_HOLDING, self.control.STATE_ACTIVE) or not message.points):
            return
        name = JOINTS[side][-1]
        if message.joint_names != [name] or len(message.points[0].positions) != 1:
            return
        raw = message.points[0].positions[0]
        if not math.isfinite(raw):
            return
        active = self.control.requested_arms in (side, 'both')
        if not active or self.gripper_anchor[side] is None:
            target = self.executed_gripper.get(side, self.feedback[name])
            self.gripper_target[side] = min(1., max(0., target))
            self.gripper_anchor[side] = (raw, self.gripper_target[side]) if active else None
        else:
            leader, commanded = self.gripper_anchor[side]
            delta = raw-leader
            target = commanded + math.copysign(max(0., abs(delta)-.02), delta)
            self.gripper_target[side] = min(1., max(0., target))
        self.grippers[side].publish(JointTrajectory(joint_names=[name], points=[
            JointTrajectoryPoint(positions=[self.gripper_target[side]])]))

    def on_button(self, side, value):
        if value == 0:
            preset_busy = getattr(self.control, f'{side}_preset_state') in (
                self.control.PRESET_WAITING, self.control.PRESET_MOVING, self.control.PRESET_HOLDING)
            pending = (any(s == side for s, _ in self.preset_queue)
                       or self.preset_inflight_side == side
                       or self.preset_wait is not None and self.preset_wait[1] == side)
            if preset_busy or pending:
                # Upstream arm-disable only disables manual teleoperation; it
                # does not cancel a preset. A source reset cancels pose sequences
                # and immediately gates both outputs to measured holds.
                self.preset_generation += 1
                self.preset_queue.clear()
                self.preset_wait = None
                self.requires_model_reset = True
                self.route_changed = time.monotonic()
                self.disable_both()
        if value != 0 and not (self.teleop_selected() and not self.requires_model_reset
                               and self.source_fresh() and self.source == 'teleop'):
            self.get_logger().warning(f'Ignored {side} command {value}: official TELEOP is not ready')
            return
        if value in (0, 1, 2):
            self.sequence += 1
            command = {0: TeleoperationCommand.COMMAND_DISABLE, 1: TeleoperationCommand.COMMAND_ENABLE,
                       2: TeleoperationCommand.COMMAND_TOGGLE}[value]
            self.buttons.publish(TeleoperationCommand(request_id=self.sequence, target_arm=side, command=command))
        elif value in (3, 4) and self.preset_client.service_is_ready():
            if len(self.preset_queue) < 16:
                self.preset_queue.append((side,value))

    def preset_done(self, future, side, value, generation):
        self.preset_future = None
        self.preset_inflight_side = None
        if generation != self.preset_generation:
            return
        try:
            result = future.result()
            if not result.accepted:
                self.get_logger().warning(f'Preset rejected: {result.message}')
            else:
                self.preset_wait = (result.transition_id, side, value, time.monotonic())
        except Exception as error:
            self.get_logger().error(f'Preset failed: {error}')

    def on_output(self, side, message):
        if (self.teleop_selected() and self.source_fresh() and self.source == 'teleop'
                and not self.requires_model_reset
                and time.monotonic()-self.feedback_seen < .25):
            # A loaded follower gripper can deflect away from its commanded
            # grasp. Retain that command across disabled-arm holds/handoffs.
            preset = getattr(self.control, f'{side}_preset_state')
            name = JOINTS[side][-1]
            if (preset == self.control.PRESET_UNUSED and message.points
                    and name in message.joint_names):
                index = message.joint_names.index(name)
                if index < len(message.points[0].positions):
                    target = self.executed_gripper.get(side, self.feedback[name])
                    if self.control.requested_arms in (side, 'both'):
                        target = self.gripper_target.get(side, target)
                    message.points[0].positions[index] = min(1., max(0., target))
            # The existing arbiter validates joint order, limits, finite values and leases.
            self.outputs[side].publish(message)
            self.output_seen[side] = time.monotonic()

    def tick(self):
        now = time.monotonic()
        desired = self.teleop_selected()
        # The official leader holds one preset request at a time. Wait for its
        # follower acknowledgement before sending the other arm's request.
        if self.preset_wait:
            transition, side, value, started = self.preset_wait
            acknowledged = (self.control.transition_id >= transition
                and getattr(self.control, f'{side}_preset_id') == value
                and getattr(self.control, f'{side}_preset_state') in (
                    self.control.PRESET_MOVING, self.control.PRESET_HOLDING))
            if acknowledged:
                self.preset_wait = None
            elif now-started > 3.:
                self.get_logger().error('Preset acknowledgement timed out; queued requests discarded')
                self.preset_wait = None
                self.preset_queue.clear()
        if (desired and not self.requires_model_reset and self.source_fresh() and self.source == 'teleop'
                and self.preset_queue and self.preset_future is None and self.preset_wait is None
                and self.preset_client.service_is_ready()):
            side, value = self.preset_queue.popleft()
            generation = self.preset_generation
            request = SetPreset.Request(target_arm=side, left_preset_id=value, right_preset_id=value)
            self.preset_inflight_side = side
            self.preset_future = self.preset_client.call_async(request)
            self.preset_future.add_done_callback(
                lambda future, s=side, v=value, g=generation: self.preset_done(future,s,v,g))
        if self.source_future is not None and self.source_future.done():
            try:
                result = self.source_future.result()
                if not result.success:
                    # A late preset request can arrive just after cancellation.
                    # Keep outputs gated and cancel again before retrying.
                    self.source_cancelled = False
                    self.get_logger().warning(f'Cyclo source request rejected: {result.message}')
            except Exception as error:
                self.get_logger().error(f'Cyclo source request failed: {error}')
            self.source_future = None
        if (desired and self.requires_model_reset and self.source_future is None
                and self.source_fresh() and self.source == 'model_action'
                and self.source_seen > self.route_changed):
            self.requires_model_reset = False
        want_teleop = desired and not self.requires_model_reset
        expected = 'teleop' if want_teleop else 'model_action'
        if self.cancel_future is not None and self.cancel_future.done():
            try:
                result = self.cancel_future.result()
                self.source_cancelled = result.success
                if not result.success:
                    self.get_logger().error(f'Pose cancellation rejected: {result.message}')
            except Exception as error:
                self.get_logger().error(f'Pose cancellation failed: {error}')
            self.cancel_future = None
        needs_cancel = self.source == 'teleop' and not want_teleop and not self.source_cancelled
        if (needs_cancel and self.cancel_future is None and now-self.cancel_request_time > .5
                and self.cancel_client.service_is_ready()):
            self.cancel_request_time = now
            self.cancel_future = self.cancel_client.call_async(Trigger.Request())
        if (self.source != expected and self.source != 'switching' and self.source_future is None
                and not needs_cancel and now-self.source_request_time > .5
                and self.source_client.service_is_ready()):
            self.disable_both()
            self.source_request_time = now
            self.source_future = self.source_client.call_async(SetBool.Request(data=want_teleop))
        # Never replay an old official trajectory while external policy/waypoints move the robot.
        standby = not desired or self.requires_model_reset or (self.source_fresh() and self.source != 'teleop')
        if self.prepared() and now-self.feedback_seen < .25 and standby:
            for side, names in JOINTS.items():
                positions = [self.feedback[name] for name in names]
                positions[-1] = min(1., max(0., self.executed_gripper.get(side, positions[-1])))
                point = JointTrajectoryPoint(positions=positions)
                self.outputs[side].publish(JointTrajectory(joint_names=names, points=[point]))
        if (bool(self.get_parameter('auto_select_teleop').value) and not self.route_attempted
                and self.prepared() and self.arbiter.get('mode') == 'NONE'
                and self.arbiter.get('sources', {}).get('teleop', {}).get('ready') is True
                and self.route_client.service_is_ready()):
            self.route_attempted = True
            self.route_future = self.route_client.call_async(Trigger.Request())
            self.route_future.add_done_callback(self.route_done)

    def route_done(self, future):
        try:
            result = future.result()
            if not result.success:
                self.get_logger().error(f'Arbiter TELEOP selection rejected: {result.message}')
        except Exception as error:
            self.get_logger().error(f'Arbiter TELEOP selection failed: {error}')


def main():
    rclpy.init()
    node = CompatibilityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
