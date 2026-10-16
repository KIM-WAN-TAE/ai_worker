import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request
from urllib.request import urlopen

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Path as PathMessage
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from std_msgs.msg import UInt32
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import xacro


LEFT_JOINTS = [f'arm_l_joint{index}' for index in range(1, 8)]
RIGHT_JOINTS = [f'arm_r_joint{index}' for index in range(1, 8)]
LEFT_COMMAND_JOINTS = LEFT_JOINTS + ['gripper_l_joint1']
RIGHT_COMMAND_JOINTS = RIGHT_JOINTS + ['gripper_r_joint1']


def quaternion_to_rpy_deg(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(
        sin_pitch) >= 1.0 else math.asin(sin_pitch)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return [math.degrees(value) for value in (roll, pitch, yaw)]


class WaypointProcess:

    def __init__(self, publish_enabled=False, tip_link='end_effector_l_link',
                 action_topic='/test/follow_joint_trajectory', arm=None,
                 max_waypoint_speed_scale=None, quiet=False, max_joint_step_rad=None):
        prefix = Path(get_package_prefix('waypoint_telep'))
        executable = prefix / 'lib' / 'waypoint_telep' / 'waypoint_telep_node'
        command = [
            str(executable),
            '--ros-args',
            '-p', f'publish_enabled:={str(publish_enabled).lower()}',
            '-p', f'direct_action_enabled:={str(publish_enabled).lower()}',
            '-p', f'tip_link:={tip_link}',
            '-p', (
                'gripper_joint:=gripper_r_joint1'
                if tip_link == 'end_effector_r_link' else
                'gripper_joint:=gripper_l_joint1'
            ),
            '-p', f'action_topic:={action_topic}',
            '-p', 'external_command_topic:=/test/external_command',
            '-p', 'command_quiet_s:=0.25',
        ]
        if max_waypoint_speed_scale is not None:
            command.extend([
                '-p', f'max_waypoint_speed_scale:={max_waypoint_speed_scale}',
            ])
        if max_joint_step_rad is not None:
            command.extend(['-p', f'max_joint_step_rad:={max_joint_step_rad}'])
        if arm is not None:
            command.extend([
                '-r', '__ns:=/waypoint_telep',
                '-r', f'__node:={arm}',
                '-p', (
                    'clicked_point_topic:='
                    f'/waypoint_telep/{arm}/clicked_point'
                ),
            ])
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL if quiet else subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

    def stop(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        output = self.process.stdout.read() if self.process.stdout else ''
        assert self.process.returncode in (0, -signal.SIGINT), output


class WaypointUiProcess:

    def __init__(self, port, arm='left', publish_enabled=False,
                 max_waypoint_speed_scale=None):
        prefix = Path(get_package_prefix('waypoint_telep'))
        executable = prefix / 'lib' / 'waypoint_telep' / 'waypoint_telep_ui'
        command = [
            str(executable),
            '--ros-args',
            '-p', f'port:={port}',
            '-p', f'arm:={arm}',
            '-p', f'publish_enabled:={str(publish_enabled).lower()}',
            '-p', 'intervention_controller_url:=disabled',
        ]
        if max_waypoint_speed_scale is not None:
            command.extend([
                '-p', f'max_waypoint_speed_scale:={max_waypoint_speed_scale}',
            ])
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.url = f'http://127.0.0.1:{port}'

    def stop(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        output = self.process.stdout.read() if self.process.stdout else ''
        assert self.process.returncode in (0, -signal.SIGINT), output


class ArbiterProcess:

    def __init__(self):
        executable = (
            Path(__file__).parents[2]
            / 'ffw_bringup' / 'ffw_bringup' / 'command_source_arbiter.py'
        )
        self.process = subprocess.Popen(
            [
                sys.executable, str(executable), '--ros-args',
                '-p', 'manage_leader_service:=false',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

    def stop(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        output = self.process.stdout.read() if self.process.stdout else ''
        assert self.process.returncode in (0, -signal.SIGINT), output


def unused_tcp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind(('127.0.0.1', 0))
        return server.getsockname()[1]


def http_json(url, path, payload=None, timeout=15):
    data = None if payload is None else json.dumps(payload).encode('utf-8')
    request = Request(
        url + path,
        data=data,
        headers={'Content-Type': 'application/json'},
        method='GET' if payload is None else 'POST',
    )
    try:
        with urlopen(request, timeout=timeout) as response:
            return response.status, json.loads(response.read().decode('utf-8'))
    except HTTPError as exception:
        detail = exception.read().decode('utf-8', errors='replace')
        exception.msg = f'{exception.msg}: {detail}'
        raise


def wait_for_http(url, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            return http_json(url, '/api/state')
        except OSError:
            time.sleep(0.05)
    raise TimeoutError(f'HTTP server did not start: {url}')


class MockGraph:

    def __init__(self):
        self.node = rclpy.create_node('waypoint_telep_mock_test')
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)

        model_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.model_publisher = self.node.create_publisher(
            String, '/robot_description', model_qos)
        # Match the live sensor stream. A reliable feedback writer can stall
        # this executor during DDS discovery and starve the TELEOP heartbeat.
        feedback_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_state_publisher = self.node.create_publisher(
            JointState, '/joint_states', feedback_qos)
        self.point_publisher = self.node.create_publisher(
            PointStamped, '/clicked_point', 10)
        self.pose_publisher = self.node.create_publisher(
            PoseStamped, '/waypoint_telep/waypoint_pose', 10)
        self.absolute_position_publisher = self.node.create_publisher(
            PointStamped, '/waypoint_telep/absolute_position', 10)
        self.absolute_orientation_publisher = self.node.create_publisher(
            QuaternionStamped, '/waypoint_telep/absolute_orientation', 10)
        self.relative_pose_publisher = self.node.create_publisher(
            PoseStamped, '/waypoint_telep/relative_waypoint', 10)
        self.frame_relative_pose_publisher = self.node.create_publisher(
            PoseStamped, '/waypoint_telep/frame_relative_waypoint', 10)
        self.relative_reference_publisher = self.node.create_publisher(
            Int32, '/waypoint_telep/relative_reference', 10)
        self.cartesian_jog_publisher = self.node.create_publisher(
            PointStamped, '/waypoint_telep/cartesian_jog', 10)
        self.joint_waypoint_publisher = self.node.create_publisher(
            JointState, '/waypoint_telep/joint_waypoint', 10)
        self.mode_publisher = self.node.create_publisher(
            String, '/waypoint_telep/path_mode', 10)
        self.dwell_publisher = self.node.create_publisher(
            Float64MultiArray, '/waypoint_telep/waypoint_dwell', 10)
        self.speed_scale_publisher = self.node.create_publisher(
            Float64MultiArray, '/waypoint_telep/waypoint_speed_scale', 10)
        self.waypoint_mode_publisher = self.node.create_publisher(
            Int32MultiArray, '/waypoint_telep/waypoint_mode', 10)
        self.joint_smoothing_publisher = self.node.create_publisher(
            Bool, '/waypoint_telep/joint_smoothing', 10)
        self.start_waypoint_publisher = self.node.create_publisher(
            UInt32, '/waypoint_telep/start_waypoint', 10)
        self.end_waypoint_publisher = self.node.create_publisher(
            UInt32, '/waypoint_telep/end_waypoint', 10)
        self.reorder_waypoint_publisher = self.node.create_publisher(
            Int32MultiArray, '/waypoint_telep/reorder_waypoint', 10)
        self.gripper_publisher = self.node.create_publisher(
            Float64MultiArray, '/waypoint_telep/waypoint_gripper', 10)
        self.remove_waypoint_publisher = self.node.create_publisher(
            UInt32, '/waypoint_telep/remove_waypoint', 10)
        self.external_publisher = self.node.create_publisher(
            JointTrajectory, '/test/external_command', 10)
        self.teleop_publishers = {
            arm: self.node.create_publisher(
                JointTrajectory, f'/intervention/teleop/arm_{arm}', 10)
            for arm in ('left', 'right')
        }
        self.final_commands = {'left': [], 'right': []}
        self.leader_commands = {'left': [], 'right': []}
        self.apply_final_commands_to_state = True
        for arm in ('left', 'right'):
            self.node.create_subscription(
                JointTrajectory,
                '/leader/joint_trajectory_command_broadcaster_'
                f'{arm}/joint_trajectory',
                lambda message, side=arm: self._final_command(side, message),
                10,
            )
            self.node.create_subscription(
                UInt8,
                f'/leader/{arm}_command',
                lambda message, side=arm: self.leader_commands[side].append(
                    message.data),
                10,
            )

        self.statuses = []
        self.paths = []
        self.trajectories = []
        self.waypoint_lists = []
        self.current_world_poses = []
        self.world_root_poses = []
        self.world_waypoint_lists = []
        self.waypoint_dwells = []
        self.waypoint_speed_scales = []
        self.waypoint_grippers = []
        self.waypoint_modes = []
        self.joint_smoothing_states = []
        self.selected_start_waypoints = []
        self.selected_end_waypoints = []
        self.waypoint_targets = []
        self.cartesian_jog_targets = []
        self.node.create_subscription(
            String, '/waypoint_telep/status', self._status_callback, model_qos)
        self.node.create_subscription(
            PathMessage, '/waypoint_telep/preview_path', self.paths.append, model_qos)
        self.node.create_subscription(
            JointTrajectory,
            '/waypoint_telep/planned_trajectory',
            self.trajectories.append,
            model_qos,
        )
        self.node.create_subscription(
            PoseArray,
            '/waypoint_telep/waypoints',
            self.waypoint_lists.append,
            model_qos,
        )
        self.node.create_subscription(
            PoseStamped,
            '/waypoint_telep/current_world_pose',
            self.current_world_poses.append,
            model_qos,
        )
        self.node.create_subscription(
            PoseStamped,
            '/waypoint_telep/world_root_pose',
            self.world_root_poses.append,
            model_qos,
        )
        self.node.create_subscription(
            PoseArray,
            '/waypoint_telep/world_waypoints',
            self.world_waypoint_lists.append,
            model_qos,
        )
        self.node.create_subscription(
            Float64MultiArray,
            '/waypoint_telep/waypoint_dwells',
            self.waypoint_dwells.append,
            model_qos,
        )
        self.node.create_subscription(
            Float64MultiArray,
            '/waypoint_telep/waypoint_speed_scales',
            self.waypoint_speed_scales.append,
            model_qos,
        )
        self.node.create_subscription(
            Float64MultiArray,
            '/waypoint_telep/waypoint_grippers',
            self.waypoint_grippers.append,
            model_qos,
        )
        self.node.create_subscription(
            Int32MultiArray,
            '/waypoint_telep/waypoint_modes',
            self.waypoint_modes.append,
            model_qos,
        )
        self.node.create_subscription(
            Bool,
            '/waypoint_telep/joint_smoothing_enabled',
            self.joint_smoothing_states.append,
            model_qos,
        )
        self.node.create_subscription(
            UInt32,
            '/waypoint_telep/selected_start_waypoint',
            self.selected_start_waypoints.append,
            model_qos,
        )
        self.node.create_subscription(
            UInt32,
            '/waypoint_telep/selected_end_waypoint',
            self.selected_end_waypoints.append,
            model_qos,
        )
        self.node.create_subscription(
            Float64MultiArray,
            '/waypoint_telep/planned_waypoint_targets',
            self.waypoint_targets.append,
            model_qos,
        )
        self.node.create_subscription(
            JointState,
            '/waypoint_telep/cartesian_jog_target',
            self.cartesian_jog_targets.append,
            10,
        )

        self.joint_names = LEFT_COMMAND_JOINTS
        self.positions = [0.0] * len(self.joint_names)
        self.teleop_positions = None
        self.velocities = [0.0] * len(self.joint_names)
        self.publish_joint_states = True
        # These streams come from independent processes on the robot. Keep
        # their timers independent here as well when DDS blocks a writer.
        self.feedback_group = MutuallyExclusiveCallbackGroup()
        self.teleop_group = MutuallyExclusiveCallbackGroup()
        self.joint_timer = self.node.create_timer(
            0.02, self._publish_joint_state, callback_group=self.feedback_group)
        self.teleop_timer = self.node.create_timer(
            0.02, self._publish_teleop_pair, callback_group=self.teleop_group)

        self.hold_action = False
        self.goal_count = 0
        self.cancel_count = 0
        self.action_server = ActionServer(
            self.node,
            FollowJointTrajectory,
            '/test/follow_joint_trajectory',
            execute_callback=self._execute_goal,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
        )
        self.clients = {}
        self.thread.start()

        xacro_path = (
            Path(get_package_share_directory('ffw_description'))
            / 'urdf' / 'ffw_f2_follower' / 'ffw_f2_follower.urdf.xacro'
        )
        document = xacro.process_file(
            str(xacro_path), mappings={'use_mock_hardware': 'true'})
        self.robot_description = document.toxml()
        self.publish_model()

    def _status_callback(self, message):
        self.statuses.append(message.data)

    def _final_command(self, arm, message):
        self.final_commands[arm].append(message)
        if not self.apply_final_commands_to_state:
            return
        indexes = {name: index for index, name in enumerate(self.joint_names)}
        if not message.points:
            return
        for name, value in zip(message.joint_names, message.points[0].positions):
            if name in indexes:
                self.positions[indexes[name]] = value

    def _publish_teleop_pair(self):
        indexes = {name: index for index, name in enumerate(self.joint_names)}
        source_positions = (
            self.positions
            if self.teleop_positions is None else self.teleop_positions
        )
        for arm in ('left', 'right'):
            message = JointTrajectory()
            message.joint_names = (
                [f'arm_l_joint{index}' for index in range(1, 8)]
                + ['gripper_l_joint1']
                if arm == 'left' else
                [f'arm_r_joint{index}' for index in range(1, 8)]
                + ['gripper_r_joint1']
            )
            point = JointTrajectoryPoint()
            message.points = [point]
            point.positions = [
                source_positions[indexes[name]] if name in indexes else 0.0
                for name in message.joint_names
            ]
            self.teleop_publishers[arm].publish(message)

    def _publish_joint_state(self):
        if not self.publish_joint_states:
            return
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = list(self.joint_names)
        message.position = list(self.positions)
        message.velocity = list(self.velocities)
        if 'lift_joint' not in message.name:
            message.name.append('lift_joint')
            message.position.append(0.0)
            message.velocity.append(0.0)
        self.joint_state_publisher.publish(message)

    def _accept_goal(self, _request):
        self.goal_count += 1
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle):
        self.cancel_count += 1
        return CancelResponse.ACCEPT

    def _execute_goal(self, goal_handle):
        while self.hold_action and not goal_handle.is_cancel_requested:
            time.sleep(0.02)
        result = FollowJointTrajectory.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
        return result

    def publish_model(self):
        message = String()
        message.data = self.robot_description
        self.model_publisher.publish(message)

    def wait_for(self, predicate, timeout=5.0, message='condition timed out'):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        raise AssertionError(f'{message}; recent statuses={self.statuses[-8:]}')

    def wait_ready(self, tip_link='end_effector_l_link'):
        self.wait_for(
            lambda: any(
                status.startswith('MODEL_READY:') and tip_link in status
                for status in self.statuses
            ),
            message='waypoint node did not load the robot model',
        )
        self.wait_for(
            lambda: self.client('/waypoint_telep/capture_current').service_is_ready(),
            message='waypoint services did not become ready',
        )
        time.sleep(0.1)

    def client(self, name):
        if name not in self.clients:
            self.clients[name] = self.node.create_client(Trigger, name)
        return self.clients[name]

    def call(self, name, timeout=5.0):
        trajectory_count = len(self.trajectories)
        target_count = len(self.waypoint_targets)
        client = self.client(name)
        assert client.wait_for_service(timeout_sec=timeout), f'missing service: {name}'
        future = client.call_async(Trigger.Request())
        self.wait_for(future.done, timeout=timeout, message=f'service timeout: {name}')
        result = future.result()
        assert result is not None
        if result.success and name in {
                '/waypoint_telep/preview', '/waypoint_telep/preview_slow_start'}:
            # The service response and published plan use independent DDS
            # channels. Wait for the plan instead of relying on their arrival
            # order, particularly while another UI is under feedback load.
            self.wait_for(
                lambda: len(self.trajectories) > trajectory_count
                and self.trajectories[-1].points
                and len(self.waypoint_targets) > target_count
                and self.waypoint_targets[-1].data,
                timeout=timeout, message='preview trajectory did not arrive')
        return result

    def publish_pose(self, pose, frame_id='arm_base_link'):
        message = PoseStamped()
        message.header.frame_id = frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose = pose
        self.pose_publisher.publish(message)

    def publish_relative_pose(self, pose, frame_id='end_effector_l_link'):
        message = PoseStamped()
        message.header.frame_id = frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose = pose
        self.relative_pose_publisher.publish(message)

    def publish_frame_relative_pose(self, pose, frame_id='world'):
        message = PoseStamped()
        message.header.frame_id = frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose = pose
        self.frame_relative_pose_publisher.publish(message)

    def publish_relative_reference(self, reference):
        message = Int32()
        message.data = int(reference)
        self.relative_reference_publisher.publish(message)

    def publish_cartesian_jog(self, x=0.0, y=0.0, z=0.0, frame_id='world'):
        message = PointStamped()
        message.header.frame_id = frame_id
        message.point.x = x
        message.point.y = y
        message.point.z = z
        self.cartesian_jog_publisher.publish(message)

    def publish_absolute_position(self, point, frame_id='world'):
        message = PointStamped()
        message.header.frame_id = frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.point = point
        self.absolute_position_publisher.publish(message)

    def publish_absolute_orientation(self, quaternion, frame_id='world'):
        message = QuaternionStamped()
        message.header.frame_id = frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.quaternion = quaternion
        self.absolute_orientation_publisher.publish(message)

    def publish_point(self, point, frame_id='arm_base_link'):
        message = PointStamped()
        message.header.frame_id = frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.point = point
        self.point_publisher.publish(message)

    def publish_joint_waypoint(self, positions, names=None):
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = list(self.joint_names if names is None else names)
        message.position = list(positions)
        self.joint_waypoint_publisher.publish(message)

    def publish_dwell(self, index, seconds):
        message = Float64MultiArray()
        message.data = [float(index), float(seconds)]
        self.dwell_publisher.publish(message)

    def publish_speed_scale(self, index, scale):
        message = Float64MultiArray()
        message.data = [float(index), float(scale)]
        self.speed_scale_publisher.publish(message)

    def publish_waypoint_mode(self, index, mode_code):
        message = Int32MultiArray()
        message.data = [int(index), int(mode_code)]
        self.waypoint_mode_publisher.publish(message)

    def publish_joint_smoothing(self, enabled):
        message = Bool()
        message.data = bool(enabled)
        self.joint_smoothing_publisher.publish(message)

    def publish_start_waypoint(self, index):
        message = UInt32()
        message.data = int(index)
        self.start_waypoint_publisher.publish(message)

    def publish_end_waypoint(self, index):
        message = UInt32()
        message.data = int(index)
        self.end_waypoint_publisher.publish(message)

    def reorder_waypoint(self, source, target):
        message = Int32MultiArray()
        message.data = [int(source), int(target)]
        self.reorder_waypoint_publisher.publish(message)

    def publish_gripper(self, index, value):
        message = Float64MultiArray()
        message.data = [float(index), float(value)]
        self.gripper_publisher.publish(message)

    def remove_waypoint(self, index):
        message = UInt32()
        message.data = int(index)
        self.remove_waypoint_publisher.publish(message)

    def current_pose(self):
        assert self.call('/waypoint_telep/clear').success
        assert self.call('/waypoint_telep/capture_current').success
        previous_count = len(self.paths)
        assert self.call('/waypoint_telep/preview').success
        self.wait_for(lambda: len(self.paths) > previous_count)
        pose = self.paths[-1].poses[0].pose
        assert self.call('/waypoint_telep/clear').success
        return pose

    def capture_two_reachable(self):
        self.positions = [0.0] * len(self.joint_names)
        time.sleep(0.1)
        assert self.call('/waypoint_telep/clear').success
        assert self.call('/waypoint_telep/capture_current').success
        self.positions[3] = 0.02
        time.sleep(0.1)
        assert self.call('/waypoint_telep/capture_current').success
        self.positions = [0.0] * len(self.joint_names)
        time.sleep(0.1)

    def close(self):
        self.action_server.destroy()
        self.executor.shutdown(timeout_sec=3.0)
        self.thread.join(timeout=3.0)
        self.node.destroy_node()


@pytest.fixture
def isolated_graph():
    previous_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = os.environ.get(
        'WAYPOINT_TEST_ROS_DOMAIN_ID', str(201 + os.getpid() % 20))
    rclpy.init()
    graph = MockGraph()
    try:
        yield graph
    finally:
        graph.close()
        rclpy.shutdown()
        if previous_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain


@pytest.mark.parametrize('mode,strength', [('JOINT', 0), ('JOINT', 60), ('LINEAR', 100)])
def test_refined_path_api_roundtrip_adaptation_and_legacy_restore(isolated_graph, mode, strength):
    graph = isolated_graph
    graph.statuses.clear()
    graph.joint_names = LEFT_COMMAND_JOINTS
    graph.positions = [0.0, 0.4, 0.1, -0.5, 0.1, 0.2, 0.0, 0.1]
    graph.velocities = [0.0] * 8
    graph.publish_joint_states = True
    process = WaypointProcess(publish_enabled=False)
    ui = WaypointUiProcess(unused_tcp_port())
    try:
        graph.wait_ready()
        wait_for_http(ui.url)
        graph.wait_for(lambda: (
            http_json(ui.url, '/api/state')[1]['joint_state_fresh'] and
            http_json(ui.url, '/api/state')[1]['current_world_pose'] is not None))
        http_json(ui.url, '/api/actions/clear', {})
        for index, offset in enumerate((0.0, 0.04, 0.08)):
            q = graph.positions.copy()
            q[0] += offset
            q[-1] += offset
            if mode == 'JOINT' and strength == 60 and index == 2:
                q[1] += 0.06
            dwell = 0.05 if index == (2 if strength == 60 else 1) else 0
            assert http_json(ui.url, '/api/waypoints/joint', {
                'positions_rad': q, 'dwell_s': dwell})[1]['success']
            assert http_json(ui.url, '/api/waypoints/mode', {'index': index + 1, 'mode': mode})[1]['success']
        assert http_json(ui.url, '/api/actions/preview', {})[1]['success']
        original = http_json(ui.url, '/api/waypoints/export')[1]
        legacy = http_json(ui.url, '/api/state')[1]['trajectory']
        settings = dict(enabled=True, subdivisions=1, smoothing_percent=strength,
                        position_tolerance_mm=0.3, orientation_tolerance_deg=0.2,
                        joint_tolerance_rad=0.002, max_refinements=8)
        assert http_json(ui.url, '/api/path-refinement', {'path_refinement': settings})[1]['success']
        result = http_json(ui.url, '/api/actions/preview', {})[1]
        assert result['success'], result
        state = http_json(ui.url, '/api/state')[1]
        events = state['path_refinement_events']
        assert events[-1]['stage'] == 'success', events
        assert events[-1]['max_position_error_mm'] <= 0.3
        assert events[-1]['max_orientation_error_deg'] <= 0.2
        assert events[-1]['max_joint_error_rad'] <= 0.002
        assert events[-1]['smoothing_percent'] == strength
        assert events[-1]['reason_code'] == 'INFO'
        if mode == 'LINEAR':
            assert any(event['stage'] == 'retry' for event in events)
        else:
            assert events[-1]['joint_subdivisions'] == 1
            assert events[-1]['knot_count'] == 3
            if strength == 60:
                assert any(event['stage'] == 'retry' for event in events)
                assert events[-1]['effective_smoothing_percent'] < strength
                assert all(event['subdivisions'] == 1 for event in events)
        assert events[-1]['effective_smoothing_percent'] == pytest.approx(
            strength / 2 ** (events[-1]['attempt'] - 1))
        assert state['waypoint_count'] == 3
        assert state['trajectory']['sample_count'] <= math.ceil(state['trajectory']['duration_s'] * 100) + 12
        generated = http_json(ui.url, '/api/waypoints/export')[1]
        assert generated['waypoints'] == original['waypoints']
        assert generated['path_refinement'] == settings
        assert http_json(ui.url, '/api/waypoints/import', generated)[1]['success']
        assert http_json(ui.url, '/api/state')[1]['path_refinement'] == settings
        # Reverse and mixed ranges use the same original waypoint arrival mapping.
        http_json(ui.url, '/api/waypoints/start', {'index': 3})
        http_json(ui.url, '/api/waypoints/end', {'index': 1})
        assert http_json(ui.url, '/api/actions/preview', {})[1]['success']
        http_json(ui.url, '/api/waypoints/mode', {'index': 2, 'mode': 'JOINT' if mode == 'LINEAR' else 'LINEAR'})
        assert http_json(ui.url, '/api/actions/preview', {})[1]['success']
        # Refined mixed/reversed paths can be assigned equal source-segment time.
        timing = http_json(ui.url, '/api/waypoints/equal-time', {'duration_s': 2})[1]
        assert timing['verified'] and timing['max_error_s'] < 1e-6
        assert http_json(ui.url, '/api/state')[1]['path_refinement'] == settings
        # A failed tight-tolerance plan clears its preceding successful preview.
        strict = {**settings, 'position_tolerance_mm': 1e-10,
                  'orientation_tolerance_deg': 1e-10, 'joint_tolerance_rad': 1e-10,
                  'max_refinements': 0, 'subdivisions': 1}
        assert http_json(ui.url, '/api/path-refinement', {'path_refinement': strict})[1]['success']
        with pytest.raises(HTTPError) as failed_preview:
            http_json(ui.url, '/api/actions/preview', {})
        assert failed_preview.value.code == 409
        failed = http_json(ui.url, '/api/state')[1]
        assert failed['path_refinement_events'][-1]['stage'] == 'failed'
        assert failed['path_refinement_events'][-1]['reason_code'] == 'TOLERANCE_EXCEEDED'
        assert failed['path_refinement_events'][-1]['segment_end_waypoint'] > 0
        assert failed['trajectory'] is None
        # Old JSON files reset the feature to disabled and retain their old trajectory.
        original.pop('path_refinement', None)
        assert http_json(ui.url, '/api/waypoints/import', original)[1]['success']
        assert not http_json(ui.url, '/api/state')[1]['path_refinement']['enabled']
        assert http_json(ui.url, '/api/actions/preview', {})[1]['success']
        restored = http_json(ui.url, '/api/state')[1]['trajectory']
        assert restored == legacy
        assert graph.goal_count == 0
    finally:
        ui.stop()
        process.stop()


@pytest.mark.parametrize('arm,mode', [('left', 'JOINT'), ('right', 'JOINT'),
                                    ('left', 'LINEAR'), ('right', 'LINEAR')])
def test_generator_refinement_equal_time_complete_sequence(isolated_graph, arm, mode):
    graph = isolated_graph
    graph.joint_names = LEFT_COMMAND_JOINTS if arm == 'left' else RIGHT_COMMAND_JOINTS
    graph.positions = [0.0, 0.4, 0.1, -0.5, 0.1, 0.2, 0.0, 0.1]
    graph.velocities = [0.0] * 8
    graph.publish_joint_states = True
    if arm == 'right':
        graph.positions[1] *= -1
    process = WaypointProcess(publish_enabled=False, tip_link=f'end_effector_{arm[0]}_link', quiet=True)
    ui = WaypointUiProcess(unused_tcp_port(), arm=arm)
    try:
        graph.wait_ready(tip_link=f'end_effector_{arm[0]}_link')
        wait_for_http(ui.url)
        graph.wait_for(lambda: http_json(ui.url, '/api/state')[1]['joint_state_fresh'])
        http_json(ui.url, '/api/actions/clear', {})
        for i, offset in enumerate((0, 0.02, 0.05, 0.08)):
            target = graph.positions.copy()
            target[0] += offset
            target[7] += offset
            http_json(ui.url, '/api/waypoints/joint', {'positions_rad': target,
                'dwell_s': 0.11 if i == 1 else 0.07 if i == 3 else 0})
            http_json(ui.url, '/api/waypoints/mode', {'index': i + 1, 'mode': mode})
        generated = http_json(ui.url, '/api/generator/tool-path', {
            'grasp_complete_waypoint': 2, 'full_path_correction': True,
            'local_offset_mm': [0.5, -0.25, 0.5]})[1]
        assert generated['success']
        source = generated['waypoint_file']['waypoints']
        settings = dict(enabled=True, subdivisions=8, smoothing_percent=60,
                        position_tolerance_mm=2, orientation_tolerance_deg=2,
                        joint_tolerance_rad=0.02, max_refinements=8)
        http_json(ui.url, '/api/path-refinement', {'path_refinement': settings})
        assert http_json(ui.url, '/api/actions/preview', {})[1]['success']
        state = http_json(ui.url, '/api/state')[1]
        if mode == 'JOINT':
            assert state['path_refinement_events'][-1]['knot_count'] == 25
            assert state['path_refinement_events'][-1]['joint_intermediate_points'] == 7

        def check_arrivals(expected):
            target_count = len(graph.waypoint_targets)
            assert http_json(ui.url, '/api/actions/preview', {})[1]['success']
            graph.wait_for(lambda: len(graph.waypoint_targets) > target_count
                           and len(graph.waypoint_targets[-1].data) == 4 * 9)
            arrivals = graph.waypoint_targets[-1].data[::9]
            for i in range(1, 4):
                dwell = 0.11 if i == 2 else 0
                assert arrivals[i] - arrivals[i - 1] - dwell == pytest.approx(expected[i - 1], abs=1e-6)
            return arrivals

        assigned = http_json(ui.url, '/api/waypoints/equal-time', {'duration_s': 1.25})[1]
        assert assigned['verified'] and assigned['max_error_s'] < 1e-6
        check_arrivals([1.25] * 3)
        excluded = http_json(ui.url, '/api/waypoints/equal-time', {
            'duration_s': 1.7, 'excluded_waypoints': [2]})[1]
        assert excluded['excluded'] == 1 and excluded['updated'] == 2
        check_arrivals([1.25, 1.7, 1.7])
        total = http_json(ui.url, '/api/waypoints/equal-time', {'total_duration_s': 6})[1]
        assert total['verified'] and total['total_error_s'] < 1e-6
        check_arrivals([(6 - 0.18) / 3] * 3)
        saved = http_json(ui.url, '/api/waypoints/export')[1]
        for old, new in zip(source, saved['waypoints']):
            for field in ('joint_positions_rad', 'gripper_rad', 'dwell_s', 'interpolation_mode'):
                assert new[field] == old[field]
        assert saved['path_refinement'] == settings
        with pytest.raises(HTTPError):
            http_json(ui.url, '/api/waypoints/equal-time', {'total_duration_s': 0.01})
        assert http_json(ui.url, '/api/waypoints/export')[1] == saved
        assert http_json(ui.url, '/api/waypoints/import', saved)[1]['success']
        check_arrivals([(6 - 0.18) / 3] * 3)
        # Timing works with a stale physical state by using waypoint one locally.
        graph.publish_joint_states = False
        graph.wait_for(lambda: not http_json(ui.url, '/api/state')[1]['joint_state_fresh'])
        offline = http_json(ui.url, '/api/waypoints/equal-time', {'duration_s': 2})[1]
        assert offline['verified'] and offline['reference'] == 'first_waypoint'
        assert graph.goal_count == 0
    finally:
        graph.publish_joint_states = True
        ui.stop()
        process.stop()


@pytest.mark.parametrize('smoothing,mode', [(False, 'joint'), (True, 'joint'), (False, 'linear')])
def test_equal_time_api_verifies_and_rolls_back_without_execution(isolated_graph, smoothing, mode):
    graph = isolated_graph
    graph.joint_names = LEFT_COMMAND_JOINTS
    graph.positions = [0.0] * 8
    graph.velocities = [0.0] * 8
    graph.publish_joint_states = True
    process = WaypointProcess(publish_enabled=False)
    ui = WaypointUiProcess(unused_tcp_port())
    try:
        graph.wait_ready()
        wait_for_http(ui.url)
        graph.wait_for(lambda: http_json(ui.url, '/api/state')[1]['joint_state_fresh'])
        http_json(ui.url, '/api/actions/clear', {})
        http_json(ui.url, f'/api/actions/select_{mode}', {})
        for q4, gripper in ((0.0, 0.0), (0.03, 0.2), (0.11, 0.2), (0.13, 0.25)):
            target = [0.0] * 8
            target[3], target[7] = q4, gripper
            http_json(ui.url, '/api/waypoints/joint', {'positions_rad': target})
        http_json(ui.url, '/api/waypoints/dwell', {'index': 2, 'dwell_s': 0.27})
        http_json(ui.url, '/api/joint-smoothing', {'enabled': smoothing})
        before = http_json(ui.url, '/api/waypoints/export')[1]
        desired = 1.23456789
        result = http_json(ui.url, '/api/waypoints/equal-time', {'duration_s': desired})[1]
        assert result['verified'] and result['updated'] == 3
        assert result['max_error_s'] < 1e-6
        graph.wait_for(lambda: len(graph.waypoint_targets[-1].data) == 4 * 9)
        arrivals = graph.waypoint_targets[-1].data[::9]
        for i in range(1, 4):
            dwell = 0.27 if i == 2 else 0.0
            assert arrivals[i] - arrivals[i - 1] - dwell == pytest.approx(desired, abs=1e-6)
        graph.wait_for(lambda: http_json(ui.url, '/api/state')[1]['waypoints'][1]['speed_scale'] != 1)
        after = http_json(ui.url, '/api/waypoints/export')[1]
        assert after['waypoints'][0] == before['waypoints'][0]
        for old, new in zip(before['waypoints'], after['waypoints']):
            assert {k: v for k, v in old.items() if k != 'speed_scale'} == {
                k: v for k, v in new.items() if k != 'speed_scale'}
        # The excluded destination keeps its speed and planned movement time.
        result = http_json(ui.url, '/api/waypoints/equal-time', {
            'duration_s': 1.6, 'excluded_waypoints': [3],
        })[1]
        assert result['verified'] and result['excluded'] == 1 and result['updated'] == 2
        graph.wait_for(lambda: http_json(ui.url, '/api/state')[1]['waypoints'][1]['speed_scale']
                       != after['waypoints'][1]['speed_scale'])
        excluded_file = http_json(ui.url, '/api/waypoints/export')[1]
        assert excluded_file['waypoints'][2] == after['waypoints'][2]
        with pytest.raises(HTTPError):
            http_json(ui.url, '/api/waypoints/equal-time', {'duration_s': 0.0001})
        assert http_json(ui.url, '/api/waypoints/export')[1] == excluded_file
        # Reloading a file must retain enough scale precision for the same timing.
        http_json(ui.url, '/api/waypoints/import', excluded_file)
        http_json(ui.url, '/api/actions/preview', {})
        graph.wait_for(lambda: len(graph.waypoint_targets[-1].data) == 4 * 9)
        arrivals = graph.waypoint_targets[-1].data[::9]
        assert arrivals[1] - arrivals[0] == pytest.approx(1.6, abs=1e-6)
        assert arrivals[2] - arrivals[1] - 0.27 == pytest.approx(desired, abs=1e-6)
        assert arrivals[3] - arrivals[2] == pytest.approx(1.6, abs=1e-6)
        # Total includes first/middle/last dwell; initial alignment is separate.
        http_json(ui.url, '/api/waypoints/dwell', {'index': 1, 'dwell_s': 0.13})
        http_json(ui.url, '/api/waypoints/dwell', {'index': 4, 'dwell_s': 0.41})
        total_before = http_json(ui.url, '/api/waypoints/export')[1]
        result = http_json(ui.url, '/api/waypoints/equal-time', {
            'total_duration_s': 8.12345678, 'excluded_waypoints': [3],
        })[1]
        assert result['mode'] == 'total' and result['verified']
        assert result['total_s'] == pytest.approx(8.12345678, abs=1e-6)
        assert result['fixed_s'] == pytest.approx(desired + 0.13 + 0.27 + 0.41, abs=1e-6)
        assert result['execution_total_s'] == pytest.approx(
            result['total_s'] + result['alignment_s'], abs=1e-6)
        total_after = http_json(ui.url, '/api/waypoints/export')[1]
        for number, (old, new) in enumerate(zip(total_before['waypoints'], total_after['waypoints']), 1):
            assert {k: v for k, v in old.items() if k != 'speed_scale'} == {
                k: v for k, v in new.items() if k != 'speed_scale'}
            if number in (1, 3):
                assert old == new
        with pytest.raises(HTTPError):
            http_json(ui.url, '/api/waypoints/equal-time', {'total_duration_s': 0.1})
        assert http_json(ui.url, '/api/waypoints/export')[1] == total_after
        result = http_json(ui.url, '/api/waypoints/equal-time', {})[1]
        assert result['verified'] and result['updated'] == 3
        if smoothing:
            profiled = http_json(ui.url, '/api/waypoints/export')[1]
            profiled['joint_timing'] = dict(speed_mps=0.03,
                preserve_start_waypoint=3, preserve_end_waypoint=3)
            http_json(ui.url, '/api/waypoints/import', profiled)
            result = http_json(ui.url, '/api/waypoints/equal-time', {
                'duration_s': 1.4, 'excluded_waypoints': [3],
            })[1]
            assert result['verified'] and result['excluded'] == 1
            assert http_json(ui.url, '/api/state')[1]['joint_timing'] is None
        # No live or fabricated feedback is needed for the offline preview.
        graph.publish_joint_states = False
        time.sleep(0.35)
        prior_feedback = http_json(ui.url, '/api/state')[1]['current_deg']
        result = http_json(ui.url, '/api/waypoints/equal-time', {'duration_s': 1.5})[1]
        assert result['verified'] and result['reference'] == 'first_waypoint'
        offline_state = http_json(ui.url, '/api/state')[1]
        assert offline_state['current_deg'] == prior_feedback
        assert not offline_state['joint_state_fresh']
        assert graph.goal_count == 0
        total_result = http_json(ui.url, '/api/waypoints/equal-time', {
            'total_duration_s': 8.0,
        })[1]
        assert total_result['reference'] == 'first_waypoint'
        assert total_result['total_s'] == pytest.approx(8.0, abs=1e-6)
    finally:
        graph.publish_joint_states = True
        try:
            ui.stop()
        finally:
            process.stop()


def test_equal_time_rolls_back_when_trajectory_verification_fails(isolated_graph):
    graph = isolated_graph
    graph.joint_names = LEFT_COMMAND_JOINTS
    graph.positions = [0.0] * 8
    graph.velocities = [0.0] * 8
    graph.publish_joint_states = True
    process = WaypointProcess(publish_enabled=False, max_joint_step_rad=1e-7)
    ui = WaypointUiProcess(unused_tcp_port())
    try:
        graph.wait_ready()
        wait_for_http(ui.url)
        http_json(ui.url, '/api/actions/clear', {})
        http_json(ui.url, '/api/actions/select_joint', {})
        for q4 in (0.0, 0.05, 0.07):
            target = [0.0] * 8
            target[3] = q4
            http_json(ui.url, '/api/waypoints/joint', {'positions_rad': target})
        before = http_json(ui.url, '/api/waypoints/export')[1]
        with pytest.raises(HTTPError) as error:
            http_json(ui.url, '/api/waypoints/equal-time', {'duration_s': 1.7})
        assert 'joint discontinuity' in str(error.value)
        assert http_json(ui.url, '/api/waypoints/export')[1] == before
        assert graph.goal_count == 0
    finally:
        try:
            ui.stop()
        finally:
            process.stop()


@pytest.mark.skipif(
    not os.environ.get('WAYPOINT_EQUAL_TIME_REFERENCE_JSON'),
    reason='실제 waypoint JSON을 지정한 검증에서만 실행합니다.',
)
def test_equal_time_actual_reference_offline_without_execution(isolated_graph):
    reference = json.loads(Path(os.environ['WAYPOINT_EQUAL_TIME_REFERENCE_JSON']).read_text())
    assert reference['arm'] == 'right'
    graph = isolated_graph
    graph.joint_names = RIGHT_COMMAND_JOINTS
    graph.positions = list(reference['waypoints'][0]['joint_positions_rad'])
    graph.velocities = [0.0] * 8
    graph.publish_joint_states = False
    process = WaypointProcess(publish_enabled=False, tip_link='end_effector_r_link',
                              max_waypoint_speed_scale=30.0, quiet=True)
    ui = WaypointUiProcess(unused_tcp_port(), arm='right', max_waypoint_speed_scale=30.0)
    try:
        graph.wait_ready(tip_link='end_effector_r_link')
        wait_for_http(ui.url)
        http_json(ui.url, '/api/waypoints/import', reference, timeout=300)
        before = http_json(ui.url, '/api/waypoints/export')[1]
        profile = reference.get('joint_timing')
        excluded = list(range(profile['preserve_start_waypoint'],
                              profile['preserve_end_waypoint'] + 1)) if profile else []
        result = http_json(ui.url, '/api/waypoints/equal-time', {
            'excluded_waypoints': excluded,
        }, timeout=120)[1]
        assert result['verified'] and result['reference'] == 'first_waypoint'
        assert result['updated'] == len(reference['waypoints']) - 1 - len(excluded)
        assert result['max_error_s'] <= 1e-6
        after = http_json(ui.url, '/api/waypoints/export')[1]
        assert len(after['waypoints']) == len(before['waypoints'])
        for index, (old, new) in enumerate(zip(before['waypoints'], after['waypoints']), 1):
            assert {k: v for k, v in old.items() if k != 'speed_scale'} == {
                k: v for k, v in new.items() if k != 'speed_scale'}
            if index == 1 or index in excluded:
                assert old['speed_scale'] == new['speed_scale']
        assert not http_json(ui.url, '/api/state')[1]['joint_state_fresh']
        assert graph.goal_count == 0
        # Real right-arm reference: strict target with insertion time preserved.
        total_target = result['total_s'] * 1.5
        total_result = http_json(ui.url, '/api/waypoints/equal-time', {
            'total_duration_s': total_target, 'excluded_waypoints': excluded,
        }, timeout=120)[1]
        assert total_result['verified'] and total_result['mode'] == 'total'
        assert total_result['total_s'] == pytest.approx(total_target, abs=1e-6)
        assert total_result['total_error_s'] <= 1e-6
        total_file = http_json(ui.url, '/api/waypoints/export')[1]
        for number in excluded:
            assert total_file['waypoints'][number - 1] == after['waypoints'][number - 1]
        assert graph.goal_count == 0
        if os.environ.get('WAYPOINT_EQUAL_TIME_REPORT'):
            Path(os.environ['WAYPOINT_EQUAL_TIME_REPORT']).write_text(
                json.dumps({'segment': result, 'total': total_result}, ensure_ascii=False, indent=2))
    finally:
        graph.joint_names = LEFT_COMMAND_JOINTS
        graph.positions = [0.0] * 8
        graph.velocities = [0.0] * 8
        graph.publish_joint_states = True
        try:
            ui.stop()
        finally:
            process.stop()


def test_continuous_joint_preview_is_identical_across_50_plans(isolated_graph):
    graph = isolated_graph
    process = WaypointProcess(publish_enabled=False)
    try:
        graph.wait_ready()
        graph.positions = [0.0] * len(graph.joint_names)
        graph.velocities = [0.0] * len(graph.joint_names)
        assert graph.call('/waypoint_telep/clear').success
        assert graph.call('/waypoint_telep/select_joint').success

        targets = []
        for joint4, joint5 in ((0.10, 0.03), (0.20, 0.06), (0.30, 0.09), (0.40, 0.12)):
            target = [0.0] * 8
            target[3] = joint4
            target[4] = joint5
            targets.append(target)
            previous_status_count = len(graph.statuses)
            graph.publish_joint_waypoint(target)
            expected_count = len(targets)
            graph.wait_for(
                lambda: f'JOINT_WAYPOINT_ADDED: count={expected_count}'
                in graph.statuses[previous_status_count:]
            )

        graph.publish_dwell(len(targets), 0.2)
        graph.wait_for(
            lambda: graph.waypoint_dwells
            and graph.waypoint_dwells[-1].data[-1] == pytest.approx(0.2)
        )

        previous_status_count = len(graph.statuses)
        graph.publish_joint_smoothing(True)
        graph.wait_for(
            lambda: 'JOINT_SMOOTHING_UPDATED: enabled=true'
            in graph.statuses[previous_status_count:]
        )
        graph.wait_for(
            lambda: graph.joint_smoothing_states
            and graph.joint_smoothing_states[-1].data is True
        )

        signatures = []
        target_signatures = []
        for _ in range(50):
            previous_trajectory_count = len(graph.trajectories)
            previous_target_count = len(graph.waypoint_targets)
            response = graph.call('/waypoint_telep/preview')
            assert response.success and 'JOINT_CONTINUOUS' in response.message
            graph.wait_for(
                lambda: len(graph.trajectories) > previous_trajectory_count
                and graph.trajectories[-1].points
            )
            graph.wait_for(
                lambda: len(graph.waypoint_targets) > previous_target_count
                and len(graph.waypoint_targets[-1].data) == len(targets) * 9
            )
            trajectory = graph.trajectories[-1]
            signatures.append(tuple(
                (
                    point.time_from_start.sec,
                    point.time_from_start.nanosec,
                    tuple(point.positions),
                )
                for point in trajectory.points
            ))
            target_signatures.append(tuple(graph.waypoint_targets[-1].data))

        assert all(signature == signatures[0] for signature in signatures[1:])
        assert all(
            signature == target_signatures[0]
            for signature in target_signatures[1:]
        )
        assert tuple(target_signatures[0][1:9]) == tuple(targets[0])

        first_time = target_signatures[0][0]
        first_index = next(
            index for index, point in enumerate(graph.trajectories[-1].points)
            if point.time_from_start.sec
            + point.time_from_start.nanosec * 1.0e-9 >= first_time - 1.0e-9
        )
        points = graph.trajectories[-1].points
        assert 0.0 < points[0].positions[3] < targets[0][3]
        assert points[first_index].positions == pytest.approx(targets[0])

        final_target_time = target_signatures[0][(len(targets) - 1) * 9]
        final_trajectory_time = (
            points[-1].time_from_start.sec
            + points[-1].time_from_start.nanosec * 1.0e-9
        )
        assert final_trajectory_time == pytest.approx(
            final_target_time + 0.2, abs=1e-6)
        assert points[-1].positions == pytest.approx(targets[-1])
        assert points[-2].positions == pytest.approx(targets[-1])

        second_time = target_signatures[0][9]
        second_index = next(
            index for index, point in enumerate(graph.trajectories[-1].points)
            if point.time_from_start.sec
            + point.time_from_start.nanosec * 1.0e-9 >= second_time - 1.0e-9
        )
        assert 0 < second_index < len(points) - 1
        before = points[second_index].positions[3] - points[second_index - 1].positions[3]
        after = points[second_index + 1].positions[3] - points[second_index].positions[3]
        assert before > 0.0
        assert after > 0.0
    finally:
        graph.publish_joint_smoothing(False)
        process.stop()


def test_world_waypoint_display_does_not_repeat_on_unchanged_joint_feedback(isolated_graph):
    graph = isolated_graph
    graph.statuses.clear()
    # Match the physical robot's 500 Hz feedback, not just the UI poll rate.
    old_period = graph.joint_timer.timer_period_ns
    graph.joint_timer.timer_period_ns = 2_000_000
    process = WaypointProcess(publish_enabled=False)
    try:
        graph.wait_ready()
        assert graph.call('/waypoint_telep/clear').success
        assert graph.call('/waypoint_telep/capture_current').success
        graph.wait_for(lambda: graph.world_waypoint_lists
                       and len(graph.world_waypoint_lists[-1].poses) == 1)
        time.sleep(0.15)
        count = len(graph.world_waypoint_lists)
        feedback_count = len(graph.current_world_poses)
        time.sleep(0.4)
        assert len(graph.world_waypoint_lists) == count
        assert len(graph.current_world_poses) > feedback_count + 5
        assert len(graph.current_world_poses) <= feedback_count + 11
        # Plan edits must bypass the frame cache and publish immediately.
        assert graph.call('/waypoint_telep/capture_current').success
        graph.wait_for(lambda: len(graph.world_waypoint_lists) > count
                       and len(graph.world_waypoint_lists[-1].poses) == 2)
        assert graph.call('/waypoint_telep/clear').success
        graph.wait_for(lambda: not graph.world_waypoint_lists[-1].poses)
        assert graph.goal_count == 0
    finally:
        graph.joint_timer.timer_period_ns = old_period
        process.stop()
        # Do not let the next process reuse this process's MODEL_READY event.
        graph.statuses.clear()


def test_waypoint_editing_input_and_both_path_modes(isolated_graph):
    graph = isolated_graph
    process = WaypointProcess(publish_enabled=False)
    try:
        graph.wait_ready()

        assert graph.call('/waypoint_telep/capture_current').success
        assert graph.call('/waypoint_telep/remove_last').success
        assert not graph.call('/waypoint_telep/remove_last').success
        assert graph.call('/waypoint_telep/clear').success

        pose = graph.current_pose()
        previous_status_count = len(graph.statuses)
        graph.publish_point(pose.position, frame_id='map')
        graph.wait_for(
            lambda: any(
                'clicked point frame must be arm_base_link' in status
                for status in graph.statuses[previous_status_count:]
            )
        )

        previous_status_count = len(graph.statuses)
        graph.publish_point(pose.position)
        graph.wait_for(
            lambda: any(
                status == 'WAYPOINT_ADDED: count=1'
                for status in graph.statuses[previous_status_count:]
            )
        )
        assert graph.call('/waypoint_telep/remove_last').success

        previous_status_count = len(graph.statuses)
        graph.publish_pose(pose)
        graph.wait_for(
            lambda: any(
                status == 'WAYPOINT_ADDED: count=1'
                for status in graph.statuses[previous_status_count:]
            )
        )
        assert graph.call('/waypoint_telep/clear').success

        world_pose = PoseStamped().pose
        world_pose.position.x = pose.position.x - 0.0753807
        world_pose.position.y = pose.position.y
        world_pose.position.z = pose.position.z + 1.407
        world_pose.orientation = pose.orientation
        previous_status_count = len(graph.statuses)
        graph.publish_pose(world_pose, frame_id='world')
        graph.wait_for(
            lambda: 'ABSOLUTE_WAYPOINT_ADDED: count=1'
            in graph.statuses[previous_status_count:]
        )
        graph.wait_for(
            lambda: graph.waypoint_lists
            and len(graph.waypoint_lists[-1].poses) == 1
        )
        stored_pose = graph.waypoint_lists[-1].poses[0]
        assert stored_pose.position.x == pytest.approx(pose.position.x, abs=1e-6)
        assert stored_pose.position.y == pytest.approx(pose.position.y, abs=1e-6)
        assert stored_pose.position.z == pytest.approx(pose.position.z, abs=1e-6)
        assert graph.call('/waypoint_telep/clear').success

        previous_status_count = len(graph.statuses)
        graph.publish_absolute_position(world_pose.position)
        graph.wait_for(
            lambda: 'ABSOLUTE_POSITION_WAYPOINT_ADDED: count=1'
            in graph.statuses[previous_status_count:]
        )
        stored_pose = graph.waypoint_lists[-1].poses[0]
        assert stored_pose.position.x == pytest.approx(pose.position.x, abs=1e-6)
        assert stored_pose.position.y == pytest.approx(pose.position.y, abs=1e-6)
        assert stored_pose.position.z == pytest.approx(pose.position.z, abs=1e-6)
        assert quaternion_to_rpy_deg(stored_pose.orientation) == pytest.approx(
            quaternion_to_rpy_deg(pose.orientation), abs=1e-5)
        assert graph.call('/waypoint_telep/clear').success

        previous_status_count = len(graph.statuses)
        graph.publish_absolute_orientation(world_pose.orientation)
        graph.wait_for(
            lambda: 'ABSOLUTE_ORIENTATION_WAYPOINT_ADDED: count=1'
            in graph.statuses[previous_status_count:]
        )
        stored_pose = graph.waypoint_lists[-1].poses[0]
        assert stored_pose.position.x == pytest.approx(pose.position.x, abs=1e-6)
        assert stored_pose.position.y == pytest.approx(pose.position.y, abs=1e-6)
        assert stored_pose.position.z == pytest.approx(pose.position.z, abs=1e-6)
        assert graph.call('/waypoint_telep/clear').success

        frame_relative = PoseStamped().pose
        frame_relative.position.x = 0.005
        fixed_yaw_rad = math.radians(0.5)
        frame_relative.orientation.z = math.sin(fixed_yaw_rad / 2.0)
        frame_relative.orientation.w = math.cos(fixed_yaw_rad / 2.0)
        previous_status_count = len(graph.statuses)
        graph.publish_frame_relative_pose(frame_relative)
        graph.wait_for(
            lambda: 'FRAME_RELATIVE_WAYPOINT_ADDED: count=1'
            in graph.statuses[previous_status_count:]
        )
        graph.wait_for(
            lambda: graph.waypoint_lists
            and len(graph.waypoint_lists[-1].poses) == 1
            and graph.world_waypoint_lists
            and len(graph.world_waypoint_lists[-1].poses) == 1
        )
        fixed_frame_target = graph.waypoint_lists[-1].poses[0]
        assert fixed_frame_target.position.x == pytest.approx(
            pose.position.x + 0.005, abs=1e-6)
        assert fixed_frame_target.position.y == pytest.approx(
            pose.position.y, abs=1e-6)
        assert fixed_frame_target.position.z == pytest.approx(
            pose.position.z, abs=1e-6)
        displayed_world_target = graph.world_waypoint_lists[-1].poses[0]
        assert displayed_world_target.position.x == pytest.approx(
            world_pose.position.x + 0.005, abs=1e-6)
        reference_quaternion = world_pose.orientation
        expected_quaternion = [
            frame_relative.orientation.w * reference_quaternion.x
            - frame_relative.orientation.z * reference_quaternion.y,
            frame_relative.orientation.w * reference_quaternion.y
            + frame_relative.orientation.z * reference_quaternion.x,
            frame_relative.orientation.w * reference_quaternion.z
            + frame_relative.orientation.z * reference_quaternion.w,
            frame_relative.orientation.w * reference_quaternion.w
            - frame_relative.orientation.z * reference_quaternion.z,
        ]
        actual_quaternion = [
            displayed_world_target.orientation.x,
            displayed_world_target.orientation.y,
            displayed_world_target.orientation.z,
            displayed_world_target.orientation.w,
        ]
        assert abs(sum(
            actual * expected
            for actual, expected in zip(actual_quaternion, expected_quaternion)
        )) == pytest.approx(1.0, abs=1e-6)
        assert graph.current_world_poses
        assert graph.current_world_poses[-1].header.frame_id == 'world'
        assert graph.world_root_poses
        assert graph.world_root_poses[-1].header.frame_id == 'world'
        assert graph.call('/waypoint_telep/clear').success

        previous_target_count = len(graph.cartesian_jog_targets)
        previous_status_count = len(graph.statuses)
        graph.publish_cartesian_jog(x=0.0025)
        graph.wait_for(
            lambda: len(graph.cartesian_jog_targets) > previous_target_count
        )
        jog_target = graph.cartesian_jog_targets[-1]
        assert list(jog_target.name) == LEFT_COMMAND_JOINTS
        assert len(jog_target.position) == len(LEFT_COMMAND_JOINTS)
        assert all(math.isfinite(value) for value in jog_target.position)
        assert 'CARTESIAN_JOG_TARGET_READY' in graph.statuses[previous_status_count:]
        assert graph.waypoint_lists and not graph.waypoint_lists[-1].poses

        previous_target_count = len(graph.cartesian_jog_targets)
        previous_status_count = len(graph.statuses)
        graph.publish_cartesian_jog(x=0.00025, y=0.00025)
        graph.wait_for(
            lambda: any(
                'Cartesian jog must contain exactly one axis' in status
                for status in graph.statuses[previous_status_count:]
            )
        )
        assert len(graph.cartesian_jog_targets) == previous_target_count

        relative = PoseStamped().pose
        relative.position.x = 0.005
        relative.orientation.w = 1.0
        previous_status_count = len(graph.statuses)
        graph.publish_relative_pose(relative)
        graph.wait_for(
            lambda: 'RELATIVE_WAYPOINT_ADDED: count=1'
            in graph.statuses[previous_status_count:]
        )
        graph.publish_relative_pose(relative)
        graph.wait_for(
            lambda: 'RELATIVE_WAYPOINT_ADDED: count=2' in graph.statuses
        )
        graph.wait_for(
            lambda: graph.waypoint_lists and len(graph.waypoint_lists[-1].poses) == 2
        )
        default_second_pose = graph.waypoint_lists[-1].poses[1]
        previous_status_count = len(graph.statuses)
        graph.publish_relative_reference(1)
        graph.wait_for(
            lambda: 'RELATIVE_REFERENCE_SELECTED: waypoint=1'
            in graph.statuses[previous_status_count:]
        )
        graph.publish_relative_pose(relative)
        graph.wait_for(
            lambda: 'RELATIVE_WAYPOINT_ADDED: count=3' in graph.statuses
        )
        graph.wait_for(
            lambda: graph.waypoint_lists and len(graph.waypoint_lists[-1].poses) == 3
        )
        selected_reference_pose = graph.waypoint_lists[-1].poses[2]
        assert selected_reference_pose.position.x == pytest.approx(
            default_second_pose.position.x, abs=1e-6)
        assert selected_reference_pose.position.y == pytest.approx(
            default_second_pose.position.y, abs=1e-6)
        assert selected_reference_pose.position.z == pytest.approx(
            default_second_pose.position.z, abs=1e-6)
        assert graph.call('/waypoint_telep/preview').success
        graph.wait_for(
            lambda: graph.waypoint_targets
            and len(graph.waypoint_targets[-1].data) == 3 * 9
        )
        targets = list(graph.waypoint_targets[-1].data)
        assert targets[0] > 0.0
        assert targets[9] > targets[0]
        assert targets[18] > targets[9]
        assert graph.call('/waypoint_telep/clear').success

        unreachable_relative = PoseStamped().pose
        unreachable_relative.position.x = 10.0
        unreachable_relative.orientation.w = 1.0
        previous_status_count = len(graph.statuses)
        graph.publish_relative_pose(unreachable_relative)
        graph.wait_for(
            lambda: any(
                'relative target IK failed' in status
                for status in graph.statuses[previous_status_count:]
            )
        )
        graph.wait_for(
            lambda: graph.waypoint_lists and not graph.waypoint_lists[-1].poses
        )

        previous_status_count = len(graph.statuses)
        target = [0.0] * 8
        target[3] = 0.02
        target[7] = 0.3
        graph.publish_joint_waypoint(target)
        graph.wait_for(
            lambda: 'JOINT_WAYPOINT_ADDED: count=1'
            in graph.statuses[previous_status_count:]
        )
        graph.publish_waypoint_mode(1, 1)
        graph.wait_for(
            lambda: graph.waypoint_modes
            and list(graph.waypoint_modes[-1].data) == [1]
        )
        response = graph.call('/waypoint_telep/preview')
        assert response.success
        assert graph.trajectories[-1].points[-1].positions[3] == pytest.approx(0.02)
        assert graph.trajectories[-1].points[-1].positions[7] == pytest.approx(0.3)
        assert graph.call('/waypoint_telep/select_linear').success
        assert list(graph.waypoint_modes[-1].data) == [1]
        assert graph.call('/waypoint_telep/preview').success
        assert graph.call('/waypoint_telep/clear').success

        previous_status_count = len(graph.statuses)
        graph.publish_joint_waypoint([0.0] * 6, names=LEFT_JOINTS[:6])
        graph.wait_for(
            lambda: any(
                'joint waypoint is missing arm_l_joint7' in status
                for status in graph.statuses[previous_status_count:]
            )
        )

        previous_status_count = len(graph.statuses)
        graph.publish_joint_waypoint([0.0] * 7, names=LEFT_JOINTS)
        graph.wait_for(
            lambda: any(
                'joint waypoint is missing gripper_l_joint1' in status
                for status in graph.statuses[previous_status_count:]
            )
        )

        graph.capture_two_reachable()
        graph.wait_for(
            lambda: graph.waypoint_lists
            and len(graph.waypoint_lists[-1].poses) == 2
        )
        first_pose = graph.waypoint_lists[-1].poses[0]
        second_pose = graph.waypoint_lists[-1].poses[1]
        previous_status_count = len(graph.statuses)
        graph.reorder_waypoint(1, 2)
        graph.wait_for(
            lambda: 'WAYPOINT_REORDERED: from=1, to=2'
            in graph.statuses[previous_status_count:]
        )
        graph.wait_for(
            lambda: graph.waypoint_lists[-1].poses[0].position.z
            == pytest.approx(second_pose.position.z, abs=1e-9)
        )
        graph.reorder_waypoint(2, 1)
        graph.wait_for(
            lambda: graph.waypoint_lists[-1].poses[0].position.z
            == pytest.approx(first_pose.position.z, abs=1e-9)
        )

        graph.publish_gripper(2, 0.4)
        graph.wait_for(
            lambda: graph.waypoint_grippers
            and list(graph.waypoint_grippers[-1].data) == pytest.approx([0.0, 0.4])
        )
        graph.publish_end_waypoint(1)
        graph.wait_for(
            lambda: graph.selected_end_waypoints
            and graph.selected_end_waypoints[-1].data == 1
        )
        response = graph.call('/waypoint_telep/preview')
        assert response.success and 'end waypoint 1' in response.message
        graph.wait_for(
            lambda: graph.waypoint_targets
            and len(graph.waypoint_targets[-1].data) == 9
        )
        graph.publish_end_waypoint(2)
        graph.wait_for(
            lambda: graph.selected_end_waypoints[-1].data == 2
        )
        response = graph.call('/waypoint_telep/select_linear')
        assert response.success and 'LINEAR' in response.message
        previous_path_count = len(graph.paths)
        response = graph.call('/waypoint_telep/preview')
        assert response.success and 'LINEAR, 2 waypoint(s)' in response.message
        graph.wait_for(lambda: len(graph.paths) > previous_path_count)
        assert len(graph.paths[-1].poses) > 2
        assert len(graph.trajectories[-1].points) > 1

        response = graph.call('/waypoint_telep/select_joint')
        assert response.success and 'JOINT' in response.message
        response = graph.call('/waypoint_telep/preview')
        assert response.success and 'LINEAR, 2 waypoint(s)' in response.message
        graph.wait_for(
            lambda: graph.waypoint_modes
            and list(graph.waypoint_modes[-1].data) == [0, 0]
        )
        baseline_duration = (
            graph.trajectories[-1].points[-1].time_from_start.sec
            + graph.trajectories[-1].points[-1].time_from_start.nanosec * 1.0e-9
        )

        previous_status_count = len(graph.statuses)
        graph.publish_speed_scale(2, 0.5)
        graph.wait_for(
            lambda: any(
                status.startswith('WAYPOINT_SPEED_UPDATED: index=2,')
                for status in graph.statuses[previous_status_count:]
            )
        )
        graph.wait_for(
            lambda: graph.waypoint_speed_scales
            and list(graph.waypoint_speed_scales[-1].data) == pytest.approx([1.0, 0.5])
        )
        assert graph.call('/waypoint_telep/preview').success
        half_speed_duration = (
            graph.trajectories[-1].points[-1].time_from_start.sec
            + graph.trajectories[-1].points[-1].time_from_start.nanosec * 1.0e-9
        )
        assert half_speed_duration > baseline_duration
        graph.publish_speed_scale(2, 1.0)
        graph.wait_for(
            lambda: graph.waypoint_speed_scales
            and list(graph.waypoint_speed_scales[-1].data) == pytest.approx([1.0, 1.0])
        )
        graph.publish_speed_scale(2, 2.0)
        graph.wait_for(
            lambda: graph.waypoint_speed_scales
            and list(graph.waypoint_speed_scales[-1].data) == pytest.approx([1.0, 2.0])
        )
        assert graph.call('/waypoint_telep/preview').success
        double_speed_duration = (
            graph.trajectories[-1].points[-1].time_from_start.sec
            + graph.trajectories[-1].points[-1].time_from_start.nanosec * 1.0e-9
        )
        assert double_speed_duration < baseline_duration
        graph.publish_speed_scale(2, 1.0)
        graph.wait_for(
            lambda: graph.waypoint_speed_scales
            and list(graph.waypoint_speed_scales[-1].data) == pytest.approx([1.0, 1.0])
        )

        previous_status_count = len(graph.statuses)
        graph.publish_waypoint_mode(1, 1)
        graph.wait_for(
            lambda: 'WAYPOINT_MODE_UPDATED: index=1, mode=JOINT'
            in graph.statuses[previous_status_count:]
        )
        graph.wait_for(
            lambda: graph.waypoint_modes
            and list(graph.waypoint_modes[-1].data) == [1, 0]
        )
        response = graph.call('/waypoint_telep/preview')
        assert response.success and 'MIXED, 2 waypoint(s)' in response.message

        previous_status_count = len(graph.statuses)
        graph.publish_joint_smoothing(True)
        graph.wait_for(
            lambda: 'JOINT_SMOOTHING_UPDATED: enabled=true'
            in graph.statuses[previous_status_count:]
        )
        response = graph.call('/waypoint_telep/preview')
        assert response.success and 'MIXED, 2 waypoint(s)' in response.message
        graph.publish_joint_smoothing(False)
        graph.wait_for(
            lambda: graph.joint_smoothing_states
            and graph.joint_smoothing_states[-1].data is False
        )

        previous_status_count = len(graph.statuses)
        graph.publish_start_waypoint(2)
        graph.wait_for(
            lambda: 'START_WAYPOINT_SELECTED: index=2'
            in graph.statuses[previous_status_count:]
        )
        response = graph.call('/waypoint_telep/preview')
        assert response.success and 'start waypoint 2' in response.message
        graph.wait_for(
            lambda: graph.waypoint_targets
            and len(graph.waypoint_targets[-1].data) == 9
        )

        graph.publish_end_waypoint(1)
        graph.wait_for(
            lambda: graph.selected_end_waypoints
            and graph.selected_end_waypoints[-1].data == 1
        )
        response = graph.call('/waypoint_telep/preview')
        assert response.success
        assert '2 waypoint(s), start waypoint 2, end waypoint 1' in response.message
        graph.wait_for(
            lambda: graph.waypoint_targets
            and len(graph.waypoint_targets[-1].data) == 18
        )
        reverse_targets = list(graph.waypoint_targets[-1].data)
        assert reverse_targets[4] == pytest.approx(0.02, abs=2e-6)
        assert reverse_targets[13] == pytest.approx(0.0, abs=1e-6)

        graph.publish_start_waypoint(1)
        graph.wait_for(
            lambda: graph.selected_start_waypoints
            and graph.selected_start_waypoints[-1].data == 1
        )
        graph.publish_end_waypoint(2)
        graph.wait_for(
            lambda: graph.selected_end_waypoints[-1].data == 2
        )
        graph.publish_waypoint_mode(1, 0)
        graph.wait_for(
            lambda: graph.waypoint_modes
            and list(graph.waypoint_modes[-1].data) == [0, 0]
        )
        assert graph.call('/waypoint_telep/preview').success

        previous_status_count = len(graph.statuses)
        graph.publish_dwell(1, 0.2)
        graph.wait_for(
            lambda: any(
                status.startswith('WAYPOINT_DWELL_UPDATED: index=1,')
                for status in graph.statuses[previous_status_count:]
            )
        )
        graph.wait_for(
            lambda: graph.waypoint_dwells
            and list(graph.waypoint_dwells[-1].data) == pytest.approx([0.2, 0.0])
        )
        assert graph.call('/waypoint_telep/preview').success
        dwell_duration = (
            graph.trajectories[-1].points[-1].time_from_start.sec
            + graph.trajectories[-1].points[-1].time_from_start.nanosec * 1.0e-9
        )
        assert dwell_duration == pytest.approx(baseline_duration + 0.2, abs=1e-6)
        dwell_points = graph.trajectories[-1].points
        assert any(
            first.positions == second.positions
            for first, second in zip(dwell_points, dwell_points[1:])
        )

        graph.publish_dwell(2, 0.15)
        graph.wait_for(
            lambda: graph.waypoint_dwells
            and list(graph.waypoint_dwells[-1].data) == pytest.approx([0.2, 0.15])
        )
        assert graph.call('/waypoint_telep/preview').success
        final_dwell_duration = (
            graph.trajectories[-1].points[-1].time_from_start.sec
            + graph.trajectories[-1].points[-1].time_from_start.nanosec * 1.0e-9
        )
        assert final_dwell_duration == pytest.approx(
            baseline_duration + 0.35, abs=1e-6)
        final_dwell_points = graph.trajectories[-1].points
        assert final_dwell_points[-1].positions == pytest.approx(
            final_dwell_points[-2].positions)

        previous_status_count = len(graph.statuses)
        graph.remove_waypoint(1)
        graph.wait_for(
            lambda: any(
                status.startswith('WAYPOINT_REMOVED: index=1, count=1')
                for status in graph.statuses[previous_status_count:]
            )
        )
        graph.wait_for(
            lambda: graph.waypoint_lists
            and len(graph.waypoint_lists[-1].poses) == 1
        )
        assert graph.call('/waypoint_telep/preview').success

        previous_status_count = len(graph.statuses)
        mode = String()
        mode.data = 'linear'
        graph.mode_publisher.publish(mode)
        graph.wait_for(
            lambda: 'DEFAULT_PATH_MODE_SELECTED: LINEAR'
            in graph.statuses[previous_status_count:]
        )

        response = graph.call('/waypoint_telep/execute')
        assert not response.success and 'preview-only mode' in response.message

        assert graph.call('/waypoint_telep/clear').success
        unreachable = PoseStamped().pose
        unreachable.position.x = 10.0
        unreachable.position.y = 10.0
        unreachable.position.z = 10.0
        unreachable.orientation.w = 1.0
        graph.publish_pose(unreachable)
        graph.wait_for(lambda: graph.statuses[-1].startswith('WAYPOINT_ADDED:'))
        response = graph.call('/waypoint_telep/preview')
        assert not response.success and 'IK failed' in response.message
    finally:
        process.stop()


def test_slow_start_preview_caps_only_the_initial_alignment(isolated_graph):
    graph = isolated_graph
    graph.statuses.clear()
    graph.trajectories.clear()
    graph.joint_names = LEFT_COMMAND_JOINTS
    graph.positions = [0.0] * 8
    graph.velocities = [0.0] * 8
    process = WaypointProcess(
        publish_enabled=False,
        max_waypoint_speed_scale=30.0,
    )
    try:
        graph.wait_ready()
        assert graph.call('/waypoint_telep/clear').success
        assert graph.call('/waypoint_telep/select_joint').success

        target = [0.0] * 8
        target[0] = 0.12908137454175383
        target[-1] = 0.4
        graph.publish_joint_waypoint(target)
        graph.wait_for(
            lambda: 'JOINT_WAYPOINT_ADDED: count=1' in graph.statuses)
        graph.publish_speed_scale(1, 24.847)
        graph.wait_for(
            lambda: graph.waypoint_speed_scales
            and graph.waypoint_speed_scales[-1].data[0]
            == pytest.approx(24.847)
        )
        graph.publish_joint_smoothing(True)
        graph.wait_for(
            lambda: graph.joint_smoothing_states
            and graph.joint_smoothing_states[-1].data is True
        )

        regular = graph.call('/waypoint_telep/preview')
        assert not regular.success
        assert 'arm_l_joint1' in regular.message
        assert 'sample step exceeds the limit' in regular.message

        previous_trajectory_count = len(graph.trajectories)
        slow_start = graph.call('/waypoint_telep/preview_slow_start')
        assert slow_start.success
        assert slow_start.message.startswith('SLOW_START, JOINT_CONTINUOUS')
        graph.wait_for(lambda: len(graph.trajectories) > previous_trajectory_count)
        points = graph.trajectories[-1].points
        assert points[-1].positions == pytest.approx(target)
        assert all(point.positions[-1] == pytest.approx(0.4) for point in points)
        assert all(
            max(abs(value - old) for value, old in zip(point.positions, previous))
            <= 0.05 + 1.0e-9
            for previous, point in zip([[0.0] * 7 + [0.4], *[
                list(point.positions) for point in points[:-1]
            ]], points)
        )
    finally:
        process.stop()


def test_execution_guards_success_stop_and_external_cancel(isolated_graph):
    graph = isolated_graph
    graph.statuses.clear()
    graph.paths.clear()
    graph.trajectories.clear()
    graph.joint_names = LEFT_COMMAND_JOINTS
    graph.positions = [0.0] * 8
    graph.velocities = [0.0] * 8
    graph.publish_joint_states = True
    graph.hold_action = False
    process = WaypointProcess(publish_enabled=True)
    try:
        graph.wait_ready()
        assert graph.call('/waypoint_telep/capture_current').success

        graph.velocities = [0.1] * 7
        time.sleep(0.1)
        response = graph.call('/waypoint_telep/execute')
        assert not response.success and 'arm is moving' in response.message
        graph.velocities = [0.0] * 8
        time.sleep(0.1)

        graph.external_publisher.publish(JointTrajectory())
        time.sleep(0.05)
        response = graph.call('/waypoint_telep/execute')
        assert not response.success and 'external command topic is active' in response.message
        time.sleep(0.3)

        previous_goal_count = graph.goal_count
        response = graph.call('/waypoint_telep/execute')
        assert response.success
        graph.wait_for(lambda: graph.goal_count > previous_goal_count)
        graph.wait_for(lambda: 'EXECUTION_COMPLETE' in graph.statuses)

        graph.hold_action = True
        previous_goal_count = graph.goal_count
        previous_cancel_count = graph.cancel_count
        assert graph.call('/waypoint_telep/execute').success
        graph.wait_for(lambda: graph.goal_count > previous_goal_count)
        graph.wait_for(lambda: 'EXECUTING' in graph.statuses)
        response = graph.call('/waypoint_telep/select_joint')
        assert not response.success and 'while executing' in response.message
        assert graph.call('/waypoint_telep/stop').success
        graph.wait_for(lambda: graph.cancel_count > previous_cancel_count)

        time.sleep(0.1)
        previous_goal_count = graph.goal_count
        previous_cancel_count = graph.cancel_count
        assert graph.call('/waypoint_telep/execute').success
        graph.wait_for(lambda: graph.goal_count > previous_goal_count)
        graph.external_publisher.publish(JointTrajectory())
        graph.wait_for(lambda: graph.cancel_count > previous_cancel_count)
        graph.wait_for(
            lambda: any('external command detected' in status for status in graph.statuses)
        )
        graph.hold_action = False

        graph.publish_joint_states = False
        time.sleep(0.35)
        response = graph.call('/waypoint_telep/execute')
        assert not response.success and 'joint state is stale' in response.message
    finally:
        graph.hold_action = False
        graph.publish_joint_states = True
        process.stop()


def test_right_arm_override_preview(isolated_graph):
    graph = isolated_graph
    graph.statuses.clear()
    graph.paths.clear()
    graph.trajectories.clear()
    graph.joint_names = RIGHT_COMMAND_JOINTS
    graph.positions = [0.0] * 8
    graph.velocities = [0.0] * 8
    graph.publish_joint_states = True
    process = WaypointProcess(
        publish_enabled=False,
        tip_link='end_effector_r_link',
    )
    try:
        graph.wait_ready(tip_link='end_effector_r_link')
        assert graph.call('/waypoint_telep/capture_current').success
        response = graph.call('/waypoint_telep/preview')
        assert response.success and 'LINEAR' in response.message
        assert graph.trajectories[-1].joint_names == RIGHT_COMMAND_JOINTS
    finally:
        process.stop()


def test_duplicate_ui_port_exits_cleanly_without_abort():
    port = unused_tcp_port()
    first = WaypointUiProcess(port)
    second = None
    try:
        assert wait_for_http(first.url)[0] == 200
        second = WaypointUiProcess(port)
        returncode = second.process.wait(timeout=5)
        output = second.process.stdout.read() if second.process.stdout else ''
        assert returncode == 1, output
        assert '같은 포트를 사용하는 기존 waypoint UI' in output
        assert first.process.poll() is None
    finally:
        if second is not None and second.process.poll() is None:
            second.process.send_signal(signal.SIGINT)
            second.process.wait(timeout=5)
        first.stop()


def test_web_ui_api_preview_workflow(isolated_graph):
    graph = isolated_graph
    graph.statuses.clear()
    graph.paths.clear()
    graph.trajectories.clear()
    graph.joint_names = LEFT_COMMAND_JOINTS
    graph.positions = [0.0] * 8
    graph.velocities = [0.0] * 8
    graph.publish_joint_states = True
    node_process = WaypointProcess(publish_enabled=False)
    ui_process = WaypointUiProcess(unused_tcp_port())
    try:
        graph.wait_ready()
        graph.wait_for(
            lambda: ui_process.process.poll() is None,
            message='UI process exited early',
        )
        deadline = time.monotonic() + 5.0
        while True:
            try:
                status, page = http_json(ui_process.url, '/api/state')
                if (page['joint_state_fresh'] and
                        http_json(ui_process.url, '/api/robot-model')[1]['status'] == 'ready'):
                    break
            except OSError:
                pass
            if time.monotonic() >= deadline:
                raise AssertionError('UI model and joint state did not become ready')
            time.sleep(0.05)
        assert status == 200
        assert page['joint_names'] == LEFT_COMMAND_JOINTS
        assert page['publish_enabled'] is False
        assert set(LEFT_JOINTS).issubset(page['robot_joint_state']['name'])
        assert len(page['robot_joint_state']['name']) == len(
            page['robot_joint_state']['position'])

        model = http_json(ui_process.url, '/api/robot-model')[1]
        assert model['status'] == 'ready'
        assert model['robot_name']
        assert model['base_link'] == 'arm_base_link'
        assert model['tip_link'] == 'end_effector_l_link'
        assert model['movable_joint_names'] == LEFT_JOINTS
        assert len(model['chain']) >= len(LEFT_JOINTS)

        with urlopen(
            ui_process.url + '/api/robot-description.urdf', timeout=3
        ) as response:
            assert response.headers.get_content_type() == 'application/xml'
            assert b'<robot' in response.read()

        with urlopen(
            ui_process.url + '/full_robot_simulator.bundle.js', timeout=3
        ) as response:
            bundle = response.read()
            assert b'FullRobotSimulator' in bundle
            assert len(bundle) > 1_000_000

        assert http_json(ui_process.url, '/api/actions/clear', {})[1]['success']
        current_pose = graph.current_pose()
        world_position = [
            current_pose.position.x - 0.0753807,
            current_pose.position.y,
            current_pose.position.z + 1.407,
        ]
        world_orientation = quaternion_to_rpy_deg(current_pose.orientation)
        result = http_json(
            ui_process.url,
            '/api/waypoints/absolute',
            {'target_mode': 'position', 'position_m': world_position},
        )[1]
        assert result['success']
        assert result['message'].startswith('ABSOLUTE_POSITION_WAYPOINT_ADDED:')
        assert http_json(ui_process.url, '/api/actions/clear', {})[1]['success']
        result = http_json(
            ui_process.url,
            '/api/waypoints/absolute',
            {'target_mode': 'orientation', 'orientation_deg': world_orientation},
        )[1]
        assert result['success']
        assert result['message'].startswith('ABSOLUTE_ORIENTATION_WAYPOINT_ADDED:')
        assert http_json(ui_process.url, '/api/actions/clear', {})[1]['success']
        result = http_json(
            ui_process.url,
            '/api/waypoints/frame-relative',
            {
                'reference': 'current',
                'translation_mm': [0, 0, 0],
                'rotation_deg': [0, 0, 0],
            },
        )[1]
        assert result['success']
        assert result['message'].startswith('FRAME_RELATIVE_WAYPOINT_ADDED:')
        assert http_json(ui_process.url, '/api/actions/clear', {})[1]['success']
        result = http_json(
            ui_process.url,
            '/api/waypoints/absolute',
            {
                'position_m': world_position,
                'orientation_deg': world_orientation,
            },
        )[1]
        assert result['success']
        result = http_json(
            ui_process.url,
            '/api/waypoints/joint',
            {'positions_rad': [0.0] * 7 + [0.3], 'dwell_s': 0.2},
        )[1]
        assert result['success']
        result = http_json(
            ui_process.url,
            '/api/waypoints/relative',
            {
                'reference': 1,
                'translation_mm': [0, 0, 0],
                'rotation_deg': [0, 0, 0],
            },
        )[1]
        assert result['success']
        assert http_json(
            ui_process.url, '/api/actions/select_linear', {})[1]['success']
        assert http_json(ui_process.url, '/api/actions/preview', {})[1]['success']
        assert http_json(
            ui_process.url, '/api/actions/select_joint', {})[1]['success']
        assert http_json(ui_process.url, '/api/actions/preview', {})[1]['success']
        graph.wait_for(lambda: bool(graph.trajectories))
        page = http_json(ui_process.url, '/api/state')[1]
        assert page['trajectory']['sample_count'] > 0
        assert page['path_mode'] == 'JOINT'
        assert page['waypoint_count'] == 3
        assert len(page['waypoints']) == 3
        assert page['waypoints'][1]['dwell_s'] == pytest.approx(0.2)
        assert all(
            waypoint['speed_scale'] == pytest.approx(1.0)
            for waypoint in page['waypoints']
        )
        assert all(len(waypoint['joint_rad']) == 8 for waypoint in page['waypoints'])
        assert all(
            waypoint['trajectory_time_s'] > 0.0 for waypoint in page['waypoints']
        )
        assert all(
            waypoint['interpolation_mode'] == 'LINEAR'
            for waypoint in page['waypoints']
        )
        assert page['max_waypoint_dwell_s'] == pytest.approx(30.0)
        assert page['max_waypoint_speed_scale'] == pytest.approx(3.0)
        assert page['world_frame'] == 'world'

        result = http_json(
            ui_process.url, '/api/waypoints/name',
            {'index': 1, 'name': '  집기   준비  '},
        )[1]
        assert result['success']
        page = http_json(ui_process.url, '/api/state')[1]
        assert page['waypoints'][0]['name'] == '집기 준비'

        result = http_json(
            ui_process.url, '/api/waypoints/reorder',
            {'from_index': 1, 'to_index': 3},
        )[1]
        assert result['success']
        graph.wait_for(
            lambda: http_json(ui_process.url, '/api/state')[1]
            ['waypoints'][2]['name'] == '집기 준비'
        )
        assert http_json(
            ui_process.url, '/api/waypoints/reorder',
            {'from_index': 3, 'to_index': 1},
        )[1]['success']

        assert http_json(
            ui_process.url, '/api/waypoints/end', {'index': 2}
        )[1]['success']
        graph.wait_for(
            lambda: http_json(ui_process.url, '/api/state')[1]
            ['end_waypoint'] == 2
        )
        exported = http_json(
            ui_process.url, '/api/waypoints/export?arm=left')[1]
        assert exported['schema'] == 'ai_worker_waypoints/v1'
        assert exported['arm'] == 'left'
        assert exported['waypoints'][0]['name'] == '집기 준비'
        assert exported['waypoints'][0]['speed_scale'] == pytest.approx(1.0)
        assert len(exported['waypoints'][0]['joint_positions_rad']) == 8

        assert http_json(
            ui_process.url, '/api/sequence/add',
            {'arm': 'left', 'start_index': 1, 'end_index': 2},
        )[1]['success']
        page = http_json(ui_process.url, '/api/state')[1]
        assert page['execution_sequence'][0]['start_name'] == '집기 준비'
        assert http_json(
            ui_process.url, '/api/sequence/clear', {})[1]['success']

        assert http_json(
            ui_process.url, '/api/waypoints/import', exported
        )[1]['success']
        graph.wait_for(
            lambda: http_json(ui_process.url, '/api/state')[1]
            ['waypoint_count'] == 3
        )
        page = http_json(ui_process.url, '/api/state')[1]
        assert page['waypoints'][0]['name'] == '집기 준비'
        assert page['start_waypoint'] == exported['start_waypoint']
        assert page['end_waypoint'] == exported['end_waypoint']

        generated = http_json(
            ui_process.url,
            '/api/generator/tool-path',
            {
                'arm': 'left',
                'grasp_complete_waypoint': 3,
                'full_path_correction': True,
                'local_offset_mm': [0, 0, 0],
            },
        )[1]
        assert generated['success']
        generated_file = generated['waypoint_file']
        assert generated_file['schema'] == 'ai_worker_waypoints/v1'
        assert generated_file['generator']['algorithm'] == (
            'relative_grasp_compensation/v1')
        assert generated_file['generator']['reference_grasp_waypoint'] == 3
        assert generated_file['generator']['grasp_complete_waypoint'] == 3
        assert generated_file['generator']['full_path_correction'] is True
        assert len(generated_file['waypoints']) == 3
        assert all(
            len(waypoint['joint_positions_rad']) == 8
            for waypoint in generated_file['waypoints']
        )

        result = http_json(
            ui_process.url,
            '/api/waypoints/mode',
            {'index': 2, 'mode': 'JOINT'},
        )[1]
        assert result['success']
        graph.wait_for(
            lambda: http_json(ui_process.url, '/api/state')[1]
            ['waypoints'][1]['interpolation_mode'] == 'JOINT'
        )
        result = http_json(
            ui_process.url,
            '/api/waypoints/start',
            {'index': 2},
        )[1]
        assert result['success']
        assert http_json(ui_process.url, '/api/actions/preview', {})[1]['success']
        graph.wait_for(
            lambda: http_json(ui_process.url, '/api/state')[1]
            ['start_waypoint'] == 2
        )
        page = http_json(ui_process.url, '/api/state')[1]
        assert page['waypoints'][0]['trajectory_time_s'] is None
        assert len(page['waypoints'][0]['joint_rad']) == 8
        assert http_json(
            ui_process.url, '/api/waypoints/start', {'index': 1}
        )[1]['success']
        assert http_json(
            ui_process.url, '/api/actions/select_joint', {}
        )[1]['success']

        result = http_json(
            ui_process.url,
            '/api/waypoints/dwell',
            {'index': 1, 'dwell_s': 0.4},
        )[1]
        assert result['success']
        graph.wait_for(
            lambda: http_json(ui_process.url, '/api/state')[1]
            ['waypoints'][0]['dwell_s'] == pytest.approx(0.4)
        )
        result = http_json(
            ui_process.url,
            '/api/waypoints/speed',
            {'index': 1, 'speed_scale': 2},
        )[1]
        assert result['success']
        graph.wait_for(
            lambda: http_json(ui_process.url, '/api/state')[1]
            ['waypoints'][0]['speed_scale'] == pytest.approx(2.0)
        )
        result = http_json(
            ui_process.url,
            '/api/waypoints/remove',
            {'index': 2},
        )[1]
        assert result['success']
        graph.wait_for(
            lambda: http_json(ui_process.url, '/api/state')[1]
            ['waypoint_count'] == 2
        )
        assert http_json(
            ui_process.url, '/api/actions/clear', {}
        )[1]['success']
        result = http_json(
            ui_process.url,
            '/api/actions/capture_current',
            {'dwell_s': 0.1},
        )[1]
        assert result['success']
        graph.wait_for(
            lambda: http_json(ui_process.url, '/api/state')[1]
            ['waypoints'][0]['dwell_s'] == pytest.approx(0.1)
        )

        with pytest.raises(HTTPError) as missing_confirmation:
            http_json(ui_process.url, '/api/actions/execute', {})
        assert missing_confirmation.value.code == 400
        with pytest.raises(HTTPError) as preview_only:
            http_json(
                ui_process.url,
                '/api/actions/execute',
                {'confirm': 'EXECUTE'},
            )
        assert preview_only.value.code == 409
        with pytest.raises(HTTPError) as slow_start_preview_only:
            http_json(
                ui_process.url,
                '/api/actions/execute_slow_start',
                {'confirm': 'EXECUTE'},
            )
        assert slow_start_preview_only.value.code == 409

        with urlopen(ui_process.url + '/', timeout=3) as response:
            page_html = response.read()
            assert b'Waypoint Telep' in page_html
            assert '선택한 팔 계산 및 재생'.encode() in page_html
            assert '양팔 경로 계산 및 동시 재생'.encode() in page_html
            assert '그리퍼 기준 상대 이동'.encode() in page_html
            assert '전역 고정 frame 기준 상대 이동'.encode() in page_html
            assert '원점 큰 축: WORLD FIXED'.encode() in page_html
            assert '다중 waypoint 목록'.encode() in page_html
            assert '추가할 waypoint 도착 후 대기'.encode() in page_html
            assert '추가할 waypoint까지 속도 배율 (기본 1)'.encode() in page_html
            assert '관절값 리스트 (rad)'.encode() in page_html
            assert 'radian 리스트로 waypoint 추가'.encode() in page_html
            assert '각 지점까지의 보간 방식·동작 속도와 도착 후 대기 시간'.encode() in page_html
            assert '8축 radian 리스트 복사'.encode() in page_html
            assert '실행 시작 지점'.encode() in page_html
            assert '6방향 LINEAR 키보드 제어'.encode() in page_html
            assert '키보드 WAYPOINT'.encode() in page_html
            assert '리더 TELEOP'.encode() in page_html
            assert '시작이 종료보다 크면 역순으로 실행'.encode() in page_html
            assert '현재 팔만 실행'.encode() in page_html
            assert '시작점 저속 정렬 후 실행'.encode() in page_html
            assert '드릴 경로 생성기'.encode() in page_html
        with urlopen(ui_process.url + '/generator', timeout=3) as response:
            generator_html = response.read()
            assert b'Drill Waypoint Generator' in generator_html
            assert '기준 파지 완료 지점 지정'.encode() in generator_html
            assert b'id="robotSimulation"' in generator_html
            assert b'id="downloadButton"' in generator_html
            assert b'/full_robot_simulator.bundle.js' in generator_html
            assert '보정 · IK · Preview 검증 후 Teleop 적용'.encode() in generator_html
        with urlopen(ui_process.url + '/generator.js', timeout=3) as response:
            generator_script = response.read()
            assert b"'/api/generator/tool-path'" in generator_script
            assert b"'/api/waypoints/import'" in generator_script
            assert b"'/api/waypoints/start'" in generator_script
            assert b"'/api/waypoints/end'" in generator_script
            assert b"'/api/actions/preview'" in generator_script
            assert b'previewFullTeleopPath' in generator_script
            assert b'validateTeleopPreview' in generator_script
            assert b'local_offset_mm' in generator_script
            assert b'full_path_correction: true' in generator_script
            assert b'new window.FullRobotSimulator' in generator_script
            assert b"ensureRobotSimulator('right')" in generator_script
        with urlopen(ui_process.url + '/app.js', timeout=3) as response:
            app_script = response.read()
            assert b'async function previewAndPlay()' in app_script
            assert b'async function previewBothAndPlay()' in app_script
            assert b'previewAndPlay().catch' in app_script
            assert b"'/api/waypoints/relative'" in app_script
            assert b"'/api/waypoints/frame-relative'" in app_script
            assert b'positions_rad: positions' in app_script
            assert b"'/api/waypoints/dwell'" in app_script
            assert b"'/api/waypoints/speed'" in app_script
            assert b"'/api/waypoints/mode'" in app_script
            assert b"'/api/waypoints/start'" in app_script
            assert b"'/api/waypoints/remove'" in app_script
            assert b"runAction('execute_slow_start'" in app_script
            assert b'copyWaypointRadians' in app_script
            assert b'renderedWaypointListKey' in app_script
            assert b"KeyW: 'front'" in app_script
            assert b"'/api/keyboard/jog'" in app_script
            assert b"'/api/keyboard/heartbeat'" in app_script
            assert b'keyboardHeartbeatWorkerSource' in app_script
            assert b'previewPoseActive' in app_script
            assert b'KEYBOARD_HEARTBEAT_REQUEST_TIMEOUT_MS = 400' in app_script
            assert b"new URL('/api/keyboard/heartbeat', window.location.href).href" in app_script
            assert b"event.data?.type !== 'heartbeat' || event.data.ok" in app_script
            assert b'if (!state.keyboard.heartbeatWorker) sendKeyboardHeartbeat().catch(() => {});' in app_script
            assert b"'/api/sequence/execute-one'" in app_script
            assert '이것만 실행'.encode() in app_script
            assert '순차 실행 목록의 모든 단계를 위에서 아래 순서로 실행'.encode() in app_script
            assert '순차 목록의 다음 단계는 실행되지 않습니다'.encode() in app_script
            assert b'if (state.renderedWaypointListKey === listRenderKey) return;' in app_script
    finally:
        try:
            ui_process.stop()
        finally:
            node_process.stop()


@pytest.mark.skipif(
    not os.environ.get('WAYPOINT_REFERENCE_JSON'),
    reason='실제 reference JSON을 지정한 수동 검증에서만 실행합니다.',
)
def test_manual_reference_generator_round_trip_and_preview(isolated_graph):
    reference_path = Path(os.environ['WAYPOINT_REFERENCE_JSON'])
    reference = json.loads(reference_path.read_text())
    assert reference['schema'] == 'ai_worker_waypoints/v1'
    assert reference['arm'] == 'right'
    grasp_complete = int(os.environ['WAYPOINT_REFERENCE_GRASP_COMPLETE'])
    preview_start = int(os.environ.get(
        'WAYPOINT_REFERENCE_PREVIEW_START', reference.get('start_waypoint', 1)))
    preview_end = int(os.environ['WAYPOINT_REFERENCE_PREVIEW_END'])
    offset_mm = [1.0, -0.5, 0.75]

    graph = isolated_graph
    graph.statuses.clear()
    graph.paths.clear()
    graph.trajectories.clear()
    graph.joint_names = RIGHT_COMMAND_JOINTS
    graph.positions = list(reference['waypoints'][0]['joint_positions_rad'])
    graph.velocities = [0.0] * 8
    graph.publish_joint_states = True
    node_process = WaypointProcess(
        publish_enabled=False,
        tip_link='end_effector_r_link',
        max_waypoint_speed_scale=30.0,
        quiet=True,
    )
    ui_process = WaypointUiProcess(
        unused_tcp_port(),
        arm='right',
        max_waypoint_speed_scale=30.0,
    )
    try:
        graph.wait_ready(tip_link='end_effector_r_link')
        wait_for_http(ui_process.url)
        imported = http_json(
            ui_process.url, '/api/waypoints/import', reference,
            timeout=300,
        )[1]
        assert imported['success']

        result = http_json(
            ui_process.url,
            '/api/generator/tool-path',
            {
                'arm': 'right',
                'grasp_complete_waypoint': grasp_complete,
                'full_path_correction': True,
                'local_offset_mm': offset_mm,
            },
            timeout=300,
        )[1]
        assert result['success']
        generated = result['waypoint_file']
        assert len(generated['waypoints']) == len(reference['waypoints'])
        assert generated['generator']['reference_grasp_waypoint'] == grasp_complete
        assert generated['generator']['grasp_complete_waypoint'] == grasp_complete
        assert generated['generator']['full_path_correction'] is True

        def rotate_vector(quaternion, vector):
            x, y, z, w = [float(value) for value in quaternion]
            norm = math.sqrt(x * x + y * y + z * z + w * w)
            x, y, z, w = x / norm, y / norm, z / norm, w / norm
            vx, vy, vz = vector
            tx = 2.0 * (y * vz - z * vy)
            ty = 2.0 * (z * vx - x * vz)
            tz = 2.0 * (x * vy - y * vx)
            return [
                vx + w * tx + y * tz - z * ty,
                vy + w * ty + z * tx - x * tz,
                vz + w * tz + x * ty - y * tx,
            ]

        sample_numbers = sorted({
            1, preview_start, grasp_complete, preview_end,
            len(reference['waypoints']),
        })
        local_m = [value / 1000.0 for value in offset_mm]
        for number in sample_numbers:
            source = reference['waypoints'][number - 1]
            target = generated['waypoints'][number - 1]
            fixed_delta = rotate_vector(
                source['orientation_xyzw'],
                local_m,
            )
            expected_position = [
                float(value) + fixed_delta[axis]
                for axis, value in enumerate(source['position_m'])
            ]
            assert target['position_m'] == pytest.approx(
                expected_position, abs=2.0e-6)
            assert target['orientation_xyzw'] == pytest.approx(
                source['orientation_xyzw'], abs=1.0e-6)
            assert target['gripper_rad'] == pytest.approx(source['gripper_rad'])
            assert len(target['joint_positions_rad']) == 8

        # Re-importing uses only the saved 8-axis targets and recomputes FK.
        # Matching poses after this round trip proves that the generated JSON is
        # directly consumable by the existing Waypoint Teleop importer.
        assert http_json(
            ui_process.url, '/api/waypoints/import', generated,
            timeout=300,
        )[1]['success']
        round_trip = http_json(
            ui_process.url, '/api/waypoints/export?arm=right',
            timeout=30,
        )[1]
        for number in sample_numbers:
            assert round_trip['waypoints'][number - 1]['position_m'] == pytest.approx(
                generated['waypoints'][number - 1]['position_m'], abs=2.0e-5)

        assert http_json(
            ui_process.url, '/api/waypoints/start', {'index': preview_start},
            timeout=30,
        )[1]['success']
        assert http_json(
            ui_process.url, '/api/waypoints/end', {'index': preview_end},
            timeout=30,
        )[1]['success']
        preview = http_json(
            ui_process.url, '/api/actions/preview', {'arm': 'right'},
            timeout=300,
        )[1]
        assert preview['success']
        page = http_json(ui_process.url, '/api/state?arm=right', timeout=30)[1]
        assert page['trajectory']['sample_count'] > 0
        assert page['start_waypoint'] == preview_start
        assert page['end_waypoint'] == preview_end
    finally:
        try:
            ui_process.stop()
        finally:
            node_process.stop()


def test_dual_arm_ui_keeps_independent_waypoints_and_trajectories(isolated_graph):
    graph = isolated_graph
    graph.joint_names = LEFT_COMMAND_JOINTS + RIGHT_COMMAND_JOINTS
    graph.positions = [0.0] * 16
    graph.velocities = [0.0] * 16
    graph.publish_joint_states = True
    left_process = WaypointProcess(
        publish_enabled=False,
        tip_link='end_effector_l_link',
        action_topic='/test/left_follow_joint_trajectory',
        arm='left',
    )
    right_process = WaypointProcess(
        publish_enabled=False,
        tip_link='end_effector_r_link',
        action_topic='/test/right_follow_joint_trajectory',
        arm='right',
    )
    ui_process = WaypointUiProcess(unused_tcp_port(), arm='both')
    try:
        deadline = time.monotonic() + 8.0
        while True:
            try:
                left = http_json(ui_process.url, '/api/state?arm=left')[1]
                right = http_json(ui_process.url, '/api/state?arm=right')[1]
                left_model = http_json(
                    ui_process.url, '/api/robot-model?arm=left')[1]
                right_model = http_json(
                    ui_process.url, '/api/robot-model?arm=right')[1]
                if (
                    left['joint_state_fresh'] and right['joint_state_fresh']
                    and left_model['status'] == 'ready'
                    and right_model['status'] == 'ready'
                    and left['current_world_pose'] is not None
                    and right['current_world_pose'] is not None
                ):
                    break
            except OSError:
                pass
            if time.monotonic() >= deadline:
                raise AssertionError('dual-arm UI did not become ready')
            time.sleep(0.05)

        assert left['available_arms'] == ['left', 'right']
        assert right['available_arms'] == ['left', 'right']
        assert left['joint_names'] == LEFT_COMMAND_JOINTS
        assert right['joint_names'] == RIGHT_COMMAND_JOINTS
        assert left_model['tip_link'] == 'end_effector_l_link'
        assert right_model['tip_link'] == 'end_effector_r_link'

        for arm in ('left', 'right'):
            assert http_json(
                ui_process.url, '/api/actions/clear', {'arm': arm}
            )[1]['success']
            assert http_json(
                ui_process.url,
                '/api/waypoints/joint',
                {'arm': arm, 'positions_deg': [0.0] * 8},
            )[1]['success']
            assert http_json(
                ui_process.url,
                '/api/waypoints/relative',
                {
                    'arm': arm,
                    'translation_mm': [0, 0, 0],
                    'rotation_deg': [0, 0, 0],
                },
            )[1]['success']
            assert http_json(
                ui_process.url, '/api/actions/preview', {'arm': arm}
            )[1]['success']

        left = http_json(ui_process.url, '/api/state?arm=left')[1]
        right = http_json(ui_process.url, '/api/state?arm=right')[1]
        assert left['waypoint_count'] == 2
        assert right['waypoint_count'] == 2
        assert left['trajectory']['joint_names'] == LEFT_COMMAND_JOINTS
        assert right['trajectory']['joint_names'] == RIGHT_COMMAND_JOINTS
        assert left['publish_enabled'] is False
        assert right['publish_enabled'] is False
    finally:
        graph.joint_names = LEFT_COMMAND_JOINTS
        graph.positions = [0.0] * 8
        graph.velocities = [0.0] * 8
        try:
            ui_process.stop()
        finally:
            try:
                right_process.stop()
            finally:
                left_process.stop()


@pytest.mark.parametrize('refinement_enabled', [False, True])
def test_ui_actual_stream_records_both_arms_and_returns_to_teleop(isolated_graph, refinement_enabled):
    graph = isolated_graph
    graph.statuses.clear()
    graph.final_commands = {'left': [], 'right': []}
    graph.leader_commands = {'left': [], 'right': []}
    graph.joint_names = (
        LEFT_COMMAND_JOINTS + RIGHT_COMMAND_JOINTS
    )
    graph.positions = [0.0] * len(graph.joint_names)
    graph.velocities = [0.0] * len(graph.joint_names)
    graph.publish_joint_states = True
    planner = WaypointProcess(publish_enabled=True)
    ui = WaypointUiProcess(unused_tcp_port(), publish_enabled=True)
    arbiter = ArbiterProcess()
    runtime_poll_stop = threading.Event()
    runtime_poll_errors = []
    runtime_poll_thread = None
    try:
        graph.wait_ready()
        graph.wait_for(
            lambda: graph.client(
                '/command_source_arbiter/select_teleop').service_is_ready(),
            message='arbiter services did not become ready',
        )
        graph.wait_for(
            lambda: json.loads(
                graph.call('/command_source_arbiter/get_status').message
            )['sources']['teleop']['ready'],
            message='teleop source did not become ready',
        )
        initial_arbiter_status = graph.call('/command_source_arbiter/get_status')
        assert json.loads(initial_arbiter_status.message)['mode'] == 'NONE'
        deadline = time.monotonic() + 5.0
        while True:
            try:
                initial_ui_state = http_json(ui.url, '/api/state')[1]
                if (initial_ui_state['joint_state_fresh'] and
                        initial_ui_state['current_world_pose'] is not None):
                    break
            except OSError:
                pass
            if time.monotonic() >= deadline:
                raise AssertionError('UI planner and joint state did not become ready')
            time.sleep(0.05)

        assert http_json(
            ui.url,
            '/api/actions/select_teleop',
            {'confirm': 'TELEOP'},
        )[1]['success']
        graph.wait_for(
            lambda: json.loads(
                graph.call('/command_source_arbiter/get_status').message
            )['mode'] == 'TELEOP',
            message='test precondition did not select teleop',
        )

        assert http_json(ui.url, '/api/actions/clear', {})[1]['success']
        assert http_json(
            ui.url,
            '/api/waypoints/joint',
            {
                'positions_rad': [
                    0.0, 0.0, 0.0, math.radians(1.0),
                    0.0, 0.0, 0.0, 0.2,
                ],
                'dwell_s': 0.3,
            },
        )[1]['success']
        assert http_json(
            ui.url,
            '/api/waypoints/joint',
            {'positions_rad': [
                0.0, 0.0, 0.0, math.radians(2.0),
                0.0, 0.0, 0.0, 0.4,
            ]},
        )[1]['success']
        for index in (1, 2):
            assert http_json(
                ui.url,
                '/api/waypoints/mode',
                {'index': index, 'mode': 'JOINT'},
            )[1]['success']

        if refinement_enabled:
            assert http_json(ui.url, '/api/path-refinement', {
                'path_refinement': {'enabled': True, 'smoothing_percent': 60},
            })[1]['success']

        def poll_runtime_while_executing():
            while not runtime_poll_stop.is_set():
                for arm in initial_ui_state['available_arms']:
                    try:
                        page = http_json(
                            ui.url, f'/api/runtime?arm={arm}')[1]
                        if 'waypoints' in page or 'trajectory' in page:
                            runtime_poll_errors.append(
                                f'{arm} runtime response included plan data')
                    except Exception as exception:
                        runtime_poll_errors.append(str(exception))
                runtime_poll_stop.wait(0.35)

        runtime_poll_thread = threading.Thread(
            target=poll_runtime_while_executing,
            daemon=True,
        )
        runtime_poll_thread.start()
        response = http_json(
            ui.url,
            '/api/actions/execute',
            {'confirm': 'EXECUTE'},
        )[1]
        assert response['success']

        graph.wait_for(
            lambda: (http_json(ui.url, '/api/runtime')[1]['status'].startswith(
                'EXECUTION_COMPLETE') and
                not http_json(ui.url, '/api/runtime')[1]['execution']['active']),
            timeout=6.0,
            message='UI waypoint stream did not complete',
        )
        graph.wait_for(
            lambda: json.loads(
                graph.call('/command_source_arbiter/get_status').message
            )['mode'] == 'TELEOP',
            timeout=2.0,
            message='completed waypoint stream did not return to teleop',
        )
        runtime_poll_stop.set()
        runtime_poll_thread.join(timeout=2.0)
        assert runtime_poll_errors == []
        assert len(graph.final_commands['left']) > 20
        assert len(graph.final_commands['right']) > 20
        assert all(
            message.header.stamp.sec == 0
            and message.header.stamp.nanosec == 0
            for arm in ('left', 'right')
            for message in graph.final_commands[arm]
        )
        assert any(
            message.points and message.points[0].positions[3] > 0.02
            for message in graph.final_commands['left']
        )
        assert any(
            message.points and message.points[0].positions[7] > 0.3
            for message in graph.final_commands['left']
        )
        dwell_target = math.radians(1.0)
        assert sum(
            1 for message in graph.final_commands['left']
            if message.points
            and abs(message.points[0].positions[3] - dwell_target) < 1.0e-8
        ) >= 20
        assert sum(
            1 for message in graph.final_commands['left']
            if message.points
            and abs(message.points[0].positions[7] - 0.2) < 1.0e-8
        ) >= 20
        assert all(
            abs(message.points[0].positions[3]) < 1.0e-9
            for message in graph.final_commands['right']
            if message.points
        )
        assert 0 in graph.leader_commands['left']
        assert 1 not in graph.leader_commands['left']
        assert graph.call('/command_source_arbiter/select_none').success
        # Stop a second run while its independent publisher is moving back to
        # the first target. It must relinquish ownership and exit, not keep
        # streaming behind a stopped UI.
        before = len(graph.final_commands['left'])
        assert http_json(
            ui.url, '/api/actions/execute', {'confirm': 'EXECUTE'}
        )[1]['success']
        graph.wait_for(
            lambda: any(
                message.points and message.points[0].positions[3] < math.radians(1.9)
                for message in graph.final_commands['left'][before:]),
            timeout=6.0, message='independent publisher did not begin the second run')
        assert http_json(ui.url, '/api/actions/stop', {
            'confirm': 'STOP_BUTTON_CLICK',
        })[1]['success']
        graph.wait_for(
            lambda: not http_json(ui.url, '/api/runtime')[1]['execution']['active'],
            timeout=4.0, message='independent publisher did not stop')
        assert json.loads(graph.call('/command_source_arbiter/get_status').message)['mode'] == 'NONE'
        assert any(event['status'].startswith('STOPPED:') for event in
                   http_json(ui.url, '/api/runtime')[1]['execution_events'])
    finally:
        runtime_poll_stop.set()
        if runtime_poll_thread is not None:
            runtime_poll_thread.join(timeout=2.0)
        try:
            if graph.client('/command_source_arbiter/select_none').service_is_ready():
                graph.call('/command_source_arbiter/select_none')
        except Exception:
            pass
        arbiter.stop()
        ui.stop()
        planner.stop()


def test_dual_arm_keyboard_jog_switches_waypoint_and_teleop_with_deadman(
    isolated_graph,
):
    graph = isolated_graph
    graph.final_commands = {'left': [], 'right': []}
    graph.joint_names = LEFT_COMMAND_JOINTS + RIGHT_COMMAND_JOINTS
    graph.positions = [0.0] * len(graph.joint_names)
    graph.positions[7] = 0.10
    graph.positions[15] = 0.20
    graph.teleop_positions = list(graph.positions)
    graph.teleop_positions[7] = 0.80
    graph.teleop_positions[15] = 0.90
    graph.apply_final_commands_to_state = False
    graph.velocities = [0.0] * len(graph.joint_names)
    graph.publish_joint_states = True
    left = WaypointProcess(
        tip_link='end_effector_l_link', arm='left',
        action_topic='/test/left_keyboard_action')
    right = WaypointProcess(
        tip_link='end_effector_r_link', arm='right',
        action_topic='/test/right_keyboard_action')
    ui = WaypointUiProcess(unused_tcp_port(), arm='both', publish_enabled=True)
    arbiter = ArbiterProcess()
    try:
        graph.wait_for(
            lambda: graph.client(
                '/command_source_arbiter/select_teleop').service_is_ready(),
            message='arbiter services did not become ready',
        )
        wait_for_http(ui.url)
        graph.wait_for(
            lambda: all(
                page['current_world_pose'] is not None and page['joint_state_fresh']
                for page in (http_json(ui.url, f'/api/state?arm={arm}')[1]
                             for arm in ('left', 'right'))),
            message='dual-arm UI planner models did not become ready',
        )

        prepared = http_json(
            ui.url, '/api/actions/prepare_teleop',
            {'arm': 'left', 'confirm': 'PREPARE_TELEOP'},
        )[1]
        assert prepared['success']
        prepared_status = json.loads(
            graph.call('/command_source_arbiter/get_status').message)
        assert prepared_status['mode'] == 'NONE'
        assert prepared_status['intervention_enabled'] is True
        assert prepared_status['sources']['teleop']['ready'] is True

        started = http_json(
            ui.url, '/api/keyboard/start',
            {'arm': 'left', 'confirm': 'KEYBOARD'},
        )[1]
        assert started['success']
        status = json.loads(
            graph.call('/command_source_arbiter/get_status').message)
        assert status['mode'] == 'WAYPOINT'
        graph.wait_for(
            lambda: graph.final_commands['left'] and graph.final_commands['right'],
            message='keyboard handoff commands did not reach both final topics',
        )
        assert graph.final_commands['left'][-1].points[0].positions[-1] == pytest.approx(
            0.80)
        assert graph.final_commands['right'][-1].points[0].positions[-1] == pytest.approx(
            0.90)

        jogged = http_json(
            ui.url, '/api/keyboard/jog',
            {'arm': 'left', 'direction': 'front', 'speed_mmps': 5},
        )[1]
        assert jogged['success']
        graph.wait_for(
            lambda: any(
                message.points
                and any(
                    abs(value) > 1.0e-8
                    for value in message.points[0].positions[:7]
                )
                for message in graph.final_commands['left']
            ),
            message='left Cartesian jog did not reach the final command topic',
        )
        assert graph.final_commands['left'][-1].points[0].positions[-1] == pytest.approx(
            0.80)
        assert graph.final_commands['right'][-1].points[0].positions[-1] == pytest.approx(
            0.90)
        page = http_json(ui.url, '/api/state?arm=left')[1]
        assert page['keyboard_control']['active'] is True
        assert page['keyboard_control']['max_speed_mmps'] == 50.0
        assert page['arbiter']['mode'] == 'WAYPOINT'

        for _ in range(8):
            heartbeat = http_json(ui.url, '/api/keyboard/heartbeat', {})[1]
            assert heartbeat['success']
            time.sleep(0.10)
        page = http_json(ui.url, '/api/state?arm=left')[1]
        assert page['keyboard_control']['active'] is True
        assert page['arbiter']['mode'] == 'WAYPOINT'

        teleop = http_json(
            ui.url, '/api/actions/select_teleop',
            {'arm': 'left', 'confirm': 'TELEOP'},
        )[1]
        assert teleop['success']
        graph.wait_for(
            lambda: json.loads(
                graph.call('/command_source_arbiter/get_status').message
            )['mode'] == 'TELEOP',
            message='keyboard-to-teleop handoff failed',
        )
        assert http_json(ui.url, '/api/state?arm=left')[1][
            'keyboard_control']['active'] is False

        assert http_json(
            ui.url, '/api/keyboard/start',
            {'arm': 'right', 'confirm': 'KEYBOARD'},
        )[1]['success']
        graph.wait_for(
            lambda: json.loads(
                graph.call('/command_source_arbiter/get_status').message
            )['mode'] == 'NONE',
            timeout=2.0,
            message='keyboard heartbeat timeout did not select NONE',
        )
    finally:
        graph.apply_final_commands_to_state = True
        graph.teleop_positions = None
        try:
            if graph.client('/command_source_arbiter/select_none').service_is_ready():
                graph.call('/command_source_arbiter/select_none')
        except Exception:
            pass
        arbiter.stop()
        ui.stop()
        right.stop()
        left.stop()


def test_dual_arm_sequence_executes_selected_ranges_in_order(isolated_graph):
    graph = isolated_graph
    graph.final_commands = {'left': [], 'right': []}
    graph.joint_names = LEFT_COMMAND_JOINTS + RIGHT_COMMAND_JOINTS
    graph.positions = [0.0] * 16
    graph.velocities = [0.0] * 16
    graph.publish_joint_states = True
    left_planner = WaypointProcess(
        tip_link='end_effector_l_link', arm='left')
    right_planner = WaypointProcess(
        tip_link='end_effector_r_link', arm='right')
    ui = WaypointUiProcess(unused_tcp_port(), arm='both', publish_enabled=True)
    arbiter = ArbiterProcess()
    try:
        graph.wait_for(
            lambda: graph.client(
                '/command_source_arbiter/select_teleop').service_is_ready(),
            message='arbiter services did not become ready',
        )
        graph.wait_for(
            lambda: json.loads(
                graph.call('/command_source_arbiter/get_status').message
            )['sources']['teleop']['ready'],
            message='teleop source did not become ready',
        )
        initial_arbiter_status = graph.call('/command_source_arbiter/get_status')
        assert json.loads(initial_arbiter_status.message)['mode'] == 'NONE'
        wait_for_http(ui.url, timeout=8.0)
        graph.wait_for(
            lambda: all(
                page['current_world_pose'] is not None and page['joint_state_fresh']
                for page in (http_json(ui.url, f'/api/state?arm={arm}')[1]
                             for arm in ('left', 'right'))),
            timeout=8.0,
            message='dual-arm planner models did not become ready',
        )
        for arm, joint4 in (('left', math.radians(1.0)),
                            ('right', math.radians(1.0))):
            assert http_json(
                ui.url, '/api/actions/clear', {'arm': arm})[1]['success']
            positions = [0.0] * 8
            positions[3] = joint4
            assert http_json(
                ui.url, '/api/waypoints/joint',
                {'arm': arm, 'positions_rad': positions})[1]['success']
            assert http_json(
                ui.url, '/api/waypoints/mode',
                {'arm': arm, 'index': 1, 'mode': 'JOINT'})[1]['success']
            assert http_json(
                ui.url, '/api/sequence/add',
                {'arm': arm, 'start_index': 1, 'end_index': 1})[1]['success']

        response = http_json(
            ui.url, '/api/sequence/execute-one',
            {'index': 2, 'confirm': 'EXECUTE'},
        )[1]
        assert response['success']
        assert '2번 단계만' in response['message']
        graph.wait_for(
            lambda: (
                not http_json(ui.url, '/api/state?arm=right')[1]
                ['execution']['active']
                and any(
                    message.points
                    and message.points[0].positions[3] > 0.01
                    for message in graph.final_commands['right']
                )
            ),
            timeout=8.0,
            message='selected right-arm sequence step did not complete',
        )
        assert all(
            not message.points or abs(message.points[0].positions[3]) < 1.0e-9
            for message in graph.final_commands['left']
        )
        assert json.loads(
            graph.call('/command_source_arbiter/get_status').message
        )['mode'] == 'TELEOP'

        graph.apply_final_commands_to_state = False
        graph.positions = [0.0] * 16
        graph.wait_for(
            lambda: (
                abs(http_json(ui.url, '/api/state?arm=left')[1]
                    ['current_deg'][3]) < 0.001
                and abs(http_json(ui.url, '/api/state?arm=right')[1]
                        ['current_deg'][3]) < 0.001
            ),
            timeout=2.0,
            message='reset joint state did not reach the UI bridge',
        )
        graph.apply_final_commands_to_state = True
        graph.final_commands = {'left': [], 'right': []}
        response = http_json(
            ui.url, '/api/sequence/execute', {'confirm': 'EXECUTE'})[1]
        assert response['success']
        graph.wait_for(
            lambda: (
                not http_json(ui.url, '/api/state?arm=right')[1]
                ['execution']['active']
                and any(
                    message.points
                    and message.points[0].positions[3] > 0.01
                    for message in graph.final_commands['left']
                )
                and any(
                    message.points
                    and message.points[0].positions[3] > 0.01
                    for message in graph.final_commands['right']
                )
            ),
            timeout=8.0,
            message='dual-arm waypoint sequence did not complete',
        )
        arbiter_status = graph.call('/command_source_arbiter/get_status')
        assert json.loads(arbiter_status.message)['mode'] == 'TELEOP'
        left_move_index = next(
            index for index, message in enumerate(graph.final_commands['left'])
            if message.points and message.points[0].positions[3] > 0.01)
        right_move_index = next(
            index for index, message in enumerate(graph.final_commands['right'])
            if message.points and message.points[0].positions[3] > 0.01)
        assert left_move_index < right_move_index
    finally:
        graph.apply_final_commands_to_state = True
        try:
            if graph.client('/command_source_arbiter/select_none').service_is_ready():
                graph.call('/command_source_arbiter/select_none')
        except Exception:
            pass
        arbiter.stop()
        ui.stop()
        right_planner.stop()
        left_planner.stop()
