#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Sungho Woo, Woojin Wie, Wonho Yun

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_lg2_mini_leader.urdf.xacro',
            description='URDF/XACRO file for the robot model.',
        ),
        DeclareLaunchArgument(
            'left_trajectory_output_topic',
            default_value='/intervention/teleop/arm_left',
            description='Left leader output; command arbitration may route it as teleop.',
        ),
        DeclareLaunchArgument(
            'right_trajectory_output_topic',
            default_value='/intervention/teleop/arm_right',
            description='Right leader output; command arbitration may route it as teleop.',
        ),
        DeclareLaunchArgument(
            'left_joystick_output_topic',
            default_value='/leader/joystick_controller_left/joint_trajectory',
            description='SensorXEL left auxiliary-joint output topic.',
        ),
        DeclareLaunchArgument(
            'right_joystick_output_topic',
            default_value='/leader/joystick_controller_right/joint_trajectory',
            description='SensorXEL right auxiliary-joint output topic.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_output_topic',
            default_value='/cmd_vel',
            description='SensorXEL mobile-base output topic.',
        ),
        DeclareLaunchArgument(
            'tact_trigger_output_topic',
            default_value='/leader/joystick_controller/tact_trigger',
            description='SensorXEL recording-trigger output topic.',
        ),
        DeclareLaunchArgument(
            'teleop_reference_mode',
            default_value='relative',
            choices=['relative', 'absolute'],
            description=(
                'Leader-to-follower mapping mode. Relative anchors both end-effector poses '
                'when an arm is enabled; absolute forwards transformed leader joints.'
            ),
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'left_port_name',
            default_value='/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_00000002-if00',
            description='Serial port for the left mini leader Dynamixel bus.',
        ),
        DeclareLaunchArgument(
            'right_port_name',
            default_value='/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_00000001-if00',
            description='Serial port for the right mini leader Dynamixel bus.',
        ),
        DeclareLaunchArgument(
            'use_sensorxel',
            default_value='true',
            description=(
                'Use the left and right SensorXEL joysticks (Dynamixel ID 10). '
                'Enabled by default for the installed mini leader hardware.'
            ),
        ),
        DeclareLaunchArgument(
            'launch_foot_switch',
            default_value=str(
                Path('/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd').exists()
            ).lower(),
            description='Whether to launch the foot switch node (auto-detected by default).',
        ),
        DeclareLaunchArgument(
            'launch_gripper_trigger',
            default_value='true',
            description=(
                'Enable the spring-gripper trigger that moves each arm to save_pose_3.'
            ),
        ),
        DeclareLaunchArgument(
            'auto_select_teleop',
            default_value='true',
            description=(
                'Select the arbiter TELEOP route after the leader broadcaster is ready; '
                'physical left/right buttons still enable each arm.'
            ),
        ),
    ]

    description_file = LaunchConfiguration('description_file')
    left_trajectory_output_topic = LaunchConfiguration('left_trajectory_output_topic')
    right_trajectory_output_topic = LaunchConfiguration('right_trajectory_output_topic')
    left_joystick_output_topic = LaunchConfiguration('left_joystick_output_topic')
    right_joystick_output_topic = LaunchConfiguration('right_joystick_output_topic')
    cmd_vel_output_topic = LaunchConfiguration('cmd_vel_output_topic')
    tact_trigger_output_topic = LaunchConfiguration('tact_trigger_output_topic')
    teleop_reference_mode = LaunchConfiguration('teleop_reference_mode')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    left_port_name = LaunchConfiguration('left_port_name')
    right_port_name = LaunchConfiguration('right_port_name')
    use_sensorxel = LaunchConfiguration('use_sensorxel')
    launch_foot_switch = LaunchConfiguration('launch_foot_switch')
    launch_gripper_trigger = LaunchConfiguration('launch_gripper_trigger')
    auto_select_teleop = LaunchConfiguration('auto_select_teleop')

    # Robot controllers config file path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffw_bringup'),
            'config',
            'ffw_lg2_mini_leader',
            'ffw_lg2_mini_leader_ai_hardware_controller.yaml',
        ]
    )

    # ros2_control Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        remappings=[
            (
                'joint_trajectory_command_broadcaster_left/joint_trajectory',
                left_trajectory_output_topic,
            ),
            (
                'joint_trajectory_command_broadcaster_right/joint_trajectory',
                right_trajectory_output_topic,
            ),
        ],
        output='both',
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ffw_description'), 'urdf', 'ffw_lg2_mini_leader', description_file]
            ),
            ' ',
            'use_mock_hardware:=', use_mock_hardware,
            ' ',
            'left_port_name:=', left_port_name,
            ' ',
            'right_port_name:=', right_port_name,
            ' ',
            'use_sensorxel:=', use_sensorxel,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '--controller-manager-timeout',
            '15',
            '--controller',
            'joint_state_broadcaster',
            '--controller',
            'joint_trajectory_command_broadcaster',
            ['--controller-ros-args=-p teleop_reference_mode:=', teleop_reference_mode],
            '--controller',
            'trigger_position_controller',
        ],
        parameters=[robot_description],
    )

    joystick_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joystick_controller',
            '--controller-manager-timeout',
            '15',
            [
                '--controller-ros-args=-r '
                '/leader/joystick_controller_left/joint_trajectory:=',
                left_joystick_output_topic,
            ],
            [
                '--controller-ros-args=-r '
                '/leader/joystick_controller_right/joint_trajectory:=',
                right_joystick_output_topic,
            ],
            ['--controller-ros-args=-r /cmd_vel:=', cmd_vel_output_topic],
            [
                '--controller-ros-args=-r '
                '/leader/joystick_controller/tact_trigger:=',
                tact_trigger_output_topic,
            ],
        ],
        parameters=[robot_description],
        condition=IfCondition(use_sensorxel),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'frame_prefix': 'leader_'}],
    )

    foot_switch_node = Node(
        package='ffw_bringup',
        executable='foot_switch_node',
        name='foot_switch_node',
        output='both',
        parameters=[{'controller_config_path': robot_controllers}],
        condition=IfCondition(
            AndSubstitution(launch_foot_switch, NotSubstitution(use_mock_hardware))
        ),
    )

    # leader_feedback_node = Node(
    #     package='ffw_joint_trajectory_command_broadcaster',
    #     executable='leader_feedback',
    #     name='leader_feedback',
    #     output='both',
    # )

    gripper_trigger_node = Node(
        package='ffw_joint_trajectory_command_broadcaster',
        executable='gripper_trigger',
        name='gripper_trigger',
        output='both',
        parameters=[{
            # SensorXEL tact switches toggle teleop arms. Independently, a
            # spring-gripper falling edge requests the saved initial pose 3.
            'enable_press_gestures': False,
            'gripper_threshold': -0.7,
            'save_pose_id': 3,
        }],
        condition=IfCondition(launch_gripper_trigger),
    )

    # Execute process to publish position command
    position_command_process = ExecuteProcess(
        name='trigger_position_command',
        cmd=[
            'ros2', 'topic', 'pub',
            '-r', '50',
            '-t', '50',
            '-p', '50',
            '/leader/trigger_position_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            'data: [0.0, 0.0]',
        ],
    )

    select_teleop_route_process = ExecuteProcess(
        name='select_teleop_route',
        cmd=[
            'ros2', 'service', 'call',
            '/command_source_arbiter/select_teleop',
            'std_srvs/srv/Trigger',
            '{}',
        ],
        output='both',
        condition=IfCondition(auto_select_teleop),
    )

    delayed_teleop_route_selection = TimerAction(
        period=1.0,
        actions=[select_teleop_route_process],
    )

    # The broadcaster initializes from follower state, then holds its most recent
    # leader action whenever teleoperation is stopped. No initial kick is needed.

    def handle_controller_spawner_exit(event, _context):
        if event.returncode == 0:
            return [position_command_process, delayed_teleop_route_selection]
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        'Leader controller startup failed; shutting down to release '
                        'the serial ports.'
                    )
                )
            )
        ]

    delay_position_command_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=handle_controller_spawner_exit,
        )
    )

    def handle_joystick_controller_spawner_exit(event, _context):
        if event.returncode == 0:
            return []
        return [
            EmitEvent(
                event=Shutdown(
                    reason='SensorXEL joystick controller startup failed.'
                )
            )
        ]

    shutdown_on_joystick_controller_failure = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joystick_controller_spawner,
            on_exit=handle_joystick_controller_spawner_exit,
        ),
        condition=IfCondition(use_sensorxel),
    )

    # Wrap everything in a namespace 'leader'
    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('leader'),
            control_node,
            robot_controller_spawner,
            joystick_controller_spawner,
            robot_state_publisher_node,
            delay_position_command_after_controllers,
            shutdown_on_joystick_controller_failure,
            # leader_feedback_node,
            gripper_trigger_node,
        ]
    )

    # Return combined LaunchDescription
    return LaunchDescription(declared_arguments + [
        leader_with_namespace, foot_switch_node])
