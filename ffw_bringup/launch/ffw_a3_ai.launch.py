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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_a3.urdf.xacro',
            description='URDF/XACRO file for the robot model.',
        ),
        DeclareLaunchArgument(
            'left_trajectory_output_topic',
            default_value='joint_trajectory_command_broadcaster_left/joint_trajectory',
            description='Left leader output; command arbitration may route it as teleop.',
        ),
        DeclareLaunchArgument(
            'right_trajectory_output_topic',
            default_value='joint_trajectory_command_broadcaster_right/joint_trajectory',
            description='Right leader output; command arbitration may route it as teleop.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/serial/by-id/usb-ROBOTIS_STM32_Virtual_ComPort_3151388A3434-if00',
            description='Serial port for the A3 leader Dynamixel bus.',
        ),
        DeclareLaunchArgument(
            'launch_foot_switch',
            default_value='true',
            description='Whether to launch the foot switch node.',
        ),
        DeclareLaunchArgument(
            'launch_gripper_trigger',
            default_value='true',
            description='Whether to launch the gripper save-pose trigger node.',
        ),
    ]

    description_file = LaunchConfiguration('description_file')
    left_trajectory_output_topic = LaunchConfiguration('left_trajectory_output_topic')
    right_trajectory_output_topic = LaunchConfiguration('right_trajectory_output_topic')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    port_name = LaunchConfiguration('port_name')
    launch_foot_switch = LaunchConfiguration('launch_foot_switch')
    launch_gripper_trigger = LaunchConfiguration('launch_gripper_trigger')

    # Robot controllers config file path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffw_bringup'),
            'config',
            'ffw_a3',
            'ffw_a3_ai_hardware_controller.yaml',
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
                [FindPackageShare('ffw_description'), 'urdf', 'ffw_a3', description_file]
            ),
            ' ',
            'use_mock_hardware:=', use_mock_hardware,
            ' ',
            'port_name:=', port_name,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_command_broadcaster',
            'spring_actuator_controller',
            'joystick_controller',
            'joint_state_broadcaster',
        ],
        parameters=[robot_description],
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
        condition=IfCondition(launch_foot_switch),
    )

    gripper_trigger_node = Node(
        package='ffw_joint_trajectory_command_broadcaster',
        executable='gripper_trigger',
        name='gripper_trigger',
        output='both',
        parameters=[{'gripper_threshold': -0.5}],
        condition=IfCondition(launch_gripper_trigger),
    )

    # Wrap everything in a namespace 'leader'
    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('leader'),
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
            gripper_trigger_node,
        ]
    )

    # Return combined LaunchDescription
    return LaunchDescription(declared_arguments + [leader_with_namespace, foot_switch_node])
