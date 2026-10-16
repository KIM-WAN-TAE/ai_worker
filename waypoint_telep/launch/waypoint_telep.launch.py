#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def planner_node(
    arm, config_file, publish_enabled, path_mode, base_link, world_frame
):
    side = 'l' if arm == 'left' else 'r'
    return Node(
        package='waypoint_telep',
        executable='waypoint_telep_node',
        namespace='waypoint_telep',
        name=arm,
        output='screen',
        parameters=[
            config_file,
            {
                'publish_enabled': ParameterValue(
                    publish_enabled, value_type=bool
                ),
                'path_mode': path_mode,
                'base_link': base_link,
                'world_frame': world_frame,
                'tip_link': f'end_effector_{side}_link',
                'gripper_joint': f'gripper_{side}_joint1',
                'action_topic': (
                    f'/arm_{side}_controller/follow_joint_trajectory'
                ),
                'external_command_topic': (
                    '/leader/joint_trajectory_command_broadcaster_'
                    f'{arm}/joint_trajectory'
                ),
                'clicked_point_topic': (
                    f'/waypoint_telep/{arm}/clicked_point'
                ),
            },
        ],
    )


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory('waypoint_telep'))
        / 'config'
        / 'waypoint_telep.yaml'
    )

    arguments = [
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('publish_enabled', default_value='false'),
        DeclareLaunchArgument('path_mode', default_value='linear'),
        DeclareLaunchArgument('start_ui', default_value='true'),
        DeclareLaunchArgument('ui_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('ui_port', default_value='8765'),
        DeclareLaunchArgument('ui_public_host', default_value=''),
        DeclareLaunchArgument('base_link', default_value='arm_base_link'),
        # F2 publishes the fixed navigation frame as odom. There is no `world`
        # frame in the live TF tree, so using world would silently lose the
        # mobile base translation and yaw.
        DeclareLaunchArgument('world_frame', default_value='odom'),
    ]

    config_file = LaunchConfiguration('config_file')
    publish_enabled = LaunchConfiguration('publish_enabled')
    path_mode = LaunchConfiguration('path_mode')
    base_link = LaunchConfiguration('base_link')
    world_frame = LaunchConfiguration('world_frame')
    left_node = planner_node(
        'left', config_file, publish_enabled, path_mode, base_link, world_frame)
    right_node = planner_node(
        'right', config_file, publish_enabled, path_mode, base_link, world_frame)

    ui = Node(
        package='waypoint_telep',
        executable='waypoint_telep_ui',
        name='waypoint_telep_ui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_ui')),
        on_exit=Shutdown(
            reason='Waypoint UI exited; stopping duplicate planner nodes.'
        ),
        parameters=[config_file, {
            'arm': 'both',
            'host': LaunchConfiguration('ui_host'),
            'port': ParameterValue(
                LaunchConfiguration('ui_port'), value_type=int
            ),
            'public_host': LaunchConfiguration('ui_public_host'),
            'publish_enabled': ParameterValue(
                publish_enabled, value_type=bool
            ),
            'path_mode': path_mode,
            'waypoint_namespace': '/waypoint_telep',
            'base_link': base_link,
            'world_frame': world_frame,
            'tip_link': '',
        }],
    )

    return LaunchDescription(arguments + [left_node, right_node, ui])
