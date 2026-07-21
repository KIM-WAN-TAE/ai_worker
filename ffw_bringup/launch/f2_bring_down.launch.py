#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Move an already-running F2 follower into its attention (bring-down) pose."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    position_file_arg = DeclareLaunchArgument(
        'position_file',
        default_value='ffw_f2_follower_bring_down_positions.yaml',
        description='Bring-down trajectory configuration in config/ffw_f2_follower.',
    )

    trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'),
        'config',
        'ffw_f2_follower',
        LaunchConfiguration('position_file'),
    ])

    executors = [
        Node(
            package='ffw_bringup',
            executable='joint_trajectory_executor',
            name=executor_name,
            parameters=[trajectory_params_file],
            output='screen',
        )
        for executor_name in (
            'arm_l_joint_trajectory_executor',
            'arm_r_joint_trajectory_executor',
            'head_joint_trajectory_executor',
            'lift_joint_trajectory_executor',
        )
    ]

    return LaunchDescription([position_file_arg, *executors])
