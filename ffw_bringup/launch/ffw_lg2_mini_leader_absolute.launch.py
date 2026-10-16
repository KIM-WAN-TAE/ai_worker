#!/usr/bin/env python3

"""Start the LG2 Mini Leader as an absolute TELEOP source.

This wrapper prepares the command-source arbiter before the underlying Mini
Leader launch selects TELEOP.  Per-arm enable remains gated by the physical
left/right buttons, so starting this launch file alone does not move an arm.
"""

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _arbiter_service(action: str) -> ExecuteProcess:
    return ExecuteProcess(
        name=f'absolute_teleop_{action}',
        cmd=[
            'ros2',
            'service',
            'call',
            f'/command_source_arbiter/{action}',
            'std_srvs/srv/Trigger',
            '{}',
        ],
        output='both',
    )


def generate_launch_description():
    mini_leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ffw_bringup'),
                'launch',
                'ffw_lg2_mini_leader_ai.launch.py',
            ])
        ),
        launch_arguments={
            'teleop_reference_mode': 'absolute',
            'auto_select_teleop': 'true',
        }.items(),
    )

    # The base launch selects TELEOP only after its controllers are active.
    # Prepare the intervention gate first so that selection is accepted.
    prepare_intervention = TimerAction(
        period=0.5,
        actions=[_arbiter_service('prepare_intervention')],
    )

    # Close the final command route before the Leader publishers disappear.
    close_route_on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=[_arbiter_service('select_none')])
    )

    return LaunchDescription([
        close_route_on_shutdown,
        mini_leader_launch,
        prepare_intervention,
    ])
