import importlib.util
import math
from pathlib import Path
import subprocess
import threading
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from ffw_bringup.command_source_arbiter import CommandSourceArbiter
import pytest
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'waypoint_telep_ui.py'
SPEC = importlib.util.spec_from_file_location('waypoint_telep_ui', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_path_refinement_user_units_defaults_and_validation():
    assert MODULE.parse_path_refinement(None)['enabled'] is False
    for strength in (0, 30, 60, 100):
        result = MODULE.parse_path_refinement({'enabled': True, 'smoothing_percent': strength})
        assert result['smoothing_percent'] == strength
        assert result['position_tolerance_mm'] == 1
    for value in ([], True, 'settings'):
        with pytest.raises(ValueError):
            MODULE.parse_path_refinement(value)
    for patch in ({'enabled': 1}, {'subdivisions': True}, {'subdivisions': 0},
                  {'subdivisions': 1.2}, {'subdivisions': 257}, {'smoothing_percent': 101},
                  {'position_tolerance_mm': 0}, {'position_tolerance_mm': float('nan')},
                  {'orientation_tolerance_deg': float('inf')}, {'joint_tolerance_rad': -1},
                  {'max_refinements': 9}, {'max_refinements': 1.5}, {'max_refinements': '3'}):
        with pytest.raises(ValueError):
            MODULE.parse_path_refinement(patch)


def test_stop_during_planning_bypasses_execution_lock_and_sets_cancellation():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.execution_lock = threading.Lock()
    bridge.execution_stop = threading.Event()
    bridge.intervention_controller_url = ''
    bridge._select_arm = lambda arm: 'left'
    bridge._execution_is_active = lambda: pytest.fail('STOP waited on the execution guard')
    bridge._deactivate_keyboard = lambda: False
    bridge._call_arbiter = lambda action: {'success': action == 'select_none'}
    bridge._set_status = lambda arm, status: None
    with bridge.execution_lock:
        assert bridge.call_action('stop', {'confirm': 'STOP_BUTTON_CLICK'})['success']
    assert bridge.execution_stop.is_set()


@pytest.mark.parametrize('expected', ['JOINT_TIMING_UPDATED', 'PATH_REFINEMENT_UPDATED'])
def test_planner_settings_wait_for_topic_match_and_their_own_ack(monkeypatch, expected):
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = threading.Lock()
    state = {'status_sequence': 0, 'status': ''}
    bridge.arm_states = {'left': state}
    actions = []
    ready = False
    published = False

    class Publisher:
        def get_subscription_count(self):
            return int(ready)

        def publish(self, message):
            nonlocal published
            assert ready and message == 'settings'
            published = True
            actions.append('publish')
            state.update(status_sequence=2, status='MODEL_READY: arm_base_link')

    def settle(_seconds):
        nonlocal ready
        actions.append('ack' if published else 'match')
        if published:
            state.update(status_sequence=3, status=expected)
        else:
            ready = True

    monkeypatch.setattr(MODULE.time, 'sleep', settle)
    assert bridge._publish_planner_setting('left', Publisher(), 'settings', expected) == expected
    assert actions == ['match', 'publish', 'ack']


def test_planner_setting_wait_reports_rejection_without_waiting_for_ack():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = threading.Lock()
    bridge.arm_states = {'left': {'status_sequence': 1, 'status': 'ERROR: invalid setting'}}
    assert bridge._wait_for_status(
        'left', 0, expected_prefix='PATH_REFINEMENT_UPDATED') == 'ERROR: invalid setting'


def test_equal_time_validates_duration_and_exclusions():
    assert MODULE.parse_equal_segment_time({}, 4) == (0.0, [])
    assert MODULE.parse_equal_segment_time(
        {'duration_s': 1.25, 'excluded_waypoints': [3, 2, 3]}, 4) == (1.25, [2, 3])
    for payload in ({'duration_s': True}, {'duration_s': float('nan')},
                    {'duration_s': -1}, {'excluded_waypoints': [True]},
                    {'excluded_waypoints': [1]}, {'excluded_waypoints': [5]},
                    {'excluded_waypoints': [2, 3, 4]}, {'excluded_waypoints': '2-3'}):
        with pytest.raises(ValueError):
            MODULE.parse_equal_segment_time(payload, 4)
    with pytest.raises(ValueError):
        MODULE.parse_equal_segment_time({}, 1)


def test_total_time_requires_positive_exclusive_target():
    assert MODULE.parse_equal_segment_time(
        {'total_duration_s': 180, 'excluded_waypoints': [3]}, 4) == (180.0, [3])
    for value in (0, -1, True, '180', None, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            MODULE.parse_equal_segment_time({'total_duration_s': value}, 4)
    with pytest.raises(ValueError):
        MODULE.parse_equal_segment_time({'total_duration_s': 10, 'duration_s': 1}, 4)


def test_joint_timing_rejects_invalid_file_before_import():
    assert MODULE.parse_joint_timing(None, 477) is None
    valid = dict(speed_mps=0.0327590555675, preserve_start_waypoint=225,
                 preserve_end_waypoint=231)
    assert MODULE.parse_joint_timing(valid, 477) == valid
    for patch in ({'speed_mps': float('nan')}, {'speed_mps': True},
                  {'speed_mps': 0.11}, {'preserve_start_waypoint': 1},
                  {'preserve_end_waypoint': 478}, {'preserve_start_waypoint': 232}):
        with pytest.raises(ValueError):
            MODULE.parse_joint_timing({**valid, **patch}, 477)


def _expanded_f2_description():
    description_share = Path(get_package_share_directory('ffw_description'))
    xacro_path = (
        description_share / 'urdf' / 'ffw_f2_follower' /
        'ffw_f2_follower.urdf.xacro'
    )
    completed = subprocess.run(
        ['xacro', str(xacro_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(completed.stdout)


def test_f2_urdf_arbiter_and_waypoint_limits_share_one_safe_contract():
    root = _expanded_f2_description()
    urdf_limits = {}
    for joint in root.findall('joint'):
        name = joint.get('name', '')
        is_arm_joint = (
            (name.startswith('arm_l_joint') or name.startswith('arm_r_joint'))
            and name[-1:] in {str(index) for index in range(1, 8)}
        )
        if not is_arm_joint and name not in {
            'gripper_l_joint1', 'gripper_r_joint1',
        }:
            continue
        limit = joint.find('limit')
        urdf_limits[name] = (float(limit.get('lower')), float(limit.get('upper')))

    for side, arm_letter in (('left', 'l'), ('right', 'r')):
        expected = [
            urdf_limits[f'arm_{arm_letter}_joint{index}']
            for index in range(1, 8)
        ] + [urdf_limits[f'gripper_{arm_letter}_joint1']]
        assert MODULE.F2_POSITION_LIMITS[side] == pytest.approx(expected)
        assert CommandSourceArbiter.F2_POSITION_LIMITS[side] == pytest.approx(expected)

    # The arm software envelope must never exceed the configured motor limits.
    gpio = {element.get('name'): element for element in root.findall('.//gpio')}
    for side, arm_letter, id_offset in (
        ('right', 'r', 0), ('left', 'l', 30),
    ):
        for index in range(1, 8):
            params = {
                param.get('name'): param.text
                for param in gpio[f'dxl{id_offset + index}'].findall('param')
            }
            ticks_per_rev = 607_500 if index == 7 else 524_288
            rad_per_tick = 2.0 * math.pi / ticks_per_rev
            hardware_lower = float(params['Min Position Limit']) * rad_per_tick
            hardware_upper = float(params['Max Position Limit']) * rad_per_tick
            software_lower, software_upper = urdf_limits[
                f'arm_{arm_letter}_joint{index}'
            ]
            assert software_lower >= hardware_lower - 1e-12, (side, index)
            assert software_upper <= hardware_upper + 1e-12, (side, index)


def test_arm_joint_names():
    assert MODULE.arm_joint_names('left') == [
        f'arm_l_joint{index}' for index in range(1, 8)
    ]
    assert MODULE.arm_joint_names('right') == [
        f'arm_r_joint{index}' for index in range(1, 8)
    ]
    with pytest.raises(ValueError):
        MODULE.arm_joint_names('center')


def test_command_joint_names_include_recorded_gripper_dimension():
    assert MODULE.command_joint_names('left')[-1] == 'gripper_l_joint1'
    assert MODULE.command_joint_names('right')[-1] == 'gripper_r_joint1'
    assert len(MODULE.command_joint_names('left')) == 8


@pytest.mark.parametrize(('value', 'expected'), [(1, 1), ('3', 3), (4.0, 4)])
def test_parse_waypoint_index(value, expected):
    assert MODULE.parse_waypoint_index({'index': value}) == expected


@pytest.mark.parametrize('value', [None, 0, -1, 1.5, True, float('nan')])
def test_parse_waypoint_index_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        MODULE.parse_waypoint_index({'index': value})


@pytest.mark.parametrize(
    ('value', 'expected'),
    [('linear', ('LINEAR', 0)), ('LINEAR', ('LINEAR', 0)),
     ('joint', ('JOINT', 1)), ('JOINT', ('JOINT', 1))],
)
def test_parse_waypoint_mode(value, expected):
    assert MODULE.parse_waypoint_mode({'mode': value}) == expected


@pytest.mark.parametrize('value', [None, '', 'mixed', 3])
def test_parse_waypoint_mode_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        MODULE.parse_waypoint_mode({'mode': value})


def test_parse_waypoint_name_normalizes_spacing_and_keeps_korean():
    assert MODULE.parse_waypoint_name({'name': '  왼팔   집기  '}) == '왼팔 집기'


@pytest.mark.parametrize('value', [None, '', '   ', 'x' * 61])
def test_parse_waypoint_name_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        MODULE.parse_waypoint_name({'name': value})


def test_parse_reorder_indices_uses_one_based_indices():
    assert MODULE.parse_reorder_indices({
        'from_index': '3', 'to_index': 1,
    }) == (3, 1)
    with pytest.raises(ValueError):
        MODULE.parse_reorder_indices({'from_index': 0, 'to_index': 1})


def test_parse_execution_step_validates_arm_and_allows_reverse_range():
    assert MODULE.parse_execution_step({
        'arm': 'right', 'start_index': 2, 'end_index': 4,
    }) == {'arm': 'right', 'start_index': 2, 'end_index': 4}
    assert MODULE.parse_execution_step({
        'arm': 'left', 'start_index': 4, 'end_index': 1,
    }) == {'arm': 'left', 'start_index': 4, 'end_index': 1}
    with pytest.raises(ValueError):
        MODULE.parse_execution_step({
            'arm': 'center', 'start_index': 1, 'end_index': 2})


def test_parse_tool_path_correction_uses_one_based_range_and_mm():
    parsed = MODULE.parse_tool_path_correction({
        'approach_start_waypoint': 3,
        'grasp_waypoint': 5,
        'grasp_complete_waypoint': 7,
        'local_offset_mm': [4, -2.5, 1],
    }, 8)
    assert parsed['approach_start_waypoint'] == 3
    assert parsed['grasp_waypoint'] == 5
    assert parsed['reference_grasp_waypoint'] == 5
    assert parsed['grasp_complete_waypoint'] == 7
    assert parsed['full_path_correction'] is False
    assert parsed['local_offset_mm'] == [4.0, -2.5, 1.0]
    assert parsed['norm_mm'] == pytest.approx(math.sqrt(23.25))


def test_parse_tool_path_correction_full_path_needs_only_grasp_complete():
    parsed = MODULE.parse_tool_path_correction({
        'grasp_complete_waypoint': 7,
        'full_path_correction': True,
        'local_offset_mm': [1, -0.5, 0.75],
    }, 8)
    assert parsed['approach_start_waypoint'] == 1
    assert parsed['grasp_waypoint'] == 1
    assert parsed['reference_grasp_waypoint'] == 7
    assert parsed['grasp_complete_waypoint'] == 7
    assert parsed['full_path_correction'] is True


@pytest.mark.parametrize('payload', [
    {
        'approach_start_waypoint': 6,
        'grasp_waypoint': 5,
        'local_offset_mm': [0, 0, 0],
    },
    {
        'approach_start_waypoint': 1,
        'grasp_waypoint': 9,
        'local_offset_mm': [0, 0, 0],
    },
    {
        'approach_start_waypoint': 1,
        'grasp_waypoint': 5,
        'grasp_complete_waypoint': 4,
        'local_offset_mm': [0, 0, 0],
    },
    {
        'approach_start_waypoint': 1,
        'grasp_waypoint': 2,
        'local_offset_mm': [50, 50, 0],
    },
    {
        'approach_start_waypoint': 1,
        'grasp_waypoint': 2,
        'local_offset_mm': [0, 0],
    },
    {
        'grasp_complete_waypoint': 3,
        'full_path_correction': 'true',
        'local_offset_mm': [0, 0, 0],
    },
])
def test_parse_tool_path_correction_rejects_invalid_input(payload):
    with pytest.raises(ValueError):
        MODULE.parse_tool_path_correction(payload, 8)


@pytest.mark.parametrize(
    ('value', 'expected'),
    [('last', -1), (None, -1), ('current', 0), (1, 1), ('3', 3)],
)
def test_parse_relative_reference(value, expected):
    assert MODULE.parse_relative_reference({'reference': value}) == expected


@pytest.mark.parametrize('value', [-2, 0, 1.5, True, 'unknown', float('nan')])
def test_parse_relative_reference_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        MODULE.parse_relative_reference({'reference': value})


def test_parse_dwell_seconds_uses_zero_default_and_bounds_value():
    assert MODULE.parse_dwell_seconds({}, 30.0) == 0.0
    assert MODULE.parse_dwell_seconds({'dwell_s': '1.5'}, 30.0) == 1.5
    for value in (-0.1, 30.1, True, float('inf')):
        with pytest.raises(ValueError):
            MODULE.parse_dwell_seconds({'dwell_s': value}, 30.0)


def test_parse_speed_scale_defaults_to_one_and_bounds_value():
    assert MODULE.parse_speed_scale({}) == 1.0
    assert MODULE.parse_speed_scale({'speed_scale': '0.5'}) == 0.5
    assert MODULE.parse_speed_scale({'speed_scale': 2}) == 2.0
    assert MODULE.parse_speed_scale({'speed_percent': 200}) == 2.0
    for value in (0, 3.1, True, float('inf')):
        with pytest.raises(ValueError):
            MODULE.parse_speed_scale({'speed_scale': value})


def test_smooth_joint_transition_keeps_every_command_within_step_limit():
    start = [0.0, -0.2, 0.4]
    target = [0.14, -0.28, 0.4]
    points = MODULE.smooth_joint_transition(start, target, 0.05)

    assert points
    assert points[-1] == pytest.approx(target)
    for previous, point in zip([start, *points[:-1]], points):
        assert max(
            abs(value - old) for value, old in zip(point, previous)
        ) <= 0.05 + 1.0e-12
    assert MODULE.smooth_joint_transition(start, [0.01, -0.2, 0.4], 0.05) == []


def test_plan_validation_bridges_start_gap_but_rejects_internal_jump():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = MODULE.threading.RLock()
    bridge.arm_states = {'left': {'joint_smoothing_enabled': False}}
    bridge.max_joint_step_rad = 0.05
    bridge._current_command_positions = lambda: {
        'left': [0.0] * 8,
        'right': [0.0] * 8,
    }
    bridge._validate_command_pair = lambda _pair: None
    trajectory = MODULE.JointTrajectory()
    trajectory.joint_names = MODULE.command_joint_names('left')
    first = MODULE.JointTrajectoryPoint()
    first.positions = [0.2] + [0.0] * 7
    first.time_from_start.nanosec = 10_000_000
    second = MODULE.JointTrajectoryPoint()
    second.positions = [0.21] + [0.0] * 7
    second.time_from_start.nanosec = 20_000_000
    trajectory.points = [first, second]

    bridge._validate_plan('left', trajectory, [0.0] * 8)
    second.positions[0] = 0.3
    with pytest.raises(RuntimeError, match='trajectory 내부 관절 간격'):
        bridge._validate_plan('left', trajectory, [0.0] * 8)


def test_continuous_joint_plan_allows_first_waypoint_gap():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = MODULE.threading.RLock()
    bridge.arm_states = {'left': {'joint_smoothing_enabled': True}}
    bridge.max_joint_step_rad = 0.05
    bridge._current_command_positions = lambda: {
        'left': [0.0] * 8,
        'right': [0.0] * 8,
    }
    bridge._validate_command_pair = lambda _pair: None
    trajectory = MODULE.JointTrajectory()
    trajectory.joint_names = MODULE.command_joint_names('left')
    point = MODULE.JointTrajectoryPoint()
    point.positions = [0.02] + [0.0] * 7
    point.time_from_start.nanosec = 10_000_000
    trajectory.points = [point]

    bridge._validate_plan('left', trajectory, [0.0] * 8)


@pytest.mark.parametrize(
    ('direction', 'expected'),
    [
        ('front', [0.00025, 0.0, 0.0]),
        ('back', [-0.00025, 0.0, 0.0]),
        ('left', [0.0, 0.00025, 0.0]),
        ('right', [0.0, -0.00025, 0.0]),
        ('up', [0.0, 0.0, 0.00025]),
        ('down', [0.0, 0.0, -0.00025]),
    ],
)
def test_parse_cartesian_jog_maps_six_world_directions(direction, expected):
    parsed_direction, vector, speed = MODULE.parse_cartesian_jog(
        {'direction': direction, 'speed_mmps': 5}, 50.0, 0.05)
    assert parsed_direction == direction
    assert vector == pytest.approx(expected)
    assert speed == 5.0


def test_parse_cartesian_jog_accepts_50_mmps_ceiling_as_2_5_mm_step():
    direction, vector, speed = MODULE.parse_cartesian_jog(
        {'direction': 'up', 'speed_mmps': 50}, 50.0, 0.05)
    assert direction == 'up'
    assert vector == pytest.approx([0.0, 0.0, 0.0025])
    assert speed == 50.0


@pytest.mark.parametrize(
    'payload',
    [
        {'direction': 'diagonal', 'speed_mmps': 5},
        {'direction': 'front', 'speed_mmps': 0},
        {'direction': 'front', 'speed_mmps': 50.1},
        {'direction': 'front', 'speed_mmps': True},
    ],
)
def test_parse_cartesian_jog_rejects_invalid_requests(payload):
    with pytest.raises(ValueError):
        MODULE.parse_cartesian_jog(payload, 50.0, 0.05)


def test_parse_joint_values_converts_degrees_to_radians():
    values = MODULE.parse_joint_values(
        {'positions_deg': [0, 30, -45, 90, 1.5, -2.5, 180, 45]}, 8)
    assert values[0] == 0.0
    assert values[1] == pytest.approx(math.pi / 6.0)
    assert values[2] == pytest.approx(-math.pi / 4.0)
    assert values[6] == pytest.approx(math.pi)
    assert values[-1] == pytest.approx(math.pi / 4.0)


def test_parse_joint_values_accepts_radian_list_without_conversion():
    expected = [
        -0.4156368881675766,
        0.774600376757722,
        -0.5792455265755045,
        -1.344654002830818,
        0.5723905499296406,
        -0.05628990438045982,
        -1.067158946493481,
        0.3,
    ]
    assert MODULE.parse_joint_values(
        {'positions_rad': expected}, 8
    ) == expected


@pytest.mark.parametrize(
    'payload',
    [
        {},
        {'positions_deg': [0] * 7},
        {'positions_deg': [0, 0, 0, 0, 0, 0, 0, None]},
        {'positions_deg': [0, 0, 0, 0, 0, 0, 0, float('nan')]},
        {'positions_deg': [0, 0, 0, 0, 0, 0, 0, True]},
        {'positions_rad': [0] * 7},
        {'positions_rad': [0, 0, 0, 0, 0, 0, 0, float('inf')]},
        {'positions_deg': [0] * 8, 'positions_rad': [0] * 8},
    ],
)
def test_parse_joint_values_rejects_invalid_input(payload):
    with pytest.raises(ValueError):
        MODULE.parse_joint_values(payload, 8)


def test_parse_relative_pose_and_quaternion():
    translation, rotation = MODULE.parse_relative_pose({
        'translation_mm': [10, -20, 30],
        'rotation_deg': [0, 0, 90],
    })
    assert translation == pytest.approx([0.01, -0.02, 0.03])
    assert rotation == pytest.approx([0.0, 0.0, math.pi / 2.0])
    quaternion = MODULE.rpy_to_quaternion(rotation)
    assert quaternion == pytest.approx([
        0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)
    ])


def test_parse_absolute_pose_uses_meters_and_converts_degrees():
    position, rotation = MODULE.parse_absolute_pose({
        'position_m': [0.4, -0.2, 1.1],
        'orientation_deg': [90, 0, -45],
    })
    assert position == pytest.approx([0.4, -0.2, 1.1])
    assert rotation == pytest.approx([math.pi / 2.0, 0.0, -math.pi / 4.0])


@pytest.mark.parametrize(
    ('mode', 'payload', 'expected_position', 'expected_rotation'),
    [
        ('position', {'position_m': [0.4, -0.2, 1.1]}, [0.4, -0.2, 1.1], None),
        ('orientation', {'orientation_deg': [90, 0, -45]}, None,
         [math.pi / 2.0, 0.0, -math.pi / 4.0]),
        ('pose', {
            'position_m': [0.4, -0.2, 1.1],
            'orientation_deg': [90, 0, -45],
        }, [0.4, -0.2, 1.1], [math.pi / 2.0, 0.0, -math.pi / 4.0]),
    ],
)
def test_parse_absolute_target_only_requires_selected_components(
    mode, payload, expected_position, expected_rotation
):
    selected, position, rotation = MODULE.parse_absolute_target({
        'target_mode': mode, **payload,
    })
    assert selected == mode
    assert (
        position == pytest.approx(expected_position)
        if position is not None else expected_position is None
    )
    assert (
        rotation == pytest.approx(expected_rotation)
        if rotation is not None else expected_rotation is None
    )


@pytest.mark.parametrize(
    'payload',
    [
        {},
        {'translation_mm': [0, 0], 'rotation_deg': [0, 0, 0]},
        {'translation_mm': [0, 0, 0], 'rotation_deg': [0, None, 0]},
        {'translation_mm': [0, True, 0], 'rotation_deg': [0, 0, 0]},
    ],
)
def test_parse_relative_pose_rejects_invalid_input(payload):
    with pytest.raises(ValueError):
        MODULE.parse_relative_pose(payload)


@pytest.mark.parametrize(
    'payload',
    [
        {},
        {'position_m': [0, 0], 'orientation_deg': [0, 0, 0]},
        {'position_m': [0, 0, 0], 'orientation_deg': [0, None, 0]},
        {'position_m': [0, True, 0], 'orientation_deg': [0, 0, 0]},
    ],
)
def test_parse_absolute_pose_rejects_invalid_input(payload):
    with pytest.raises(ValueError):
        MODULE.parse_absolute_pose(payload)


def test_parse_absolute_target_rejects_unknown_mode():
    with pytest.raises(ValueError):
        MODULE.parse_absolute_target({'target_mode': 'xyzrpy-ish'})


def test_browser_urls_are_clickable_and_never_use_unspecified_bind_address():
    assert MODULE.browser_urls('localhost', 8766) == [
        'http://127.0.0.1:8766'
    ]
    assert MODULE.browser_urls(
        '0.0.0.0', 8766, '192.168.6.2'
    ) == ['http://192.168.6.2:8766']
    detected = MODULE.browser_urls('0.0.0.0', 8766)
    assert 'http://127.0.0.1:8766' in detected
    assert all('0.0.0.0' not in url for url in detected)


def test_parse_urdf_chain_keeps_fixed_segments_and_joint_metadata():
    model = MODULE.parse_urdf_chain(
        """
        <robot name="test_f2">
          <link name="base"/>
          <link name="shoulder"/>
          <link name="wrist"/>
          <link name="tool"/>
          <joint name="joint1" type="revolute">
            <parent link="base"/><child link="shoulder"/>
            <origin xyz="0 0 0.2" rpy="0 0 1.57"/>
            <axis xyz="0 1 0"/><limit lower="-1" upper="2"/>
          </joint>
          <joint name="joint2" type="continuous">
            <parent link="shoulder"/><child link="wrist"/>
            <origin xyz="0.3 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="tool_fixed" type="fixed">
            <parent link="wrist"/><child link="tool"/>
            <origin xyz="0.1 0 0"/>
          </joint>
        </robot>
        """,
        'base',
        'tool',
    )
    assert model['robot_name'] == 'test_f2'
    assert model['movable_joint_names'] == ['joint1', 'joint2']
    assert [joint['name'] for joint in model['chain']] == [
        'joint1', 'joint2', 'tool_fixed'
    ]
    assert model['chain'][0]['origin_xyz'] == [0.0, 0.0, 0.2]
    assert model['chain'][0]['axis'] == [0.0, 1.0, 0.0]
    assert model['chain'][0]['limit_lower'] == -1.0
    assert model['chain'][1]['limit_lower'] == pytest.approx(-math.pi)
    assert model['chain'][2]['type'] == 'fixed'


def test_parse_urdf_chain_rejects_missing_chain():
    with pytest.raises(ValueError, match='URDF chain not found'):
        MODULE.parse_urdf_chain(
            '<robot name="test"><link name="base"/><link name="tip"/></robot>',
            'base',
            'tip',
        )


def test_waypoint_stream_keeps_both_recorded_arm_topics_and_returns_teleop():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.completion_timeout_s = 0.1
    bridge.completion_tolerance_rad = 0.05
    bridge.max_start_velocity_radps = 0.02
    bridge.max_schedule_lag_s = 0.05
    bridge.max_joint_step_rad = 0.05
    bridge.control_rate_hz = 100.0
    bridge.execution_stop = MODULE.threading.Event()
    bridge.execution_lock = MODULE.threading.Lock()
    bridge.lock = MODULE.threading.RLock()
    bridge.all_current_velocities = {
        name: 0.0 for name in MODULE.arm_joint_names('left')
    }
    bridge.execution_arm = 'left'
    bridge.execution_thread = None
    bridge.arbiter_mode = 'WAYPOINT'
    bridge.arbiter_fault = ''
    published = []
    calls = []
    statuses = []
    current = {
        'left': [0.02] * 7 + [0.4],
        'right': [0.1] * 7 + [0.4],
    }
    bridge._publish_waypoint_pair = lambda pair: published.append({
        arm: list(values) for arm, values in pair.items()
    })
    bridge._current_command_positions = lambda: current
    bridge._call_arbiter = lambda action: calls.append(action) or {
        'success': True, 'message': action
    }
    bridge._select_relative_teleop_route = (
        lambda: calls.append('select_relative_teleop')
    )
    bridge._set_status = lambda arm, status: statuses.append((arm, status))

    trajectory = JointTrajectory()
    trajectory.joint_names = MODULE.command_joint_names('left')
    for nanoseconds, value, gripper in (
        (1_000_000, 0.01, 0.35), (2_000_000, 0.02, 0.4)
    ):
        point = JointTrajectoryPoint()
        point.positions = [value] * 7 + [gripper]
        point.time_from_start.nanosec = nanoseconds
        trajectory.points.append(point)
    hold = {
        'left': [0.0] * 7 + [0.3],
        'right': [0.1] * 7 + [0.4],
    }

    MODULE.WaypointUiBridge._run_waypoint_trajectory(
        bridge, 'left', trajectory, hold, 'TELEOP'
    )

    assert len(published) == 2
    assert all(pair['right'] == hold['right'] for pair in published)
    assert published[0]['left'][-1] == pytest.approx(0.35)
    assert published[-1]['left'] == [0.02] * 7 + [0.4]
    assert calls == ['select_relative_teleop']
    assert statuses == [('left', 'EXECUTION_COMPLETE: teleop handoff ready')]


@pytest.mark.parametrize('publish_delay', [0.0, 0.025])
def test_dense_stream_preserves_samples_without_catch_up_bursts(monkeypatch, publish_delay):
    clock = [0.0]
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: clock[0])

    class FakeStop:
        def is_set(self):
            return False

        def wait(self, seconds):
            clock[0] += seconds
            return False

    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.control_rate_hz = 100.0
    bridge.max_schedule_lag_s = 0.05
    bridge.max_joint_step_rad = 0.05
    bridge.execution_stop = FakeStop()
    bridge.lock = MODULE.threading.RLock()
    bridge.arbiter_mode = 'WAYPOINT'
    bridge.arbiter_fault = ''
    sent = []

    def publish(pair):
        sent.append((clock[0], list(pair['right'])))
        clock[0] += publish_delay

    bridge._publish_waypoint_pair = publish
    trajectory = JointTrajectory()
    for i in range(500):
        point = JointTrajectoryPoint()
        point.positions = [i * 0.0001] * 8
        point.time_from_start.sec = (i + 1) // 100
        point.time_from_start.nanosec = ((i + 1) % 100) * 10_000_000
        trajectory.points.append(point)
    hold = {'left': [0.1] * 8, 'right': [0.0] * 8}
    result = bridge._stream_trajectory('right', trajectory, hold)
    assert [values for _, values in sent] == [list(p.positions) for p in trajectory.points]
    assert all(b[0] - a[0] >= .01 + publish_delay - 1e-9 for a, b in zip(sent, sent[1:]))
    assert result['right'] == list(trajectory.points[-1].positions)
    assert result['left'] == hold['left']


@pytest.mark.parametrize('failure', ['stop', 'mode', 'fault', 'lag'])
def test_dense_stream_keeps_stop_ownership_and_large_delay_checks(monkeypatch, failure):
    clock = [0.0]
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: clock[0])

    class FakeStop:
        def is_set(self):
            return failure == 'stop'

        def wait(self, seconds):
            clock[0] += seconds + (0.06 if failure == 'lag' else 0.0)
            return False

    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.control_rate_hz = 100.0
    bridge.max_schedule_lag_s = 0.05
    bridge.max_joint_step_rad = 0.05
    bridge.execution_stop = FakeStop()
    bridge.lock = MODULE.threading.RLock()
    bridge.arbiter_mode = 'NONE' if failure == 'mode' else 'WAYPOINT'
    bridge.arbiter_fault = 'stale' if failure == 'fault' else ''
    sent = []
    bridge._publish_waypoint_pair = lambda pair: sent.append(pair)
    trajectory = JointTrajectory()
    point = JointTrajectoryPoint()
    point.positions = [0.0] * 8
    point.time_from_start.nanosec = 10_000_000
    trajectory.points = [point]
    with pytest.raises(InterruptedError if failure == 'stop' else RuntimeError):
        bridge._stream_trajectory('right', trajectory, {'left': [0.0] * 8, 'right': [0.0] * 8})
    assert not sent


def test_execution_failure_history_survives_followup_status_and_keeps_progress():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = MODULE.threading.RLock()
    bridge.arm_states = {'left': {'status_sequence': 0}}
    bridge.arbiter_mode = 'NONE'
    bridge.arbiter_fault = 'source timeout'
    messages = []

    class Logger:
        error = warning = info = lambda self, message: messages.append(message)

    bridge.get_logger = lambda: Logger()
    bridge._record_stream_progress('left', 'trajectory', 700, 2073, 6.99)
    bridge._set_status('left', 'ERROR: WAYPOINT control lost')
    bridge._set_status('left', 'PREVIEW_READY')
    bridge._set_status('left', 'EXECUTION_COMPLETE: teleop handoff ready')
    events = bridge.arm_states['left']['execution_events']
    assert len(events) == 2
    assert events[0]['status'] == 'ERROR: WAYPOINT control lost'
    assert events[0]['progress']['sample_index'] == 700
    assert events[0]['arbiter_fault'] == 'source timeout'
    assert 'WAYPOINT_EXECUTION left:' in messages[0]
    bridge._record_stream_progress('left', 'trajectory', 1, 10, 0.0)
    assert events[0]['progress']['sample_index'] == 700
    for _ in range(25):
        bridge._set_status('left', 'STOPPED: requested')
    assert len(bridge.arm_states['left']['execution_events']) == 20


def test_completion_ignores_gripper_position_error():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.completion_timeout_s = 0.02
    bridge.completion_tolerance_rad = 0.05
    bridge.max_start_velocity_radps = 0.02
    bridge.execution_stop = MODULE.threading.Event()
    bridge.lock = MODULE.threading.RLock()
    bridge.all_current_velocities = {
        name: 0.0 for name in MODULE.arm_joint_names('right')
    }
    bridge._current_command_positions = lambda: {
        'right': [0.04] * 7 + [0.95],
    }
    bridge._publish_waypoint_pair = lambda _pair: None
    target = {
        'left': [0.0] * 8,
        'right': [0.0] * 8,
    }

    MODULE.WaypointUiBridge._wait_for_arm_target(bridge, 'right', target)


def test_completion_error_reports_only_arm_error():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.completion_timeout_s = 0.001
    bridge.completion_tolerance_rad = 0.05
    bridge.max_start_velocity_radps = 0.02
    bridge.execution_stop = MODULE.threading.Event()
    bridge.lock = MODULE.threading.RLock()
    bridge.all_current_velocities = {
        name: 0.0 for name in MODULE.arm_joint_names('right')
    }
    bridge._current_command_positions = lambda: {
        'right': [0.0, 0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.11],
    }
    bridge._publish_waypoint_pair = lambda _pair: None
    target = {
        'left': [0.0] * 8,
        'right': [0.0] * 8,
    }

    with pytest.raises(
        RuntimeError,
        match=r'arm_r_joint5=0\.0600 rad \(팔 허용 0\.0500\)',
    ):
        MODULE.WaypointUiBridge._wait_for_arm_target(bridge, 'right', target)


def test_sequence_completion_defaults_to_teleop_handoff_from_none():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.execution_lock = MODULE.threading.Lock()
    bridge.execution_arm = 'sequence'
    statuses = []
    completion_modes = []
    bridge._set_status = lambda arm, status: statuses.append((arm, status))
    bridge._hold_waypoint_lease = lambda _hold, stop: stop.wait(1.0)
    bridge._select_start_waypoint = lambda _arm, _index: None
    bridge._select_end_waypoint = lambda _arm, _index: None
    bridge._fresh_plan = lambda _arm: object()
    bridge._validate_plan = lambda _arm, _trajectory, _start: None
    bridge._stream_trajectory = lambda _arm, _trajectory, pair: pair
    bridge._wait_for_arm_target = lambda _arm, _pair: None
    bridge._complete_execution = lambda mode: completion_modes.append(mode)
    hold = {
        'left': [0.0] * 8,
        'right': [0.0] * 8,
    }
    steps = [
        {'arm': 'left', 'start_index': 1, 'end_index': 2},
        {'arm': 'right', 'start_index': 1, 'end_index': 2},
    ]

    MODULE.WaypointUiBridge._run_execution_sequence(bridge, steps, hold)

    assert completion_modes == ['TELEOP']
    assert bridge.execution_arm is None
    assert ('left', 'EXECUTION_SEQUENCE_COMPLETE: teleop handoff ready') in statuses
    assert ('right', 'EXECUTION_SEQUENCE_COMPLETE: teleop handoff ready') in statuses


def _action_bridge(*, publish_enabled=True, execution_active=False):
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.available_arms = ['left', 'right']
    bridge.default_arm = 'left'
    bridge.publish_enabled = publish_enabled
    bridge.execution_stop = MODULE.threading.Event()
    bridge.lock = MODULE.threading.RLock()
    bridge.keyboard_condition = MODULE.threading.Condition(bridge.lock)
    bridge.keyboard_active = False
    bridge.keyboard_hold = None
    bridge.keyboard_last_heartbeat = 0.0
    bridge.teleop_ready_timeout_s = 0.1
    bridge.intervention_controller_url = ''
    bridge._execution_is_active = lambda: execution_active
    bridge.calls = []
    bridge.statuses = []
    bridge.get_logger = lambda: type('Logger', (), {
        'info': lambda self, message: None,
        'error': lambda self, message: None,
    })()
    def call_arbiter(action):
        bridge.calls.append(action)
        if action == 'get_status':
            return {
                'success': True,
                'message': MODULE.json.dumps({
                    'mode': 'NONE',
                    'fault': '',
                    'sources': {
                        'teleop': {'ready': True, 'reason': 'ready'},
                    },
                }),
            }
        return {'success': True, 'message': action}

    bridge._call_arbiter = call_arbiter
    bridge._call_planner_action = lambda arm, action: {
        'success': True, 'message': f'{arm}:{action}',
    }
    bridge._set_status = lambda arm, status: bridge.statuses.append((arm, status))
    return bridge


def test_keyboard_handoff_keeps_final_action_grippers_instead_of_state():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = MODULE.threading.RLock()
    state = {
        'left': [0.0] * 7 + [0.10],
        'right': [0.0] * 7 + [0.20],
    }
    bridge.previous_action_positions = {
        'final': {
            'left': [0.0] * 7 + [0.80],
            'right': [0.0] * 7 + [0.90],
        },
        'teleop': {
            'left': [0.0] * 7 + [0.60],
            'right': [0.0] * 7 + [0.70],
        },
    }
    bridge._current_command_positions = lambda: {
        arm: list(values) for arm, values in state.items()
    }

    positions, actions = bridge._keyboard_handoff_positions()

    assert positions['left'] == [0.0] * 7 + [0.80]
    assert positions['right'] == [0.0] * 7 + [0.90]
    assert actions == {'left': 0.80, 'right': 0.90}


def test_keyboard_handoff_rejects_missing_previous_gripper_action():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = MODULE.threading.RLock()
    bridge.joint_state_stamp = 0.0
    bridge.all_current_positions = {}
    bridge.arm_states = {
        arm: {'start_waypoint': 1, 'waypoint_grippers': []}
        for arm in MODULE.ARMS
    }
    bridge.previous_action_positions = {
        'final': {'left': None, 'right': None},
        'teleop': {'left': None, 'right': [0.0] * 8},
    }

    with pytest.raises(RuntimeError, match='left 그리퍼의 이전 action과 fresh joint state'):
        bridge._keyboard_handoff_positions()


def test_keyboard_handoff_initializes_missing_actions_from_fresh_feedback():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = MODULE.threading.RLock()
    bridge.joint_state_stamp = MODULE.time.monotonic()
    bridge.all_current_positions = {
        'gripper_l_joint1': 0.101,
        'gripper_r_joint1': 0.199,
    }
    bridge.arm_states = {
        'left': {'start_waypoint': 1, 'waypoint_grippers': [0.10]},
        'right': {'start_waypoint': 2, 'waypoint_grippers': [0.30, 0.20]},
    }
    bridge.previous_action_positions = {
        'final': {'left': None, 'right': None},
        'teleop': {'left': None, 'right': None},
    }
    bridge._current_command_positions = lambda: {
        'left': [0.0] * 7 + [0.101],
        'right': [0.0] * 7 + [0.199],
    }

    positions, actions = bridge._keyboard_handoff_positions()

    assert actions == {'left': 0.101, 'right': 0.199}
    assert positions['left'][-1] == pytest.approx(0.101)
    assert positions['right'][-1] == pytest.approx(0.199)


def test_single_arm_handoff_holds_inactive_gripper_without_waypoints():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = MODULE.threading.RLock()
    bridge.joint_state_stamp = MODULE.time.monotonic()
    bridge.all_current_positions = {
        'gripper_l_joint1': 0.12,
        'gripper_r_joint1': 0.20,
    }
    bridge.arm_states = {
        'left': {'start_waypoint': 1, 'waypoint_grippers': []},
        'right': {'start_waypoint': 1, 'waypoint_grippers': [0.20]},
    }
    bridge.previous_action_positions = {
        'final': {'left': None, 'right': None},
        'teleop': {'left': None, 'right': None},
    }
    bridge._current_command_positions = lambda: {
        'left': [0.0] * 7 + [0.12],
        'right': [0.0] * 7 + [0.20],
    }

    positions, actions = bridge._keyboard_handoff_positions(
        active_arms={'right'})

    assert actions == {'left': 0.12, 'right': 0.20}
    assert positions['left'][-1] == pytest.approx(0.12)
    assert positions['right'][-1] == pytest.approx(0.20)


def test_keyboard_handoff_uses_active_start_waypoint_gripper():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = MODULE.threading.RLock()
    bridge.joint_state_stamp = MODULE.time.monotonic()
    bridge.all_current_positions = {
        'gripper_l_joint1': 0.15,
        'gripper_r_joint1': 0.20,
    }
    bridge.arm_states = {
        'left': {'start_waypoint': 1, 'waypoint_grippers': [0.10]},
        'right': {'start_waypoint': 1, 'waypoint_grippers': [0.20]},
    }
    bridge.previous_action_positions = {
        'final': {'left': [0.0] * 7 + [0.80], 'right': None},
        'teleop': {'left': None, 'right': None},
    }

    bridge._current_command_positions = lambda: {
        'left': [0.0] * 7 + [0.15],
        'right': [0.0] * 7 + [0.20],
    }

    positions, actions = bridge._keyboard_handoff_positions(
        active_arms={'left'})

    assert actions == {'left': 0.10, 'right': 0.20}
    assert positions['left'][-1] == pytest.approx(0.10)
    assert positions['right'][-1] == pytest.approx(0.20)


def test_keyboard_handoff_uses_explicit_sequence_start_waypoint_gripper():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = MODULE.threading.RLock()
    bridge.joint_state_stamp = MODULE.time.monotonic()
    bridge.all_current_positions = {
        'gripper_l_joint1': 0.15,
        'gripper_r_joint1': 0.25,
    }
    bridge.arm_states = {
        'left': {'start_waypoint': 1, 'waypoint_grippers': [0.10, 0.20]},
        'right': {'start_waypoint': 1, 'waypoint_grippers': [0.30, 0.40]},
    }
    bridge.previous_action_positions = {
        'final': {'left': None, 'right': [0.0] * 7 + [0.90]},
        'teleop': {'left': None, 'right': None},
    }
    bridge._current_command_positions = lambda: {
        'left': [0.0] * 7 + [0.15],
        'right': [0.0] * 7 + [0.25],
    }

    positions, actions = bridge._keyboard_handoff_positions(
        active_arms={'left'}, active_start_waypoints={'left': 2})

    assert actions == {'left': 0.20, 'right': 0.90}
    assert positions['left'][-1] == pytest.approx(0.20)
    assert positions['right'][-1] == pytest.approx(0.90)


def test_keyboard_handoff_rejects_missing_active_start_gripper():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.lock = MODULE.threading.RLock()
    bridge.arm_states = {
        'left': {'start_waypoint': 2, 'waypoint_grippers': [0.10]},
        'right': {'start_waypoint': 1, 'waypoint_grippers': [0.20]},
    }
    bridge.previous_action_positions = {
        'final': {'left': None, 'right': [0.0] * 7 + [0.20]},
        'teleop': {'left': None, 'right': None},
    }

    with pytest.raises(RuntimeError, match='시작 waypoint 그리퍼 값이 없습니다'):
        bridge._keyboard_handoff_positions(active_arms={'left'})


def test_select_teleop_requires_confirmation_and_uses_arbiter():
    bridge = _action_bridge()
    with pytest.raises(ValueError, match='텔레옵 시작 확인'):
        bridge.call_action('select_teleop', {'arm': 'left'})

    result = bridge.call_action(
        'select_teleop', {'arm': 'left', 'confirm': 'TELEOP'})
    assert result['success'] is True
    assert bridge.calls == [
        'get_status', 'prepare_intervention', 'get_status', 'select_teleop',
    ]


def test_select_teleop_is_disabled_in_preview_only_mode():
    bridge = _action_bridge(publish_enabled=False)
    result = bridge.call_action(
        'select_teleop', {'arm': 'left', 'confirm': 'TELEOP'})
    assert result['success'] is False
    assert bridge.calls == []


def test_select_teleop_relinquishes_keyboard_control_first():
    bridge = _action_bridge()
    bridge.keyboard_active = True
    bridge.keyboard_hold = {
        'left': [0.0] * 8,
        'right': [0.0] * 8,
    }
    result = bridge.call_action(
        'select_teleop', {'arm': 'right', 'confirm': 'TELEOP'})
    assert result['success'] is True
    assert bridge.keyboard_active is False
    assert bridge.keyboard_hold is None
    assert bridge.calls == [
        'get_status', 'prepare_intervention', 'get_status', 'select_teleop',
    ]


def test_prepared_relative_teleop_selects_immediately_without_restarting_service():
    bridge = _action_bridge()

    def call_arbiter(action):
        bridge.calls.append(action)
        if action == 'get_status':
            return {
                'success': True,
                'message': MODULE.json.dumps({
                    'mode': 'NONE',
                    'fault': '',
                    'intervention_enabled': True,
                    'sources': {
                        'teleop': {'ready': True, 'reason': 'ready'},
                    },
                }),
            }
        return {'success': True, 'message': action}

    bridge._call_arbiter = call_arbiter

    result = bridge.call_action(
        'select_teleop', {'arm': 'left', 'confirm': 'TELEOP'})

    assert result['success'] is True
    assert bridge.calls == ['get_status', 'select_teleop']


def test_prepare_teleop_connects_input_but_does_not_select_route():
    bridge = _action_bridge()

    with pytest.raises(ValueError, match='미리 연결 확인'):
        bridge.call_action('prepare_teleop', {'arm': 'left'})

    result = bridge.call_action('prepare_teleop', {
        'arm': 'left', 'confirm': 'PREPARE_TELEOP',
    })

    assert result['success'] is True
    assert bridge.calls == ['prepare_intervention', 'get_status']
    assert '제어권은 NONE' in result['message']
    assert bridge.statuses == [
        ('left', 'TELEOP_STANDBY_CONNECTING: Mini Leader 연결 중'),
        ('left', 'TELEOP_STANDBY_READY: relative input connected'),
    ]


def test_prepare_teleop_uses_controller_owned_standby():
    bridge = _action_bridge()
    bridge.intervention_controller_url = 'http://127.0.0.1:8779'
    controller_calls = []
    bridge._call_intervention_controller = lambda action: (
        controller_calls.append(action) or {'ok': True}
    )

    result = bridge.call_action('prepare_teleop', {
        'arm': 'left', 'confirm': 'PREPARE_TELEOP',
    })

    assert result['success'] is True
    assert controller_calls == ['standby']
    assert bridge.calls == ['get_status']


def test_select_teleop_uses_controller_owned_manual_session():
    bridge = _action_bridge()
    bridge.intervention_controller_url = 'http://127.0.0.1:8779'
    controller_calls = []
    selected = {'value': False}

    def call_controller(action):
        controller_calls.append(action)
        if action == 'teleop_start':
            selected['value'] = True
        return {'ok': True}

    def call_arbiter(action):
        bridge.calls.append(action)
        assert action == 'get_status'
        return {
            'success': True,
            'message': MODULE.json.dumps({
                'mode': 'TELEOP' if selected['value'] else 'NONE',
                'fault': '',
                'intervention_enabled': selected['value'],
                'sources': {
                    'teleop': {
                        'ready': selected['value'],
                        'reason': 'ready' if selected['value'] else 'not received',
                    },
                },
            }),
        }

    bridge._call_intervention_controller = call_controller
    bridge._call_arbiter = call_arbiter

    result = bridge.call_action(
        'select_teleop', {'arm': 'left', 'confirm': 'TELEOP'})

    assert result['success'] is True
    assert controller_calls == ['standby', 'teleop_start']
    assert bridge.calls == ['get_status', 'get_status']


def test_select_teleop_waits_for_relative_input_and_fails_closed():
    bridge = _action_bridge()
    bridge.teleop_ready_timeout_s = 0.01

    def call_arbiter(action):
        bridge.calls.append(action)
        if action == 'get_status':
            return {
                'success': True,
                'message': MODULE.json.dumps({
                    'mode': 'NONE',
                    'fault': '',
                    'sources': {
                        'teleop': {
                            'ready': False,
                            'reason': 'left/teleop: not received',
                        },
                    },
                }),
            }
        return {'success': True, 'message': action}

    bridge._call_arbiter = call_arbiter

    with pytest.raises(RuntimeError, match='상대 TELEOP 입력 준비 시간이 초과'):
        bridge.call_action(
            'select_teleop', {'arm': 'left', 'confirm': 'TELEOP'})

    assert bridge.calls[0:3] == [
        'get_status', 'prepare_intervention', 'get_status',
    ]
    assert bridge.calls[-1] == 'select_none'
    assert 'select_teleop' not in bridge.calls


@pytest.mark.parametrize('execution_active', [False, True])
def test_confirmed_stop_always_selects_none(execution_active):
    bridge = _action_bridge(execution_active=execution_active)
    result = bridge.call_action('stop', {
        'arm': 'right',
        'confirm': MODULE.STOP_BUTTON_CONFIRMATION,
    })
    assert result['success'] is True
    assert bridge.calls == ['select_none']
    assert bridge.execution_stop.is_set()
    assert bridge.statuses == [(
        'right', 'STOP_REQUESTED: all arm command sources disabled')]


@pytest.mark.parametrize('stop_phase', ['stream_startup', 'initial_plan'])
def test_stop_received_during_preparation_prevents_execution_and_teleop_restore(stop_phase):
    bridge = _action_bridge()
    bridge.execution_lock = threading.Lock()
    bridge.execution_thread = None
    bridge._keyboard_is_active = lambda: False
    bridge._arbiter_mode = lambda: 'TELEOP'
    hold = {'left': [0.0] * 8, 'right': [0.0] * 8}
    bridge._keyboard_handoff_positions = lambda **kwargs: (hold, {})
    bridge._publish_waypoint_pair = lambda pair: None
    def open_stream(pair):
        bridge.calls.append('open_stream')
        if stop_phase == 'stream_startup':
            bridge.call_action('stop', {'confirm': MODULE.STOP_BUTTON_CONFIRMATION})

    bridge._open_stream_session = open_stream
    bridge._close_stream_session = lambda: bridge.calls.append('close_stream')
    bridge._wait_until_stationary = lambda actions: hold
    bridge._hold_waypoint_lease = lambda pair, stop: stop.wait(1.0)
    bridge._validate_plan = lambda *args: None

    def plan_then_stop(arm, slow_start):
        assert stop_phase == 'initial_plan'
        bridge.call_action('stop', {'confirm': MODULE.STOP_BUTTON_CONFIRMATION})
        return object()

    bridge._fresh_plan = plan_then_stop
    with pytest.raises(InterruptedError, match='정지가 요청되어 실행을 취소'):
        bridge._start_waypoint_execution('left')
    assert bridge.execution_thread is None
    assert bridge.execution_stop.is_set()
    expected = ['select_none', 'open_stream']
    if stop_phase == 'initial_plan':
        expected.append('select_waypoint')
    assert bridge.calls == expected + ['select_none', 'select_none', 'close_stream']


@pytest.mark.parametrize('stop_phase', ['stream_startup', 'stationary_wait'])
def test_stop_during_sequence_preparation_prevents_execution_and_teleop_restore(stop_phase):
    bridge = _action_bridge()
    bridge.execution_lock = threading.Lock()
    bridge.execution_thread = None
    bridge.execution_sequence = [{'arm': 'left', 'start_index': 1, 'end_index': 2}]
    bridge.arm_states = {'left': {'waypoint_count': 2}}
    bridge._keyboard_is_active = lambda: False
    bridge._arbiter_mode = lambda: 'TELEOP'
    hold = {'left': [0.0] * 8, 'right': [0.0] * 8}
    bridge._keyboard_handoff_positions = lambda **kwargs: (hold, {})
    bridge._publish_waypoint_pair = lambda pair: None
    bridge._close_stream_session = lambda: bridge.calls.append('close_stream')

    def stop():
        bridge.call_action('stop', {'confirm': MODULE.STOP_BUTTON_CONFIRMATION})

    def open_stream(pair):
        bridge.calls.append('open_stream')
        if stop_phase == 'stream_startup':
            stop()

    def wait_stationary(actions):
        assert stop_phase == 'stationary_wait'
        stop()
        return hold

    bridge._open_stream_session = open_stream
    bridge._wait_until_stationary = wait_stationary
    with pytest.raises(InterruptedError, match='정지가 요청되어 실행을 취소'):
        bridge._start_execution_steps({'confirm': 'EXECUTE'}, None)
    assert bridge.execution_thread is None
    assert bridge.execution_stop.is_set()
    expected = ['select_none', 'open_stream']
    if stop_phase == 'stationary_wait':
        expected.append('select_waypoint')
    assert bridge.calls == expected + ['select_none', 'select_none', 'close_stream']


@pytest.mark.parametrize('confirmation', [None, '', 'STOP', 'EXECUTE'])
def test_stop_rejects_requests_not_confirmed_by_ui_button(confirmation):
    bridge = _action_bridge(execution_active=True)
    payload = {'arm': 'right'}
    if confirmation is not None:
        payload['confirm'] = confirmation

    with pytest.raises(ValueError, match='UI의 정지 요청 버튼'):
        bridge.call_action('stop', payload)

    assert bridge.calls == []
    assert bridge.execution_stop.is_set() is False
    assert bridge.statuses == []


def test_web_stop_requires_a_trusted_pointer_click_and_confirmation():
    app = (SCRIPT.parents[1] / 'web' / 'app.js').read_text()

    assert "stopButton.addEventListener('pointerdown'" in app
    assert 'event.isTrusted' in app
    assert 'event.detail > 0' in app
    assert "runAction('stop', { confirm: STOP_BUTTON_CONFIRMATION })" in app


def test_web_periodic_refresh_uses_lightweight_runtime_endpoint():
    app = (SCRIPT.parents[1] / 'web' / 'app.js').read_text()

    assert 'async function refreshRuntime(force = false)' in app
    assert 'request(`/api/runtime?arm=${arm}`)' in app
    assert 'window.setInterval(() => { refreshRuntime()' in app
    assert 'window.setInterval(refresh, 350)' not in app


def test_web_loads_robot_meshes_through_the_ui_origin():
    root = SCRIPT.parents[1]
    app = (root / 'web' / 'app.js').read_text()
    generator = (root / 'web' / 'generator.js').read_text()
    server = SCRIPT.read_text()

    assert 'const assetOrigin = window.location.origin;' in app
    assert 'const assetOrigin = window.location.origin;' in generator
    assert ':7080' not in app
    assert ':7080' not in generator
    assert "request.path.startswith('/urdf/')" in server
    assert "'robot_asset_roots': robot_asset_roots" in server


def test_generator_automatically_validates_full_teleop_path_before_download():
    root = SCRIPT.parents[1]
    generator = (root / 'web' / 'generator.js').read_text()

    assert "request('/api/waypoints/start'" in generator
    assert "request('/api/waypoints/end'" in generator
    assert "request('/api/actions/preview'" in generator
    assert 'validateTeleopPreview(runtime, waypointFile)' in generator
    assert "state.generated?.generator?.validation?.passed === true" in generator
    assert "collision_checked: false" in generator


def test_web_uses_independent_speed_for_each_keyboard_direction():
    root = SCRIPT.parents[1]
    html = (root / 'web' / 'index.html').read_text()
    app = (root / 'web' / 'app.js').read_text()

    for direction in ('front', 'back', 'left', 'right', 'up', 'down'):
        assert f'class="keyboard-speed" data-direction="{direction}"' in html
    assert html.count('class="keyboard-speed"') == 6
    assert '`.keyboard-speed[data-direction="${direction}"]`' in app
    assert 'speed_mmps: speed' in app


def test_web_sends_radian_list_without_degree_round_trip():
    app = (SCRIPT.parents[1] / 'web' / 'app.js').read_text()

    assert 'positions_rad: positions' in app
    assert 'positions_deg: positions.map((value) => value * 180 / Math.PI)' not in app


def test_web_exposes_relative_teleop_preconnect_button():
    root = SCRIPT.parents[1]
    html = (root / 'web' / 'index.html').read_text()
    app = (root / 'web' / 'app.js').read_text()

    assert 'id="prepareTeleop"' in html
    assert '텔레옵 미리 연결' in html
    assert "runAction('prepare_teleop', { confirm: 'PREPARE_TELEOP' })" in app
    assert "? '텔레옵 연결됨'" in app


def test_simulator_exposes_fixed_world_frame_and_applies_root_tf():
    root = SCRIPT.parents[1]
    html = (root / 'web' / 'index.html').read_text()
    app = (root / 'web' / 'app.js').read_text()
    simulator = (root / 'web' / 'full_robot_simulator.source.js').read_text()

    assert '원점 큰 축: WORLD FIXED' in html
    assert 'setWorldTransform(' in app
    assert "this.worldAxes = new THREE.AxesHelper(0.5)" in simulator
    assert "textSprite(`WORLD FIXED (${frameName || 'world'})`)" in simulator
    assert 'this.modelGroup.position.set' in simulator
    assert 'this.modelGroup.quaternion.set' in simulator


def test_simulator_exposes_synchronized_six_dof_pose_editor():
    root = SCRIPT.parents[1]
    html = (root / 'web' / 'index.html').read_text()
    app = (root / 'web' / 'app.js').read_text()
    simulator = (root / 'web' / 'full_robot_simulator.source.js').read_text()

    for field in ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'):
        assert f'id="poseEditor{field}"' in html
    assert 'data-pose-mode="translate"' in html
    assert 'data-pose-mode="rotate"' in html
    assert 'id="poseAddWaypoint"' in html
    assert "request('/api/waypoints/absolute'" in app
    assert "target_mode: 'pose'" in app
    assert 'function handlePoseGizmoChange(pose)' in app
    assert "import { TransformControls }" in simulator
    assert "this.transformControls.setMode(this.poseEditorMode)" in simulator
    assert "this.transformControls.addEventListener('objectChange'" in simulator


def test_runtime_snapshot_omits_large_plan_payloads():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.default_arm = 'right'
    bridge.available_arms = ['right']
    bridge.lock = MODULE.threading.RLock()
    bridge.arm_configs = {
        'right': {'joint_names': MODULE.command_joint_names('right')},
    }
    bridge.arm_states = {
        'right': {
            'current_positions': {
                name: 0.1 for name in MODULE.command_joint_names('right')
            },
            'current_stamp': MODULE.time.monotonic(),
            'status': 'EXECUTING: WAYPOINT 10.000 s',
            'plan_revision': 42,
            'waypoint_names': ['large plan must stay out of runtime'],
            'waypoints': [{'index': index} for index in range(590)],
            'trajectory': object(),
        },
    }
    bridge.all_current_positions = {'arm_r_joint1': 0.1}
    bridge.execution_sequence = []
    bridge.execution_thread = None
    bridge.execution_arm = None
    bridge.keyboard_active = False
    bridge.world_frame = 'world'
    bridge.keyboard_jog_period_s = 0.05
    bridge.keyboard_jog_max_speed_mmps = 50.0
    bridge.keyboard_deadman_timeout_s = 1.0
    bridge.arbiter_mode = 'NONE'
    bridge.arbiter_fault = ''
    bridge.arbiter_intervention_enabled = True
    bridge.arbiter_teleop_ready = True
    bridge.arbiter_teleop_reason = 'ready'

    snapshot = bridge.runtime_snapshot('right')

    assert snapshot['plan_revision'] == 42
    assert snapshot['arbiter']['intervention_enabled'] is True
    assert snapshot['arbiter']['teleop_ready'] is True
    assert snapshot['joint_state_fresh'] is True
    assert snapshot['current_deg'] == pytest.approx([math.degrees(0.1)] * 8, abs=0.001)
    assert 'waypoints' not in snapshot
    assert 'trajectory' not in snapshot


def test_full_snapshot_formats_trajectory_without_holding_command_lock():
    bridge = object.__new__(MODULE.WaypointUiBridge)
    bridge.default_arm = 'left'
    bridge.available_arms = ['left']
    bridge.lock = MODULE.threading.RLock()
    bridge.arm_configs = {'left': {}}
    bridge.arm_states = {'left': {'current_stamp': 0.0, 'trajectory': None}}
    bridge.all_current_positions = {}
    bridge.execution_sequence = []
    formatting = MODULE.threading.Event()
    release = MODULE.threading.Event()
    errors = []

    class FormattingComplete(Exception):
        pass

    def format_trajectory(_message):
        formatting.set()
        release.wait(timeout=2.0)
        raise FormattingComplete()

    bridge._trajectory_snapshot = format_trajectory

    def snapshot_request():
        try:
            bridge.snapshot('left')
        except FormattingComplete:
            pass
        except Exception as exception:
            errors.append(exception)

    request = MODULE.threading.Thread(target=snapshot_request)
    request.start()
    try:
        assert formatting.wait(timeout=1.0)
        acquired = bridge.lock.acquire(timeout=0.02)
        if acquired:
            bridge.lock.release()
        assert acquired, 'UI formatting blocked the command stream lock'
    finally:
        release.set()
        request.join(timeout=2.0)
    assert not request.is_alive()
    assert errors == []
