"""File library tests; all planner operations are mocked."""
import copy
import importlib.util
from pathlib import Path
import threading

import pytest

SCRIPT = Path(__file__).parents[1] / 'scripts' / 'waypoint_telep_ui.py'
SPEC = importlib.util.spec_from_file_location('waypoint_file_ui', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def waypoint_file(arm='left', name='A'):
    return dict(schema='ai_worker_waypoints/v1', arm=arm,
                joint_smoothing_enabled=True, start_waypoint=1, end_waypoint=1,
                waypoints=[dict(name=name, joint_positions_rad=[0.0] * 8,
                                dwell_s=1.5, speed_scale=0.5,
                                interpolation_mode='JOINT', is_interpolated=False)])


@pytest.fixture
def bridge():
    b = object.__new__(MODULE.WaypointUiBridge)
    b.default_arm = 'left'
    b.available_arms = ['left', 'right']
    b.lock = threading.RLock()
    b.execution_lock = threading.RLock()
    b.waypoint_file_lock = threading.RLock()
    b.waypoint_files = {}
    b.active_waypoint_files = dict(left=None, right=None)
    b.next_waypoint_file_id = 1
    b.arm_states = {arm: dict(waypoint_count=0, plan_revision=0) for arm in b.available_arms}
    b.max_waypoint_dwell_s = 30
    b.max_waypoint_speed_scale = 3
    b.max_uniform_tcp_speed_mps = 1.8
    b._execution_is_active = lambda: False
    b._keyboard_is_active = lambda: False
    b.live = {arm: dict(waypoint_file(arm), waypoints=[]) for arm in b.available_arms}
    b.applied = []
    b.export_waypoints = lambda arm: copy.deepcopy(b.live[arm])

    def apply(payload, validated):
        b.applied.append(copy.deepcopy(payload))
        b.live[validated['arm']] = copy.deepcopy(payload)
        b.arm_states[validated['arm']]['waypoint_count'] = len(payload['waypoints'])
        return dict(success=True)

    b._apply_waypoint_file = apply
    return b


def register(b, *files):
    return b.add_waypoint_files(dict(files=[dict(name='same.json', waypoint_file=f) for f in files]))['ids']


def test_registration_preserves_current_plans_and_keeps_duplicate_names_independent(bridge):
    ids = register(bridge, waypoint_file(), waypoint_file(), waypoint_file('right'), waypoint_file('right'))
    assert len(set(ids)) == 4
    assert [f['id'] for f in bridge.waypoint_files_snapshot('left')] == ids[:2]
    assert [f['id'] for f in bridge.waypoint_files_snapshot('right')] == ids[2:]
    assert not any(f['active'] for f in bridge.waypoint_files_snapshot('left'))
    assert bridge.applied == []
    assert bridge.arm_states['left']['waypoint_count'] == 0


def test_batch_validation_is_atomic_and_preserves_source_objects(bridge):
    good = waypoint_file()
    original = copy.deepcopy(good)
    bad = waypoint_file('right')
    bad['waypoints'][0]['joint_positions_rad'] = [999] * 8
    with pytest.raises(ValueError, match='same.json'):
        register(bridge, good, bad)
    assert bridge.waypoint_files == {}
    assert bridge.applied == []
    assert good == original
    file_id, = register(bridge, good)
    good['waypoints'].clear()
    assert len(bridge.waypoint_files[file_id]['waypoint_file']['waypoints']) == 1


def test_switching_restores_edits_and_keeps_other_arm_active(bridge):
    a, b, r = register(bridge, waypoint_file(name='A'), waypoint_file(name='B'), waypoint_file('right'))
    bridge.activate_waypoint_file(dict(arm='left', id=a))
    bridge.activate_waypoint_file(dict(arm='right', id=r))
    bridge.live['left']['waypoints'][0].update(name='편집', dwell_s=2.5, speed_scale=0.8)
    bridge.activate_waypoint_file(dict(arm='left', id=b))
    assert bridge.active_waypoint_files == dict(left=b, right=r)
    assert bridge.live['left']['waypoints'][0]['name'] == 'B'
    bridge.activate_waypoint_file(dict(arm='left', id=a))
    assert bridge.live['left']['waypoints'][0]['name'] == '편집'
    assert bridge.live['left']['waypoints'][0]['dwell_s'] == 2.5
    assert bridge.live['left']['waypoints'][0]['speed_scale'] == 0.8
    count = len(bridge.applied)
    bridge.activate_waypoint_file(dict(arm='left', id=a))
    assert len(bridge.applied) == count, 'reactivating the current file must preserve edits'


def test_first_activation_preserves_unregistered_work(bridge):
    bridge.live['left'] = waypoint_file(name='기존 작업')
    bridge.arm_states['left']['waypoint_count'] = 1
    a, = register(bridge, waypoint_file())
    bridge.activate_waypoint_file(dict(arm='left', id=a))
    saved = next(f for f in bridge.waypoint_files_snapshot('left') if not f['active'])
    assert saved['name'] == '현재 작업 (자동 보관)'
    bridge.activate_waypoint_file(dict(arm='left', id=saved['id']))
    assert bridge.live['left']['waypoints'][0]['name'] == '기존 작업'


@pytest.mark.parametrize('guard', ['_execution_is_active', '_keyboard_is_active'])
def test_active_control_blocks_switching_and_direct_import(bridge, guard):
    a, = register(bridge, waypoint_file())
    setattr(bridge, guard, lambda: True)
    for operation, payload in ((bridge.activate_waypoint_file, dict(arm='left', id=a)),
                               (bridge.import_waypoints, waypoint_file())):
        with pytest.raises(RuntimeError):
            operation(payload)
    assert bridge.applied == []
    assert bridge.active_waypoint_files['left'] is None


def test_remove_rejects_active_or_wrong_arm_and_does_not_change_live_plan(bridge):
    a, b = register(bridge, waypoint_file(), waypoint_file())
    bridge.activate_waypoint_file(dict(arm='left', id=a))
    for payload in (dict(arm='left', id=a), dict(arm='right', id=b), dict(arm='left', id='missing')):
        with pytest.raises(ValueError):
            bridge.remove_waypoint_file(payload)
    with pytest.raises(ValueError):
        bridge.activate_waypoint_file(dict(arm='right', id=a))
    bridge.remove_waypoint_file(dict(arm='left', id=b))
    assert len(bridge.waypoint_files_snapshot('left')) == 1
    assert len(bridge.applied) == 1


def test_failed_activation_preserves_previous_file_without_marking_partial_plan_active(bridge):
    a, b = register(bridge, waypoint_file(), waypoint_file(name='B'))
    bridge.activate_waypoint_file(dict(arm='left', id=a))
    bridge.live['left']['waypoints'][0]['name'] = '복구할 편집'

    def fail(*_args):
        raise RuntimeError('planner timeout')

    bridge._apply_waypoint_file = fail
    with pytest.raises(RuntimeError, match='planner timeout'):
        bridge.activate_waypoint_file(dict(arm='left', id=b))
    assert bridge.active_waypoint_files['left'] is None
    assert bridge.waypoint_files[a]['waypoint_file']['waypoints'][0]['name'] == '복구할 편집'
    assert bridge.waypoint_files[b]['waypoint_file']['waypoints'][0]['name'] == 'B'


def test_import_migrates_legacy_false_flags_and_respects_new_explicit_provenance(bridge):
    payload = waypoint_file(name='왼팔 지점 1-보간1 (1→2)')
    payload['waypoints'][0]['is_interpolated'] = False
    assert bridge._validate_waypoint_file(payload)['flags'] == [True]
    payload['interpolated_provenance_version'] = 2
    assert bridge._validate_waypoint_file(payload)['flags'] == [False]


def test_refined_timing_profile_roundtrip_accepts_reference_modes_without_changing_geometry(bridge):
    payload = waypoint_file()
    payload['joint_smoothing_enabled'] = False
    payload['path_refinement'] = dict(enabled=True, smoothing_percent=60)
    payload['waypoints'][0]['interpolation_mode'] = 'LINEAR'
    payload['joint_timing'] = dict(speed_mps=.02, preserve_start_waypoint=0, preserve_end_waypoint=0)
    original = copy.deepcopy(payload)
    parsed = bridge._validate_waypoint_file(payload)
    assert parsed['refinement']['enabled']
    assert parsed['timing']['speed_mps'] == .02
    assert parsed['parsed'][0]['mode'] == 'LINEAR'
    assert payload == original
