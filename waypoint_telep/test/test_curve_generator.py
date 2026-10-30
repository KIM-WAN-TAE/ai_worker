"""Geometric contracts independent of ROS, with dense samples between knots."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

SPEC = importlib.util.spec_from_file_location('curve_generator',
    Path(__file__).parents[1] / 'scripts' / 'waypoint_curve_generator.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize('strength', [0., .3, .6, 1.])
@pytest.mark.parametrize('angle', [0., 1.2, 3.14])
def test_curve_endpoints_bounds_and_straight_limit(strength, angle):
    a = np.array([.5, .02, -.14])
    b = np.array([.4948, .0228, -.167])
    c = b + np.array([0., 0., -.025])
    width = .003
    for u in np.linspace(0, 1, 257):
        actual = MODULE.curve_position(a, b, c, u, strength, width, angle)
        straight = a+u*(b-a)
        assert np.linalg.norm(actual-straight) <= strength*width + 1e-8
        if strength == 0:
            np.testing.assert_array_equal(actual, straight)
    np.testing.assert_allclose(MODULE.curve_position(a, b, c, 0, strength, width, angle), a, atol=1e-15)
    np.testing.assert_allclose(MODULE.curve_position(a, b, c, 1, strength, width, angle), b, atol=1e-15)


def test_rotations_use_shortest_arc_and_accept_equivalent_quaternions():
    q = [0., 0., 0., 1.]
    np.testing.assert_allclose(MODULE.slerp(q, [0., 0., 0., -1.], .5), q)
    np.testing.assert_allclose(MODULE.slerp(q, [0., 0., 1., 0.], .5),
                               [0., 0., 2**-.5, 2**-.5])
    with pytest.raises(ValueError):
        MODULE.quaternion([0, 0, 0, 0])


@pytest.mark.parametrize('segments', [[], 'bad', [None],
    [dict(start_waypoint=1, end_waypoint=1)],
    [dict(start_waypoint=True, end_waypoint=2)],
    [dict(start_waypoint=1, end_waypoint=4)],
    [dict(start_waypoint=1, end_waypoint=2, mode='wrong')],
    [dict(start_waypoint=1, end_waypoint=2, curve_percent=float('nan'))],
    [dict(start_waypoint=1, end_waypoint=2), dict(start_waypoint=3, end_waypoint=2)]])
def test_reject_invalid_or_disconnected_segments_before_ik(segments):
    config = dict(arm='right', payload=dict(segments=segments,
        waypoint_file=dict(schema='ai_worker_waypoints/v1', arm='right', waypoints=[{}, {}, {}])))
    with pytest.raises(ValueError):
        MODULE.generate(config)


@pytest.mark.parametrize('field,value,reason', [('dwell_s', 1., '대기'),
    ('joint_positions_rad', [0., 0., 0., 0., 0., 0., 0., .8], '그리퍼')])
def test_selected_segments_must_not_discard_internal_actions(field, value, reason):
    points = [dict(joint_positions_rad=[0.]*7+[.9], dwell_s=0.) for _ in range(3)]
    points[1][field] = value
    config = dict(arm='right', payload=dict(segments=[dict(start_waypoint=1,end_waypoint=3)],
        waypoint_file=dict(schema='ai_worker_waypoints/v1', arm='right', waypoints=points)))
    with pytest.raises(ValueError, match=reason):
        MODULE.generate(config)


@pytest.mark.parametrize('mode', ['position', 'orientation', 'both'])
def test_start_noise_sampling_is_bounded_reproducible_and_mode_specific(mode):
    settings = MODULE.parse_start_noise(dict(enabled=True, mode=mode, sampling='random',
        translation_min_mm=[1., -2., 0.], translation_max_mm=[3., 2., 0.],
        rotation_min_deg=[-3., 1., 0.], rotation_max_deg=[3., 2., 0.]))
    samples = [MODULE.sample_start_noise(settings, 42, i) for i in range(30)]
    assert samples == [MODULE.sample_start_noise(settings, 42, i) for i in range(30)]
    assert len({str(s) for s in samples}) == 30
    for sample in samples:
        for kind, unit, key in [('translation', 'mm', 'translation_mm'), ('rotation', 'deg', 'rotation_rpy_deg')]:
            assert all(lo <= v <= hi for lo, v, hi in zip(settings[f'{kind}_min_{unit}'], sample[key], settings[f'{kind}_max_{unit}']))
        if mode == 'position':
            assert sample['rotation_rpy_deg'] == [0.]*3
        if mode == 'orientation':
            assert sample['translation_mm'] == [0.]*3


def test_start_noise_translation_is_in_base_frame_and_rotation_is_local():
    a = dict(position_m=[.5, .02, -.14], orientation_xyzw=[0., 0., 2**-.5, 2**-.5])
    p, q = MODULE.noisy_start_pose(a, dict(translation_mm=[1., -2., 3.], rotation_rpy_deg=[10., 0., 0.]))
    np.testing.assert_allclose(p, [.501, .018, -.137], atol=1e-15)
    expected = MODULE.K.Rotation.RotZ(np.pi/2)*MODULE.K.Rotation.RotX(np.pi/18)
    assert MODULE.K.diff(MODULE.K.Rotation.Quaternion(*q), expected).Norm() < 1e-12
    p, q = MODULE.noisy_start_pose(a, dict(translation_mm=[0.]*3, rotation_rpy_deg=[0.]*3))
    np.testing.assert_allclose(p, a['position_m'], atol=1e-15)
    np.testing.assert_allclose(q, a['orientation_xyzw'], atol=1e-15)


@pytest.mark.parametrize('patch', [dict(enabled=1), dict(mode='other'), dict(sampling='other'),
    dict(translation_min_mm=[2.]*3, translation_max_mm=[1.]*3),
    dict(translation_min_mm=[float('nan')]*3), dict(translation_max_mm=[51.]*3),
    dict(mode='both', rotation_max_deg=[31.]*3), dict(translation_mm=[6.]*3),
    dict(translation_mm=[True, 0, 0]), dict(translation_min_mm=[0.])])
def test_invalid_start_noise_bounds_and_values_are_rejected(patch):
    with pytest.raises(ValueError):
        MODULE.parse_start_noise(dict(dict(enabled=True), **patch))
