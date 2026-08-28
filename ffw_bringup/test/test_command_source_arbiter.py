import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ffw_bringup.command_source_arbiter import (
    CommandSourceArbiter,
    CommandSourceArbiterCore,
    MODE_NONE,
    MODE_POLICY,
    MODE_TELEOP,
)


JOINTS = {"left": ["l1", "l2"], "right": ["r1", "r2"]}
LIMITS = {
    "left": [(-1.0, 1.0), (-0.5, 0.5)],
    "right": [(-1.0, 1.0), (-0.5, 0.5)],
}


def trajectory(names, values):
    msg = JointTrajectory()
    msg.joint_names = list(names)
    point = JointTrajectoryPoint()
    point.positions = list(values)
    msg.points = [point]
    return msg


def ready_pair(core, source, now=1.0):
    core.update(source, "left", trajectory(JOINTS["left"], [0.1, 0.2]), now)
    core.update(source, "right", trajectory(JOINTS["right"], [0.3, 0.4]), now + 0.01)


def test_starts_fail_closed_and_requires_fresh_arm_pair():
    core = CommandSourceArbiterCore(JOINTS)
    assert core.mode == MODE_NONE
    success, message = core.select(MODE_POLICY, 1.0)
    assert not success
    assert "not received" in message

    ready_pair(core, MODE_POLICY)
    success, _ = core.select(MODE_POLICY, 1.02)
    assert success
    assert core.mode == MODE_POLICY


def test_rejects_joint_order_and_non_finite_values():
    core = CommandSourceArbiterCore(JOINTS)
    assert not core.update(
        MODE_TELEOP, "left", trajectory(["l2", "l1"], [0.0, 0.0]), 1.0)
    assert not core.update(
        MODE_TELEOP, "right", trajectory(JOINTS["right"], [0.0, math.nan]), 1.0)
    ready, reason = core.readiness(MODE_TELEOP, 1.0)
    assert not ready
    assert "mismatch" in reason


def test_active_invalid_message_latches_none():
    core = CommandSourceArbiterCore(JOINTS)
    ready_pair(core, MODE_TELEOP)
    assert core.select(MODE_TELEOP, 1.02)[0]
    core.update(MODE_TELEOP, "left", trajectory(JOINTS["left"], [0.0]), 1.03)
    assert core.mode == MODE_NONE
    assert "position length mismatch" in core.fault


def test_position_limit_violation_rejects_whole_source_sample():
    core = CommandSourceArbiterCore(JOINTS, position_limits=LIMITS)
    assert not core.update(
        MODE_POLICY,
        "right",
        trajectory(JOINTS["right"], [0.0, 0.500001]),
        1.0,
    )
    sample = core.samples[MODE_POLICY]["right"]
    assert "position limit violation" in sample.error
    assert "r2=0.500001000" in sample.error


def test_active_position_limit_violation_latches_none():
    core = CommandSourceArbiterCore(JOINTS, position_limits=LIMITS)
    ready_pair(core, MODE_POLICY)
    assert core.select(MODE_POLICY, 1.02)[0]

    assert not core.update(
        MODE_POLICY,
        "left",
        trajectory(JOINTS["left"], [1.000001, 0.0]),
        1.03,
    )
    assert core.mode == MODE_NONE
    assert "position limit violation" in core.fault


def test_f2_right_j7_observed_ttrtc_output_is_rejected():
    core = CommandSourceArbiterCore(
        CommandSourceArbiter.DEFAULT_JOINTS,
        position_limits=CommandSourceArbiter.F2_POSITION_LIMITS,
    )
    values = [0.0] * len(CommandSourceArbiter.DEFAULT_JOINTS["right"])
    values[6] = 1.8201

    assert not core.update(
        MODE_POLICY,
        "right",
        trajectory(CommandSourceArbiter.DEFAULT_JOINTS["right"], values),
        1.0,
    )
    sample = core.samples[MODE_POLICY]["right"]
    assert "arm_r_joint7=1.820100000" in sample.error
    assert "1.570796327" in sample.error


def test_watchdog_latches_stale_source_and_requires_explicit_clear():
    core = CommandSourceArbiterCore(JOINTS, max_age_s=0.1)
    ready_pair(core, MODE_POLICY)
    assert core.select(MODE_POLICY, 1.02)[0]
    ok, _ = core.watchdog(1.25)
    assert not ok
    assert core.mode == MODE_NONE
    assert "lease lost" in core.fault
    ready_pair(core, MODE_POLICY, now=1.3)
    assert not core.select(MODE_POLICY, 1.32)[0]
    core.clear_fault()
    assert core.select(MODE_POLICY, 1.32)[0]


def test_pair_skew_is_rejected():
    core = CommandSourceArbiterCore(JOINTS, max_pair_skew_s=0.02)
    core.update(MODE_POLICY, "left", trajectory(JOINTS["left"], [0.0, 0.0]), 1.0)
    core.update(MODE_POLICY, "right", trajectory(JOINTS["right"], [0.0, 0.0]), 1.04)
    success, message = core.select(MODE_POLICY, 1.04)
    assert not success
    assert "skew" in message
