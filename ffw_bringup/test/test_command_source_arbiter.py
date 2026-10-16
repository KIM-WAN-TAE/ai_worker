import math
from types import SimpleNamespace

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ffw_bringup.command_source_arbiter import (
    CommandSourceArbiter,
    CommandSourceArbiterCore,
    MODE_NONE,
    MODE_POLICY,
    MODE_TELEOP,
    MODE_WAYPOINT,
)


JOINTS = {"left": ["l1", "l2"], "right": ["r1", "r2"]}
LIMITS = {
    "left": [(-1.0, 1.0), (-0.5, 0.5)],
    "right": [(-1.0, 1.0), (-0.5, 0.5)],
}


def trajectory(names, values, duration_s=0.0):
    msg = JointTrajectory()
    msg.joint_names = list(names)
    point = JointTrajectoryPoint()
    point.positions = list(values)
    seconds = int(duration_s)
    point.time_from_start.sec = seconds
    point.time_from_start.nanosec = int(
        round((float(duration_s) - seconds) * 1_000_000_000)
    )
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


def test_waypoint_requires_a_fresh_pair_and_is_watchdog_guarded():
    core = CommandSourceArbiterCore(JOINTS, max_age_s=0.1)
    ready_pair(core, MODE_WAYPOINT)
    success, _ = core.select(MODE_WAYPOINT, 1.02)
    assert success
    assert core.mode == MODE_WAYPOINT
    ok, reason = core.watchdog(1.25)
    assert not ok
    assert core.mode == MODE_NONE
    assert 'waypoint lease lost' in reason


def test_selected_policy_uses_validated_trajectory_duration_as_lease():
    core = CommandSourceArbiterCore(
        JOINTS,
        max_age_s=0.1,
        policy_trajectory_completion_grace_s=0.1,
    )
    core.update(
        MODE_POLICY,
        "left",
        trajectory(JOINTS["left"], [0.1, 0.2], duration_s=5.0),
        1.0,
    )
    core.update(
        MODE_POLICY,
        "right",
        trajectory(JOINTS["right"], [0.3, 0.4], duration_s=5.0),
        1.01,
    )

    assert core.select(MODE_POLICY, 1.02)[0]
    assert core.watchdog(5.9)[0]
    ok, reason = core.watchdog(6.2)
    assert not ok
    assert "policy lease lost" in reason


def test_policy_trajectory_allows_bounded_post_completion_inference_gap():
    core = CommandSourceArbiterCore(
        JOINTS,
        max_age_s=0.1,
        command_lease_grace_s=0.1,
        policy_trajectory_completion_grace_s=1.0,
    )
    for arm, values in (("left", [0.1, 0.2]), ("right", [0.3, 0.4])):
        core.update(
            MODE_POLICY,
            arm,
            trajectory(JOINTS[arm], values, duration_s=5.0),
            1.0,
        )

    assert core.select(MODE_POLICY, 1.02)[0]
    assert core.watchdog(6.5)[0]
    ok, reason = core.watchdog(7.1)
    assert not ok
    assert "policy lease lost" in reason


def test_policy_streaming_command_keeps_short_watchdog_grace():
    core = CommandSourceArbiterCore(
        JOINTS,
        max_age_s=0.1,
        command_lease_grace_s=0.1,
        policy_trajectory_completion_grace_s=1.0,
    )
    ready_pair(core, MODE_POLICY)

    assert core.select(MODE_POLICY, 1.02)[0]
    ok, reason = core.watchdog(1.25)
    assert not ok
    assert "policy lease lost" in reason


def test_cached_trajectory_lease_begins_when_selection_forwards_it():
    core = CommandSourceArbiterCore(JOINTS, max_age_s=0.25)
    for arm, values in (("left", [0.1, 0.2]), ("right", [0.3, 0.4])):
        core.update(
            MODE_POLICY,
            arm,
            trajectory(JOINTS[arm], values, duration_s=1.0),
            1.0,
        )

    assert core.select(MODE_POLICY, 1.2)[0]
    assert core.watchdog(2.25)[0]


def test_long_trajectory_does_not_allow_stale_policy_selection():
    core = CommandSourceArbiterCore(JOINTS, max_age_s=0.1)
    for arm, values in (("left", [0.1, 0.2]), ("right", [0.3, 0.4])):
        core.update(
            MODE_POLICY,
            arm,
            trajectory(JOINTS[arm], values, duration_s=5.0),
            1.0,
        )

    success, reason = core.select(MODE_POLICY, 1.2)
    assert not success
    assert "stale" in reason


def test_new_streaming_command_replaces_prior_long_policy_lease():
    core = CommandSourceArbiterCore(JOINTS, max_age_s=0.1)
    for arm, values in (("left", [0.1, 0.2]), ("right", [0.3, 0.4])):
        core.update(
            MODE_POLICY,
            arm,
            trajectory(JOINTS[arm], values, duration_s=5.0),
            1.0,
        )
    assert core.select(MODE_POLICY, 1.01)[0]

    ready_pair(core, MODE_POLICY, now=2.0)
    ok, _ = core.watchdog(2.25)
    assert not ok


def test_trajectory_duration_above_lease_limit_is_rejected():
    core = CommandSourceArbiterCore(JOINTS, max_command_lease_s=60.0)
    valid = core.update(
        MODE_POLICY,
        "left",
        trajectory(JOINTS["left"], [0.1, 0.2], duration_s=60.1),
        1.0,
    )
    assert not valid
    assert "lease limit" in core.samples[MODE_POLICY]["left"].error


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


def test_f2_right_j7_nominal_endpoint_is_accepted():
    core = CommandSourceArbiterCore(
        CommandSourceArbiter.DEFAULT_JOINTS,
        position_limits=CommandSourceArbiter.F2_POSITION_LIMITS,
    )
    values = [0.0] * len(CommandSourceArbiter.DEFAULT_JOINTS["right"])
    values[6] = 1.8201

    assert core.update(
        MODE_POLICY,
        "right",
        trajectory(CommandSourceArbiter.DEFAULT_JOINTS["right"], values),
        1.0,
    )


def test_f2_right_j7_above_nominal_endpoint_is_rejected():
    core = CommandSourceArbiterCore(
        CommandSourceArbiter.DEFAULT_JOINTS,
        position_limits=CommandSourceArbiter.F2_POSITION_LIMITS,
    )
    values = [0.0] * len(CommandSourceArbiter.DEFAULT_JOINTS["right"])
    values[6] = 1.820101

    assert not core.update(
        MODE_POLICY,
        "right",
        trajectory(CommandSourceArbiter.DEFAULT_JOINTS["right"], values),
        1.0,
    )
    sample = core.samples[MODE_POLICY]["right"]
    assert "arm_r_joint7=1.820101000" in sample.error
    assert "1.820100000" in sample.error


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


def test_active_pair_skew_must_persist_before_faulting():
    core = CommandSourceArbiterCore(
        JOINTS, max_age_s=0.5, max_pair_skew_s=0.05)
    ready_pair(core, MODE_WAYPOINT)
    assert core.select(MODE_WAYPOINT, 1.02)[0]

    core.update(
        MODE_WAYPOINT, "left",
        trajectory(JOINTS["left"], [0.1, 0.2]), 1.10,
    )
    assert core.watchdog(1.10) == (True, "arm pair skew grace")
    assert core.watchdog(1.14) == (True, "arm pair skew grace")
    ok, reason = core.watchdog(1.16)
    assert not ok
    assert "arm pair skew" in reason
    assert core.mode == MODE_NONE


def test_active_pair_skew_grace_resets_when_pair_recovers():
    core = CommandSourceArbiterCore(
        JOINTS, max_age_s=0.5, max_pair_skew_s=0.05)
    ready_pair(core, MODE_WAYPOINT)
    assert core.select(MODE_WAYPOINT, 1.02)[0]

    core.update(
        MODE_WAYPOINT, "left",
        trajectory(JOINTS["left"], [0.1, 0.2]), 1.10,
    )
    assert core.watchdog(1.10)[0]
    core.update(
        MODE_WAYPOINT, "right",
        trajectory(JOINTS["right"], [0.3, 0.4]), 1.11,
    )
    assert core.watchdog(1.12) == (True, "ready")
    assert core._pair_skew_started_s is None


def test_prepare_intervention_does_not_clear_a_latched_fault():
    arbiter = object.__new__(CommandSourceArbiter)
    arbiter.core = CommandSourceArbiterCore(JOINTS)
    arbiter.core.latch_fault("operator review required")
    published = []
    arbiter._publish_status = lambda: published.append(True)
    response = SimpleNamespace(success=None, message="")

    result = CommandSourceArbiter._prepare_intervention(arbiter, None, response)

    assert result is response
    assert not response.success
    assert response.message == "fault latched: operator review required"
    assert arbiter.core.fault == "operator review required"
    assert arbiter.core.mode == MODE_NONE
    assert published == [True]


def test_select_teleop_waits_for_live_sample_without_enabling_leader_arms():
    arbiter = object.__new__(CommandSourceArbiter)
    arbiter.core = CommandSourceArbiterCore(JOINTS)
    arbiter._intervention_enabled = True
    ready_pair(arbiter.core, MODE_TELEOP)
    arbiter._now_s = lambda: 1.02
    arbiter._publisher_conflict = lambda: ''
    published = {'left': [], 'right': []}
    arbiter._publish_executed_command = (
        lambda arm, msg: published[arm].append(msg)
    )
    arbiter._publish_status = lambda: None
    response = SimpleNamespace(success=None, message='')

    result = CommandSourceArbiter._select_teleop(arbiter, None, response)

    assert result is response
    assert response.success
    assert arbiter.core.mode == MODE_TELEOP
    assert all(not items for items in published.values())

    arbiter._now_s = lambda: 1.03
    fresh_left = trajectory(JOINTS['left'], [0.15, 0.25])
    CommandSourceArbiter._source_callback(
        arbiter, MODE_TELEOP, 'left', fresh_left)

    assert published == {'left': [fresh_left], 'right': []}
    assert not hasattr(arbiter, '_enable_leader_arms')


def test_select_teleop_requires_explicit_intervention_prepare():
    arbiter = object.__new__(CommandSourceArbiter)
    arbiter.core = CommandSourceArbiterCore(JOINTS)
    arbiter._intervention_enabled = False
    ready_pair(arbiter.core, MODE_TELEOP)
    published = []
    arbiter._publish_status = lambda: published.append(True)
    response = SimpleNamespace(success=None, message='')

    result = CommandSourceArbiter._select_teleop(arbiter, None, response)

    assert result is response
    assert not response.success
    assert 'prepare_intervention' in response.message
    assert arbiter.core.mode == MODE_NONE
    assert published == [True]


def test_release_teleop_does_not_change_policy_ownership():
    arbiter = object.__new__(CommandSourceArbiter)
    arbiter.core = CommandSourceArbiterCore(JOINTS)
    ready_pair(arbiter.core, MODE_POLICY)
    assert arbiter.core.select(MODE_POLICY, 1.02)[0]
    released = []
    published = []
    arbiter._release_leader_arms = lambda: released.append(True)
    arbiter._publish_status = lambda: published.append(True)
    response = SimpleNamespace(success=None, message='')

    result = CommandSourceArbiter._release_teleop(arbiter, None, response)

    assert result is response
    assert response.success
    assert arbiter.core.mode == MODE_POLICY
    assert released == [True]
    assert published == [True]


def test_selected_command_is_published_to_final_and_executed_action_topics():
    arbiter = object.__new__(CommandSourceArbiter)
    final = []
    executed = []
    arbiter._final_publishers = {
        'left': SimpleNamespace(publish=final.append),
    }
    arbiter._executed_action_publishers = {
        'left': SimpleNamespace(publish=executed.append),
    }
    message = trajectory(JOINTS['left'], [0.1, 0.2])

    CommandSourceArbiter._publish_executed_command(arbiter, 'left', message)

    assert final == [message]
    assert executed == [message]


def test_periodic_status_also_records_the_current_intervention_mode():
    arbiter = object.__new__(CommandSourceArbiter)
    status_messages = []
    intervention_modes = []
    arbiter._status_payload = lambda: {"mode": MODE_POLICY}
    arbiter._status_publisher = SimpleNamespace(publish=status_messages.append)
    arbiter._publish_intervention_mode = lambda: intervention_modes.append(True)

    CommandSourceArbiter._publish_status(arbiter)

    assert len(status_messages) == 1
    assert '"mode": "POLICY"' in status_messages[0].data
    assert intervention_modes == [True]


def test_policy_cannot_take_over_an_active_waypoint_lease():
    arbiter = object.__new__(CommandSourceArbiter)
    arbiter.core = CommandSourceArbiterCore(JOINTS)
    ready_pair(arbiter.core, MODE_WAYPOINT)
    assert arbiter.core.select(MODE_WAYPOINT, 1.02)[0]
    published = []
    arbiter._publish_status = lambda: published.append(True)
    response = SimpleNamespace(success=None, message='')

    result = CommandSourceArbiter._select_policy(arbiter, None, response)

    assert result is response
    assert not response.success
    assert 'active WAYPOINT lease' in response.message
    assert arbiter.core.mode == MODE_WAYPOINT
    assert published == [True]
