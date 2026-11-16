"""Offline SG2 pose and controller contracts; never starts a ROS node."""

from pathlib import Path
import unittest

from logic import FOLLOWER_MODELS, SafetyState, gate_action, gate_intervention_start
from recovery_config import (
    CUSTOM_TARGET_KEYS, _joint_limits, _load_repeated_ros_sections,
    default_config_path, load_custom_recovery_groups, load_recovery_groups,
)


class SG2ConfigurationTests(unittest.TestCase):
    def test_all_startup_executors_match_f2_including_steering(self):
        f2 = _load_repeated_ros_sections(default_config_path("f2"))
        sg2 = _load_repeated_ros_sections(default_config_path("sg2"))
        self.assertEqual(len(sg2), 5)
        self.assertIn("swerve_steering_joint_trajectory_executor", sg2)
        # Includes every step, joint order, duration, tolerance and ROS topic.
        self.assertEqual(sg2, f2)

    def test_f2_startup_pose_is_valid_for_every_sg2_joint(self):
        expected = load_recovery_groups("f2")
        actual = load_recovery_groups("sg2")
        self.assertEqual(actual, expected)
        limits = _joint_limits("sg2", {j for group in actual for j in group.joints})
        for group in actual:
            for step in group.steps:
                for name, value in zip(group.joints, step.positions):
                    low, high = limits[name]
                    self.assertLessEqual(low, value, (name, step.name))
                    self.assertLessEqual(value, high, (name, step.name))
        targets = {CUSTOM_TARGET_KEYS[g.executor_name]: list(g.steps[-1].positions)
                   for g in actual}
        self.assertEqual(len(load_custom_recovery_groups("sg2", targets)), 4)
        targets["lift"] = [99.0]
        with self.assertRaises(ValueError):
            load_custom_recovery_groups("sg2", targets)

    def test_sg2_recovery_does_not_enable_f2_bring_down(self):
        state = SafetyState(physical_control_enabled=True, follower_up=True,
                            controllers_ready=True, feedback_fresh=True, dxl_ok=True,
                            selected_follower="sg2")
        self.assertTrue(gate_action("recover_init", state).ok)
        self.assertFalse(gate_action("bring_down", state).ok)
        self.assertFalse(FOLLOWER_MODELS["sg2"]["bring_down"])

    def test_sg2_intervention_keeps_fault_and_motion_guards(self):
        from dataclasses import replace
        state = SafetyState(physical_control_enabled=True, follower_up=True,
                            controllers_ready=True, feedback_fresh=True, dxl_ok=True,
                            selected_follower="sg2", inference_present=True)
        self.assertTrue(gate_intervention_start(state, inference_phase="PAUSED").ok)
        for change in ({"estop_active": True}, {"feedback_fresh": False},
                       {"command_stream_active": True}, {"critical_fault": True}):
            self.assertFalse(gate_intervention_start(
                replace(state, **change), inference_phase="PAUSED").ok)


if __name__ == "__main__":
    unittest.main()
