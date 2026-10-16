import unittest

try:
    from .logic import FOLLOWER_MODELS, LEADER_MODELS, SafetyState, diagnose_lines, gate_action, gate_arbiter_fault_reset, gate_intervention_preflight, gate_intervention_reset, gate_intervention_start, gate_leader_standby_start, gate_paused_pose_move, intervention_resume_confirmed, operating_mode, parse_s6_status, sustained_failure_elapsed
    from .recovery_config import default_config_path, load_custom_recovery_groups, load_recovery_groups
except ImportError:
    from logic import FOLLOWER_MODELS, LEADER_MODELS, SafetyState, diagnose_lines, gate_action, gate_arbiter_fault_reset, gate_intervention_preflight, gate_intervention_reset, gate_intervention_start, gate_leader_standby_start, gate_paused_pose_move, intervention_resume_confirmed, operating_mode, parse_s6_status, sustained_failure_elapsed
    from recovery_config import default_config_path, load_custom_recovery_groups, load_recovery_groups


class SafetyGateTest(unittest.TestCase):
    def test_standby_failure_must_outlive_its_grace(self):
        self.assertFalse(sustained_failure_elapsed(None, 20.0, 3.0))
        self.assertFalse(sustained_failure_elapsed(10.0, 12.999, 3.0))
        self.assertTrue(sustained_failure_elapsed(10.0, 13.0, 3.0))
        self.assertFalse(sustained_failure_elapsed(10.0, 9.0, 3.0))

    def test_intervention_resume_accepts_slow_start_and_rejects_unsafe_routes(self):
        base = {
            "inference_phase": "SYNCING",
            "arbiter_available": True,
            "arbiter_mode": "POLICY",
            "arbiter_fault": "",
            "publisher_conflict": "",
            "intervention_enabled": False,
        }
        self.assertTrue(intervention_resume_confirmed(**base))
        self.assertTrue(intervention_resume_confirmed(
            **{**base, "inference_phase": "INFERENCING"}
        ))
        for override in (
            {"inference_phase": "PAUSED"},
            {"arbiter_available": False},
            {"arbiter_mode": "NONE"},
            {"arbiter_fault": "policy lease lost"},
            {"publisher_conflict": "left: duplicate"},
            {"intervention_enabled": True},
        ):
            self.assertFalse(intervention_resume_confirmed(
                **{**base, **override}
            ))

    def test_supported_leader_launches_include_operational_a3(self):
        self.assertEqual(LEADER_MODELS["a3"]["launch"], "ffw_a3_ai.launch.py")
        self.assertEqual(
            LEADER_MODELS["mini"]["launch"],
            "ffw_lg2_mini_leader_ai.launch.py",
        )
        self.assertEqual(set(LEADER_MODELS), {"a3", "mini", "a2", "lg2"})

    def test_only_f2_exposes_recovery_profile(self):
        self.assertTrue(FOLLOWER_MODELS["f2"]["recover_init"])
        self.assertTrue(FOLLOWER_MODELS["f2"]["recovery_yaml"].endswith("f2_follower_initial_positions.yaml"))
        self.assertFalse(FOLLOWER_MODELS["sg2"]["recover_init"])

    def healthy_f2(self, **overrides):
        values = dict(
            physical_control_enabled=True,
            follower_up=True,
            controllers_ready=True,
            feedback_fresh=True,
            dxl_ok=True,
            selected_follower="f2",
        )
        values.update(overrides)
        return SafetyState(**values)

    def test_bring_down_requires_leader_off(self):
        result = gate_action("bring_down", self.healthy_f2(leader_up=True))
        self.assertFalse(result.ok)
        self.assertTrue(any("Leader" in item for item in result.failures))

    def test_bring_up_blocks_even_stopped_inference_container(self):
        state = SafetyState(physical_control_enabled=True, inference_present=True)
        result = gate_action("follower_start", state)
        self.assertFalse(result.ok)
        self.assertTrue(any("container" in item for item in result.failures))

    def test_leader_start_requires_healthy_follower_first(self):
        missing = gate_action("leader_start", SafetyState(physical_control_enabled=True))
        self.assertFalse(missing.ok)
        self.assertTrue(any("Follower를 먼저" in item for item in missing.failures))

        unhealthy = gate_action(
            "leader_start",
            SafetyState(physical_control_enabled=True, follower_up=True),
        )
        self.assertFalse(unhealthy.ok)
        self.assertTrue(any("controller" in item for item in unhealthy.failures))
        self.assertTrue(any("DXL" in item for item in unhealthy.failures))

        healthy = gate_action("leader_start", self.healthy_f2())
        self.assertTrue(healthy.ok)

    def test_recovery_happy_path_has_workspace_warning(self):
        result = gate_action("recover_init", self.healthy_f2())
        self.assertTrue(result.ok)
        self.assertTrue(any("작업 공간" in item for item in result.warnings))

    def test_only_active_pose_cancel_bypasses_control_lock(self):
        state = SafetyState(follower_up=True)
        self.assertFalse(gate_action("follower_stop", state).ok)
        self.assertTrue(gate_action("operation_cancel", state).ok)

    def test_unmanaged_runtime_blocks_duplicate_start_and_fake_stop(self):
        state = SafetyState(
            physical_control_enabled=True,
            follower_up=True,
            follower_managed=False,
        )
        self.assertFalse(gate_action("follower_start", state).ok)
        stopped = gate_action("follower_stop", state)
        self.assertFalse(stopped.ok)
        self.assertTrue(any("UI 밖" in item for item in stopped.failures))

    def test_motion_blocks_on_estop(self):
        result = gate_action("recover_init", self.healthy_f2(estop_active=True))
        self.assertFalse(result.ok)
        self.assertTrue(any("E-stop" in item for item in result.failures))

    def test_concurrent_user_action_is_blocked(self):
        result = gate_action("leader_start", SafetyState(physical_control_enabled=True, control_busy=True))
        self.assertFalse(result.ok)
        self.assertTrue(any("다른 사용자" in item for item in result.failures))

    def test_configured_estop_monitor_must_be_fresh(self):
        result = gate_action("recover_init", self.healthy_f2(estop_signal_ready=False))
        self.assertFalse(result.ok)
        self.assertTrue(any("감시 토픽" in item for item in result.failures))

    def test_intervention_requires_confirmed_paused_inference(self):
        running = gate_intervention_start(
            self.healthy_f2(inference_present=True),
            "INFERENCING",
        )
        self.assertFalse(running.ok)
        self.assertTrue(any("PAUSED" in item for item in running.failures))

        paused = gate_intervention_start(
            self.healthy_f2(inference_present=True),
            "PAUSED",
        )
        self.assertTrue(paused.ok)

    def test_operator_pose_move_requires_quiet_paused_recording_ready_state(self):
        base = dict(
            inference_phase="PAUSED",
            inference_status_ready=True,
            recording_phase="READY",
            recording_status_ready=True,
            intervention_active=False,
            arbiter_available=True,
            arbiter_mode="NONE",
            arbiter_fault="",
            publisher_conflict="",
            intervention_enabled=False,
        )
        allowed = gate_paused_pose_move(
            self.healthy_f2(inference_present=True),
            **base,
        )
        self.assertTrue(allowed.ok)

        ready_allowed = gate_paused_pose_move(
            self.healthy_f2(inference_present=True),
            **{**base, "inference_phase": "READY"},
        )
        self.assertTrue(ready_allowed.ok)

        saving_allowed = gate_paused_pose_move(
            self.healthy_f2(inference_present=True),
            **{**base, "recording_phase": "SAVING"},
        )
        self.assertTrue(saving_allowed.ok)

        for override in (
            {"inference_phase": "INFERENCING"},
            {"recording_phase": "RECORDING"},
            {"intervention_active": True},
            {"arbiter_mode": "POLICY"},
            {"intervention_enabled": True},
        ):
            blocked = gate_paused_pose_move(
                self.healthy_f2(inference_present=True),
                **{**base, **override},
            )
            self.assertFalse(blocked.ok)

    def test_operator_pose_lists_are_checked_against_f2_joint_limits(self):
        groups = load_recovery_groups("f2")
        targets_by_executor = {
            group.executor_name: list(group.steps[-1].positions)
            for group in groups
        }
        targets = {
            "left_arm": targets_by_executor["arm_l_joint_trajectory_executor"],
            "right_arm": targets_by_executor["arm_r_joint_trajectory_executor"],
            "head": targets_by_executor["head_joint_trajectory_executor"],
            "lift": targets_by_executor["lift_joint_trajectory_executor"],
        }
        custom = load_custom_recovery_groups("f2", targets, duration=8)
        self.assertTrue(all(group.steps[0].name == "custom" for group in custom))
        self.assertTrue(all(group.duration == 8 for group in custom))

        with self.assertRaisesRegex(ValueError, "8개"):
            load_custom_recovery_groups(
                "f2",
                {**targets, "left_arm": [0.0]},
                duration=8,
            )
        unsafe = {**targets, "left_arm": list(targets["left_arm"])}
        unsafe["left_arm"][0] = 99.0
        with self.assertRaisesRegex(ValueError, "허용 범위"):
            load_custom_recovery_groups("f2", unsafe, duration=8)

    def test_intervention_rejects_recent_policy_command(self):
        result = gate_intervention_start(
            self.healthy_f2(
                inference_present=True,
                command_stream_active=True,
            ),
            "PAUSED",
        )
        self.assertFalse(result.ok)
        self.assertTrue(any("최근 trajectory" in item for item in result.failures))

    def test_intervention_is_limited_to_validated_f2_profile(self):
        result = gate_intervention_start(
            self.healthy_f2(inference_present=True, selected_follower="sg2"),
            "PAUSED",
        )
        self.assertFalse(result.ok)
        self.assertTrue(any("F2 + A3" in item for item in result.failures))

    def test_faulted_intervention_can_reset_only_after_outputs_are_quiet(self):
        allowed = gate_intervention_reset(
            "PAUSED_SAFE",
            leader_up=False,
            command_recent=False,
            recording_phase="READY",
        )
        self.assertTrue(allowed.ok)
        warm_standby_allowed = gate_intervention_reset(
            "READY_TO_RESUME",
            leader_up=True,
            command_recent=False,
            recording_phase="READY",
            leader_standby=True,
        )
        self.assertTrue(warm_standby_allowed.ok)
        saving_allowed = gate_intervention_reset(
            "READY_TO_RESUME",
            leader_up=True,
            command_recent=False,
            recording_phase="SAVING",
            leader_standby=True,
        )
        self.assertTrue(saving_allowed.ok)

        for overrides in (
            {"phase": "READY"},
            {"leader_up": True},
            {"command_recent": True},
            {"recording_phase": "RECORDING"},
        ):
            values = {
                "phase": "READY_TO_RESUME",
                "leader_up": False,
                "command_recent": False,
                "recording_phase": "READY",
            }
            values.update(overrides)
            self.assertFalse(gate_intervention_reset(**values).ok)

    def test_intervention_preflight_requires_typed_status_and_clean_arbiter(self):
        values = {
            "state": self.healthy_f2(inference_present=True),
            "inference_phase": "INFERENCING",
            "inference_status_ready": True,
            "recording_phase": "READY",
            "recording_status_ready": True,
            "arbiter_available": True,
            "arbiter_mode": "POLICY",
            "arbiter_fault": "",
            "publisher_conflict": "",
            "intervention_enabled": False,
        }
        self.assertTrue(gate_intervention_preflight(**values).ok)

        for override in (
            {"inference_status_ready": False},
            {"recording_status_ready": False},
            {"arbiter_available": False},
            {"arbiter_fault": "policy lease lost"},
            {"publisher_conflict": "left: duplicate"},
            {"intervention_enabled": True},
            {"recording_phase": "SAVING"},
        ):
            blocked = {**values, **override}
            self.assertFalse(gate_intervention_preflight(**blocked).ok)

    def test_paused_preflight_requires_arbiter_none(self):
        values = {
            "state": self.healthy_f2(inference_present=True),
            "inference_phase": "PAUSED",
            "inference_status_ready": True,
            "recording_phase": "READY",
            "recording_status_ready": True,
            "arbiter_available": True,
            "arbiter_mode": "POLICY",
            "arbiter_fault": "",
            "publisher_conflict": "",
            "intervention_enabled": False,
        }
        self.assertFalse(gate_intervention_preflight(**values).ok)
        self.assertTrue(gate_intervention_preflight(
            **{**values, "arbiter_mode": "NONE"}
        ).ok)

    def test_warm_standby_is_allowed_without_treating_leader_as_conflict(self):
        values = {
            "state": self.healthy_f2(
                inference_present=True,
                leader_up=True,
                leader_stream_active=True,
                leader_standby=True,
            ),
            "inference_phase": "INFERENCING",
            "inference_status_ready": True,
            "recording_phase": "READY",
            "recording_status_ready": True,
            "arbiter_available": True,
            "arbiter_mode": "POLICY",
            "arbiter_fault": "",
            "publisher_conflict": "",
            "intervention_enabled": False,
        }
        self.assertTrue(gate_intervention_preflight(**values).ok)
        self.assertTrue(gate_intervention_start(values["state"], "PAUSED").ok)

        # An inference episode stays open across the policy-to-human handoff.
        self.assertTrue(gate_intervention_preflight(
            **{**values, "recording_phase": "RECORDING"}
        ).ok)

    def test_warm_standby_start_preserves_policy_ownership(self):
        values = {
            "state": self.healthy_f2(inference_present=True),
            "inference_phase": "INFERENCING",
            "inference_status_ready": True,
            "recording_phase": "READY",
            "recording_status_ready": True,
            "arbiter_available": True,
            "arbiter_mode": "POLICY",
            "arbiter_fault": "",
            "publisher_conflict": "",
            "intervention_enabled": False,
        }
        allowed = gate_leader_standby_start(**values)
        self.assertTrue(allowed.ok)
        self.assertTrue(any("격리" in item for item in allowed.warnings))

        self.assertTrue(gate_leader_standby_start(
            **{**values, "recording_phase": "RECORDING"}
        ).ok)

        self.assertFalse(gate_leader_standby_start(
            **{**values, "arbiter_mode": "TELEOP"}
        ).ok)
        self.assertFalse(gate_leader_standby_start(
            **{**values, "intervention_enabled": True}
        ).ok)

    def test_fault_reset_requires_quiet_typed_none_without_conflict(self):
        values = {
            "inference_phase": "PAUSED",
            "inference_status_ready": True,
            "leader_up": False,
            "command_recent": False,
            "recording_phase": "READY",
            "arbiter_available": True,
            "arbiter_mode": "NONE",
            "publisher_conflict": "",
        }
        self.assertTrue(gate_arbiter_fault_reset(**values).ok)
        self.assertTrue(gate_arbiter_fault_reset(
            **{
                **values,
                "leader_up": True,
                "leader_standby": True,
            }
        ).ok)
        self.assertTrue(gate_arbiter_fault_reset(
            **{
                **values,
                "inference_phase": "CONNECTED",
                "inference_status_ready": False,
            }
        ).ok)
        for override in (
            {"inference_phase": "INFERENCING"},
            {"inference_phase": "LOADING"},
            {"inference_phase": "SYNCING"},
            {"leader_up": True},
            {"command_recent": True},
            {"recording_phase": "RECORDING"},
            {"arbiter_available": False},
            {"arbiter_mode": "TELEOP"},
            {"publisher_conflict": "right: duplicate"},
        ):
            self.assertFalse(gate_arbiter_fault_reset(
                **{**values, **override}
            ).ok)


class DiagnosticsTest(unittest.TestCase):
    def test_diagnostics_are_deduplicated(self):
        rows = diagnose_lines([
            "failed to open port /dev/follower",
            "could not open port /dev/follower",
            "Waiting for action server...",
        ])
        self.assertEqual({row["title"] for row in rows}, {"로봇 포트 연결 실패", "Action Server 대기"})

    def test_s6_parser(self):
        parsed = parse_s6_status("up (pid 321) 18 seconds")
        self.assertEqual(parsed["state"], "up")
        self.assertEqual(parsed["pid"], 321)
        self.assertEqual(parsed["uptime_s"], 18)


class RecoveryConfigTest(unittest.TestCase):
    def test_f2_recovery_plan_is_loaded_from_all_repeated_yaml_sections(self):
        path = default_config_path("f2")
        groups = load_recovery_groups("f2", path)
        self.assertEqual(
            {group.executor_name for group in groups},
            {
                "arm_l_joint_trajectory_executor",
                "arm_r_joint_trajectory_executor",
                "head_joint_trajectory_executor",
                "lift_joint_trajectory_executor",
            },
        )
        self.assertEqual([step.name for step in groups[0].steps], ["step0", "step1"])
        head = next(group for group in groups if group.executor_name == "head_joint_trajectory_executor")
        self.assertEqual([step.name for step in head.steps], ["home"])
        self.assertNotIn("swerve", " ".join(group.executor_name for group in groups))

    def test_unimplemented_sg2_recovery_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "지원하지 않는 복구 모델"):
            load_recovery_groups("sg2")


class OperatingModeTest(unittest.TestCase):
    def test_intervention_is_not_reported_as_runtime_conflict(self):
        mode = operating_mode(
            leader_up=True,
            inference_present=True,
            inference_active=True,
            inference_phase="PAUSED",
            intervention_active=True,
        )
        self.assertEqual(mode["name"], "intervention")
        self.assertFalse(mode["conflict"])

    def test_leader_runtime_selects_teleop_mode(self):
        mode = operating_mode(leader_up=True, inference_present=False, inference_active=False)
        self.assertEqual(mode["name"], "teleop")

    def test_paused_inference_session_remains_inference_mode(self):
        mode = operating_mode(
            leader_up=False,
            inference_present=True,
            inference_active=True,
            inference_phase="PAUSED",
        )
        self.assertEqual(mode["name"], "inference")
        self.assertIn("PAUSED", mode["detail"])

    def test_simultaneous_leader_and_inference_is_conflict(self):
        mode = operating_mode(
            leader_up=True,
            inference_present=True,
            inference_active=True,
            inference_phase="INFERENCING",
        )
        self.assertEqual(mode["name"], "conflict")
        self.assertTrue(mode["conflict"])


if __name__ == "__main__":
    unittest.main()
