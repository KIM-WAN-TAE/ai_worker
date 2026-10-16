"""Contract tests for the controller/UI lifecycle split."""

from pathlib import Path
import unittest


DOCKER = Path(__file__).resolve().parents[1]


class InterventionLifecycleTest(unittest.TestCase):
    def test_optional_rtc_type_cannot_disable_inference_status_subscriptions(self):
        source = (DOCKER / "bringup_control_ui" / "server.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn(
            "from interfaces.msg import InferenceStatus, RecordingStatus, RtcStatus",
            source,
        )
        self.assertIn("from interfaces.msg import InferenceStatus", source)
        self.assertIn("from interfaces.msg import RecordingStatus", source)
        self.assertIn("from interfaces.msg import RtcStatus", source)

    def test_controller_is_in_default_s6_bundle(self):
        for name in ("Dockerfile.amd64", "Dockerfile.arm64"):
            dockerfile = (DOCKER / name).read_text(encoding="utf-8")
            self.assertIn("contents.d/intervention_controller", dockerfile)
            self.assertIn("s6-rc.d/intervention_controller/run", dockerfile)

    def test_optional_ui_runs_only_a_proxy(self):
        command = (DOCKER / "bringup_control_ui" / "ai_worker_control").read_text(
            encoding="utf-8"
        )
        self.assertIn("ui_proxy.py", command)
        self.assertNotIn(
            'runner="/root/ros2_ws/src/ai_worker/docker/bringup_control_ui/run.sh"',
            command,
        )

    def test_internal_and_ui_ports_are_distinct(self):
        compose = (DOCKER / "docker-compose.yml").read_text(encoding="utf-8")
        self.assertIn(
            "INTERVENTION_CONTROL_PORT=${INTERVENTION_CONTROL_PORT:-8779}",
            compose,
        )
        self.assertIn("BRINGUP_CONTROL_PORT=${BRINGUP_CONTROL_PORT:-8780}", compose)

    def test_intervention_a3_outputs_are_remapped_to_arbiter_inputs(self):
        runner = (DOCKER / "s6-services" / "common" / "ros2_service_run.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "left_trajectory_output_topic:=/intervention/teleop/arm_left",
            runner,
        )
        self.assertIn(
            "right_trajectory_output_topic:=/intervention/teleop/arm_right",
            runner,
        )
        self.assertIn("teleop_reference_mode:=relative", runner)
        self.assertIn('[ "${ISOLATED_LEADER_START}" = true ]', runner)
        self.assertIn("auto_select_teleop:=false", runner)
        self.assertIn('LEASE_PURPOSE}" = "standby"', runner)
        self.assertIn(
            "cmd_vel_output_topic:=/intervention/teleop/cmd_vel",
            runner,
        )
        self.assertIn(
            "tact_trigger_output_topic:=/intervention/teleop/tact_trigger",
            runner,
        )

    def test_leader_preflight_uses_one_ros_graph_process(self):
        runner = (DOCKER / "s6-services" / "common" / "ros2_service_run.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn("Running combined ROS graph preflight", runner)
        self.assertIn("get_service_names_and_types", runner)
        self.assertIn("get_publishers_info_by_topic", runner)
        self.assertIn("AVATAR_GRAPH_PREFLIGHT=true", runner)
        self.assertIn(
            '[ "${SERVICE_NAME}" = "avatar_bringup" ] && '
            '[ "${AVATAR_GRAPH_PREFLIGHT}" != true ]',
            runner,
        )

    def test_intervention_supports_a3_and_mini_relative_leaders(self):
        avatar = (
            DOCKER / "s6-services" / "avatar_bringup" / "run"
        ).read_text(encoding="utf-8")
        a3_launch = (
            DOCKER.parent / "ffw_bringup" / "launch" / "ffw_a3_ai.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn('LEADER_TYPE}" = "mini"', avatar)
        self.assertIn("ffw_lg2_mini_leader_ai.launch.py", avatar)
        self.assertIn("choices=['relative', 'absolute']", a3_launch)
        self.assertIn(
            "['--controller-ros-args=-p teleop_reference_mode:=', teleop_reference_mode]",
            a3_launch,
        )

    def test_mini_leader_launch_supports_arbiter_output_remap(self):
        launch = (
            DOCKER.parent
            / "ffw_bringup"
            / "launch"
            / "ffw_lg2_mini_leader_ai.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("'left_trajectory_output_topic'", launch)
        self.assertIn("'right_trajectory_output_topic'", launch)
        self.assertIn("default_value='/intervention/teleop/arm_left'", launch)
        self.assertIn("default_value='/intervention/teleop/arm_right'", launch)
        self.assertIn("left_trajectory_output_topic,", launch)
        self.assertIn("right_trajectory_output_topic,", launch)
        self.assertIn("'teleop_reference_mode'", launch)
        self.assertIn("choices=['relative', 'absolute']", launch)
        self.assertIn("default_value='relative'", launch)
        self.assertIn(
            "['--controller-ros-args=-p teleop_reference_mode:=', teleop_reference_mode]",
            launch,
        )
        self.assertIn("'save_pose_id': 3", launch)
        self.assertIn("'launch_gripper_trigger'", launch)
        self.assertIn("condition=IfCondition(launch_gripper_trigger)", launch)
        gripper_argument = launch.index("'launch_gripper_trigger'")
        self.assertIn(
            "default_value='true'",
            launch[gripper_argument:gripper_argument + 500],
        )
        controller_config = (
            DOCKER.parent
            / "ffw_bringup"
            / "config"
            / "ffw_lg2_mini_leader"
            / "ffw_lg2_mini_leader_ai_hardware_controller.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "right_save_pose_4: [-0.8787313, -0.419364, 0.7832051, "
            "-1.6154256, -0.9121433, 0.3984994, 0.4185584, 0.9]",
            controller_config,
        )
        self.assertNotIn("condition=UnlessCondition(use_sensorxel)", launch)
        self.assertIn("'auto_select_teleop'", launch)
        self.assertIn("default_value='true'", launch)
        self.assertIn("'/command_source_arbiter/select_teleop'", launch)
        self.assertIn("physical left/right buttons still enable each arm", launch)
        for argument in (
            "left_joystick_output_topic",
            "right_joystick_output_topic",
            "cmd_vel_output_topic",
            "tact_trigger_output_topic",
        ):
            self.assertIn(f"'{argument}'", launch)
        self.assertIn("['--controller-ros-args=-r /cmd_vel:='", launch)
        self.assertIn(
            "'/leader/joystick_controller/tact_trigger:='",
            launch,
        )

    def test_select_teleop_keeps_physical_arm_button_gate(self):
        arbiter = (
            DOCKER.parent
            / "ffw_bringup"
            / "ffw_bringup"
            / "command_source_arbiter.py"
        ).read_text(encoding="utf-8")
        select_start = arbiter.index("    def _select_teleop")
        select_end = arbiter.index("\n    def ", select_start + 5)
        select_body = arbiter[select_start:select_end]
        self.assertIn("return self._select(MODE_TELEOP, response)", select_body)
        self.assertNotIn("_enable_leader_arms", select_body)

    def test_intervention_controller_owns_leader_lifecycle(self):
        launch = (
            DOCKER.parent
            / "ffw_bringup"
            / "launch"
            / "f2_follower_ai.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("parameters=[{'manage_leader_service': False}]", launch)

    def test_intervention_handoff_selects_and_closes_arbiter(self):
        source = (DOCKER / "bringup_control_ui" / "server.py").read_text(
            encoding="utf-8"
        )
        for action in (
            "prepare_intervention",
            "select_none",
            "select_teleop",
            "disable_intervention",
        ):
            self.assertIn(f'"{action}"', source)

        start = source.index('if action == "start":')
        end = source.index('elif action == "end":', start)
        start_block = source[start:end]
        self.assertLess(
            start_block.index('require_arbiter_action("prepare_intervention")'),
            start_block.index('change_service("leader", True)'),
        )
        self.assertLess(
            start_block.index('change_service("leader", True)'),
            start_block.index('"select_teleop"'),
        )

        reset = source.index('elif action == "reset":', end)
        end_block = source[end:reset]
        self.assertLess(
            end_block.index('require_arbiter_action("select_none")'),
            end_block.index('change_service("leader", False)'),
        )
        self.assertLess(
            end_block.index('change_service("leader", False)'),
            end_block.index('require_arbiter_action("disable_intervention")'),
        )

    def test_manual_relative_teleop_is_owned_by_controller(self):
        source = (DOCKER / "bringup_control_ui" / "server.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"teleop_start"', source)
        self.assertIn('"teleop_stop"', source)
        self.assertIn('def manual_teleop_failures(', source)
        self.assertIn(
            'ros["recording"]["phase"] not in {"READY", "RECORDING"}',
            source,
        )
        start = source.index('elif action == "teleop_start":')
        stop = source.index('elif action == "teleop_stop":', start)
        start_block = source[start:stop]
        self.assertLess(
            start_block.index('require_arbiter_action("prepare_intervention")'),
            start_block.index('"select_teleop"'),
        )
        self.assertIn('leader_standby.transition("IN_USE")', start_block)
        stop_end = source.index('elif action == "reset_fault":', stop)
        stop_block = source[stop:stop_end]
        self.assertIn('require_arbiter_action("select_none")', stop_block)
        self.assertIn('require_arbiter_action("disable_intervention")', stop_block)
        self.assertIn('leader_standby.transition("READY")', stop_block)


if __name__ == "__main__":
    unittest.main()
