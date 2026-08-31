"""Contract tests for the controller/UI lifecycle split."""

from pathlib import Path
import unittest


DOCKER = Path(__file__).resolve().parents[1]


class InterventionLifecycleTest(unittest.TestCase):
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
        self.assertIn('[ "${INTERVENTION_HANDOFF}" = true ]', runner)

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


if __name__ == "__main__":
    unittest.main()
