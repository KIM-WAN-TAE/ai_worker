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


if __name__ == "__main__":
    unittest.main()
