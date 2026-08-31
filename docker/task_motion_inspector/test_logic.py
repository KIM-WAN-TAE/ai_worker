import math
import unittest

from logic import (
    ARM_DXL_MAP,
    DEFAULT_LIMITS,
    PH42_RAD_PER_TICK,
    Y_SERIES_RAD_PER_TICK,
    ConditionLatch,
    classify_joint,
    parse_f2_limits,
    safe_session_name,
    trajectory_range_status,
)


ROBOT_DESCRIPTION = """
<robot name="ffw_f2_follower">
  <joint name="arm_r_joint1" type="revolute">
    <limit lower="-3.14" upper="3.14" effort="1000" velocity="4.8"/>
  </joint>
  <joint name="arm_l_joint7" type="revolute">
    <limit lower="-1.8201" upper="1.5804" effort="1000" velocity="4.8"/>
  </joint>
  <joint name="arm_r_joint7" type="revolute">
    <limit lower="-1.5804" upper="1.8201" effort="1000" velocity="4.8"/>
  </joint>
  <ros2_control name="ffw_f2_follower" type="system">
    <gpio name="dxl1">
      <param name="Max Position Limit">262144</param>
      <param name="Min Position Limit">-262144</param>
    </gpio>
    <gpio name="dxl7">
      <param name="Max Position Limit">151875</param>
      <param name="Min Position Limit">-131875</param>
    </gpio>
    <gpio name="dxl37">
      <param name="Max_Position_Limit">131875</param>
      <param name="Min_Position_Limit">-151875</param>
    </gpio>
  </ros2_control>
</robot>
"""


class MotionLogicTest(unittest.TestCase):
    def test_session_name_is_path_safe(self):
        self.assertEqual(safe_session_name("../../my test"), "my_test")

    def test_runtime_description_updates_current_j7_limits(self):
        limits = parse_f2_limits(ROBOT_DESCRIPTION)
        self.assertEqual(limits["arm_r_joint7"]["ticks"], [-131875, 151875])
        self.assertEqual(limits["arm_l_joint7"]["ticks"], [-151875, 131875])
        self.assertAlmostEqual(
            limits["arm_r_joint7"]["hardware"][1],
            151875 * PH42_RAD_PER_TICK,
        )
        self.assertEqual(
            limits["arm_r_joint7"]["source"],
            "runtime URDF + runtime ros2_control",
        )

    def test_runtime_description_uses_y_series_resolution_for_j1(self):
        limits = parse_f2_limits(ROBOT_DESCRIPTION)
        right_j1 = limits["arm_r_joint1"]
        self.assertEqual(right_j1["model"], "YM080-230-R099")
        self.assertEqual(right_j1["ticks_per_rev"], 524288)
        self.assertEqual(right_j1["ticks"], [-262144, 262144])
        self.assertAlmostEqual(right_j1["rad_per_tick"], Y_SERIES_RAD_PER_TICK)
        self.assertAlmostEqual(right_j1["hardware"][0], -math.pi)
        self.assertAlmostEqual(right_j1["hardware"][1], math.pi)

    def test_all_arm_motor_models_and_resolutions_are_mapped(self):
        self.assertEqual(len(ARM_DXL_MAP), 14)
        self.assertEqual(set(DEFAULT_LIMITS), {spec["joint"] for spec in ARM_DXL_MAP.values()})
        for spec in ARM_DXL_MAP.values():
            index = int(spec["joint"].rsplit("joint", 1)[1])
            if index <= 2:
                self.assertEqual(spec["model"], "YM080-230-R099")
                self.assertEqual(spec["ticks_per_rev"], 524288)
            elif index <= 6:
                self.assertEqual(spec["model"], "YM070-210-R099")
                self.assertEqual(spec["ticks_per_rev"], 524288)
            else:
                self.assertEqual(spec["model"], "PH42-020-S300-R")
                self.assertEqual(spec["ticks_per_rev"], 607500)

    def test_j7_outside_hardware_limit_is_visible(self):
        values = {
            "command": math.radians(100),
            "reference": math.radians(100),
            "feedback": math.radians(90),
            "tracking_warning": True,
        }
        flags = classify_joint(values, DEFAULT_LIMITS["arm_r_joint7"])
        self.assertIn("OUTSIDE_DXL_RANGE", flags)
        self.assertIn("TRACKING_WARNING", flags)

    def test_pipeline_difference_is_classified(self):
        values = {"source": 1.0, "command": 0.8, "reference": 0.7, "output": 0.6, "feedback": 0.5}
        flags = classify_joint(values, None)
        self.assertEqual(
            flags,
            ["SOURCE_TO_COMMAND", "COMMAND_TO_REFERENCE", "CONTROLLER_OUTPUT", "TRACKING_WARNING"],
        )

    def test_hard_limit_saturation_has_specific_flag(self):
        values = {
            "reference": 1.8,
            "feedback": 1.57,
            "dxl_present": DEFAULT_LIMITS["arm_r_joint7"]["hardware"][1],
            "hard_limit_saturation": True,
        }
        flags = classify_joint(values, DEFAULT_LIMITS["arm_r_joint7"])
        self.assertIn("HARD_LIMIT_SATURATION", flags)

    def test_recorded_range_outside_hardware_limit_is_rejected(self):
        limits = DEFAULT_LIMITS["arm_r_joint7"]
        self.assertEqual(trajectory_range_status(-1.0, math.radians(100), limits), "outside")

    def test_recorded_range_near_hardware_limit_is_visible(self):
        limits = DEFAULT_LIMITS["arm_r_joint7"]
        self.assertEqual(trajectory_range_status(-1.0, limits["hardware"][1], limits), "near")

    def test_condition_latch_debounces_open_and_close(self):
        latch = ConditionLatch(on_delay_s=0.2, off_delay_s=0.3)
        self.assertIsNone(latch.update(True, 1.0))
        self.assertIsNone(latch.update(True, 1.1))
        self.assertEqual(latch.update(True, 1.21), "opened")
        self.assertTrue(latch.active)
        self.assertIsNone(latch.update(False, 2.0))
        self.assertIsNone(latch.update(False, 2.2))
        self.assertEqual(latch.update(False, 2.31), "closed")
        self.assertFalse(latch.active)


if __name__ == "__main__":
    unittest.main()
