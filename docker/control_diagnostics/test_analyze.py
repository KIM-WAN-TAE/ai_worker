import unittest
import numpy as np
from analyze import analyze


def fixture(torque=True, moving_command=False, moving_feedback=False, missing_torque=False):
    for t in np.arange(0, 6, .01):
        reference = .03*np.sin(2*np.pi*3*t) if moving_command else 0.
        feedback = .02*np.sin(2*np.pi*3*t) if moving_feedback else 0.
        yield dict(topic='/arm_l_controller/controller_state', receive_monotonic_ns=int(t*1e9),
            message=dict(joint_names=['arm_l_joint1'], reference=dict(positions=[reference]),
                         feedback=dict(positions=[feedback])))
        if not missing_torque:
            yield dict(topic='/ffw_follower/dxl_state', receive_monotonic_ns=int(t*1e9),
                       message=dict(id=[31], torque_state=[torque]))


class DiagnosticTests(unittest.TestCase):
    def classifications(self, **kwargs):
        report = analyze(fixture(**kwargs))
        return [w['classification'] for w in report['joints']['arm_l_joint1']['windows']]

    def test_fixed_reference_with_feedback_oscillation_is_only_a_candidate(self):
        labels = self.classifications(moving_feedback=True)
        self.assertTrue(labels)
        self.assertEqual(set(labels), {'downstream_motion_candidate'})

    def test_moving_command_is_not_misdiagnosed_as_motor_fault(self):
        self.assertEqual(set(self.classifications(moving_command=True, moving_feedback=True)), {'changing_reference'})

    def test_torque_off_and_missing_torque_cannot_establish_stability(self):
        for kwargs in (dict(torque=False), dict(missing_torque=True)):
            self.assertEqual(set(self.classifications(**kwargs)), {'inconclusive_torque_or_telemetry'})

    def test_fixed_reference_quiet_when_energized(self):
        self.assertEqual(set(self.classifications()), {'quiet_under_fixed_reference'})

    def test_untrusted_torque_publisher_cannot_establish_on_or_off(self):
        for torque in (True, False):
            report = analyze(fixture(torque=torque, moving_feedback=True),
                             torque_telemetry_trusted=False)
            windows = report['joints']['arm_l_joint1']['windows']
            self.assertTrue(windows)
            self.assertEqual({w['classification'] for w in windows},
                             {'inconclusive_untrusted_torque_telemetry'})
            self.assertGreater(windows[0]['feedback_rms_rad'], .002)

    def test_telemetry_gap_does_not_become_motor_noise(self):
        rows = [r for r in fixture(moving_feedback=True) if not 2.3e9 < r['receive_monotonic_ns'] < 4.9e9]
        report = analyze(rows)
        self.assertEqual(report['joints']['arm_l_joint1']['windows'], [])


if __name__ == '__main__':
    unittest.main()
