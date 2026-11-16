import unittest
import numpy as np
from analyze_motion import fit_tracking


class TrackingTests(unittest.TestCase):
    def test_known_delay_is_recovered(self):
        t = np.arange(0, 8, .01)
        wave = lambda x: .2*np.sin(2.3*x)+.1*np.sin(5.1*x)
        result = fit_tracking(np.c_[t, wave(t)], np.c_[t, wave(t-.165)])
        self.assertTrue(result['usable'])
        self.assertAlmostEqual(result['best_fit_lag_ms'], 165)
        self.assertLess(result['fitted_rmse_deg'], .01)

    def test_stationary_data_cannot_identify_delay(self):
        t = np.arange(0, 8, .01)
        self.assertFalse(fit_tracking(np.c_[t, t*0], np.c_[t, t*0])['usable'])

    def test_missing_samples_do_not_get_bridged_into_delay(self):
        t = np.arange(0, 8, .01)
        data = np.c_[t, np.sin(t)]
        self.assertFalse(fit_tracking(data[(t < 3) | (t > 3.2)], data)['usable'])


if __name__ == '__main__':
    unittest.main()
