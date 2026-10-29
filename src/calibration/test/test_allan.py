"""Mathematical/unit regressions; not evidence of a measured IMU calibration."""

import unittest

import numpy as np

from spr_calibration.allan import analyze_allan, fit_power_law, overlapping_allan
from spr_calibration.common import CalibrationError


class AllanContractTests(unittest.TestCase):
    def test_short_static_recording_cannot_pass_formal_gate(self):
        ns = np.arange(200, dtype=np.int64) * 10_000_000 + 1
        with self.assertRaisesRegex(CalibrationError, ">="):
            analyze_allan(ns, np.zeros((200, 6)))

    def test_nonfinite_samples_are_rejected(self):
        with self.assertRaises(CalibrationError):
            overlapping_allan(np.full((100, 6), np.nan), 0.01)

    def test_rate_allan_matches_definition(self):
        # Direct comparison with the overlapping, adjacent block-mean definition.
        rates = np.arange(120, dtype=float)
        tau, deviation, counts = overlapping_allan(rates, 0.1, [1, 2, 5])
        expected = []
        for size in (1, 2, 5):
            difference = [rates[start + size:start + 2 * size].mean() - rates[start:start + size].mean()
                          for start in range(len(rates) - 2 * size + 1)]
            expected.append(np.sqrt(np.mean(np.square(difference)) / 2))
        np.testing.assert_allclose(tau, [0.1, 0.2, 0.5])
        np.testing.assert_allclose(deviation[:, 0], expected)
        np.testing.assert_equal(counts, [119, 117, 111])

    def test_noise_coefficients_follow_kalibr_continuous_time_units(self):
        tau = np.geomspace(0.01, 100, 20)
        white = fit_power_law(tau, 0.004 / np.sqrt(tau), -0.5)
        walk = fit_power_law(tau, 0.0002 * np.sqrt(tau / 3), 0.5)
        self.assertAlmostEqual(white["coefficient"], 0.004)
        self.assertAlmostEqual(walk["coefficient"], 0.0002)

    def test_no_default_random_walk_is_invented(self):
        tau = np.geomspace(0.01, 100, 20)
        with self.assertRaisesRegex(CalibrationError, "record longer"):
            fit_power_law(tau, 0.004 / np.sqrt(tau), 0.5)


if __name__ == "__main__":
    unittest.main()
