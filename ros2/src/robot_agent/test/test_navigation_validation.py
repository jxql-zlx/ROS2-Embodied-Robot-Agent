"""Counterexamples for odometry based cancellation stop verification."""

import unittest

from robot_agent.navigation_validation import validate_stop_samples


def _samples():
    return [
        (1.0 + index * 0.1, (0.0, 0.0), 0.0, (10, index * 100_000_000), 0.0, True)
        for index in range(6)
    ]


class TestNavigationValidation(unittest.TestCase):
    def test_accepts_fresh_continuous_stationary_samples(self):
        self.assertEqual(validate_stop_samples(_samples()), (0.0, 0.0))

    def test_rejects_repeated_or_nonincreasing_message_stamps(self):
        samples = _samples()
        samples[3] = (*samples[3][:3], samples[2][3], *samples[3][4:])
        with self.assertRaisesRegex(ValueError, "timestamps did not increase"):
            validate_stop_samples(samples)

    def test_rejects_receive_gap_inside_settle_window(self):
        samples = _samples()
        samples[3] = (samples[3][0] + 0.6, *samples[3][1:])
        samples[4] = (samples[4][0] + 0.6, *samples[4][1:])
        samples[5] = (samples[5][0] + 0.6, *samples[5][1:])
        with self.assertRaisesRegex(ValueError, "gap over"):
            validate_stop_samples(samples)

    def test_rejects_nonfinite_pose_and_motion(self):
        samples = _samples()
        samples[2] = (samples[2][0], (float("nan"), 0.0), *samples[2][2:])
        with self.assertRaisesRegex(ValueError, "non-finite"):
            validate_stop_samples(samples)

    def test_rejects_translation_and_rotation_drift(self):
        moved = _samples()
        moved[-1] = (moved[-1][0], (0.04, 0.0), *moved[-1][2:])
        with self.assertRaisesRegex(ValueError, "drifted"):
            validate_stop_samples(moved)

        rotated = _samples()
        rotated[-1] = (*rotated[-1][:4], 0.04, rotated[-1][5])
        with self.assertRaisesRegex(ValueError, "rotated"):
            validate_stop_samples(rotated)

    def test_rejects_speed_above_stop_threshold(self):
        samples = _samples()
        samples[-1] = (*samples[-1][:2], 0.02, *samples[-1][3:])
        with self.assertRaisesRegex(ValueError, "speed"):
            validate_stop_samples(samples)


if __name__ == "__main__":
    unittest.main()
