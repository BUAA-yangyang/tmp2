"""No-progress must distinguish a stuck robot from legitimate recovery yaw."""

import math
import unittest

from a1_exploration.frontier import NoProgressWatchdog


class NoProgressWatchdogTest(unittest.TestCase):
    def watchdog(self):
        return NoProgressWatchdog(20.0, 0.20, 0.35)

    def test_stationary_pose_stalls_at_timeout(self):
        watchdog = self.watchdog()
        watchdog.observe(1.0, 2.0, 3.0, 0.0)
        before = watchdog.observe(20.9, 2.0, 3.0, 0.0)
        at_timeout = watchdog.observe(21.0, 2.0, 3.0, 0.0)
        self.assertFalse(before.stalled)
        self.assertTrue(at_timeout.stalled)

    def test_in_place_rotation_counts_as_progress(self):
        watchdog = self.watchdog()
        watchdog.observe(1.0, 2.0, 3.0, 0.0)
        progress = watchdog.observe(15.0, 2.0, 3.0, 0.36)
        later = watchdog.observe(25.0, 2.0, 3.0, 0.36)
        self.assertTrue(progress.progressed)
        self.assertFalse(progress.stalled)
        self.assertFalse(later.stalled)

    def test_translation_counts_as_progress(self):
        watchdog = self.watchdog()
        watchdog.observe(1.0, 0.0, 0.0, 0.0)
        progress = watchdog.observe(19.0, 0.21, 0.0, 0.0)
        self.assertTrue(progress.progressed)
        self.assertFalse(watchdog.observe(30.0, 0.21, 0.0, 0.0).stalled)

    def test_wrapped_yaw_uses_shortest_angle(self):
        watchdog = self.watchdog()
        watchdog.observe(1.0, 0.0, 0.0, math.pi - 0.10)
        observation = watchdog.observe(10.0, 0.0, 0.0, -math.pi + 0.10)
        self.assertAlmostEqual(observation.turned_rad, 0.20, places=6)
        self.assertFalse(observation.progressed)

    def test_clock_rollback_resets_instead_of_instantly_stalling(self):
        watchdog = self.watchdog()
        watchdog.observe(100.0, 0.0, 0.0, 0.0)
        reset = watchdog.observe(2.0, 0.0, 0.0, 0.0)
        self.assertFalse(reset.stalled)
        self.assertIn("backwards", reset.reason)
        self.assertFalse(watchdog.observe(21.9, 0.0, 0.0, 0.0).stalled)

    def test_reset_starts_a_fresh_goal_window(self):
        watchdog = self.watchdog()
        watchdog.observe(1.0, 0.0, 0.0, 0.0)
        watchdog.observe(20.0, 0.0, 0.0, 0.0)
        watchdog.reset()
        self.assertFalse(watchdog.observe(30.0, 0.0, 0.0, 0.0).stalled)

    def test_invalid_thresholds_fail_closed(self):
        with self.assertRaises(ValueError):
            NoProgressWatchdog(0.0, 0.2, 0.35)
        with self.assertRaises(ValueError):
            NoProgressWatchdog(20.0, -0.2, 0.35)
        with self.assertRaises(ValueError):
            NoProgressWatchdog(20.0, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
