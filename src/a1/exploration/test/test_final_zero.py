#!/usr/bin/env python3
import unittest

from a1_exploration.final_zero import FinalZeroMonitor


class FinalZeroMonitorTest(unittest.TestCase):
    def setUp(self):
        self.monitor = FinalZeroMonitor(
            epsilon=0.01, freshness_s=0.25, settle_s=0.50
        )

    def test_freshness_and_time_since_nonzero_are_independent(self):
        self.monitor.observe(10.00, (0.2, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.monitor.observe(10.10, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.monitor.observe(10.55, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        result = self.monitor.evaluate(10.60)
        self.assertTrue(result["ready"])
        self.assertAlmostEqual(result["message_age"], 0.05)
        self.assertAlmostEqual(result["zero_duration"], 0.60)

    def test_stale_zero_fails_closed(self):
        self.monitor.observe(1.0, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        result = self.monitor.evaluate(1.6)
        self.assertFalse(result["ready"])
        self.assertEqual(result["reason"], "command message is stale")

    def test_fresh_zero_without_settle_interval_fails(self):
        self.monitor.observe(2.0, (0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.monitor.observe(2.4, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.monitor.observe(2.45, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        result = self.monitor.evaluate(2.45)
        self.assertFalse(result["ready"])
        self.assertEqual(
            result["reason"], "zero command has not settled long enough"
        )

    def test_clock_rollback_fails_closed(self):
        self.monitor.observe(8.0, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.monitor.observe(7.9, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertFalse(self.monitor.evaluate(8.6)["ready"])
        self.assertIn("clock", self.monitor.evaluate(8.6)["reason"])

    def test_evaluate_time_rollback_fails_closed(self):
        self.monitor.observe(5.0, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        result = self.monitor.evaluate(4.9)
        self.assertFalse(result["ready"])
        self.assertEqual(result["reason"], "ROS clock moved backwards")

    def test_nonfinite_command_fails_closed(self):
        self.monitor.observe(
            1.0, (float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0)
        )
        self.assertFalse(self.monitor.evaluate(2.0)["ready"])

    def test_hidden_twist_axis_is_not_accepted_as_zero(self):
        self.monitor.observe(1.0, (0.0, 0.0, 0.02, 0.0, 0.0, 0.0))
        self.monitor.observe(1.6, (0.0, 0.0, 0.02, 0.0, 0.0, 0.0))
        result = self.monitor.evaluate(1.6)
        self.assertFalse(result["ready"])
        self.assertEqual(result["reason"], "latest command is non-zero")

    def test_planar_only_sample_fails_closed(self):
        self.monitor.observe(1.0, (0.0, 0.0, 0.0))
        result = self.monitor.evaluate(1.1)
        self.assertFalse(result["ready"])
        self.assertIn("clock", result["reason"])


if __name__ == "__main__":
    unittest.main()
