"""P1 regression: transient navigation failures must never become permanent.

Root cause this guards against: a frontier goal whose terminal yaw pointed into
an obstacle made move_base rotate in place, collide, run recovery and ABORT.
Those aborts were recorded like proven-unreachable results, so after
``maximum_failures`` attempts the location became permanently excluded.  Once
enough locations were excluded the explorer reported "no eligible frontier" and
returned home after exploring only the first room.
"""
import unittest

from a1_exploration.frontier import (
    FailedGoal, failed_goal_state, record_failure,
)


class TransientFailuresNeverEscalate(unittest.TestCase):
    def setUp(self):
        self.entries = []
        self.radius = 0.75
        self.cooldown = 4.0
        self.maximum = 2

    def state(self, now):
        return failed_goal_state(
            self.entries, 1.0, 1.0, self.radius, now, self.maximum)

    def test_transient_failures_never_become_permanent(self):
        for i in range(10):
            record_failure(self.entries, 1.0, 1.0, self.radius,
                           float(i), self.cooldown, kind="transient")
        self.assertNotEqual(self.state(1000.0), "permanent")
        self.assertEqual(self.state(1000.0), "available")

    def test_unreachable_failures_do_escalate(self):
        record_failure(self.entries, 1.0, 1.0, self.radius, 0.0,
                       self.cooldown, kind="unreachable")
        self.assertEqual(self.state(1000.0), "available")
        record_failure(self.entries, 1.0, 1.0, self.radius, 1.0,
                       self.cooldown, kind="unreachable")
        self.assertEqual(self.state(1000.0), "permanent")

    def test_transient_does_not_top_up_unreachable_budget(self):
        record_failure(self.entries, 1.0, 1.0, self.radius, 0.0,
                       self.cooldown, kind="unreachable")
        for i in range(5):
            record_failure(self.entries, 1.0, 1.0, self.radius,
                           float(i), self.cooldown, kind="transient")
        self.assertNotEqual(self.state(1000.0), "permanent")

    def test_transient_still_applies_retry_cooldown(self):
        record_failure(self.entries, 1.0, 1.0, self.radius, 100.0,
                       self.cooldown, kind="transient")
        self.assertEqual(self.state(101.0), "cooldown")
        self.assertEqual(self.state(105.0), "available")

    def test_default_kind_is_unreachable_backward_compatible(self):
        record_failure(self.entries, 1.0, 1.0, self.radius, 0.0, self.cooldown)
        record_failure(self.entries, 1.0, 1.0, self.radius, 1.0, self.cooldown)
        self.assertEqual(self.state(1000.0), "permanent")

    def test_failed_goal_defaults_to_zero_unreachable(self):
        self.assertEqual(
            FailedGoal(x=0.0, y=0.0, failures=3,
                       retry_after=0.0).unreachable_failures, 0)


if __name__ == "__main__":
    unittest.main()
