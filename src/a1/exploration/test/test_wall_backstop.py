"""Regression: a sim-second wait budget must not be capped by a wall constant.

`reset_map_after_entry_door_opens()` waits a budget expressed in SIM seconds
but also carries a wall-clock backstop.  That backstop used to be
`min(60.0, max(8.0, budget * wall_factor))`.  The 60 s ceiling meant that at the
observed RTF (~0.25) the wait really only lasted ~15 sim seconds no matter how
large the configured budget was: raising `timeouts/entry_map` from 15 to 45 had
literally no effect, and the measured failure gap was 60.015 s -- the constant,
not the budget.
"""
import unittest

from a1_exploration.final_zero import wall_backstop_seconds


class WallBackstopScalesWithSimBudget(unittest.TestCase):
    def test_scales_with_budget_and_factor(self):
        self.assertEqual(wall_backstop_seconds(45.0, 5.0), 225.0)
        self.assertEqual(wall_backstop_seconds(15.0, 5.0), 75.0)

    def test_not_capped_at_sixty(self):
        # The exact regression: 45 sim s * 5 must not collapse to 60 s.
        self.assertGreater(wall_backstop_seconds(45.0, 5.0), 60.0)

    def test_raising_budget_actually_raises_backstop(self):
        self.assertGreater(
            wall_backstop_seconds(45.0, 5.0), wall_backstop_seconds(15.0, 5.0))

    def test_floor_protects_tiny_budgets(self):
        self.assertEqual(wall_backstop_seconds(0.1, 1.0), 8.0)
        self.assertEqual(wall_backstop_seconds(0.0, 5.0), 8.0)

    def test_backstop_still_finite_so_a_stalled_clock_cannot_hang(self):
        value = wall_backstop_seconds(600.0, 5.0)
        self.assertTrue(value == value and value < float("inf"))
