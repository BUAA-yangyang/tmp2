"""Bounded corridor probing must neither deadlock nor invent unreachability."""

import unittest

from a1_exploration.frontier import (
    corridor_gate_decision,
    corridor_probe_goal_state,
    failed_goal_state,
    record_failure,
)


class CorridorGateTest(unittest.TestCase):
    def test_missing_probe_holds_then_releases_without_completion_claim(self):
        stalls = 0
        for expected in (1, 2, 3):
            decision = corridor_gate_decision(
                True, False, True, False, stalls, 3
            )
            stalls = decision.stalls
            self.assertEqual(stalls, expected)
            self.assertTrue(decision.hold_frontiers)
            self.assertFalse(decision.released)
            self.assertNotIn("complete", decision.reason)
        released = corridor_gate_decision(
            True, False, True, False, stalls, 3
        )
        self.assertFalse(released.hold_frontiers)
        self.assertTrue(released.released)
        self.assertNotIn("complete", released.reason)

    def test_successful_probe_resets_consecutive_stalls(self):
        decision = corridor_gate_decision(
            True, False, True, True, 9, 3
        )
        self.assertEqual(decision.stalls, 0)
        self.assertFalse(decision.hold_frontiers)
        self.assertFalse(decision.released)

    def test_disabled_exhausted_or_missing_entry_never_holds_frontiers(self):
        for enabled, exhausted, has_entry in (
            (False, False, True),
            (True, True, True),
            (True, False, False),
        ):
            decision = corridor_gate_decision(
                enabled, exhausted, has_entry, False, 2, 3
            )
            self.assertFalse(decision.hold_frontiers)
            self.assertFalse(decision.released)
            self.assertEqual(decision.stalls, 2)

    def test_repeated_transient_probe_failures_are_probe_only_budget(self):
        entries = []
        for now in (0.0, 10.0):
            record_failure(
                entries, 1.0, 2.0, 0.75, now, 4.0,
                kind="transient",
            )
        self.assertEqual(
            corridor_probe_goal_state(
                entries, 1.0, 2.0, 0.75, 100.0, 2
            ),
            "attempt_budget",
        )
        self.assertEqual(
            failed_goal_state(entries, 1.0, 2.0, 0.75, 100.0, 2),
            "available",
            "normal frontier selection must not inherit probe-only retirement",
        )

    def test_probe_respects_cooldown_before_attempt_budget(self):
        entries = []
        record_failure(
            entries, 1.0, 2.0, 0.75, 10.0, 4.0,
            kind="transient",
        )
        record_failure(
            entries, 1.0, 2.0, 0.75, 11.0, 4.0,
            kind="transient",
        )
        self.assertEqual(
            corridor_probe_goal_state(
                entries, 1.0, 2.0, 0.75, 12.0, 2
            ),
            "cooldown",
        )

    def test_actual_unreachable_probe_stays_permanent(self):
        entries = []
        for now in (0.0, 10.0):
            record_failure(
                entries, 1.0, 2.0, 0.75, now, 4.0,
                kind="unreachable",
            )
        self.assertEqual(
            corridor_probe_goal_state(
                entries, 1.0, 2.0, 0.75, 100.0, 2
            ),
            "permanent",
        )

    def test_invalid_limits_fail_closed(self):
        with self.assertRaises(ValueError):
            corridor_gate_decision(True, False, True, False, -1, 3)
        with self.assertRaises(ValueError):
            corridor_gate_decision(True, False, True, False, 0, -1)
        with self.assertRaises(ValueError):
            corridor_probe_goal_state([], 0.0, 0.0, 0.75, 0.0, 0)


if __name__ == "__main__":
    unittest.main()
