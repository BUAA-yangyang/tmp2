#!/usr/bin/env python3
"""B2 regression: gate the next navigation goal on a real zero-velocity
settle after any non-success outcome (aborted, cancelled, preempted, a
bounded-backout recovery retry, or a timeout).

Evidence from a real run: move_base ABORTED at t=141.340, and a new
MoveBaseGoal was already sent at t=141.414 -- 74 ms later, while the robot
was still decelerating. `interstitial_zero_gate` is the pure, importable
decision rule the node polls (on ROS/sim time only) before selecting or
sending the next goal. It reuses FinalZeroMonitor rather than a second
zero-detector.

/cmd_vel is published unconditionally at 50 Hz by cmd_vel_guard, so it must
be fresh, zero, and settled. /cmd_vel_nav is move_base's raw local-planner
output and legitimately STOPS publishing once a goal is cancelled/aborted,
so it is satisfied by EITHER a fresh settled zero OR having gone silent
longer than the twist_mux (0.5 s) + cmd_vel_guard (0.7 s) = 1.2 s timeout
budget.
"""
import time
import unittest

from a1_exploration.final_zero import (
    CMD_VEL_NAV_SILENCE_TIMEOUT_S,
    FinalZeroMonitor,
    cmd_vel_nav_satisfied,
    interstitial_zero_gate,
)

EPSILON = 0.01
FRESHNESS_S = 0.25
SETTLE_S = 0.50
ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
MOVING = (0.2, 0.0, 0.0, 0.0, 0.0, 0.0)


def monitor():
    return FinalZeroMonitor(EPSILON, FRESHNESS_S, SETTLE_S)


def never_observed_nav():
    """A /cmd_vel_nav monitor that has never received a message -- the
    common case once a goal is cancelled/aborted and move_base stops
    publishing altogether. Silence (even total silence) satisfies it."""
    return monitor()


class SettleDurationGatesTheDecision(unittest.TestCase):
    """settled < 0.5 s must not proceed; >= 0.5 s must proceed."""

    def test_settled_under_half_second_is_not_allowed(self):
        cmd_vel = monitor()
        cmd_vel.observe(100.0, MOVING)
        cmd_vel.observe(100.05, ZERO)
        cmd_vel.observe(100.20, ZERO)
        now = 100.30  # fresh (age 0.10s), but only 0.30s since last motion
        nav = never_observed_nav()
        result = interstitial_zero_gate(
            cmd_vel.evaluate(now), nav.evaluate(now)
        )
        self.assertFalse(result["allowed"])
        self.assertFalse(result["fail_closed"])

    def test_settled_at_least_half_second_is_allowed(self):
        cmd_vel = monitor()
        cmd_vel.observe(100.0, MOVING)
        cmd_vel.observe(100.05, ZERO)
        cmd_vel.observe(100.50, ZERO)
        now = 100.55  # fresh (age 0.05s), 0.55s since last motion
        nav = never_observed_nav()
        result = interstitial_zero_gate(
            cmd_vel.evaluate(now), nav.evaluate(now)
        )
        self.assertTrue(result["allowed"])


class CmdVelNavSilenceIsAcceptable(unittest.TestCase):
    def test_silent_beyond_budget_is_satisfied(self):
        nav = monitor()
        nav.observe(100.0, MOVING)
        nav.observe(100.05, ZERO)
        now = 100.05 + CMD_VEL_NAV_SILENCE_TIMEOUT_S + 0.01
        result = nav.evaluate(now)
        self.assertFalse(result["ready"])  # no longer fresh
        self.assertTrue(cmd_vel_nav_satisfied(result))

    def test_stale_but_inside_budget_and_nonzero_is_not_satisfied(self):
        nav = monitor()
        nav.observe(100.0, MOVING)
        now = 100.0 + CMD_VEL_NAV_SILENCE_TIMEOUT_S - 0.05
        result = nav.evaluate(now)
        self.assertFalse(result["ready"])
        self.assertFalse(cmd_vel_nav_satisfied(result))

    def test_never_received_counts_as_silent(self):
        nav = monitor()
        result = nav.evaluate(100.0)
        self.assertEqual(result["reason"], "no command observed")
        self.assertTrue(cmd_vel_nav_satisfied(result))

    def test_gate_passes_the_real_fixed_case(self):
        # The real run: /cmd_vel settled and fresh; /cmd_vel_nav had gone
        # silent 1.854s ago because move_base stopped publishing after the
        # goal was aborted/cancelled.
        cmd_vel = monitor()
        cmd_vel.observe(98.146, MOVING)
        cmd_vel.observe(98.20, ZERO)
        cmd_vel.observe(100.00, ZERO)
        nav = monitor()
        nav.observe(98.06, MOVING)
        nav.observe(98.16, ZERO)
        now = 100.014
        result = interstitial_zero_gate(
            cmd_vel.evaluate(now), nav.evaluate(now)
        )
        self.assertTrue(result["allowed"])


class CmdVelIsMandatory(unittest.TestCase):
    """/cmd_vel is published unconditionally: staleness or non-zero must
    never be waved through by nav-topic silence."""

    def test_stale_cmd_vel_is_never_allowed(self):
        cmd_vel = monitor()
        cmd_vel.observe(100.0, ZERO)
        now = 100.0 + FRESHNESS_S + 0.5  # stale, though plenty settled
        nav = never_observed_nav()
        result = interstitial_zero_gate(
            cmd_vel.evaluate(now), nav.evaluate(now)
        )
        self.assertFalse(result["allowed"])

    def test_nonzero_cmd_vel_is_never_allowed(self):
        cmd_vel = monitor()
        cmd_vel.observe(100.0, MOVING)
        now = 100.01
        nav = never_observed_nav()
        result = interstitial_zero_gate(
            cmd_vel.evaluate(now), nav.evaluate(now)
        )
        self.assertFalse(result["allowed"])


class ClockRollbackFailsClosed(unittest.TestCase):
    def test_cmd_vel_clock_rollback_fails_closed(self):
        cmd_vel = monitor()
        cmd_vel.observe(100.0, ZERO)
        cmd_vel.observe(99.9, ZERO)  # rollback
        nav = never_observed_nav()
        result = interstitial_zero_gate(
            cmd_vel.evaluate(100.5), nav.evaluate(100.5)
        )
        self.assertFalse(result["allowed"])
        self.assertTrue(result["fail_closed"])

    def test_cmd_vel_nav_clock_rollback_fails_closed(self):
        cmd_vel = monitor()
        cmd_vel.observe(100.0, MOVING)
        cmd_vel.observe(100.05, ZERO)
        cmd_vel.observe(100.50, ZERO)
        nav = monitor()
        nav.observe(100.0, ZERO)
        nav.observe(99.9, ZERO)  # rollback
        result = interstitial_zero_gate(
            cmd_vel.evaluate(100.55), nav.evaluate(100.55)
        )
        self.assertFalse(result["allowed"])
        self.assertTrue(result["fail_closed"])

    def test_missing_cmd_vel_fails_closed(self):
        # The mandatory /cmd_vel heartbeat has never arrived at all.
        cmd_vel = monitor()
        nav = never_observed_nav()
        result = interstitial_zero_gate(
            cmd_vel.evaluate(100.0), nav.evaluate(100.0)
        )
        self.assertFalse(result["allowed"])
        self.assertTrue(result["fail_closed"])


class DecisionIsSimTimeOnly(unittest.TestCase):
    def test_wall_clock_delay_does_not_change_the_verdict(self):
        cmd_vel = monitor()
        cmd_vel.observe(100.0, MOVING)
        cmd_vel.observe(100.05, ZERO)
        cmd_vel.observe(100.50, ZERO)
        nav = never_observed_nav()
        now = 100.55
        first = interstitial_zero_gate(
            cmd_vel.evaluate(now), nav.evaluate(now)
        )
        time.sleep(0.05)
        second = interstitial_zero_gate(
            cmd_vel.evaluate(now), nav.evaluate(now)
        )
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
