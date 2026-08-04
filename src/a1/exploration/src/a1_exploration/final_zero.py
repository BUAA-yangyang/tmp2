"""ROS-independent final-command settling monitor.

The caller supplies ROS/simulation timestamps. Wall time may bound how long a
test waits, but it never proves that the simulated robot has been stopped long
enough.
"""

import math


class FinalZeroMonitor:
    def __init__(self, epsilon, freshness_s, settle_s, component_count=6):
        values = (epsilon, freshness_s, settle_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("final-zero parameters must be finite")
        if epsilon < 0.0 or freshness_s <= 0.0 or settle_s <= 0.0:
            raise ValueError("invalid final-zero parameters")
        if (
            isinstance(component_count, bool)
            or not isinstance(component_count, int)
            or component_count < 1
        ):
            raise ValueError("component_count must be a positive integer")
        self.epsilon = epsilon
        self.freshness_s = freshness_s
        self.settle_s = settle_s
        self.component_count = component_count
        self.reset()

    def reset(self):
        self.last_message_stamp = None
        self.last_nonzero_stamp = None
        self.zero_epoch_stamp = None
        self.last_values = (float("nan"),) * self.component_count
        self.clock_valid = True

    def observe(self, stamp_s, values):
        values = tuple(float(value) for value in values)
        if (
            not math.isfinite(stamp_s)
            or len(values) != self.component_count
            or not all(math.isfinite(value) for value in values)
        ):
            self.clock_valid = False
            return
        if (
            self.last_message_stamp is not None
            and stamp_s < self.last_message_stamp
        ):
            self.clock_valid = False
        self.last_message_stamp = stamp_s
        self.last_values = values
        if any(abs(value) > self.epsilon for value in values):
            self.last_nonzero_stamp = stamp_s
            self.zero_epoch_stamp = None
        elif self.zero_epoch_stamp is None:
            # If no non-zero message was observed in this action, require a
            # full settling interval beginning with the first fresh zero.
            self.zero_epoch_stamp = stamp_s

    def evaluate(self, now_s):
        result = {
            "ready": False,
            "reason": "no command observed",
            "message_age": float("inf"),
            "zero_duration": 0.0,
            "values": self.last_values,
        }
        if not math.isfinite(now_s) or not self.clock_valid:
            result["reason"] = "ROS clock invalid or moved backwards"
            return result
        if self.last_message_stamp is None:
            return result
        if (
            now_s < self.last_message_stamp
            or (
                self.last_nonzero_stamp is not None
                and now_s < self.last_nonzero_stamp
            )
            or (
                self.zero_epoch_stamp is not None
                and now_s < self.zero_epoch_stamp
            )
        ):
            self.clock_valid = False
            result["reason"] = "ROS clock moved backwards"
            return result

        result["message_age"] = now_s - self.last_message_stamp
        if result["message_age"] > self.freshness_s:
            result["reason"] = "command message is stale"
            return result
        if any(abs(value) > self.epsilon for value in self.last_values):
            result["reason"] = "latest command is non-zero"
            return result
        if self.zero_epoch_stamp is None:
            result["reason"] = "zero epoch not established"
            return result
        settle_reference = (
            self.last_nonzero_stamp
            if self.last_nonzero_stamp is not None
            else self.zero_epoch_stamp
        )
        result["zero_duration"] = now_s - settle_reference
        if result["zero_duration"] < self.settle_s:
            result["reason"] = "zero command has not settled long enough"
            return result
        result["ready"] = True
        result["reason"] = "fresh command and zero settling interval satisfied"
        return result


# B2: after any non-success navigation outcome (aborted, cancelled,
# preempted, a bounded-backout recovery retry, or a timeout) the
# exploration node must not select or send the next goal until commanded
# velocity has settled at zero. /cmd_vel is published unconditionally at
# 50 Hz by cmd_vel_guard, so it is mandatory: it must stay fresh, zero, and
# settled (FinalZeroMonitor.evaluate()["ready"]). /cmd_vel_nav is
# move_base's raw local-planner output and legitimately STOPS publishing
# once a goal is cancelled/aborted -- twist_mux's navigation source timeout
# is 0.5 s (a1/cmd_mux/config/twist_mux.yaml) and cmd_vel_guard's
# input_timeout is 0.7 s (a1/cmd_mux/config/guard.yaml), so mux+guard need
# up to the sum of those two timeouts to notice the source went silent and
# start zeroing /cmd_vel on their own. Silence longer than this budget on
# /cmd_vel_nav is therefore equivalent evidence to an observed fresh zero,
# not a fault; requiring freshness on it unconditionally is wrong and has
# previously caused a false failure.
CMD_VEL_NAV_SILENCE_TIMEOUT_S = 0.5 + 0.7


def cmd_vel_nav_satisfied(
        cmd_vel_nav_result, silence_timeout_s=CMD_VEL_NAV_SILENCE_TIMEOUT_S):
    """Whether a FinalZeroMonitor.evaluate() result for /cmd_vel_nav permits
    proceeding to the next goal.

    Unlike /cmd_vel, /cmd_vel_nav is satisfied by EITHER a fresh settled
    zero OR the topic having gone silent longer than `silence_timeout_s`
    (including never having published at all, where message_age is +inf).
    A clock fault on this topic is never satisfied -- it fails closed.
    """
    if "clock" in cmd_vel_nav_result["reason"]:
        return False
    return (
        cmd_vel_nav_result["ready"]
        or cmd_vel_nav_result["message_age"] > silence_timeout_s
    )


def interstitial_zero_gate(
        cmd_vel_result, cmd_vel_nav_result,
        cmd_vel_nav_silence_timeout_s=CMD_VEL_NAV_SILENCE_TIMEOUT_S):
    """Pure decision: may the node select/send the next navigation goal?

    Both arguments are FinalZeroMonitor.evaluate(now_s) dicts sampled at the
    same ROS/simulation `now_s` -- never wall time. Wall time may only ever
    bound how long a caller waits for `allowed` to become True; it must
    never influence the verdict itself (see `evaluate`'s use of `now_s`).

    - /cmd_vel is mandatory (published unconditionally at 50 Hz): it must
      be fresh, zero, and settled for the full interstitial window, i.e.
      `cmd_vel_result["ready"]`.
    - /cmd_vel_nav is optional once a goal is no longer active: see
      `cmd_vel_nav_satisfied`.
    - A clock fault on either topic, or /cmd_vel never having published a
      single message ("no command observed"), fails closed: `fail_closed`
      is True and `allowed` is False, distinct from an ordinary "not yet"
      (stale/non-zero/not-settled) verdict that may still resolve on its
      own with more waiting.
    """
    cmd_vel_clock_fault = "clock" in cmd_vel_result["reason"]
    cmd_vel_nav_clock_fault = "clock" in cmd_vel_nav_result["reason"]
    cmd_vel_missing = cmd_vel_result["reason"] == "no command observed"
    fail_closed = (
        cmd_vel_clock_fault or cmd_vel_nav_clock_fault or cmd_vel_missing
    )

    cmd_vel_ok = (not cmd_vel_clock_fault) and cmd_vel_result["ready"]
    nav_ok = (not cmd_vel_nav_clock_fault) and cmd_vel_nav_satisfied(
        cmd_vel_nav_result, cmd_vel_nav_silence_timeout_s
    )
    allowed = (not fail_closed) and cmd_vel_ok and nav_ok

    if cmd_vel_clock_fault or cmd_vel_nav_clock_fault:
        reason = "ROS/simulation clock invalid or moved backwards"
    elif cmd_vel_missing:
        reason = "mandatory /cmd_vel has never published a message"
    elif not cmd_vel_ok:
        reason = (
            "/cmd_vel is not fresh/zero/settled: %s" % cmd_vel_result["reason"]
        )
    elif not nav_ok:
        reason = (
            "/cmd_vel_nav is neither a fresh settled zero nor silent past "
            "the mux+guard timeout budget"
        )
    else:
        reason = "cmd_vel fresh/zero/settled and cmd_vel_nav satisfied"

    return {
        "allowed": allowed,
        "fail_closed": fail_closed,
        "reason": reason,
        "cmd_vel": cmd_vel_result,
        "cmd_vel_nav": cmd_vel_nav_result,
    }


def wall_backstop_seconds(sim_budget_s, wall_factor, floor_s=8.0):
    """Wall-clock backstop for a wait whose real budget is in SIM seconds.

    A wall backstop must exist so a stalled `/clock` cannot hang the action
    forever, but it must scale with the configured sim budget.  A previously
    hardcoded ``min(60.0, ...)`` ceiling silently capped every such wait at 60
    real seconds: at the observed RTF of ~0.25 that is only ~15 sim seconds, so
    raising the sim budget had no effect at all and the wait died on the wall
    cap instead (measured gap 60.015 s against a 45 s sim budget).

    ``wall_factor`` is the configured "wall seconds tolerated per sim second",
    so the backstop is simply the sim budget scaled by it, never below
    ``floor_s``.
    """
    budget = float(sim_budget_s) * float(wall_factor)
    return max(float(floor_s), budget)
