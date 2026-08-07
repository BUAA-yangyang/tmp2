#!/usr/bin/env python3
"""Deciding when a standing body has actually stopped turning.

Split out of multifloor_mission_node so the decision can be tested without a
simulator.  It is a decision rule, not a controller: it never commands
anything and it never refuses to answer.
"""

import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class YawQuiescence:
    """Sustained |dyaw| over a fixed sim-time lookback.

    Two failure modes it exists to prevent, both visible in mf61:

    * Calling the body still while it is mid-unwind.  On transfer 1->2 the
      estimator's 0.5 s yaw delta dips to 0.16 deg at t+0.5 s while 3.5 deg of
      unwind is still to come, so a single sample under the threshold proves
      nothing.  Quiescence has to be sustained, and it cannot start before a
      minimum hold.
    * Never calling it, which would turn a tidy-up wait into a mission failure.
      Nothing here raises or blocks; the caller owns the budget and is expected
      to proceed when it expires.

    Absence of evidence is not evidence of stillness: with no sample old enough
    to compare against, the answer is "not quiet", never "quiet".
    """

    def __init__(self, reference_max_age_s, quiescent_yaw, minimum_hold_s):
        if reference_max_age_s <= 0.0:
            raise ValueError("reference_max_age_s must be positive")
        if quiescent_yaw <= 0.0:
            raise ValueError("quiescent_yaw must be positive")
        if minimum_hold_s < 0.0:
            raise ValueError("minimum_hold_s must not be negative")
        self.reference_max_age_s = float(reference_max_age_s)
        self.quiescent_yaw = float(quiescent_yaw)
        self.minimum_hold_s = float(minimum_hold_s)
        self._samples = []
        self._start_s = None
        self._start_yaw = None
        self._quiet_since = None
        # Diagnostics the caller publishes so a bad transform can never be
        # silent: what the last window measured, and how much heading the old
        # code would have handed to the cross-generation transform unnoticed.
        self.window_yaw = None
        self.total_yaw = None

    def observe(self, now_s, yaw):
        """Feed one pose sample.  Returns True once the body counts as still."""
        now_s = float(now_s)
        yaw = float(yaw)
        if self._start_s is None:
            self._start_s = now_s
            self._start_yaw = yaw
            self._samples = [(now_s, yaw)]
            self.total_yaw = 0.0
            return False
        if now_s <= self._samples[-1][0]:
            # A repeat of the pose we already have carries no new information
            # about motion, and must not be allowed to age into a reference.
            return self.settled(self._samples[-1][0])
        self._samples.append((now_s, yaw))
        horizon = now_s - 2.0 * self.reference_max_age_s
        self._samples = [item for item in self._samples if item[0] >= horizon]
        reference = None
        for older in self._samples:
            if now_s - older[0] >= self.reference_max_age_s:
                reference = older
        if reference is None:
            self.window_yaw = None
            self._quiet_since = None
        else:
            self.window_yaw = abs(normalize_angle(yaw - reference[1]))
            if self.window_yaw <= self.quiescent_yaw:
                if self._quiet_since is None:
                    self._quiet_since = now_s
            else:
                self._quiet_since = None
        self.total_yaw = normalize_angle(yaw - self._start_yaw)
        return self.settled(now_s)

    def settled(self, now_s):
        if self._start_s is None or self._quiet_since is None:
            return False
        return (float(now_s) - self._start_s >= self.minimum_hold_s and
                float(now_s) - self._quiet_since >= self.reference_max_age_s)
