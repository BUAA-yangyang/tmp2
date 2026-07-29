"""ROS-independent final-command settling monitor.

The caller supplies ROS/simulation timestamps. Wall time may bound how long a
test waits, but it never proves that the simulated robot has been stopped long
enough.
"""

import math


class FinalZeroMonitor:
    def __init__(self, epsilon, freshness_s, settle_s):
        values = (epsilon, freshness_s, settle_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("final-zero parameters must be finite")
        if epsilon < 0.0 or freshness_s <= 0.0 or settle_s <= 0.0:
            raise ValueError("invalid final-zero parameters")
        self.epsilon = epsilon
        self.freshness_s = freshness_s
        self.settle_s = settle_s
        self.reset()

    def reset(self):
        self.last_message_stamp = None
        self.last_nonzero_stamp = None
        self.zero_epoch_stamp = None
        self.last_values = (float("nan"),) * 3
        self.clock_valid = True

    def observe(self, stamp_s, values):
        values = tuple(float(value) for value in values)
        if (
            not math.isfinite(stamp_s)
            or len(values) != 3
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
