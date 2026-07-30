#!/usr/bin/env python3
import unittest

from a1_exploration.entry_speed_limit import (
    EntrySpeedLimitError,
    EntrySpeedLimiter,
)


class Parameter:
    def __init__(self, name="", value=0.0):
        self.name = name
        self.value = value


class Config:
    def __init__(self, values=None):
        self.doubles = [
            Parameter(name, value) for name, value in (values or {}).items()
        ]


class Request:
    def __init__(self):
        self.config = Config()


class Response:
    def __init__(self, values):
        self.config = Config(values)


class FakeDynamicReconfigure:
    def __init__(self, values):
        self.values = dict(values)
        self.requests = []
        self.omit_on_call = {}
        self.fail_on_call = {}

    def __call__(self, request):
        call = len(self.requests)
        requested = {
            parameter.name: parameter.value
            for parameter in request.config.doubles
        }
        self.requests.append(requested)
        if call in self.fail_on_call:
            raise RuntimeError(self.fail_on_call[call])
        self.values.update(requested)
        response = dict(self.values)
        response.pop(self.omit_on_call.get(call, ""), None)
        return Response(response)


LIVE = {
    "max_vel_x": 0.20,
    "max_vel_y": 0.0,
    "max_vel_trans": 0.20,
    "max_vel_theta": 0.30,
    "min_vel_x": 0.08,
    "min_vel_trans": 0.08,
    "min_vel_theta": 0.25,
    "sim_time": 2.0,
}
LIMITS = {
    "max_vel_x": 0.15,
    "max_vel_y": 0.0,
    "max_vel_trans": 0.15,
    "max_vel_theta": 0.01,
    "min_vel_x": 0.15,
    "min_vel_trans": 0.15,
    "min_vel_theta": 0.0,
    "sim_time": 0.5,
}


def limiter(service, limits=None):
    return EntrySpeedLimiter(
        service,
        Request,
        Parameter,
        LIMITS if limits is None else limits,
    )


class EntrySpeedLimiterTest(unittest.TestCase):
    def test_apply_queries_live_config_and_restore_is_exact(self):
        service = FakeDynamicReconfigure(LIVE)
        subject = limiter(service)
        subject.apply()
        self.assertTrue(subject.active)
        self.assertEqual(service.requests[0], {})
        self.assertEqual(service.requests[1], LIMITS)
        self.assertEqual(service.values, LIMITS)
        subject.restore()
        self.assertFalse(subject.active)
        self.assertEqual(service.requests[2], LIVE)
        self.assertEqual(service.values, LIVE)

    def test_limit_may_not_increase_live_maximum(self):
        service = FakeDynamicReconfigure(LIVE)
        unsafe = dict(LIMITS)
        unsafe["max_vel_theta"] = 0.31
        with self.assertRaisesRegex(
                EntrySpeedLimitError, "must not increase"):
            limiter(service, unsafe).apply()
        self.assertEqual(service.requests, [{}])
        self.assertEqual(service.values, LIVE)

    def test_minimum_may_increase_to_avoid_unstable_sub_gait_commands(self):
        service = FakeDynamicReconfigure(LIVE)
        subject = limiter(service)
        subject.apply()
        self.assertEqual(service.values["min_vel_trans"], 0.15)

    def test_missing_live_parameter_fails_before_write(self):
        service = FakeDynamicReconfigure(LIVE)
        service.omit_on_call[0] = "max_vel_trans"
        with self.assertRaisesRegex(EntrySpeedLimitError, "missing"):
            limiter(service).apply()
        self.assertEqual(service.requests, [{}])
        self.assertEqual(service.values, LIVE)

    def test_apply_verification_failure_rolls_back(self):
        service = FakeDynamicReconfigure(LIVE)
        service.omit_on_call[1] = "max_vel_x"
        with self.assertRaisesRegex(EntrySpeedLimitError, "rolled back"):
            limiter(service).apply()
        self.assertEqual(service.requests[2], LIVE)
        self.assertEqual(service.values, LIVE)

    def test_apply_and_rollback_failure_reports_both(self):
        service = FakeDynamicReconfigure(LIVE)
        service.fail_on_call[1] = "apply transport"
        service.fail_on_call[2] = "rollback transport"
        with self.assertRaisesRegex(EntrySpeedLimitError, "rollback failed"):
            limiter(service).apply()

    def test_restore_failure_keeps_profile_active_for_fail_closed_retry(self):
        service = FakeDynamicReconfigure(LIVE)
        subject = limiter(service)
        subject.apply()
        service.fail_on_call[2] = "restore transport"
        with self.assertRaisesRegex(EntrySpeedLimitError, "restore failed"):
            subject.restore()
        self.assertTrue(subject.active)
        self.assertEqual(service.values, LIMITS)

    def test_invalid_nonfinite_or_zero_motion_limit_is_rejected(self):
        invalid = dict(LIMITS)
        invalid["max_vel_x"] = float("nan")
        with self.assertRaises(EntrySpeedLimitError):
            limiter(FakeDynamicReconfigure(LIVE), invalid)

    def test_minimum_speed_may_not_exceed_corresponding_maximum(self):
        invalid = dict(LIMITS)
        invalid["min_vel_trans"] = 0.16
        with self.assertRaisesRegex(
                EntrySpeedLimitError, "min_vel_trans"):
            limiter(FakeDynamicReconfigure(LIVE), invalid)
        invalid = dict(LIMITS)
        invalid["min_vel_theta"] = 0.02
        with self.assertRaisesRegex(
                EntrySpeedLimitError, "min_vel_theta"):
            limiter(FakeDynamicReconfigure(LIVE), invalid)
        invalid = dict(LIMITS)
        invalid["max_vel_trans"] = 0.0
        with self.assertRaises(EntrySpeedLimitError):
            limiter(FakeDynamicReconfigure(LIVE), invalid)


if __name__ == "__main__":
    unittest.main()
