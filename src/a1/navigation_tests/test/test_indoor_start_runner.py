#!/usr/bin/env python3
"""Unit tests for fail-closed indoor-start runner decisions."""

import importlib.util
import json
import os
import tempfile
import unittest


SCRIPT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "scripts",
        "indoor_start_runner.py",
    )
)
SPEC = importlib.util.spec_from_file_location("indoor_start_runner", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class IndoorStartRunnerTest(unittest.TestCase):
    def write_json(self, value):
        descriptor, path = tempfile.mkstemp(suffix=".json", text=True)
        with os.fdopen(descriptor, "w") as stream:
            if isinstance(value, str):
                stream.write(value)
            else:
                json.dump(value, stream)
        self.addCleanup(lambda: os.path.exists(path) and os.unlink(path))
        return path

    def test_pause_requires_completed_startup_and_three_stable_samples(self):
        stability = MODULE.PauseStability(required=3)
        paused = "time_step: 0.002\npause: True\nsuccess: True\n"
        self.assertFalse(
            stability.observe(False, True, True, True, True, paused)
        )
        self.assertFalse(
            stability.observe(True, True, False, True, True, paused)
        )
        self.assertFalse(
            stability.observe(True, True, True, True, True, paused)
        )
        self.assertFalse(
            stability.observe(True, True, True, True, True, paused)
        )
        self.assertTrue(
            stability.observe(True, True, True, True, True, paused)
        )

    def test_unpaused_or_failed_pause_resets_stability(self):
        stability = MODULE.PauseStability(required=2)
        paused = "pause: True\n"
        self.assertFalse(
            stability.observe(True, True, True, True, True, paused)
        )
        self.assertFalse(
            stability.observe(
                True, True, True, True, True, "pause: False\n"
            )
        )
        self.assertFalse(
            stability.observe(True, True, True, True, True, paused)
        )
        self.assertTrue(
            stability.observe(True, True, True, True, True, paused)
        )
        self.assertFalse(
            stability.observe(True, True, True, True, False, paused)
        )

    def test_top_level_success_ignores_nested_success(self):
        self.assertTrue(
            MODULE.top_level_success(
                self.write_json({"success": True})
            )
        )
        self.assertFalse(
            MODULE.top_level_success(
                self.write_json(
                    {"success": False, "safe_stop": {"success": True}}
                )
            )
        )

    def test_result_fails_closed_on_missing_invalid_or_non_boolean_value(self):
        for value in (
            {"safe_stop": {"success": True}},
            {"success": 1},
            [],
            "{not valid json",
        ):
            with self.subTest(value=value):
                with self.assertRaises(MODULE.RunnerFailure):
                    MODULE.top_level_success(self.write_json(value))


if __name__ == "__main__":
    unittest.main()
