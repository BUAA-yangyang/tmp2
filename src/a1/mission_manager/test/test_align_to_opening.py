#!/usr/bin/env python3
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


HERE = Path(__file__).resolve()
REPOSITORY_SCRIPTS = HERE.parents[1] / "scripts"
SCRIPTS = REPOSITORY_SCRIPTS if REPOSITORY_SCRIPTS.is_dir() else HERE.parent
sys.path.insert(0, str(SCRIPTS))

from align_to_opening import opening_bearing  # noqa: E402


def angle_error(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def car_grid(openings=(), corridor_half_width=0.70, fill=-1):
    resolution = 0.05
    width = height = 161
    origin = -0.5 * width * resolution
    message = SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin, y=origin))),
        data=[fill] * (width * height),
    )
    for row in range(height):
        y = origin + (row + 0.5) * resolution
        for column in range(width):
            x = origin + (column + 0.5) * resolution
            known = abs(x) <= 0.65 and abs(y) <= 0.65
            for bearing in openings:
                forward = x * math.cos(bearing) + y * math.sin(bearing)
                lateral = -x * math.sin(bearing) + y * math.cos(bearing)
                if (0.0 <= forward <= 3.20 and
                        abs(lateral) <= corridor_half_width):
                    known = True
            if known:
                message.data[row * width + column] = 0
    return message


class OpeningBearingTest(unittest.TestCase):
    def assertBearingAlmostEqual(self, actual, expected, tolerance=0.08):
        self.assertIsNotNone(actual)
        self.assertLess(abs(angle_error(actual, expected)), tolerance)

    def test_finds_rotated_unique_opening_without_layout_constant(self):
        expected = -0.58
        diagnostics = {}
        actual = opening_bearing(
            car_grid((expected,)), 0.0, 0.0, diagnostics=diagnostics)
        self.assertBearingAlmostEqual(actual, expected)
        self.assertTrue(diagnostics["accepted"])
        self.assertEqual(diagnostics["reason"], "unique_opening")

    def test_circular_lobe_crossing_pi_is_joined(self):
        actual = opening_bearing(car_grid((math.pi,)), 0.0, 0.0)
        self.assertBearingAlmostEqual(actual, math.pi)

    def test_two_equally_deep_openings_fail_closed(self):
        diagnostics = {}
        actual = opening_bearing(
            car_grid((0.0, math.pi)), 0.0, 0.0,
            diagnostics=diagnostics)
        self.assertIsNone(actual)
        self.assertEqual(diagnostics["reason"], "ambiguous_runner_up")

    def test_overwide_open_space_fails_closed(self):
        diagnostics = {}
        actual = opening_bearing(
            car_grid(fill=0), 0.0, 0.0, diagnostics=diagnostics)
        self.assertIsNone(actual)
        self.assertEqual(diagnostics["reason"], "no_plausible_lobe")

    def test_footprint_strip_rejects_center_ray_only_gap(self):
        message = car_grid((0.0,), corridor_half_width=0.10)
        centre_only = opening_bearing(
            message, 0.0, 0.0, half_width=0.0,
            minimum_lobe_width=0.03)
        footprint = opening_bearing(
            message, 0.0, 0.0, half_width=0.22,
            minimum_lobe_width=0.03)
        self.assertBearingAlmostEqual(centre_only, 0.0)
        self.assertIsNone(footprint)

    def test_unknown_map_does_not_invent_an_opening(self):
        self.assertIsNone(opening_bearing(car_grid(), 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
