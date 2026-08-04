#!/usr/bin/env python3
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from elevator_exit import (  # noqa: E402
    apply_forward_speed_floor,
    bounded_exit_step,
    choose_corridor_side,
    known_free_run_in_grid,
)


def grid(width=20, height=20, resolution=0.1, fill=0,
         origin=(-1.0, -1.0)):
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin[0], y=origin[1]))),
        data=[fill] * (width * height),
    )


def set_cell(message, x, y, value):
    info = message.info
    column = int(math.floor((x - info.origin.position.x) / info.resolution))
    row = int(math.floor((y - info.origin.position.y) / info.resolution))
    message.data[row * info.width + column] = value


class ElevatorExitGeometryTest(unittest.TestCase):
    def test_known_free_run_reaches_requested_bound(self):
        message = grid()
        run = known_free_run_in_grid(
            message, 0.0, 0.0, 0.0, 0.8, 65, half_width=0.2)
        self.assertAlmostEqual(run, 0.8)

    def test_unknown_and_occupied_cells_stop_the_run(self):
        unknown = grid()
        set_cell(unknown, 0.45, 0.0, -1)
        self.assertLess(known_free_run_in_grid(
            unknown, 0.0, 0.0, 0.0, 0.8, 65), 0.5)
        occupied = grid()
        set_cell(occupied, 0.55, 0.0, 80)
        self.assertLess(known_free_run_in_grid(
            occupied, 0.0, 0.0, 0.0, 0.8, 65), 0.6)

    def test_footprint_strip_rejects_side_obstacle(self):
        message = grid()
        set_cell(message, 0.45, 0.2, 100)
        centre_only = known_free_run_in_grid(
            message, 0.0, 0.0, 0.0, 0.8, 65, half_width=0.0)
        footprint = known_free_run_in_grid(
            message, 0.0, 0.0, 0.0, 0.8, 65, half_width=0.2)
        self.assertAlmostEqual(centre_only, 0.8)
        self.assertLess(footprint, 0.5)

    def test_negative_coordinates_use_floor_not_integer_truncation(self):
        message = grid(origin=(-2.0, -2.0))
        set_cell(message, -0.55, -0.5, 100)
        run = known_free_run_in_grid(
            message, -1.0, -0.5, 0.0, 0.8, 65)
        self.assertLess(run, 0.5)

    def test_bounded_step_waits_for_margin_and_caps_distance(self):
        self.assertEqual(bounded_exit_step(1.2, 0.75, 0.8, 0.65, 0.15), 0.0)
        self.assertAlmostEqual(
            bounded_exit_step(1.2, 0.95, 0.8, 0.65, 0.15), 0.8)
        self.assertAlmostEqual(
            bounded_exit_step(0.2, 0.85, 0.8, 0.65, 0.15), 0.65)

    def test_return_specific_minimum_uses_only_proven_strip(self):
        # mf33 failure geometry: 1.50 m known-free, with 0.35 m held back.
        self.assertEqual(
            bounded_exit_step(2.166, 1.50, 2.40, 1.40, 0.35), 0.0)
        self.assertAlmostEqual(
            bounded_exit_step(2.166, 1.50, 2.40, 0.65, 0.35), 1.15)

    def test_forward_speed_floor_only_boosts_slow_straight_motion(self):
        self.assertEqual(
            apply_forward_speed_floor(0.38, 0.05, 0.55, 0.25),
            (0.55, True))
        self.assertEqual(
            apply_forward_speed_floor(0.60, 0.05, 0.55, 0.25),
            (0.60, False))
        self.assertEqual(
            apply_forward_speed_floor(-0.20, 0.05, 0.55, 0.25),
            (-0.20, False))
        self.assertEqual(
            apply_forward_speed_floor(0.38, 0.40, 0.55, 0.25),
            (0.38, False))
        self.assertEqual(
            apply_forward_speed_floor(0.0, 0.0, 0.55, 0.25),
            (0.0, False))

    def test_corridor_choice_requires_run_and_non_ambiguous_advantage(self):
        self.assertIsNone(choose_corridor_side(1.8, 1.9, 2.0, 0.4))
        self.assertEqual(choose_corridor_side(2.4, 1.8, 2.0, 0.4), "left")
        self.assertEqual(choose_corridor_side(1.7, 2.5, 2.0, 0.4), "right")
        self.assertIsNone(choose_corridor_side(2.4, 2.6, 2.0, 0.4))


if __name__ == "__main__":
    unittest.main()
