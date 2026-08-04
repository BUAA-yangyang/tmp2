"""Return-anchor substitution rules for the outdoor RECORD_START leg."""

import math
import unittest

import numpy as np

from a1_exploration.frontier import (
    GridSpec,
    nearest_known_free_anchor,
    return_anchor_selection,
)


class ReturnAnchorSelectionTest(unittest.TestCase):
    def test_accepts_anchor_strictly_inside_the_tolerance(self):
        decision = return_anchor_selection(
            (0.213, -0.162), (-0.010, -0.134), 0.40, 0.10
        )
        self.assertTrue(decision["accepted"])
        self.assertAlmostEqual(decision["offset_m"], 0.2247, places=3)
        self.assertAlmostEqual(decision["limit_m"], 0.30, places=9)

    def test_rejects_anchor_outside_the_admissible_offset(self):
        decision = return_anchor_selection(
            (0.35, -0.134), (-0.010, -0.134), 0.40, 0.10
        )
        self.assertFalse(decision["accepted"])
        self.assertGreater(decision["offset_m"], decision["limit_m"])
        self.assertIn("beyond", decision["reason"])

    def test_rejects_when_no_anchor_was_found(self):
        decision = return_anchor_selection(
            None, (-0.010, -0.134), 0.40, 0.10
        )
        self.assertFalse(decision["accepted"])
        self.assertIsNone(decision["offset_m"])

    def test_rejects_when_the_margin_consumes_the_tolerance(self):
        decision = return_anchor_selection(
            (0.0, 0.0), (0.0, 0.0), 0.40, 0.40
        )
        self.assertFalse(decision["accepted"])

    def test_substitution_can_never_exceed_the_return_tolerance(self):
        tolerance = 0.40
        margin = 0.10
        start = (-0.010, -0.134)
        for radius in np.arange(0.0, 0.61, 0.01):
            for angle in np.arange(0.0, 2.0 * math.pi, 0.35):
                anchor = (
                    start[0] + radius * math.cos(angle),
                    start[1] + radius * math.sin(angle),
                )
                decision = return_anchor_selection(
                    anchor, start, tolerance, margin
                )
                if decision["accepted"]:
                    self.assertLessEqual(
                        decision["offset_m"], tolerance - margin + 1e-12
                    )

    def test_rejects_non_finite_geometry(self):
        with self.assertRaises(ValueError):
            return_anchor_selection(
                (float("nan"), 0.0), (0.0, 0.0), 0.40, 0.10
            )
        with self.assertRaises(ValueError):
            return_anchor_selection((0.0, 0.0), (0.0, 0.0), 0.0, 0.10)


class ReturnAnchorFromOccupancyTest(unittest.TestCase):
    """The unobserved RECORD_START cell reproduced from integration_fix16."""

    def setUp(self):
        self.spec = GridSpec(
            width=40, height=40, resolution=0.075,
            origin_x=-1.5, origin_y=-1.5,
        )
        grid = np.full((self.spec.height, self.spec.width), -1, dtype=np.int16)
        # Everything forward of x = 0.2 m was raytraced free during the
        # entrance transit; the spawn cell and everything behind it never was.
        for row in range(self.spec.height):
            for col in range(self.spec.width):
                x = self.spec.origin_x + (col + 0.5) * self.spec.resolution
                if x >= 0.2:
                    grid[row, col] = 0
        self.data = grid.reshape(-1)
        self.start = (-0.010, -0.134)

    def test_record_start_cell_is_unknown(self):
        cell = self.spec.world_to_cell(*self.start)
        grid = np.asarray(self.data, dtype=np.int16).reshape(
            (self.spec.height, self.spec.width)
        )
        self.assertEqual(int(grid[cell]), -1)

    def test_nearest_free_anchor_is_admissible(self):
        anchor = nearest_known_free_anchor(
            self.data, self.spec, self.start, 0.60, 20
        )
        self.assertIsNotNone(anchor)
        decision = return_anchor_selection(anchor, self.start, 0.40, 0.10)
        self.assertTrue(decision["accepted"], decision["reason"])

    def test_no_admissible_anchor_when_the_free_band_is_far(self):
        grid = np.asarray(self.data, dtype=np.int16).reshape(
            (self.spec.height, self.spec.width)
        ).copy()
        for row in range(self.spec.height):
            for col in range(self.spec.width):
                x = self.spec.origin_x + (col + 0.5) * self.spec.resolution
                if x < 0.9:
                    grid[row, col] = -1
        anchor = nearest_known_free_anchor(
            grid.reshape(-1), self.spec, self.start, 0.60, 20
        )
        decision = return_anchor_selection(anchor, self.start, 0.40, 0.10)
        self.assertFalse(decision["accepted"])


if __name__ == "__main__":
    unittest.main()
