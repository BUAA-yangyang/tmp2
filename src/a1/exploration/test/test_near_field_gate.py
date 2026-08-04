#!/usr/bin/env python3
"""Regression for the fixed-seed reverse-door boundary deadlock."""

import math
import unittest

import numpy as np

from a1_exploration.frontier import GridSpec, first_near_field_blocker


class NearFieldGateTest(unittest.TestCase):
    def setUp(self):
        self.spec = GridSpec(
            width=80,
            height=80,
            resolution=0.075,
            origin_x=-3.0,
            origin_y=-3.0,
        )

    def grid_with_obstacle(self, x, y):
        grid = np.zeros((self.spec.height, self.spec.width), dtype=np.int16)
        row, col = self.spec.world_to_cell(x, y)
        grid[row, col] = 100
        return grid.reshape(-1)

    def blocker(self, data, half_width):
        return first_near_field_blocker(
            data,
            self.spec,
            body_x=0.0,
            body_y=0.0,
            yaw=0.0,
            direction=-1.0,
            distance=0.70,
            half_width=half_width,
            occupied_threshold=65,
        )

    def test_old_outer_boundary_cell_is_not_inside_reverse_footprint_band(self):
        # yaw=0 and direction=-1 maps lateral=-0.30 to y=+0.30.
        data = self.grid_with_obstacle(-0.40, 0.30)
        old = self.blocker(data, 0.30)
        bounded = self.blocker(data, 0.22)
        self.assertIsNotNone(old)
        self.assertAlmostEqual(old.lateral, -0.30, places=6)
        self.assertIsNone(bounded)

    def test_central_obstacle_still_fails_closed(self):
        data = self.grid_with_obstacle(-0.40, 0.0)
        blocker = self.blocker(data, 0.22)
        self.assertIsNotNone(blocker)
        self.assertLessEqual(abs(blocker.lateral), 0.075)

    def test_unknown_cells_keep_the_existing_explicit_obstacle_semantics(self):
        data = np.full(
            self.spec.width * self.spec.height, -1, dtype=np.int16
        )
        self.assertIsNone(self.blocker(data, 0.22))

    def test_non_finite_geometry_is_rejected(self):
        data = np.zeros(self.spec.width * self.spec.height, dtype=np.int16)
        with self.assertRaises(ValueError):
            first_near_field_blocker(
                data, self.spec, 0.0, 0.0, math.nan, -1.0,
                0.70, 0.22, 65,
            )


if __name__ == "__main__":
    unittest.main()
