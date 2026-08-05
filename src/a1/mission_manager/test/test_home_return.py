#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from home_return import (  # noqa: E402
    compose,
    inverse,
    nearest_known_free_anchor,
    propagate_home_transform,
    transform_pose,
    warp_occupancy_grid,
)


class HomeReturnTransformTest(unittest.TestCase):
    def assert_pose_close(self, actual, expected, places=9):
        self.assertAlmostEqual(actual[0], expected[0], places=places)
        self.assertAlmostEqual(actual[1], expected[1], places=places)
        self.assertAlmostEqual(
            math.atan2(math.sin(actual[2] - expected[2]),
                       math.cos(actual[2] - expected[2])),
            0.0,
            places=places,
        )

    def test_inverse_round_trip(self):
        transform = (2.3, -1.7, 0.83)
        self.assert_pose_close(compose(transform, inverse(transform)), (0.0, 0.0, 0.0))

    def test_elevator_transfer_preserves_physical_base_pose(self):
        home_from_source = (4.0, -2.0, 0.7)
        source_base = (1.2, 0.4, -0.3)
        target_base = (-0.2, 0.1, 0.05)
        home_from_target = propagate_home_transform(
            home_from_source, source_base, target_base)
        before = transform_pose(home_from_source, source_base)
        after = transform_pose(home_from_target, target_base)
        self.assert_pose_close(after, before)

    def test_transform_chain_returns_to_home_coordinates(self):
        home_from_current = (0.0, 0.0, 0.0)
        transfers = (
            ((2.0, 1.0, 0.2), (0.1, -0.2, 0.0)),
            ((3.0, -1.0, -0.4), (0.2, 0.1, 0.1)),
            ((-2.0, 0.5, 0.5), (0.0, 0.0, -0.2)),
        )
        expected = (0.0, 0.0, 0.0)
        for source_base, target_base in transfers:
            expected = transform_pose(home_from_current, source_base)
            home_from_current = propagate_home_transform(
                home_from_current, source_base, target_base)
            self.assert_pose_close(
                transform_pose(home_from_current, target_base), expected)


class HomeReturnGridTest(unittest.TestCase):
    def test_identity_warp_preserves_grid(self):
        data = list(range(20))
        self.assertEqual(
            warp_occupancy_grid(data, 5, 4, 1.0, -2.0, -1.0, (0.0, 0.0, 0.0)),
            data,
        )

    def test_translation_samples_source_coordinates(self):
        data = [-1] * 25
        data[2 * 5 + 3] = 100
        transformed = warp_occupancy_grid(
            data, 5, 5, 1.0, 0.0, 0.0, (1.0, 0.0, 0.0))
        self.assertEqual(transformed[2 * 5 + 2], 100)
        self.assertEqual(transformed[2 * 5 + 3], -1)

    def test_nearest_free_anchor_uses_cell_centres(self):
        data = [-1] * 25
        data[2 * 5 + 3] = 0
        anchor = nearest_known_free_anchor(
            data, 5, 5, 1.0, 0.0, 0.0, 2.5, 2.5, 1.1)
        self.assertEqual(anchor, (3.5, 2.5))

    def test_anchor_rejects_out_of_grid_pose(self):
        self.assertIsNone(nearest_known_free_anchor(
            [0] * 9, 3, 3, 1.0, 0.0, 0.0, -1.0, 0.0, 1.0))


if __name__ == "__main__":
    unittest.main()
