#!/usr/bin/env python3
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from corridor_axis import (  # noqa: E402
    estimate_corridor_axis,
    generation_is_new,
    measurement_matches_identity,
    stamped_snapshot_can_bind,
)


def wall(identifier, start, end, confidence=0.9, stable=True,
         status="observed"):
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    return SimpleNamespace(
        detection_id=identifier,
        start=SimpleNamespace(x=start[0], y=start[1]),
        end=SimpleNamespace(x=end[0], y=end[1]),
        length=length,
        confidence=confidence,
        stable=stable,
        status=status,
    )


class CorridorAxisTest(unittest.TestCase):
    def estimate(self, walls, reference=0.30):
        return estimate_corridor_axis(
            walls=walls,
            robot_xy=(0.0, 0.0),
            reference_yaw=reference,
            maximum_correction=0.65,
            parallel_tolerance=0.30,
            minimum_wall_length=0.70,
            minimum_width=1.50,
            maximum_width=3.60,
            maximum_midpoint_distance=8.0,
        )

    def test_opposite_stable_walls_correct_declared_axis(self):
        true_yaw = 0.0
        estimate = self.estimate([
            wall(11, (-3.0, 1.1), (3.0, 1.1)),
            wall(22, (3.0, -1.1), (-3.0, -1.1)),
        ])
        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.yaw, true_yaw, places=6)
        self.assertAlmostEqual(estimate.width, 2.2, places=6)
        self.assertAlmostEqual(estimate.correction, -0.30, places=6)
        self.assertEqual((estimate.left_id, estimate.right_id), (11, 22))

    def test_axis_sign_is_selected_near_declared_forward_direction(self):
        estimate = self.estimate([
            wall(1, (3.0, 1.0), (-3.0, 1.0)),
            wall(2, (-3.0, -1.0), (3.0, -1.0)),
        ], reference=0.20)
        self.assertIsNotNone(estimate)
        self.assertLess(abs(estimate.yaw), 1.0e-6)

    def test_same_side_or_perpendicular_walls_do_not_form_corridor(self):
        same_side = self.estimate([
            wall(1, (-3.0, 1.0), (3.0, 1.0)),
            wall(2, (-3.0, 2.8), (3.0, 2.8)),
        ])
        self.assertIsNone(same_side)
        perpendicular = self.estimate([
            wall(3, (1.0, -3.0), (1.0, 3.0)),
            wall(4, (-1.0, -3.0), (-1.0, 3.0)),
        ])
        self.assertIsNone(perpendicular)

    def test_unstable_and_out_of_range_segments_are_rejected(self):
        estimate = self.estimate([
            wall(1, (-3.0, 1.1), (3.0, 1.1), stable=False),
            wall(2, (-3.0, -1.1), (3.0, -1.1)),
            wall(3, (20.0, 1.1), (26.0, 1.1)),
        ])
        self.assertIsNone(estimate)

    def test_generation_and_measurement_identity_fail_closed(self):
        self.assertFalse(generation_is_new(None, "4"))
        self.assertFalse(generation_is_new("5", None))
        self.assertFalse(generation_is_new("5", ""))
        self.assertFalse(generation_is_new("4", "4"))
        self.assertTrue(generation_is_new("5", "4"))
        measurement = SimpleNamespace(
            localization_generation=5, floor_session_id=9
        )
        self.assertTrue(measurement_matches_identity(measurement, 5, 9))
        self.assertFalse(measurement_matches_identity(measurement, -1, 9))
        self.assertFalse(measurement_matches_identity(measurement, 4, 9))
        self.assertFalse(measurement_matches_identity(
            SimpleNamespace(localization_generation="", floor_session_id=9),
            5,
            9,
        ))

    def test_unlabelled_snapshot_requires_post_reset_stamp_and_identity(self):
        self.assertTrue(stamped_snapshot_can_bind(101.0, 100.0, 5, 9))
        self.assertTrue(stamped_snapshot_can_bind(100.0, 100.0, 5, 9))
        self.assertFalse(stamped_snapshot_can_bind(99.9, 100.0, 5, 9))
        self.assertFalse(stamped_snapshot_can_bind(0.0, 0.0, 5, 9))
        self.assertFalse(stamped_snapshot_can_bind(float("nan"), 100.0, 5, 9))
        self.assertFalse(stamped_snapshot_can_bind(101.0, 100.0, -1, 9))
        self.assertFalse(stamped_snapshot_can_bind(101.0, 100.0, 5, ""))


if __name__ == "__main__":
    unittest.main()
