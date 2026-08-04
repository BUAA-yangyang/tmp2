"""ROS-independent tests for perceived room-station midpoint bounds."""

import math
import unittest

from a1_exploration.frontier import room_axis_bounds


class RoomAxisBoundsTest(unittest.TestCase):
    def test_nearest_same_side_stations_define_lower_and_upper_midpoints(self):
        neighbours = [
            (40.0, 1.5),
            (10.0, 1.5),
            (30.0, 1.5),
            (5.0, 1.5),
        ]
        self.assertEqual(
            room_axis_bounds(20.0, 1.5, neighbours, 3.0),
            (15.0, 25.0),
        )

    def test_opposite_side_and_centreline_detections_do_not_clip(self):
        neighbours = [(10.0, -1.5), (30.0, -1.5), (40.0, 0.0)]
        self.assertEqual(
            room_axis_bounds(20.0, 1.5, neighbours, 3.0),
            (None, None),
        )

    def test_duplicate_station_observations_are_ignored(self):
        neighbours = [(20.0, 1.5), (21.0, 1.4), (30.0, 1.5)]
        self.assertEqual(
            room_axis_bounds(20.0, 1.5, neighbours, 3.0),
            (None, 25.0),
        )

    def test_order_does_not_change_bounds(self):
        forward = [(10.0, -1.5), (30.0, -1.5), (40.0, -1.5)]
        reverse = list(reversed(forward))
        self.assertEqual(
            room_axis_bounds(20.0, -1.5, forward, 3.0),
            room_axis_bounds(20.0, -1.5, reverse, 3.0),
        )

    def test_missing_neighbour_keeps_that_axis_bound_open(self):
        self.assertEqual(
            room_axis_bounds(20.0, 1.5, [(10.0, 1.5)], 3.0),
            (15.0, None),
        )

    def test_invalid_geometry_fails_closed(self):
        with self.assertRaises(ValueError):
            room_axis_bounds(20.0, 0.0, [], 3.0)
        with self.assertRaises(ValueError):
            room_axis_bounds(20.0, 1.5, [(math.nan, 1.5)], 3.0)
        with self.assertRaises(ValueError):
            room_axis_bounds(20.0, 1.5, [], -1.0)


if __name__ == "__main__":
    unittest.main()
