#!/usr/bin/env python3
import math
import unittest

import numpy as np

from a1_exploration.frontier import (
    FailedGoal,
    GridSpec,
    coverage_ratio,
    extract_frontiers,
    failed_goal_state,
    point_in_start_aligned_scope,
    record_failure,
    start_aligned_scope_mask,
)


class FrontierCoreTest(unittest.TestCase):
    def setUp(self):
        self.spec = GridSpec(
            width=30,
            height=30,
            resolution=0.2,
            origin_x=-3.0,
            origin_y=-3.0,
        )

    def test_clusters_unknown_boundary_and_places_goal_in_known_free(self):
        grid = np.full((30, 30), -1, dtype=np.int8)
        grid[8:22, 8:22] = 0
        frontiers = extract_frontiers(
            grid.ravel(),
            self.spec,
            robot_xy=(0.0, 0.0),
            min_frontier_length_m=0.4,
            obstacle_clearance_m=0.2,
            minimum_goal_distance_m=0.1,
        )
        self.assertTrue(frontiers)
        goal = frontiers[0]
        row, col = goal.goal_cell
        self.assertEqual(grid[row, col], 0)
        self.assertGreaterEqual(goal.length_m, 0.4)
        self.assertTrue(math.isfinite(goal.yaw))

        expanded = np.full((30, 30), -1, dtype=np.int8)
        expanded[4:26, 4:26] = 0
        next_goal = extract_frontiers(
            expanded.ravel(),
            self.spec,
            robot_xy=(goal.goal_x, goal.goal_y),
            min_frontier_length_m=0.4,
            obstacle_clearance_m=0.2,
            minimum_goal_distance_m=0.1,
        )[0]
        self.assertGreater(
            math.hypot(
                next_goal.goal_x - goal.goal_x,
                next_goal.goal_y - goal.goal_y,
            ),
            0.7,
            "an expanded closed frontier ring must advance the target",
        )

    def test_obstacle_clearance_removes_unsafe_frontier(self):
        grid = np.full((20, 20), -1, dtype=np.int8)
        grid[5:15, 5:15] = 0
        grid[5, 5:15] = 100
        spec = GridSpec(20, 20, 0.2, -2.0, -2.0)
        frontiers = extract_frontiers(
            grid.ravel(),
            spec,
            robot_xy=(0.0, 0.0),
            min_frontier_length_m=0.2,
            obstacle_clearance_m=0.4,
            minimum_goal_distance_m=0.1,
        )
        self.assertTrue(frontiers)
        self.assertTrue(all(item.goal_cell[0] > 5 for item in frontiers))

    def test_coverage_and_failure_lifecycle(self):
        grid = np.full((10, 10), -1, dtype=np.int8)
        grid[:5, :] = 0
        self.assertAlmostEqual(coverage_ratio(grid.ravel()), 0.5)

        failures = []
        item = record_failure(failures, 1.0, 2.0, 0.5, 10.0, 3.0)
        self.assertEqual(item.failures, 1)
        self.assertEqual(
            failed_goal_state(failures, 1.1, 2.0, 0.5, 11.0, 2),
            "cooldown",
        )
        record_failure(failures, 1.1, 2.0, 0.5, 14.0, 3.0)
        self.assertEqual(
            failed_goal_state(failures, 1.0, 2.0, 0.5, 20.0, 2),
            "permanent",
        )

    def test_start_aligned_scope_rotates_and_uses_cell_centers(self):
        spec = GridSpec(10, 10, 1.0, -5.0, -5.0)
        east = start_aligned_scope_mask(
            spec,
            start_xy=(0.0, 0.0),
            start_yaw=0.0,
            forward_distance_m=3.0,
            rear_distance_m=1.0,
            lateral_half_width_m=2.0,
        )
        self.assertTrue(east[spec.world_to_cell(2.5, 1.5)])
        self.assertFalse(east[spec.world_to_cell(-1.5, 0.5)])
        self.assertFalse(east[spec.world_to_cell(0.5, 2.5)])

        north = start_aligned_scope_mask(
            spec,
            start_xy=(0.0, 0.0),
            start_yaw=math.pi / 2.0,
            forward_distance_m=3.0,
            rear_distance_m=1.0,
            lateral_half_width_m=2.0,
        )
        self.assertTrue(north[spec.world_to_cell(-1.5, 2.5)])
        self.assertFalse(north[spec.world_to_cell(2.5, 0.5)])
        self.assertTrue(
            point_in_start_aligned_scope(
                0.0, 2.0, (0.0, 0.0), math.pi / 2.0,
                3.0, 1.0, 2.0,
            )
        )

    def test_scope_clips_frontier_goal_and_coverage(self):
        grid = np.full((30, 30), -1, dtype=np.int8)
        grid[5:25, 5:25] = 0
        allowed = start_aligned_scope_mask(
            self.spec,
            start_xy=(0.0, 0.0),
            start_yaw=0.0,
            forward_distance_m=2.0,
            rear_distance_m=1.0,
            lateral_half_width_m=1.5,
        )
        frontiers = extract_frontiers(
            grid.ravel(),
            self.spec,
            robot_xy=(0.0, 0.0),
            min_frontier_length_m=0.2,
            obstacle_clearance_m=0.2,
            minimum_goal_distance_m=0.1,
            allowed_mask=allowed,
        )
        self.assertTrue(frontiers)
        for frontier in frontiers:
            self.assertTrue(all(allowed[cell] for cell in frontier.cells))
            self.assertTrue(allowed[frontier.goal_cell])

        scoped_grid = np.full((30, 30), -1, dtype=np.int8)
        scoped_grid[allowed] = 0
        self.assertAlmostEqual(
            coverage_ratio(scoped_grid.ravel(), allowed), 1.0
        )
        scoped_grid[~allowed] = 0
        scoped_grid[allowed] = -1
        self.assertAlmostEqual(
            coverage_ratio(scoped_grid.ravel(), allowed), 0.0
        )

    def test_invalid_scope_fails_closed(self):
        with self.assertRaises(ValueError):
            start_aligned_scope_mask(
                self.spec,
                start_xy=(0.0, 0.0),
                start_yaw=0.0,
                forward_distance_m=1.0,
                rear_distance_m=0.1,
                lateral_half_width_m=1.0,
                boundary_margin_m=0.2,
            )


if __name__ == "__main__":
    unittest.main()
