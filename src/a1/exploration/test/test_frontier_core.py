#!/usr/bin/env python3
import math
import unittest

import numpy as np

from a1_exploration.frontier import (
    FailedGoal,
    GridSpec,
    NoFrontierEvidence,
    coverage_ratio,
    dominant_axis_correction,
    extract_frontiers,
    map_margin_mask,
    failed_goal_state,
    has_pending_retry,
    known_cell_count,
    known_free_path_exists,
    local_plan_is_acceptable,
    nearest_known_free_anchor,
    occupancy_content_fingerprint,
    point_in_polygon,
    point_in_start_aligned_scope,
    polygon_mask,
    record_failure,
    segment_corridor_mask,
    start_aligned_scope_mask,
    transform_local_polygon,
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

    def test_entry_relative_concave_roi_rotates_and_keeps_narrow_entrance(self):
        local = (
            (-2.0, -0.5),
            (0.0, -0.5),
            (0.0, -2.0),
            (4.0, -2.0),
            (4.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.5),
            (-2.0, 0.5),
        )
        world = transform_local_polygon(
            local, anchor_xy=(1.0, -1.0), anchor_yaw=math.pi / 2.0
        )
        self.assertTrue(point_in_polygon(1.0, -2.5, world))
        self.assertFalse(point_in_polygon(2.0, -2.5, world))
        self.assertTrue(point_in_polygon(0.0, 2.0, world))

        mask = polygon_mask(self.spec, world)
        self.assertTrue(mask[self.spec.world_to_cell(1.0, -2.5)])
        self.assertFalse(mask[self.spec.world_to_cell(2.0, -2.5)])
        self.assertTrue(mask[self.spec.world_to_cell(0.0, 2.0)])

    def test_roi_boundary_margin_and_invalid_polygon_fail_closed(self):
        polygon = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
        self.assertTrue(point_in_polygon(0.0, 0.0, polygon, 0.2))
        self.assertFalse(point_in_polygon(0.9, 0.0, polygon, 0.2))
        with self.assertRaises(ValueError):
            polygon_mask(self.spec, ((0.0, 0.0), (1.0, 0.0)))
        with self.assertRaises(ValueError):
            polygon_mask(
                self.spec,
                ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)),
            )

    def test_near_frontier_goal_is_rejected_by_effective_nav_distance(self):
        grid = np.full((30, 30), -1, dtype=np.int8)
        grid[12:18, 12:18] = 0
        loose = extract_frontiers(
            grid.ravel(),
            self.spec,
            robot_xy=(0.0, 0.0),
            min_frontier_length_m=0.2,
            obstacle_clearance_m=0.1,
            goal_standoff_m=0.35,
            minimum_goal_distance_m=0.1,
        )
        self.assertTrue(loose)
        strict = extract_frontiers(
            grid.ravel(),
            self.spec,
            robot_xy=(0.0, 0.0),
            min_frontier_length_m=0.2,
            obstacle_clearance_m=0.1,
            goal_standoff_m=0.35,
            minimum_goal_distance_m=0.70,
        )
        self.assertTrue(
            all(item.distance_m >= 0.70 for item in strict),
            "goal distance must include standoff plus move_base xy tolerance",
        )

    def test_map_fingerprint_ignores_headers_and_tracks_content(self):
        grid = np.full((30, 30), -1, dtype=np.int8)
        first = occupancy_content_fingerprint(grid.ravel(), self.spec)
        second = occupancy_content_fingerprint(grid.ravel(), self.spec)
        self.assertEqual(first, second)
        grid[10, 10] = 0
        self.assertNotEqual(
            first, occupancy_content_fingerprint(grid.ravel(), self.spec)
        )

    def test_no_frontier_evidence_does_not_count_repeated_content(self):
        evidence = NoFrontierEvidence(distinct_required=3, stable_duration=2.0)
        version = ("same-content",)
        first = evidence.observe(version, 10.0)
        second = evidence.observe(version, 10.5)
        self.assertFalse(first["complete"])
        self.assertFalse(second["complete"])
        self.assertEqual(second["count"], 1)
        stable = evidence.observe(version, 12.1)
        self.assertTrue(stable["complete"])
        self.assertIn("unchanged map content", stable["reason"])

        rollback = evidence.observe(version, 11.0)
        self.assertFalse(rollback["complete"])
        self.assertEqual(rollback["count"], 0)

    def test_retry_cooldown_remains_pending_without_current_frontier(self):
        failures = [FailedGoal(1.0, 2.0, failures=1, retry_after=14.0)]
        self.assertTrue(has_pending_retry(failures, 13.0, 2))
        self.assertFalse(has_pending_retry(failures, 15.0, 2))
        failures[0].failures = 2
        self.assertFalse(has_pending_retry(failures, 13.0, 2))

    def test_entry_passage_uses_only_known_free_grid_connectivity(self):
        spec = GridSpec(12, 8, 0.5, -3.0, -2.0)
        grid = np.full((8, 12), -1, dtype=np.int8)
        grid[3:5, 1:10] = 0
        grid[3:5, 6] = 100
        self.assertFalse(
            known_free_path_exists(
                grid.ravel(), spec, (-2.0, 0.0), (1.5, 0.0)
            )
        )
        before_known = known_cell_count(grid.ravel())
        grid[3:5, 6] = 0
        self.assertTrue(
            known_free_path_exists(
                grid.ravel(), spec, (-2.0, 0.0), (1.5, 0.0)
            )
        )
        self.assertGreaterEqual(known_cell_count(grid.ravel()), before_known)

    def test_entry_anchor_uses_nearest_local_known_free_cell(self):
        spec = GridSpec(9, 9, 0.2, -0.9, -0.9)
        grid = np.full((9, 9), -1, dtype=np.int8)
        anchor = (0.0, 0.0)
        center = spec.world_to_cell(*anchor)
        grid[center] = -1
        grid[center[0], center[1] + 1] = 0
        self.assertEqual(
            nearest_known_free_anchor(
                grid.ravel(), spec, anchor, search_radius_m=0.30
            ),
            spec.cell_to_world((center[0], center[1] + 1)),
        )

    def test_entry_anchor_fails_closed_without_local_free_evidence(self):
        spec = GridSpec(9, 9, 0.2, -0.9, -0.9)
        grid = np.full((9, 9), -1, dtype=np.int8)
        self.assertIsNone(
            nearest_known_free_anchor(
                grid.ravel(), spec, (0.0, 0.0), search_radius_m=0.30
            )
        )
        grid[0, 0] = 0
        self.assertIsNone(
            nearest_known_free_anchor(
                grid.ravel(), spec, (0.0, 0.0), search_radius_m=0.30
            )
        )

    def test_entry_anchor_is_deterministic_and_respects_allowed_mask(self):
        spec = GridSpec(7, 7, 1.0, -3.5, -3.5)
        grid = np.full((7, 7), -1, dtype=np.int8)
        grid[3, 2] = 0
        grid[3, 4] = 0
        anchor = (0.0, 0.0)
        self.assertEqual(
            nearest_known_free_anchor(
                grid.ravel(), spec, anchor, search_radius_m=1.1
            ),
            spec.cell_to_world((3, 2)),
        )
        allowed = np.ones(grid.shape, dtype=bool)
        allowed[3, 2] = False
        self.assertEqual(
            nearest_known_free_anchor(
                grid.ravel(),
                spec,
                anchor,
                search_radius_m=1.1,
                allowed_mask=allowed,
            ),
            spec.cell_to_world((3, 4)),
        )

    def test_local_anchor_does_not_open_unknown_or_cross_an_obstacle(self):
        spec = GridSpec(12, 8, 0.5, -3.0, -2.0)
        grid = np.full((8, 12), -1, dtype=np.int8)
        grid[3:5, 1:10] = 0
        grid[3:5, 6] = 100
        start = (-2.0, 0.0)
        start_cell = spec.world_to_cell(*start)
        grid[start_cell] = -1
        seed = nearest_known_free_anchor(
            grid.ravel(), spec, start, search_radius_m=0.8
        )
        self.assertIsNotNone(seed)
        self.assertFalse(
            known_free_path_exists(
                grid.ravel(), spec, seed, (1.5, 0.0)
            )
        )

    def test_entry_corridor_rejects_a_known_free_side_route(self):
        spec = GridSpec(20, 14, 0.5, -5.0, -3.5)
        grid = np.full((14, 20), -1, dtype=np.int8)
        start = (-3.5, 0.0)
        goal = (3.5, 0.0)
        start_cell = spec.world_to_cell(*start)
        goal_cell = spec.world_to_cell(*goal)
        grid[6:8, 3:18] = 0
        grid[6:8, 10] = 100
        grid[3:7, 3] = 0
        grid[3:7, 17] = 0
        grid[3, 3:18] = 0
        self.assertTrue(
            known_free_path_exists(grid.ravel(), spec, start, goal)
        )
        corridor = segment_corridor_mask(spec, start, goal, 0.75)
        self.assertTrue(corridor[start_cell])
        self.assertTrue(corridor[goal_cell])
        self.assertFalse(
            known_free_path_exists(
                grid.ravel(),
                spec,
                start,
                goal,
                allowed_mask=corridor,
            )
        )
        grid[6:8, 10] = 0
        self.assertTrue(
            known_free_path_exists(
                grid.ravel(),
                spec,
                start,
                goal,
                allowed_mask=corridor,
            )
        )

    def test_entry_plan_requires_near_direct_finite_corridor_path(self):
        start = (0.0, -3.2)
        goal = (0.0, 0.3)
        direct = [
            (0.0, -3.2),
            (0.05, -2.0),
            (-0.05, -0.8),
            (0.0, 0.3),
        ]
        limits = (0.75, 0.60, 1.50, 0.75)
        self.assertTrue(
            local_plan_is_acceptable(direct, start, goal, *limits)
        )
        self.assertFalse(
            local_plan_is_acceptable(
                [(0.0, -3.2), (-2.0, -2.0), (0.0, 0.3)],
                start,
                goal,
                *limits
            )
        )
        self.assertFalse(
            local_plan_is_acceptable(
                [(0.0, -3.2), (float("nan"), -1.0), (0.0, 0.3)],
                start,
                goal,
                *limits
            )
        )
        self.assertFalse(
            local_plan_is_acceptable(
                [(1.0, -3.2), (0.0, 0.3)],
                start,
                goal,
                *limits
            )
        )


class MapMarginMaskTest(unittest.TestCase):
    """The ROI-vs-grid margin, which mf08 hit as a hard mission abort."""

    def setUp(self):
        # 20 x 20 m at 0.5 m, origin (-10, -10): cell centers -9.75 .. 9.75.
        self.spec = GridSpec(40, 40, 0.5, -10.0, -10.0)

    def test_margin_keeps_only_cells_that_clear_every_edge(self):
        mask = map_margin_mask(self.spec, 2.0)
        self.assertTrue(mask[self.spec.world_to_cell(0.0, 0.0)])
        self.assertTrue(mask[self.spec.world_to_cell(-7.75, 7.75)])
        # 2.0 m margin excludes centers outside [-8, 8].
        self.assertFalse(mask[self.spec.world_to_cell(-8.25, 0.0)])
        self.assertFalse(mask[self.spec.world_to_cell(0.0, 9.75)])
        self.assertTrue(map_margin_mask(self.spec, 0.0).all())

    def test_zero_area_and_invalid_margins_fail_closed(self):
        self.assertFalse(map_margin_mask(self.spec, 10.0).any())
        with self.assertRaises(ValueError):
            map_margin_mask(self.spec, -1.0)
        with self.assertRaises(ValueError):
            map_margin_mask(self.spec, float("nan"))
        with self.assertRaises(ValueError):
            map_margin_mask(GridSpec(0, 40, 0.5, 0.0, 0.0), 1.0)

    def test_overhanging_roi_clips_to_the_margin_without_losing_the_floor(self):
        # An ROI whose far end overhangs the grid, as on mf08's floor 1.
        polygon = transform_local_polygon(
            ((0.0, -4.0), (30.0, -4.0), (30.0, 4.0), (0.0, 4.0)),
            (0.0, 0.0),
            0.0,
        )
        allowed = polygon_mask(self.spec, polygon)
        clipped = allowed & map_margin_mask(self.spec, 2.0)
        self.assertTrue(clipped.any())
        self.assertLess(int(clipped.sum()), int(allowed.sum()))
        # Everything up to the margin is kept; nothing beyond it survives.
        self.assertTrue(clipped[self.spec.world_to_cell(7.75, 0.0)])
        self.assertFalse(clipped[self.spec.world_to_cell(9.75, 0.0)])


class DominantAxisCorrectionTest(unittest.TestCase):
    """Re-measuring an elevator-delivered entry axis from wall directions."""

    def test_recovers_the_measured_mf08_seventeen_degree_error(self):
        declared = math.radians(107.3)
        walls = [(math.radians(90.0), 4.0), (math.radians(-90.5), 3.0)]
        correction, weight, used = dominant_axis_correction(
            declared, walls, 0.61
        )
        self.assertAlmostEqual(math.degrees(correction), -17.5, places=1)
        self.assertEqual(used, 2)
        self.assertAlmostEqual(weight, 7.0)

    def test_antiparallel_walls_fold_onto_the_same_axis(self):
        correction, _weight, used = dominant_axis_correction(
            0.0,
            [(math.radians(10.0), 1.0), (math.radians(-170.0), 1.0)],
            0.61,
        )
        self.assertEqual(used, 2)
        self.assertAlmostEqual(math.degrees(correction), 10.0, places=6)

    def test_cross_walls_are_discarded_rather_than_averaged_in(self):
        result = dominant_axis_correction(
            0.0,
            [(math.radians(88.0), 9.0), (math.radians(5.0), 1.0)],
            0.61,
        )
        self.assertIsNotNone(result)
        correction, weight, used = result
        self.assertEqual(used, 1)
        self.assertAlmostEqual(weight, 1.0)
        self.assertAlmostEqual(math.degrees(correction), 5.0, places=6)

    def test_no_usable_evidence_returns_none_so_the_caller_keeps_its_axis(self):
        self.assertIsNone(dominant_axis_correction(0.0, [], 0.61))
        self.assertIsNone(
            dominant_axis_correction(0.0, [(math.radians(45.0), 2.0)], 0.61)
        )
        self.assertIsNone(
            dominant_axis_correction(0.0, [(0.0, 0.0), (0.1, -1.0)], 0.61)
        )
        self.assertIsNone(
            dominant_axis_correction(
                0.0, [(float("nan"), 1.0)], 0.61
            )
        )

    def test_a_correction_may_never_exceed_its_bound(self):
        # Two walls straddling the bound average to a value inside it; the
        # bound is enforced per wall first, so only the admissible one counts.
        correction, _weight, used = dominant_axis_correction(
            0.0,
            [(math.radians(34.0), 1.0), (math.radians(50.0), 1.0)],
            0.61,
        )
        self.assertEqual(used, 1)
        self.assertLessEqual(abs(correction), 0.61)
        with self.assertRaises(ValueError):
            dominant_axis_correction(0.0, [(0.0, 1.0)], 0.0)
        with self.assertRaises(ValueError):
            dominant_axis_correction(0.0, [(0.0, 1.0)], 2.0)


if __name__ == "__main__":
    unittest.main()
