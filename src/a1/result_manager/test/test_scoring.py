"""Unit tests for the scoring/timing contract.  No ROS, no sim."""

from __future__ import annotations

import math
import unittest

from a1_result_manager.scoring import (
    ABORTED,
    NOT_STARTED,
    RUNNING,
    STOPPED,
    MissionClock,
    official_payload,
    return_verdict,
    world_anchor_from_start,
)


class MissionClockTest(unittest.TestCase):
    def test_no_time_before_the_mission_starts(self):
        clock = MissionClock()
        clock.note_node_start(100.0)
        self.assertEqual(clock.state, NOT_STARTED)
        self.assertFalse(clock.valid)
        # 120 s of startup handshake must not become exploration time.
        self.assertEqual(clock.elapsed(220.0), 0.0)
        self.assertEqual(clock.elapsed_since_node_start(220.0), 120.0)

    def test_elapsed_counts_from_start_not_from_node_construction(self):
        clock = MissionClock()
        clock.note_node_start(100.0)
        clock.start(160.0)
        self.assertEqual(clock.state, RUNNING)
        self.assertEqual(clock.elapsed(400.0), 240.0)

    def test_stop_latches_and_later_reads_do_not_grow(self):
        clock = MissionClock()
        clock.note_node_start(0.0)
        clock.start(10.0)
        clock.stop(310.0)
        self.assertEqual(clock.state, STOPPED)
        self.assertTrue(clock.valid)
        self.assertEqual(clock.elapsed(310.0), 300.0)
        # The old code kept counting until the process was killed.
        self.assertEqual(clock.elapsed(9999.0), 300.0)

    def test_a_repeated_start_cannot_reset_a_running_clock(self):
        # status_pub is latched, so a late subscriber replays the start event.
        clock = MissionClock()
        clock.start(10.0)
        self.assertFalse(clock.start(200.0))
        self.assertEqual(clock.elapsed(310.0), 300.0)

    def test_stop_without_start_is_refused(self):
        clock = MissionClock()
        self.assertFalse(clock.stop(50.0))
        self.assertEqual(clock.state, NOT_STARTED)
        self.assertFalse(clock.valid)

    def test_abort_freezes_the_time_but_marks_the_run_invalid(self):
        clock = MissionClock()
        clock.start(10.0)
        clock.abort(130.0)
        self.assertEqual(clock.state, ABORTED)
        self.assertFalse(clock.valid)
        self.assertEqual(clock.elapsed(9999.0), 120.0)

    def test_abort_before_start_reports_zero_not_a_process_lifetime(self):
        clock = MissionClock()
        clock.note_node_start(0.0)
        clock.abort(500.0)
        self.assertFalse(clock.valid)
        self.assertEqual(clock.elapsed(600.0), 0.0)

    def test_no_accumulation_across_runs(self):
        # The old node read the previous run's exploration_time back in as an
        # offset, so a second round started at the first round's total.
        first = MissionClock()
        first.start(0.0)
        first.stop(300.0)
        second = MissionClock()
        second.start(0.0)
        second.stop(100.0)
        self.assertEqual(second.elapsed(100.0), 100.0)


class WorldAnchorTest(unittest.TestCase):
    ROBOT_START = {"x": 0.0, "y": -3.2, "z": 0.6, "yaw": 1.5708}

    def test_the_robot_start_pose_maps_onto_itself(self):
        anchor = world_anchor_from_start(
            generation=3, map_pose=(0.0, 0.0, 0.3, 0.0),
            world_start=self.ROBOT_START)
        world = anchor.apply((0.0, 0.0, 0.3))
        self.assertAlmostEqual(world[0], 0.0, places=6)
        self.assertAlmostEqual(world[1], -3.2, places=6)
        self.assertAlmostEqual(world[2], 0.6, places=6)

    def test_rotation_is_applied_not_just_translation(self):
        # map yaw 0 vs world yaw pi/2: a point 1 m ahead in map is 1 m along
        # +y in world.  Translating without rotating would put it at +x and
        # every detection would be mirrored about the building diagonal.
        anchor = world_anchor_from_start(
            generation=0, map_pose=(0.0, 0.0, 0.3, 0.0),
            world_start={"x": 0.0, "y": -3.2, "z": 0.6, "yaw": math.pi / 2.0})
        world = anchor.apply((1.0, 0.0, 0.3))
        self.assertAlmostEqual(world[0], 0.0, places=6)
        self.assertAlmostEqual(world[1], -2.2, places=6)

    def test_distances_are_preserved(self):
        anchor = world_anchor_from_start(
            generation=1, map_pose=(4.0, -7.0, 0.31, 2.1),
            world_start=self.ROBOT_START)
        a = (1.0, 2.0, 0.5)
        b = (3.5, -0.5, 1.25)
        before = math.dist(a, b)
        after = math.dist(anchor.apply(a), anchor.apply(b))
        self.assertAlmostEqual(before, after, places=9)

    def test_generation_travels_with_the_anchor(self):
        anchor = world_anchor_from_start(
            generation=7, map_pose=(0.0, 0.0, 0.3, 0.0),
            world_start=self.ROBOT_START)
        self.assertEqual(anchor.generation, 7)

    def test_z_offset_lands_on_the_expected_floor_heights(self):
        # Truth spheres sit at floor_z + 0.15 -> 0.15 / 2.75 / 5.35.  The
        # anchor's z term has to carry a detection on the map's second floor to
        # roughly 2.75, not to 2.75 + spawn height.
        anchor = world_anchor_from_start(
            generation=0, map_pose=(0.0, 0.0, 0.31, 0.0),
            world_start={"x": 0.0, "y": -3.2, "z": 0.31, "yaw": 0.0})
        self.assertAlmostEqual(anchor.apply((1.0, 1.0, 2.75))[2], 2.75, places=6)


class ReturnVerdictTest(unittest.TestCase):
    def test_unmeasured_is_not_the_same_as_failed(self):
        self.assertIsNone(return_verdict(None, 1.0))
        self.assertIsNone(return_verdict(1.0, None))

    def test_within_tolerance(self):
        self.assertTrue(return_verdict(0.8, 1.0))
        self.assertTrue(return_verdict(1.0, 1.0))
        self.assertFalse(return_verdict(1.01, 1.0))


class OfficialPayloadTest(unittest.TestCase):
    def test_exactly_the_two_documented_fields(self):
        payload = official_payload(123.456, [(1.0, 2.0, 3.0)])
        self.assertEqual(set(payload), {"exploration_time",
                                        "detected_danger_sources"})

    def test_time_is_rounded_to_two_decimals_like_the_documented_sample(self):
        payload = official_payload(98.7649, [])
        self.assertEqual(payload["exploration_time"], 98.76)

    def test_each_detection_is_a_three_element_position(self):
        payload = official_payload(1.0, [(1.23456, -2.0, 0.15)])
        item = payload["detected_danger_sources"][0]
        self.assertEqual(set(item), {"position"})
        self.assertEqual(len(item["position"]), 3)
        self.assertEqual(item["position"][0], 1.235)

    def test_empty_detection_list_still_carries_the_time(self):
        payload = official_payload(42.0, [])
        self.assertEqual(payload["detected_danger_sources"], [])
        self.assertEqual(payload["exploration_time"], 42.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
