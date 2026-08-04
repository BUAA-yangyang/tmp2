#!/usr/bin/env python3
"""Structural guards for the generation-local option-A elevator transaction."""

import ast
from pathlib import Path
import unittest


NODE = (Path(__file__).resolve().parents[1] / "scripts" /
        "multifloor_mission_node.py")


class OptionAContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = NODE.read_text()
        cls.tree = ast.parse(cls.source)
        cls.mission = next(
            node for node in cls.tree.body
            if isinstance(node, ast.ClassDef) and
            node.name == "MultiFloorMission")
        cls.methods = {
            node.name: node for node in cls.mission.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }

    def method_source(self, name):
        node = self.methods[name]
        return ast.get_source_segment(self.source, node)

    def calls_in(self, name):
        calls = []
        for node in ast.walk(self.methods[name]):
            if not isinstance(node, ast.Call):
                continue
            if isinstance(node.func, ast.Attribute):
                calls.append(node.func.attr)
            elif isinstance(node.func, ast.Name):
                calls.append(node.func.id)
        return calls

    def test_exit_does_not_wait_for_or_derive_from_upper_doorway(self):
        body = self.method_source("exit_upper_floor_without_doorway")
        self.assertNotIn("floor in self.elevator", body)
        self.assertNotIn("elevator_poses", body)
        self.assertNotIn("align_to_car_opening", body)
        self.assertNotIn("car opening measured from inside", body)

    def test_exit_uses_bounded_steps_and_two_corridor_probes(self):
        calls = self.calls_in("exit_upper_floor_without_doorway")
        self.assertIn("bounded_exit_step", calls)
        self.assertIn("choose_corridor_side", calls)
        self.assertGreaterEqual(calls.count("known_free_run"), 3)
        self.assertIn("NAVIGATION_NO_PROGRESS", self.source)

    def test_transfer_yaw_stays_provisional_until_map_alignment(self):
        transfer_body = self.method_source("transfer")
        align_body = self.method_source("align_to_upper_floor_opening")
        exit_body = self.method_source("exit_upper_floor_without_doorway")
        self.assertNotIn("self.arrival_exit_yaws[target]", transfer_body)
        self.assertIn("provisional_arrival_yaw=arrival_yaw", transfer_body)
        self.assertIn("stable_identity", align_body)
        self.assertIn("self.opening_stable_samples", align_body)
        self.assertIn("self.arrival_exit_yaws[floor] = target_yaw", align_body)
        self.assertIn("align_to_upper_floor_opening", exit_body)

    def test_opening_waits_for_decidable_footprint_before_detection_budget(self):
        body = self.method_source("align_to_upper_floor_opening")
        scan_call = body.index("self.actively_scan_upper_floor_elevator(floor)")
        readiness = body.index("readiness_deadline =")
        self.assertLess(scan_call, readiness)
        self.assertIn("self.opening_map_ready_wait_wall", body)
        self.assertIn("self.opening_map_ready_probe", body)
        self.assertIn("known_free_run_in_grid", body)
        self.assertIn("ELEVATOR_OPENING_MAP_READY", body)
        self.assertIn("detection_deadline =", body)
        self.assertIn("ready_identity != identity_after", body)

    def test_opening_active_scan_is_identity_bound_bounded_and_rotation_only(self):
        body = self.method_source("actively_scan_upper_floor_elevator")
        self.assertIn("self.current_mapping_identity()", body)
        self.assertIn("2.0 * math.pi", body)
        self.assertIn("self.angle_error(current_yaw, previous_yaw)", body)
        self.assertIn("self.opening_active_scan_timeout_wall", body)
        self.assertIn("self.opening_active_scan_timeout_sim", body)
        self.assertIn("self.opening_active_scan_no_progress_wall", body)
        self.assertIn("command.angular.z = self.opening_active_scan_speed", body)
        self.assertGreaterEqual(body.count("self.behavior_cmd_pub.publish(Twist())"),
                                1)
        self.assertNotIn("command.linear", body)
        self.assertNotIn("self.navigate", body)
        self.assertIn("ELEVATOR_OPENING_ACTIVE_SCAN_READY", body)

    def test_return_point_faces_inward_and_turn_has_no_pi_fallback(self):
        exit_body = self.method_source("exit_upper_floor_without_doorway")
        turn_body = self.method_source("turn_inside_elevator_before_transfer")
        self.assertIn("yaw_out + math.pi", exit_body)
        self.assertNotIn("target_yaw = start_yaw + math.pi", turn_body)
        self.assertIn("raise MissionFailure", turn_body)

    def test_upper_return_retraces_lobby_b_before_in_car_a(self):
        exit_body = self.method_source("exit_upper_floor_without_doorway")
        complete_body = self.method_source(
            "complete_upper_floor_and_return_to_a")
        return_body = self.method_source(
            "return_upper_floor_over_traversed_segments")
        markers_body = self.method_source("publish_markers")
        self.assertIn("ELEVATOR_EXIT_CLEAR", exit_body)
        self.assertIn("self.elevator_lobby_return_points[floor]", exit_body)
        self.assertIn("return entry, point_a, point_b", exit_body)
        self.assertIn("self.return_upper_floor_over_traversed_segments(",
                      complete_body)
        point_b_call = return_body.index("self.navigate(\n            point_b")
        point_a_call = return_body.index(
            "self.enter_upper_floor_car_from_lobby(floor, point_a, point_b)")
        self.assertLess(point_b_call, point_a_call)
        self.assertIn("self.elevator_return_points.get(self.floor)",
                      markers_body)
        self.assertIn("self.elevator_lobby_return_points.get(self.floor)",
                      markers_body)
        self.assertIn("Marker.DELETEALL", markers_body)

    def test_upper_b_to_a_entry_is_continuous_and_speed_floored(self):
        body = self.method_source("enter_upper_floor_car_from_lobby")
        self.assertIn("self.exit_minimum_distance <= ab_distance", body)
        self.assertIn("self.upper_return_heading_tolerance", body)
        self.assertIn("self.upper_transfer_safe_inset", body)
        self.assertIn("safe_point.pose.position.x", body)
        self.assertIn("inward_progress < ab_distance", body)
        self.assertIn("self.elevator_safe_return_points[floor]", body)
        self.assertIn("self.arm_car_entry_speed_floor", body)
        self.assertIn("self.navigate", body)
        self.assertNotIn("self.advance_map_checked", body)
        self.assertNotIn("self.travel_to_map_checked", body)

    def test_upper_exploration_returns_to_c_and_return_has_no_step_policy(self):
        explore_body = self.method_source("explore_floor")
        return_body = self.method_source(
            "return_upper_floor_over_traversed_segments")
        self.assertIn("goal.LEGACY_RETURN_TO_START", explore_body)
        self.assertIn("goal.STAY_ON_FLOOR if main_entrance", explore_body)
        self.assertIn("entry_error > self.nav_arrival_band", explore_body)
        self.assertNotIn("travel_to_map_checked", self.methods)
        self.assertNotIn("advance_map_checked", return_body)
        self.assertNotIn("return_minimum_goal", self.source)
        self.assertNotIn("return_handover_distance", self.source)


if __name__ == "__main__":
    unittest.main()
