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

    def test_upper_exploration_holds_endpoint_and_mission_returns_via_c(self):
        explore_body = self.method_source("explore_floor")
        complete_body = self.method_source(
            "complete_upper_floor_and_return_to_a")
        return_body = self.method_source(
            "return_upper_floor_over_traversed_segments")
        self.assertIn("goal.completion_mode = goal.STAY_ON_FLOOR", explore_body)
        self.assertIn("endpoint_to_entry_m", explore_body)
        self.assertNotIn("entry_error > self.nav_arrival_band", explore_body)
        self.assertIn("achieved.header.frame_id != entry.header.frame_id",
                      explore_body)
        explore_call = complete_body.index(
            "self.explore_floor(floor, entry, False)")
        return_to_c_call = complete_body.index("self.navigate(\n                staging,")
        retrace_call = complete_body.index(
            "self.return_upper_floor_over_traversed_segments(")
        self.assertLess(explore_call, return_to_c_call)
        self.assertLess(return_to_c_call, retrace_call)
        self.assertIn("entry_error > self.nav_arrival_band", return_body)

    def test_endpoint_to_c_stages_on_c_position_with_the_c_to_b_bearing(self):
        """mf49: C's own yaw points away from B, so using it turns twice."""
        body = self.method_source("complete_upper_floor_and_return_to_a")
        self.assertIn("staging = copy.deepcopy(entry)", body)
        # Position comes from entry (the deep copy), heading from C->B.
        self.assertIn("point_b.pose.position.y - entry.pose.position.y", body)
        self.assertIn("point_b.pose.position.x - entry.pose.position.x", body)
        self.assertIn("set_yaw(staging.pose.orientation", body)
        self.assertNotIn("staging.pose.position.x =", body)
        self.assertNotIn("staging.pose.position.y =", body)
        # The declared entry pose must survive untouched: the measured C-B
        # geometry check downstream is stated against its axis.
        self.assertNotIn("set_yaw(entry.pose.orientation", body)
        self.assertNotIn(
            'self.navigate(entry, "floor %d return to corridor point C" '
            '% floor)', body)

    def test_every_goal_waits_for_the_shared_move_base_to_go_idle(self):
        """mf51 floor 2: the explorer had not finished handing move_base back."""
        navigate_body = self.method_source("navigate")
        settle_body = self.method_source("settle_move_base_handover")
        # The handshake must precede send_goal, not merely exist.
        settle_call = navigate_body.index("self.settle_move_base_handover(")
        send_goal = navigate_body.index("self.move.send_goal(goal)")
        self.assertLess(settle_call, send_goal)
        # PENDING/ACTIVE/PREEMPTING/RECALLING are the states that corrupt the
        # SimpleActionClient if a new goal lands on top of them.
        self.assertIn("busy = (0, 1, 6, 7)", settle_body)
        self.assertIn("self.move.cancel_all_goals()", settle_body)
        # Idle client must cost nothing, and a client that never had a goal
        # must not raise out of the guard itself.
        self.assertIn("if state not in busy:", settle_body)
        self.assertIn("except Exception:", settle_body)
        # Bounded on both clocks, like every other wait in this node.
        self.assertIn("self.move_handover_timeout_sim", settle_body)
        self.assertIn("self.move_handover_timeout_wall", settle_body)
        self.assertIn("MOVE_BASE_HANDOVER", settle_body)
        # It is a handshake only: no goal, tolerance or envelope may change.
        self.assertNotIn("send_goal", settle_body)
        self.assertNotIn("cmd_vel", settle_body)

    def test_demo_stops_at_floor_zero_unless_the_real_return_is_enabled(self):
        """The 2.3/3.5 m tail is a placeholder, not a return-to-start."""
        body = self.method_source("run")
        gate = body.index("if not self.final_return_to_start:")
        rebuild = body.index("entrance_inside = self.make_pose(")
        self.assertLess(gate, rebuild)
        # The hardcoded offsets must sit behind the gate, not in front of it.
        self.assertIn("2.3 * fx", body[rebuild:])
        self.assertIn("3.5 * fx", body[rebuild:])
        self.assertNotIn("2.3 * fx", body[:gate])
        # A run without the return leg must not silently publish a number that
        # reads like the PDF's "explore fully AND return to start" figure.
        early = body[gate:rebuild]
        self.assertIn("MISSION_COMPLETE", early)
        self.assertIn("return_to_start_performed=False", early)
        self.assertIn("does NOT include a return leg", early)
        self.assertIn("return_to_start_performed=True", body[rebuild:])

    def test_return_alignment_precedes_the_c_to_b_goal_and_never_translates(self):
        return_body = self.method_source(
            "return_upper_floor_over_traversed_segments")
        align_body = self.method_source("align_return_bearing_in_place")
        align_call = return_body.index(
            "self.align_return_bearing_in_place(")
        goal_call = return_body.index("self.navigate(\n            point_b")
        self.assertLess(align_call, goal_call)
        # C is re-verified AFTER the turn, not only before it.
        self.assertIn("settled_entry_error > self.nav_arrival_band",
                      return_body)
        self.assertIn("settled_heading_error > self.return_align_tolerance",
                      return_body)
        self.assertLess(align_call, return_body.index("settled_entry_error"))
        # Rotation only, on its own parameter block, with both clocks bounded.
        self.assertNotIn("command.linear", align_body)
        self.assertNotIn("self.navigate", align_body)
        self.assertIn("command.angular.z = math.copysign", align_body)
        self.assertIn("self.return_align_timeout_sim", align_body)
        self.assertIn("self.return_align_timeout_wall", align_body)
        self.assertIn("self.return_align_max_speed", align_body)
        self.assertNotIn("self.opening_turn_max_speed", align_body)
        self.assertIn("UPPER_FLOOR_RETURN_ALIGNMENT", align_body)
        self.assertIn("UPPER_FLOOR_RETURN_ALIGNMENT_READY", align_body)
        # Every exit path -- success, timeout, exception -- leaves the
        # behavior channel at zero, because it outranks navigation.
        self.assertIn("finally:", align_body)
        finally_block = align_body[align_body.index("finally:"):]
        self.assertIn("self.behavior_cmd_pub.publish(Twist())", finally_block)
        self.assertNotIn("travel_to_map_checked", self.methods)
        self.assertNotIn("advance_map_checked", return_body)
        self.assertNotIn("return_minimum_goal", self.source)
        self.assertNotIn("return_handover_distance", self.source)


if __name__ == "__main__":
    unittest.main()
