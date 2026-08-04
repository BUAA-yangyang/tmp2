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

    def test_transfer_records_generation_local_arrival_heading(self):
        body = self.method_source("transfer")
        self.assertIn("self.arrival_exit_yaws[target]", body)
        self.assertIn("arrival_exit_yaw=arrival_yaw", body)

    def test_return_point_faces_inward_and_turn_has_no_pi_fallback(self):
        exit_body = self.method_source("exit_upper_floor_without_doorway")
        turn_body = self.method_source("turn_inside_elevator_before_transfer")
        self.assertIn("yaw_out + math.pi", exit_body)
        self.assertNotIn("target_yaw = start_yaw + math.pi", turn_body)
        self.assertIn("raise MissionFailure", turn_body)


if __name__ == "__main__":
    unittest.main()
