#!/usr/bin/env python3
"""Structural integration guards for the remembered floor-zero return."""

import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "scripts" / "multifloor_mission_node.py"
WORKSPACE_A1 = ROOT.parent


class HomeReturnContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = NODE.read_text()
        tree = ast.parse(cls.source)
        mission = next(
            item for item in tree.body
            if isinstance(item, ast.ClassDef) and item.name == "MultiFloorMission")
        cls.methods = {
            item.name: item for item in mission.body
            if isinstance(item, ast.FunctionDef)
        }

    def method_source(self, name):
        return ast.get_source_segment(self.source, self.methods[name])

    def test_transfer_propagates_home_frame_from_pre_and_post_restart_poses(self):
        body = self.method_source("transfer")
        self.assertLess(body.index("departure = self.current_pose()"),
                        body.index("response = self.call_elevator"))
        self.assertLess(body.index("arrival = self.current_pose()"),
                        body.index("propagate_home_transform("))
        self.assertIn("self.home_from_current = home_from_target", body)

    def test_run_snapshots_before_first_transfer_and_restores_after_final_transfer(self):
        body = self.method_source("run")
        self.assertLess(body.index("self.capture_home_snapshot(spawn, entry0)"),
                        body.index("self.transfer(0, 1)"))
        self.assertLess(body.index("self.transfer(2, 0)"),
                        body.index("targets = self.restore_home_map()"))
        self.assertIn("stop_at_lobby=True", body)
        self.assertIn("self.return_to_home(targets)", body)
        self.assertIn("special_test and not self.home_return_after_special_test", body)
        self.assertNotIn("floor-zero elevator after relocalization", body)
        self.assertNotIn("2.3 * fx", body)

    def test_online_map_relay_stops_only_after_home_restore(self):
        body = self.method_source("on_floor_grid")
        self.assertIn("publish_active = not self.home_map_restored", body)
        self.assertIn("self.active_map_pub.publish(message)", body)
        restore = self.method_source("restore_home_map")
        self.assertIn("warp_occupancy_grid(", restore)
        self.assertIn("self.home_map_restored = True", restore)
        self.assertIn("self.active_map_pub.publish(restored)", restore)

    def test_completion_verifies_pose_and_final_velocity(self):
        body = self.method_source("return_to_home")
        self.assertIn("position_error > self.home_return_position_tolerance", body)
        self.assertIn("yaw_error > self.home_return_yaw_tolerance", body)
        self.assertIn("self.wait_for_final_zero()", body)
        self.assertIn('"HOME_REACHED"', body)

    def test_only_multifloor_launch_selects_the_active_map(self):
        multifloor = (ROOT / "launch" / "multifloor_test.launch").read_text()
        navigation = (
            WORKSPACE_A1 / "navigation" / "launch" /
            "navigation_floor_mapping.launch").read_text()
        single = (
            WORKSPACE_A1 / "navigation_tests" / "launch" /
            "single_floor_exploration_dev.launch").read_text()
        self.assertIn('/a1/navigation/active_map', multifloor)
        self.assertIn('static_map_topic" default="/a1/floor_mapping/map', navigation)
        self.assertIn('static_map_topic" default="/a1/floor_mapping/map', single)


if __name__ == "__main__":
    unittest.main()
