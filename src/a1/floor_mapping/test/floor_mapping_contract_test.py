#!/usr/bin/env python3
import pathlib
import sys
import unittest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]

class ContractTest(unittest.TestCase):
    def test_required_files_and_topics(self):
        for name in ("package.xml", "CMakeLists.txt", "README.md", "config/floor_mapping.yaml", "launch/floor_mapping.launch", "src/floor_mapping_node.cpp"):
            self.assertTrue((ROOT / name).is_file(), name)
        cfg = yaml.safe_load((ROOT / "config/floor_mapping.yaml").read_text())
        self.assertEqual(cfg["frames"]["odom"], "odom")
        self.assertEqual(cfg["frames"]["sensor"], "laser_livox")
        for topic in ("obstacle_cloud", "occupancy_grid", "status", "diagnostics"):
            self.assertTrue(cfg["topics"][topic].startswith("/a1/floor_mapping/"))
        self.assertGreaterEqual(cfg["ground"]["floor_change_frames"], 2)
        self.assertGreaterEqual(cfg["ground"]["invalid_frame_tolerance"], 0)
        self.assertGreaterEqual(cfg["recovery"]["valid_frames"], 1)
        self.assertGreaterEqual(cfg["queues"]["pointcloud"], 1)
        self.assertGreater(cfg["timeouts"]["tf_queue_ros"], 0)
        self.assertGreater(cfg["timeouts"]["tf_queue_wall"], 0)
        source = (ROOT / "src/floor_mapping_node.cpp").read_text()
        for field in ("pointcloud_age_sec", "pointcloud_input_age_sec", "tf_pending_clouds", "odom_age_sec", "last_success_tf_age_sec", "occupied_cells", "processing_time_ms", "minimum_boundary_margin_m", "floor_session_id"):
            self.assertIn(field, source)

    def test_phase_three_delivery_assets(self):
        for name in ("scripts/floor_mapping_route_runner.py", "scripts/floor_mapping_health_gate.py", "config/validation_route.yaml", "config/validation_route_extended.yaml", "config/costmap_mapping_sources.yaml"):
            self.assertTrue((ROOT / name).is_file(), name)
        route = yaml.safe_load((ROOT / "config/validation_route.yaml").read_text())
        self.assertTrue(route["segments"])
        for segment in route["segments"]:
            self.assertLessEqual(abs(segment.get("linear", 0)), route["limits"]["linear"])
            self.assertLessEqual(abs(segment.get("angular", 0)), route["limits"]["angular"])

    def test_no_forbidden_dependencies(self):
        source = (ROOT / "src/floor_mapping_node.cpp").read_text()
        for forbidden in ("Odometry_gazebo", "/ground_truth", "generated_building", "/a1_nav/", "cmd_vel"):
            self.assertNotIn(forbidden, source)
        self.assertNotIn("src/a1/navigation", "\n".join(str(p) for p in ROOT.rglob("*")))

if __name__ == "__main__":
    unittest.main()
