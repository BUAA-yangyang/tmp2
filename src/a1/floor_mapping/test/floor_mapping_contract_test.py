#!/usr/bin/env python3
import pathlib
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
        self.assertGreaterEqual(cfg["recovery"]["valid_frames"], 1)
        source = (ROOT / "src/floor_mapping_node.cpp").read_text()
        for field in ("pointcloud_age_sec", "odom_age_sec", "last_success_tf_age_sec", "occupied_cells", "processing_time_ms", "minimum_boundary_margin_m"):
            self.assertIn(field, source)

    def test_no_forbidden_dependencies(self):
        source = (ROOT / "src/floor_mapping_node.cpp").read_text()
        for forbidden in ("Odometry_gazebo", "/ground_truth", "generated_building", "/a1_nav/", "cmd_vel"):
            self.assertNotIn(forbidden, source)
        self.assertNotIn("src/a1/navigation", "\n".join(str(p) for p in ROOT.rglob("*")))

if __name__ == "__main__":
    unittest.main()
