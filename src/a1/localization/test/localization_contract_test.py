#!/usr/bin/env python3
import pathlib
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]

class LocalizationContractTest(unittest.TestCase):
    def test_xml(self):
        ET.parse(ROOT / "package.xml")
        for path in (ROOT / "launch").glob("*.launch"):
            ET.parse(path)

    def test_fast_lio_sim_contract(self):
        text = (ROOT / "config" / "fast_lio_a1.yaml").read_text()
        self.assertIn("lidar_type: 4", text)
        self.assertIn("imu_topic: /trunk_imu", text)
        self.assertIn("extrinsic_est_en: false", text)
        self.assertIn("extrinsic_T: [0.2, 0.0, 0.08]", text)

    def test_no_truth_or_legacy_converter(self):
        launch_text = "\n".join(p.read_text(errors="ignore") for p in (ROOT / "launch").glob("*.launch"))
        source = (ROOT / "src" / "pointcloud_adapter.cpp").read_text()
        self.assertNotIn("pointcloud2livox.py", launch_text)
        self.assertNotIn("/ground_truth/", source)
        self.assertNotIn("/Odometry_gazebo", source)

    def test_stability_gate_is_acceptance_only(self):
        text = (ROOT / "scripts" / "localization_stability_gate.py").read_text()
        self.assertIn("ground_truth/base_w", text)
        self.assertIn("never republishes truth", text)

    def test_validation_recorder_is_acceptance_only(self):
        text = (ROOT / "scripts" / "localization_validation_recorder.py").read_text()
        self.assertIn("ground_truth/base_w", text)
        self.assertIn("never fed into localization", text)
        self.assertNotIn("Publisher(", text)

    def test_standard_frame_contract(self):
        frames = (ROOT / "config" / "frames.yaml").read_text()
        launch = "\n".join(path.read_text() for path in
                            (ROOT / "launch").glob("*.launch"))
        source = (ROOT / "src" / "localization_pose_adapter.cpp").read_text()
        self.assertIn("odom_frame: odom", frames)
        self.assertIn("base_frame: base", frames)
        self.assertIn("output_odom_topic: /a1/localization/odom", frames)
        self.assertIn("output_registered_cloud_topic: /a1/localization/cloud_registered", frames)
        self.assertIn("output_map_topic: /a1/localization/map", frames)
        self.assertIn("imu_to_base_translation: [0.0, 0.0, 0.0]", frames)
        self.assertIn("unknown_twist_variance: 1000000.0", frames)
        self.assertIn('publish/tf_en" type="bool" value="false"', launch)
        self.assertIn("/a1_localization/fast_lio/odom_raw", launch)
        self.assertIn("localization_pose_adapter", launch)
        self.assertIn("transform.child_frame_id = base_frame_", source)
        self.assertNotIn('sendTransform( tf::StampedTransform', source)

    def test_health_gate_contract(self):
        frames = (ROOT / "config" / "frames.yaml").read_text()
        source = (ROOT / "src" / "localization_pose_adapter.cpp").read_text()
        self.assertIn("health_enabled: true", frames)
        self.assertIn("monitor_clock: true", frames)
        self.assertIn("/a1/localization/status", frames)
        self.assertIn("WAITING_FOR_SENSORS", source)
        self.assertIn("INITIALIZING", source)
        self.assertIn("TRACKING", source)
        self.assertIn("DEGRADED", source)
        self.assertIn("LOST", source)
        self.assertIn('state_ != State::TRACKING', source)
        self.assertNotIn("/ground_truth/", source)

    def test_controlled_reinitialization_contract(self):
        launch = (ROOT / "launch" / "localization.launch").read_text()
        estimator = (ROOT / "launch" / "localization_estimator.launch").read_text()
        supervisor = (ROOT / "scripts" / "localization_supervisor.py").read_text()
        config = (ROOT / "config" / "supervisor.yaml").read_text()
        self.assertIn("localization_supervisor.py", launch)
        self.assertNotIn("fastlio_mapping", launch)
        self.assertIn("fastlio_mapping", estimator)
        self.assertIn("localization_pose_adapter", estimator)
        self.assertIn("start_new_session=True", supervisor)
        self.assertIn('child_env.pop("ROS_NAMESPACE", None)', supervisor)
        self.assertIn("refusing to create a duplicate estimator", supervisor)
        self.assertIn("reinitialization_required", supervisor)
        self.assertIn("WAITING_FOR_INPUT_SETTLING", supervisor)
        self.assertIn("/a1/localization/reinitialize", config)
        self.assertNotIn("/ground_truth/", supervisor)

    def test_online_map_product_contract(self):
        launch = (ROOT / "launch" / "localization.launch").read_text()
        config = (ROOT / "config" / "map.yaml").read_text()
        source = (ROOT / "src" / "localization_map_manager.cpp").read_text()
        fast_lio = (ROOT.parents[1] / "third_party" / "FAST_LIO" / "src" /
                    "laserMapping.cpp").read_text()
        self.assertIn("localization_map_manager", launch)
        self.assertIn("/a1/localization/save_map", config)
        self.assertIn("expected_frame: odom", config)
        self.assertIn("overwrite: false", config)
        self.assertIn("savePCDFileBinaryCompressed", source)
        self.assertIn("loadPCDFile", source)
        self.assertIn("pcd_sha256", source)
        self.assertIn("rename(temporary_directory", source)
        self.assertIn("map_publish_interval", fast_lio)
        self.assertNotIn("OccupancyGrid", source)

    def test_a1_imu_to_base_is_identity(self):
        robot = (ROOT.parents[1] / "unitree_guide" / "unitree_ros" / "robots" /
                 "a1_description" / "xacro" / "robot.xacro").read_text()
        self.assertIn('<joint name="floating_base" type="fixed">', robot)
        self.assertIn('<origin rpy="0 0 0" xyz="0 0 0"/>', robot)
        self.assertIn('<joint name="imu_joint" type="fixed">', robot)

if __name__ == "__main__":
    unittest.main()
