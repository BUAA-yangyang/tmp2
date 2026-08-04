#!/usr/bin/env python3
import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


CALF_JOINTS = {
    "FR_calf_joint",
    "FL_calf_joint",
    "RR_calf_joint",
    "RL_calf_joint",
}


class GazeboCalfJointLimitConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source_root = pathlib.Path(__file__).resolve().parents[3]
        description = (
            source_root
            / "unitree_guide"
            / "unitree_ros"
            / "robots"
            / "a1_description"
        )
        with (description / "config" / "robot_control.yaml").open() as stream:
            cls.control = yaml.safe_load(stream)["a1_gazebo"]
        cls.urdf = ET.parse(description / "urdf" / "a1.urdf").getroot()

    def test_only_calf_software_position_gates_are_disabled(self):
        overrides = self.control["joint_limits"]
        self.assertEqual(CALF_JOINTS, set(overrides))
        for joint_name in CALF_JOINTS:
            self.assertIs(False, overrides[joint_name]["has_position_limits"])

    def test_calf_urdf_still_has_bounded_physical_and_effort_limits(self):
        joints = {
            joint.attrib["name"]: joint
            for joint in self.urdf.findall("joint")
        }
        for joint_name in CALF_JOINTS:
            joint = joints[joint_name]
            self.assertEqual("revolute", joint.attrib["type"])
            limits = joint.find("limit")
            self.assertIsNotNone(limits)
            self.assertLess(float(limits.attrib["lower"]), -2.6)
            self.assertGreater(float(limits.attrib["upper"]), -1.0)
            self.assertGreater(float(limits.attrib["velocity"]), 0.0)
            self.assertGreater(float(limits.attrib["effort"]), 0.0)

    def test_unitree_controller_targets_each_overridden_joint(self):
        for joint_name in CALF_JOINTS:
            leg = joint_name.split("_", 1)[0]
            controller = self.control[f"{leg}_calf_controller"]
            self.assertEqual(joint_name, controller["joint"])


if __name__ == "__main__":
    unittest.main()
