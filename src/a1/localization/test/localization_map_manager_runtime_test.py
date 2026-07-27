#!/usr/bin/env python3
import hashlib
import math
import pathlib
import shutil
import struct
import time
import unittest

import rospy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger


class LocalizationMapManagerRuntimeTest(unittest.TestCase):
    ROOT = pathlib.Path("/tmp/a1_localization_map_manager_test")

    @classmethod
    def setUpClass(cls):
        shutil.rmtree(str(cls.ROOT), ignore_errors=True)
        cls.status_pub = rospy.Publisher("/test/localization/status", DiagnosticStatus,
                                         queue_size=1, latch=True)
        cls.map_pub = rospy.Publisher("/test/localization/map", PointCloud2,
                                      queue_size=1, latch=True)
        rospy.wait_for_service("/test/localization/save_map", timeout=10)
        cls.save = rospy.ServiceProxy("/test/localization/save_map", Trigger)
        time.sleep(0.5)

    def setUp(self):
        self.map_id = "test_{}".format(time.time_ns())
        rospy.set_param("/localization_map_manager_test/map_id", self.map_id)
        rospy.set_param("/localization_map_manager_test/overwrite", False)
        self.publish_status(False)
        time.sleep(0.1)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(str(cls.ROOT), ignore_errors=True)

    def publish_status(self, tracking):
        message = DiagnosticStatus()
        message.level = DiagnosticStatus.OK if tracking else DiagnosticStatus.ERROR
        message.message = "TRACKING" if tracking else "LOST"
        message.values = [KeyValue("results_valid", "true" if tracking else "false")]
        self.status_pub.publish(message)

    def publish_cloud(self, points, frame="world"):
        message = PointCloud2()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = frame
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        message.is_bigendian = False
        message.point_step = 16
        message.row_step = message.point_step * message.width
        message.is_dense = all(math.isfinite(value) for point in points for value in point)
        message.data = b"".join(struct.pack("<ffff", *point) for point in points)
        self.map_pub.publish(message)

    def test_rejects_unavailable_invalid_and_wrong_frame_maps(self):
        self.assertFalse(self.save().success)
        self.publish_status(True)
        self.publish_cloud([(0, 0, 0, 1), (1, 1, 1, 2), (2, 2, 2, 3)], "odom")
        time.sleep(0.2)
        self.assertFalse(self.save().success)
        self.publish_cloud([(0, 0, 0, 1), (1, 1, 1, 2), (float("nan"), 2, 2, 3)])
        time.sleep(0.2)
        self.assertFalse(self.save().success)

    def test_atomic_product_metadata_checksum_and_overwrite_protection(self):
        points = [(-1, 2, 0.5, 10), (3, -2, 1.5, 20), (0, 4, -0.5, 30)]
        self.publish_status(True)
        time.sleep(0.2)
        self.publish_cloud(points)
        time.sleep(0.3)
        result = self.save()
        self.assertTrue(result.success, result.message)
        product = self.ROOT / self.map_id
        pcd = product / "map.pcd"
        metadata = (product / "metadata.yaml").read_text()
        self.assertTrue(pcd.is_file())
        self.assertIn("point_count: 3", metadata)
        self.assertIn("frame: world", metadata)
        self.assertIn("alignment_mode: fixed_start", metadata)
        self.assertIn("min: [-1", metadata)
        self.assertIn("max: [3", metadata)
        self.assertIn(hashlib.sha256(pcd.read_bytes()).hexdigest(), metadata)
        self.assertFalse(any(path.name.startswith(".{}".format(self.map_id))
                             for path in self.ROOT.iterdir()))
        self.assertFalse(self.save().success)

    def test_lost_state_discards_cached_map(self):
        self.publish_status(True)
        time.sleep(0.2)
        self.publish_cloud([(0, 0, 0, 1), (1, 1, 1, 2), (2, 2, 2, 3)])
        time.sleep(0.2)
        self.publish_status(False)
        time.sleep(0.2)
        self.publish_status(True)
        time.sleep(0.2)
        self.assertFalse(self.save().success)


if __name__ == "__main__":
    rospy.init_node("localization_map_manager_runtime_test")
    import rostest
    rostest.rosrun("a1_localization", "localization_map_manager_runtime_test",
                   LocalizationMapManagerRuntimeTest)
