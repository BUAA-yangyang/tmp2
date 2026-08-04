#!/usr/bin/env python3
import math
import threading
import time
import unittest

import rospy
import rostest
import tf2_ros
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from std_msgs.msg import String


class LocalizationPoseAdapterRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("localization_pose_adapter_runtime_test", anonymous=True)
        cls.odom_pub = rospy.Publisher(
            "/a1_localization/fast_lio/odom_raw", Odometry, queue_size=1
        )
        cls.cloud_pub = rospy.Publisher(
            "/a1_localization/fast_lio/cloud_registered_raw", PointCloud2, queue_size=1
        )
        cls.map_pub = rospy.Publisher(
            "/a1_localization/fast_lio/map_raw", PointCloud2, queue_size=1
        )
        cls.sensor_cloud_pub = rospy.Publisher(
            "/a1_localization/livox_pointcloud", PointCloud2, queue_size=1
        )
        cls.imu_pub = rospy.Publisher("/trunk_imu", Imu, queue_size=1)
        cls.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)
        cls.controller_pub = rospy.Publisher("/a1/controller/state", String, queue_size=1,
                                             latch=True)
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer)

    def wait_for_connections(self):
        deadline = time.monotonic() + 5.0
        publishers = [self.odom_pub, self.cloud_pub, self.map_pub, self.sensor_cloud_pub,
                      self.imu_pub, self.clock_pub, self.controller_pub]
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if all(pub.get_num_connections() for pub in publishers):
                return
            time.sleep(0.05)
        self.fail("pose adapter did not subscribe to all raw input topics")

    @staticmethod
    def raw_odom(stamp, x=1.25):
        raw = Odometry()
        raw.header.stamp = stamp
        raw.header.frame_id = "camera_init"
        raw.child_frame_id = "body"
        raw.pose.pose.position.x = x
        raw.pose.pose.position.y = -0.5
        raw.pose.pose.position.z = 0.3
        raw.pose.pose.orientation.z = math.sin(0.2)
        raw.pose.pose.orientation.w = math.cos(0.2)
        raw.pose.covariance[0] = 0.04
        return raw

    def publish_healthy_sample(self, stamp, x=1.25):
        sensor_cloud = PointCloud2()
        sensor_cloud.header.stamp = stamp
        sensor_cloud.header.frame_id = "laser_livox"
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        self.sensor_cloud_pub.publish(sensor_cloud)
        self.imu_pub.publish(imu)
        self.clock_pub.publish(Clock(clock=stamp))
        self.odom_pub.publish(self.raw_odom(stamp, x))

    @staticmethod
    def status_values(status):
        return {entry.key: entry.value for entry in status.values}

    def test_health_gate_standard_outputs_and_invalidation(self):
        self.wait_for_connections()
        self.controller_pub.publish(String(data="fixed stand"))
        received_odom = []
        received_cloud = []
        received_map = []
        received_status = []
        odom_sub = rospy.Subscriber(
            "/a1/localization/odom", Odometry, received_odom.append, queue_size=10
        )
        cloud_sub = rospy.Subscriber(
            "/a1/localization/cloud_registered", PointCloud2,
            received_cloud.append, queue_size=10
        )
        map_sub = rospy.Subscriber(
            "/a1/localization/map", PointCloud2, received_map.append, queue_size=10
        )
        status_sub = rospy.Subscriber(
            "/a1/localization/status", DiagnosticStatus,
            received_status.append, queue_size=20
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if odom_sub.get_num_connections() and cloud_sub.get_num_connections() \
                    and map_sub.get_num_connections() \
                    and status_sub.get_num_connections():
                break
            time.sleep(0.05)

        # Fewer than initialization_samples must not leak a trusted result.
        for index in range(2):
            self.publish_healthy_sample(rospy.Time.now() + rospy.Duration(index * 0.01))
            time.sleep(0.05)
        self.assertFalse(received_odom, "odometry leaked during INITIALIZING")

        tracking_stamp = rospy.Time.now() + rospy.Duration(0.03)
        for index in range(5):
            tracking_stamp = rospy.Time.now() + rospy.Duration(0.03 + index * 0.01)
            self.publish_healthy_sample(tracking_stamp)
            time.sleep(0.05)
            if received_odom:
                break
        self.assertTrue(received_odom, "standard odometry was not received after health gate")
        self.assertTrue(any(message.message == "TRACKING" for message in received_status))

        cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=tracking_stamp, frame_id="camera_init"), [(1.0, 0.0, 0.0)]
        )
        self.cloud_pub.publish(cloud)
        self.map_pub.publish(cloud)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not received_cloud:
            time.sleep(0.02)
        self.assertTrue(received_cloud, "registered cloud was not published while TRACKING")
        self.assertTrue(received_map, "world-aligned map was not published while TRACKING")

        output = received_odom[-1]
        self.assertEqual(output.header.frame_id, "odom")
        self.assertEqual(output.child_frame_id, "base")
        self.assertAlmostEqual(output.pose.pose.position.x, 1.25, places=6)
        self.assertAlmostEqual(output.pose.covariance[0], 0.04, places=6)
        # Consecutive FAST-LIO poses now provide the body twist consumed by
        # DWA, so a tracked output carries the configured trusted variance.
        self.assertAlmostEqual(output.twist.covariance[0], 0.04, places=6)
        self.assertEqual(received_cloud[-1].header.frame_id, "world")
        self.assertEqual(received_map[-1].header.frame_id, "world")
        transform = self.tf_buffer.lookup_transform(
            "odom", "base", rospy.Time(0), rospy.Duration(2.0)
        )
        self.assertAlmostEqual(transform.transform.translation.x, 1.25, places=6)
        world_base = self.tf_buffer.lookup_transform(
            "world", "base", rospy.Time(0), rospy.Duration(2.0)
        )
        self.assertAlmostEqual(world_base.transform.translation.x, 0.0, places=5)
        self.assertAlmostEqual(world_base.transform.translation.y, -3.2, places=5)
        self.assertAlmostEqual(world_base.transform.translation.z, 0.6, places=5)

        # Stop all inputs: warn first, then LOST. No further public result may appear.
        odom_count = len(received_odom)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if received_status and received_status[-1].message == "LOST":
                break
            time.sleep(0.05)
        self.assertEqual(received_status[-1].message, "LOST")
        values = self.status_values(received_status[-1])
        self.assertEqual(values["results_valid"], "false")
        self.assertEqual(values["reason"], "INPUT_TIMEOUT_LOST")
        self.cloud_pub.publish(cloud)
        time.sleep(0.2)
        self.assertEqual(len(received_odom), odom_count)
        self.assertEqual(len(received_cloud), 1, "cloud leaked after localization became LOST")

        # Recovery also requires the configured number of consecutive healthy samples.
        for index in range(5):
            self.publish_healthy_sample(rospy.Time.now() + rospy.Duration(index * 0.01))
            time.sleep(0.05)
            if len(received_odom) > odom_count:
                break
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and len(received_odom) == odom_count:
            time.sleep(0.02)
        self.assertGreater(len(received_odom), odom_count)
        self.assertEqual(received_status[-1].message, "TRACKING")
        self.assertEqual(self.status_values(received_status[-1])["results_valid"], "true")

        # A non-finite estimator output invalidates immediately and is never forwarded.
        odom_count = len(received_odom)
        invalid = self.raw_odom(rospy.Time.now() + rospy.Duration(0.1))
        invalid.pose.pose.position.x = float("nan")
        self.odom_pub.publish(invalid)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and received_status[-1].message != "LOST":
            time.sleep(0.02)
        self.assertEqual(self.status_values(received_status[-1])["reason"],
                         "ODOM_NON_FINITE_OR_ZERO_STAMP")
        self.assertEqual(len(received_odom), odom_count)

        for index in range(5):
            self.publish_healthy_sample(rospy.Time.now() + rospy.Duration(0.2 + index * 0.01))
            time.sleep(0.05)
            if received_status[-1].message == "TRACKING":
                break
        self.assertEqual(received_status[-1].message, "TRACKING")

        # Small per-frame changes that evade the jump gate must still be
        # rejected when they accumulate during an explicit stationary state.
        drift_start = rospy.Time.now() + rospy.Duration(0.3)
        for index in range(6):
            self.publish_healthy_sample(
                drift_start + rospy.Duration(index * 0.05), 1.25 + index * 0.03)
            time.sleep(0.06)
            if received_status[-1].message == "LOST":
                break
        self.assertEqual(received_status[-1].message, "LOST")
        self.assertEqual(self.status_values(received_status[-1])["reason"],
                         "STATIONARY_TRANSLATION_DRIFT")
        self.assertEqual(self.status_values(received_status[-1])["results_valid"], "false")

        for index in range(5):
            self.publish_healthy_sample(
                rospy.Time.now() + rospy.Duration(0.7 + index * 0.01), 1.40)
            time.sleep(0.05)
            if received_status[-1].message == "TRACKING":
                break
        self.assertEqual(received_status[-1].message, "TRACKING")

        # A simulation reset is represented by /clock going backwards.
        self.clock_pub.publish(Clock(clock=rospy.Time(1.0)))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and received_status[-1].message != "LOST":
            time.sleep(0.02)
        self.assertEqual(received_status[-1].message, "LOST")
        self.assertEqual(self.status_values(received_status[-1])["reason"],
                         "CLOCK_TIME_ROLLBACK")
        self.assertEqual(self.status_values(received_status[-1])["results_valid"], "false")
        self.assertEqual(
            self.status_values(received_status[-1])["reinitialization_required"], "true"
        )

        # Even healthy-looking samples cannot clear a reset fault without restarting
        # the estimator/adapter process.
        odom_count = len(received_odom)
        for index in range(5):
            stamp = rospy.Time(2.0 + index * 0.01)
            self.publish_healthy_sample(stamp)
            time.sleep(0.05)
        self.assertEqual(received_status[-1].message, "LOST")
        self.assertEqual(len(received_odom), odom_count)

        odom_sub.unregister()
        cloud_sub.unregister()
        map_sub.unregister()
        status_sub.unregister()


if __name__ == "__main__":
    rostest.rosrun(
        "a1_localization",
        "localization_pose_adapter_runtime_test",
        LocalizationPoseAdapterRuntimeTest,
    )
