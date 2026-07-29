#!/usr/bin/env python3
import math
import time
import unittest

import rospy
import rostest
import tf2_ros
from a1_navigation_interfaces.msg import DoorwayArray, WallSegmentArray
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class DoorWallRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("floor_mapping_door_wall_test")
        cls.doorways = []
        cls.walls = []
        cls.statuses = []
        cls.obstacle_clouds = []
        cls.clearing_clouds = []
        cls.cloud_pub = rospy.Publisher("/test/door_wall/cloud", PointCloud2, queue_size=2)
        cls.odom_pub = rospy.Publisher("/test/door_wall/odom", Odometry, queue_size=2)
        cls.loc_pub = rospy.Publisher("/test/door_wall/localization", DiagnosticStatus, queue_size=1, latch=True)
        cls.sup_pub = rospy.Publisher("/test/door_wall/supervisor", DiagnosticStatus, queue_size=1, latch=True)
        rospy.Subscriber("/test/door_wall/doorways", DoorwayArray, cls.doorways.append)
        rospy.Subscriber("/test/door_wall/walls", WallSegmentArray, cls.walls.append)
        rospy.Subscriber("/test/door_wall/structure_status", DiagnosticStatus, cls.statuses.append)
        rospy.Subscriber("/test/door_wall/obstacle_cloud", PointCloud2, cls.obstacle_clouds.append)
        rospy.Subscriber("/test/door_wall/clearing_cloud", PointCloud2, cls.clearing_clouds.append)
        cls.tf = tf2_ros.TransformBroadcaster()
        time.sleep(1.0)

    def publish_health(self, generation=1):
        self.loc_pub.publish(DiagnosticStatus(values=[KeyValue("state", "TRACKING"), KeyValue("results_valid", "true")]))
        self.sup_pub.publish(DiagnosticStatus(values=[KeyValue("generation", str(generation))]))

    @staticmethod
    def append_wall(points, x, y_min, y_max):
        y = y_min
        while y <= y_max + 1e-6:
            for height in (0.10, 0.42, 0.78, 1.16):
                points.append((x, y, height - 0.5, 2.0))
            y += 0.10

    def frame(self, open_state=True):
        stamp = rospy.Time.now()
        transforms = []
        for child in ("laser_livox", "base"):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "odom"
            transform.child_frame_id = child
            transform.transform.translation.z = 0.5
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id="odom")
        odom.child_frame_id = "base"
        odom.pose.pose.position.z = 0.5
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        points = [(ix * 0.12, iy * 0.12, -0.5, 1.0)
                  for ix in range(-10, 12) for iy in range(-10, 11)
                  if math.hypot(ix * 0.12, iy * 0.12) > 0.5]
        self.append_wall(points, 3.0, -2.0, -0.65)
        self.append_wall(points, 3.0, 0.65, 2.0)
        if open_state:
            self.append_wall(points, 5.0, -0.90, 0.90)
        else:
            self.append_wall(points, 3.0, -0.55, 0.55)
        fields = [PointField("x", 0, PointField.FLOAT32, 1), PointField("y", 4, PointField.FLOAT32, 1),
                  PointField("z", 8, PointField.FLOAT32, 1), PointField("intensity", 12, PointField.FLOAT32, 1)]
        self.cloud_pub.publish(point_cloud2.create_cloud(Header(stamp=stamp, frame_id="laser_livox"), fields, points))

    def wait_door(self, expected_state, minimum_session=0, timeout=12.0):
        end = time.time() + timeout
        while time.time() < end and not rospy.is_shutdown():
            for array in reversed(self.doorways):
                if array.doorways and array.doorways[0].floor_session_id >= minimum_session and array.doorways[0].state == expected_state:
                    return array.doorways[0]
            time.sleep(0.04)
        detail = [(d.floor_session_id, d.state, d.detection_id) for a in self.doorways for d in a.doorways]
        self.fail("door state %s not reached: %s" % (expected_state, detail[-8:]))

    def test_open_close_and_session_contract(self):
        self.publish_health(1)
        for _ in range(12):
            self.frame(True)
            time.sleep(0.12)
        doorway = self.wait_door(1)  # Doorway.OPEN
        self.assertTrue(self.walls and self.walls[-1].walls)
        self.assertTrue(doorway.stable)
        self.assertFalse(doorway.control_id_matched)
        self.assertEqual(doorway.control_door_id, "")
        self.assertGreater(doorway.usable_width, 0.70)
        old_id = doorway.detection_id
        old_session = doorway.floor_session_id

        for _ in range(7):
            self.frame(False)
            time.sleep(0.12)
        closed = self.wait_door(2, old_session)  # Doorway.CLOSED
        self.assertEqual(closed.detection_id, old_id)
        self.assertAlmostEqual(closed.usable_width, 0.0, places=3)

        self.publish_health(2)
        for _ in range(12):
            self.frame(True)
            time.sleep(0.12)
        next_doorway = self.wait_door(1, old_session + 1)
        self.assertGreater(next_doorway.floor_session_id, old_session)
        self.assertTrue(self.statuses)
        self.assertEqual(dict((item.key, item.value) for item in self.statuses[-1].values)["results_valid"], "true")

        self.assertTrue(self.obstacle_clouds and self.clearing_clouds)
        obstacle_z = [point[2] for point in point_cloud2.read_points(self.obstacle_clouds[-1], field_names=("x", "y", "z"), skip_nans=True)]
        clearing_z = [point[2] for point in point_cloud2.read_points(self.clearing_clouds[-1], field_names=("x", "y", "z"), skip_nans=True)]
        self.assertNotIn(-0.5, [round(z, 3) for z in obstacle_z])
        self.assertIn(-0.5, [round(z, 3) for z in clearing_z])


if __name__ == "__main__":
    rostest.rosrun("a1_floor_mapping", "floor_mapping_door_wall", DoorWallRuntimeTest)
