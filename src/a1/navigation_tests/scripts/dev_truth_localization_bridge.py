#!/usr/bin/env python3
"""DEV-ONLY Gazebo truth adapter for floor_mapping integration.

This test helper does not publish TF and never belongs in competition bringup.
`state_from_gazebo` remains the sole dev TF publisher.  The helper only exposes
the already dev-only `/Odometry_gazebo` through the public localization topic
and emits explicit DEV_TRUTH health/generation diagnostics so floor_mapping can
exercise its normal production-facing contract.
"""

import math
import threading
import time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import PointCloud2


def finite_pose(message):
    pose = message.pose.pose
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    norm = sum(value * value for value in values[3:])
    return all(math.isfinite(value) for value in values) and norm > 1e-8


class DevTruthLocalizationBridge:
    def __init__(self):
        self.lock = threading.Lock()
        self.raw_odom_topic = rospy.get_param(
            "~raw_odom_topic", "/Odometry_gazebo"
        )
        self.pointcloud_topic = rospy.get_param(
            "~pointcloud_topic", "/a1_localization/livox_pointcloud"
        )
        self.output_odom_topic = rospy.get_param(
            "~output_odom_topic", "/a1/localization/odom"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/a1/localization/status"
        )
        self.supervisor_topic = rospy.get_param(
            "~supervisor_status_topic",
            "/a1/localization/supervisor_status",
        )
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.input_timeout = float(
            rospy.get_param("~input_timeout", 1.0)
        )
        self.generation = int(rospy.get_param("~generation", 1))
        self.last_odom_stamp = rospy.Time()
        self.last_cloud_stamp = rospy.Time()
        self.last_odom_wall = 0.0
        self.last_cloud_wall = 0.0
        self.last_error = "waiting_for_inputs"

        self.odom_pub = rospy.Publisher(
            self.output_odom_topic, Odometry, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, DiagnosticStatus, queue_size=1, latch=True
        )
        self.supervisor_pub = rospy.Publisher(
            self.supervisor_topic,
            DiagnosticStatus,
            queue_size=1,
            latch=True,
        )
        rospy.Subscriber(
            self.raw_odom_topic, Odometry, self.odom_callback, queue_size=10
        )
        rospy.Subscriber(
            self.pointcloud_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=2,
        )
        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_callback)
        rospy.logwarn(
            "DEV-ONLY localization bridge active: %s -> %s; "
            "Gazebo truth must never be used in competition bringup",
            self.raw_odom_topic,
            self.output_odom_topic,
        )

    def odom_callback(self, message):
        valid = (
            message.header.frame_id == self.odom_frame
            and message.child_frame_id == self.base_frame
            and not message.header.stamp.is_zero()
            and finite_pose(message)
        )
        with self.lock:
            if (
                not self.last_odom_stamp.is_zero()
                and message.header.stamp < self.last_odom_stamp
            ):
                self.generation += 1
                self.last_cloud_stamp = rospy.Time()
                self.last_cloud_wall = 0.0
                self.last_error = "time_regression_new_generation"
            if not valid:
                self.last_error = "invalid_dev_truth_odom"
                return
            self.last_odom_stamp = message.header.stamp
            self.last_odom_wall = time.monotonic()
        self.odom_pub.publish(message)

    def cloud_callback(self, message):
        if message.header.stamp.is_zero() or not message.header.frame_id:
            with self.lock:
                self.last_error = "invalid_pointcloud_contract"
            return
        with self.lock:
            if (
                not self.last_cloud_stamp.is_zero()
                and message.header.stamp <= self.last_cloud_stamp
            ):
                self.last_error = "pointcloud_time_regression"
                return
            self.last_cloud_stamp = message.header.stamp
            self.last_cloud_wall = time.monotonic()

    @staticmethod
    def item(key, value):
        return KeyValue(key=key, value=str(value))

    def timer_callback(self, _event):
        now = time.monotonic()
        with self.lock:
            odom_age = (
                now - self.last_odom_wall
                if self.last_odom_wall else float("inf")
            )
            cloud_age = (
                now - self.last_cloud_wall
                if self.last_cloud_wall else float("inf")
            )
            healthy = (
                odom_age <= self.input_timeout
                and cloud_age <= self.input_timeout
            )
            generation = self.generation
            reason = "DEV_TRUTH_HEALTHY" if healthy else self.last_error

        status = DiagnosticStatus()
        status.level = (
            DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
        )
        status.name = "a1_localization/dev_truth_bridge"
        status.hardware_id = "gazebo_truth_dev_only"
        status.message = "TRACKING" if healthy else "WAITING_FOR_SENSORS"
        status.values = [
            self.item("state", status.message),
            self.item("reason", reason),
            self.item("results_valid", str(healthy).lower()),
            self.item("localization_mode", "DEV_TRUTH_GAZEBO"),
            self.item("competition_legal", "false"),
            self.item("odom_wall_age_sec", "%.6f" % odom_age),
            self.item("pointcloud_wall_age_sec", "%.6f" % cloud_age),
        ]
        self.status_pub.publish(status)

        supervisor = DiagnosticStatus()
        supervisor.level = status.level
        supervisor.name = "a1_localization/dev_truth_supervisor"
        supervisor.hardware_id = "gazebo_truth_dev_only"
        supervisor.message = "RUNNING" if healthy else "WAITING_FOR_SENSORS"
        supervisor.values = [
            self.item("generation", generation),
            self.item("mode", "DEV_TRUTH_GAZEBO"),
            self.item("competition_legal", "false"),
        ]
        self.supervisor_pub.publish(supervisor)


if __name__ == "__main__":
    rospy.init_node("dev_truth_localization_bridge")
    DevTruthLocalizationBridge()
    rospy.spin()
