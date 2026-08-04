#!/usr/bin/env python3
"""Reference downstream gate for mapping-derived navigation commands."""
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def status_values(message):
    return {item.key: item.value for item in message.values}


def status_is_usable(message, expected_generation=None, expected_session=None):
    values = status_values(message)
    if message.message != "MAPPING" or values.get("map_valid") != "true" or values.get("obstacle_cloud_valid") != "true":
        return False
    if expected_generation is not None and values.get("localization_generation") != str(expected_generation):
        return False
    if expected_session is not None and values.get("floor_session_id") != str(expected_session):
        return False
    try:
        return float(values.get("pointcloud_age_sec", "inf")) <= 0.5 and float(values.get("last_success_tf_age_sec", "inf")) <= 0.5
    except ValueError:
        return False


class HealthGate:
    def __init__(self):
        self.lock = threading.Lock()
        self.allowed = False
        self.last_status_wall = None
        self.timeout = float(rospy.get_param("~status_wall_timeout", rospy.get_param("~status_timeout", 5.0)))
        self.expected_generation = rospy.get_param("~expected_generation", None)
        self.expected_session = rospy.get_param("~expected_floor_session_id", None)
        self.allowed_pub = rospy.Publisher("~mapping_usable", Bool, queue_size=1, latch=True)
        self.stop_pub = rospy.Publisher(rospy.get_param("~stop_topic", "/cmd_vel"), Twist, queue_size=1)
        rospy.Subscriber(rospy.get_param("~status_topic", "/a1/floor_mapping/status"), DiagnosticStatus, self.on_status, queue_size=1)
        self.watchdog = threading.Thread(target=self.watchdog_loop, daemon=True)
        self.watchdog.start()

    def set_allowed(self, allowed):
        changed = allowed != self.allowed
        self.allowed = allowed
        self.allowed_pub.publish(Bool(allowed))
        if not allowed:
            self.stop_pub.publish(Twist())
        if changed:
            rospy.loginfo("mapping health gate %s", "OPEN" if allowed else "CLOSED")

    def on_status(self, message):
        with self.lock:
            self.last_status_wall = time.monotonic()
            self.set_allowed(status_is_usable(message, self.expected_generation, self.expected_session))

    def watchdog_loop(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.last_status_wall is None or time.monotonic() - self.last_status_wall > self.timeout:
                    self.set_allowed(False)
            time.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node("floor_mapping_health_gate")
    HealthGate()
    rospy.spin()
