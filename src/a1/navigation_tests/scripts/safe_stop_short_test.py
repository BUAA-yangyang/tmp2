#!/usr/bin/env python3
"""Short Gazebo mode-transition test for MOVE_BASE -> guarded FIXEDSTAND."""

import importlib.util
import json
import math
import os
import tempfile
import threading
import time

from geometry_msgs.msg import Twist, WrenchStamped
import rosgraph
import rospy
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool

ACCEPTANCE_SCRIPT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "single_floor_gazebo_acceptance.py",
)
ACCEPTANCE_SPEC = importlib.util.spec_from_file_location(
    "a1_single_floor_acceptance_helpers", ACCEPTANCE_SCRIPT
)
ACCEPTANCE_HELPERS = importlib.util.module_from_spec(ACCEPTANCE_SPEC)
ACCEPTANCE_SPEC.loader.exec_module(ACCEPTANCE_HELPERS)

JOY_TOPIC = ACCEPTANCE_HELPERS.JOY_TOPIC
MOTOR_COMMAND_TOPICS = ACCEPTANCE_HELPERS.MOTOR_COMMAND_TOPICS
SafeStandGyroFilter = ACCEPTANCE_HELPERS.SafeStandGyroFilter
controller_graph_diagnostic = (
    ACCEPTANCE_HELPERS.controller_graph_diagnostic
)
evaluate_controller_probe = ACCEPTANCE_HELPERS.evaluate_controller_probe
safe_stand_edge_evidence = ACCEPTANCE_HELPERS.safe_stand_edge_evidence
safe_stand_edge_is_valid = ACCEPTANCE_HELPERS.safe_stand_edge_is_valid


FOOT_TOPICS = (
    "/visual/FR_foot_contact/the_force",
    "/visual/FL_foot_contact/the_force",
    "/visual/RR_foot_contact/the_force",
    "/visual/RL_foot_contact/the_force",
)


def quaternion_to_roll_pitch(quaternion):
    sin_roll = 2.0 * (
        quaternion.w * quaternion.x + quaternion.y * quaternion.z
    )
    cos_roll = 1.0 - 2.0 * (
        quaternion.x * quaternion.x + quaternion.y * quaternion.y
    )
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (
        quaternion.w * quaternion.y - quaternion.z * quaternion.x
    )
    pitch = (
        math.copysign(math.pi / 2.0, sin_pitch)
        if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    )
    return roll, pitch


class SafeStopShortTest:
    def __init__(self):
        self.output = os.path.abspath(rospy.get_param("~output"))
        self.command_sim_s = float(rospy.get_param("~command_sim_s", 1.5))
        self.wall_timeout_s = float(rospy.get_param("~wall_timeout_s", 180.0))
        self.master = rosgraph.Master(rospy.get_name())
        self.lock = threading.RLock()
        self.ready = False
        self.safe_stand_ready = False
        self.safe_stand_edge = None
        self.safe_stand_gyro_filter = SafeStandGyroFilter()
        self.filtered_gyro = None
        self.controller_node = None
        self.controller_probe_diagnostic = None
        self.joy_message_stamp = None
        self.motor_command_stamps = {
            topic: None for topic in MOTOR_COMMAND_TOPICS
        }
        self.foot_forces = [None] * 4
        self.foot_stamps = [None] * 4
        self.imu = None
        self.final_cmd = None
        self.final_cmd_stamp = None
        self.max_roll = 0.0
        self.max_pitch = 0.0

        self.joy_pub = rospy.Publisher(JOY_TOPIC, Joy, queue_size=2)
        self.cmd_pub = rospy.Publisher("/cmd_vel_nav", Twist, queue_size=2)
        rospy.Subscriber(
            "/a1/controller_ready", Bool, self.ready_callback, queue_size=10
        )
        rospy.Subscriber(
            "/a1/safe_stand_ready",
            Bool,
            self.safe_stand_callback,
            queue_size=10,
        )
        rospy.Subscriber("/trunk_imu", Imu, self.imu_callback, queue_size=20)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=20)
        rospy.Subscriber(
            JOY_TOPIC, Joy, self.joy_callback, queue_size=2
        )
        for topic in MOTOR_COMMAND_TOPICS:
            rospy.Subscriber(
                topic,
                rospy.AnyMsg,
                lambda message, motor_topic=topic:
                self.motor_command_callback(message, motor_topic),
                queue_size=1,
            )
        for index, topic in enumerate(FOOT_TOPICS):
            rospy.Subscriber(
                topic,
                WrenchStamped,
                lambda message, leg=index: self.foot_callback(message, leg),
                queue_size=20,
            )

    def ready_callback(self, message):
        with self.lock:
            was_ready = self.ready
            self.ready = bool(message.data)
            if was_ready and not message.data:
                self.safe_stand_gyro_filter.reset()
                self.filtered_gyro = None

    def safe_stand_callback(self, message):
        with self.lock:
            rising_edge = bool(message.data) and not self.safe_stand_ready
            self.safe_stand_ready = bool(message.data)
            if rising_edge and self.safe_stand_edge is None:
                self.safe_stand_edge = safe_stand_edge_evidence(
                    rospy.Time.now().to_sec(),
                    self.foot_forces,
                    self.foot_stamps,
                    self.imu,
                    self.filtered_gyro,
                )

    def foot_callback(self, message, index):
        with self.lock:
            self.foot_forces[index] = abs(message.wrench.force.z)
            self.foot_stamps[index] = rospy.Time.now().to_sec()

    def imu_callback(self, message):
        now = rospy.Time.now().to_sec()
        roll, pitch = quaternion_to_roll_pitch(message.orientation)
        gyro = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        with self.lock:
            self.imu = (roll, pitch, gyro)
            self.filtered_gyro = self.safe_stand_gyro_filter.update(
                now, gyro
            )
            self.max_roll = max(self.max_roll, abs(roll))
            self.max_pitch = max(self.max_pitch, abs(pitch))

    def cmd_callback(self, message):
        with self.lock:
            self.final_cmd = (
                message.linear.x,
                message.linear.y,
                message.angular.z,
            )
            self.final_cmd_stamp = rospy.Time.now().to_sec()

    def joy_callback(self, _message):
        with self.lock:
            self.joy_message_stamp = rospy.Time.now().to_sec()

    def motor_command_callback(self, _message, topic):
        with self.lock:
            self.motor_command_stamps[topic] = rospy.Time.now().to_sec()

    def publish_button(self, index, wall_s=0.35):
        deadline = time.monotonic() + wall_s
        pressed = Joy(axes=[0.0] * 6, buttons=[0] * 11)
        pressed.buttons[index] = 1
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.joy_pub.publish(pressed)
            time.sleep(0.02)
        self.joy_pub.publish(Joy(axes=[0.0] * 6, buttons=[0] * 11))

    def controller_probe(self):
        try:
            graph = controller_graph_diagnostic(
                self.master.getSystemState(), rospy.resolve_name
            )
        except Exception as error:
            graph = {
                "joy_subscribers": [],
                "motor_publishers": [],
                "intersection": [],
                "motor_topic_publishers": {},
                "motor_topics_by_publisher": {},
                "graph_error": repr(error),
            }
        with self.lock:
            probe = evaluate_controller_probe(
                graph,
                rospy.Time.now().to_sec(),
                self.joy_message_stamp,
                dict(self.motor_command_stamps),
            )
            self.controller_node = probe["selected_node"]
            self.controller_probe_diagnostic = probe
        return probe

    def wait_for_live_controller_path(self):
        last_probe = None

        def live():
            nonlocal last_probe
            self.joy_pub.publish(
                Joy(axes=[0.0] * 6, buttons=[0] * 11)
            )
            last_probe = self.controller_probe()
            return last_probe["ready"]

        try:
            self.wait_for(live, "unique live A1 controller path", 10.0)
        except Exception as error:
            raise RuntimeError(
                "%s: %s" % (
                    error,
                    json.dumps(last_probe, sort_keys=True),
                )
            )

    def wait_for(self, predicate, description, sim_timeout_s):
        wall_deadline = time.monotonic() + self.wall_timeout_s
        start_sim = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
            now_sim = rospy.Time.now().to_sec()
            if predicate():
                return
            if now_sim < start_sim:
                raise RuntimeError(
                    "ROS/simulation clock moved backwards while waiting for "
                    + description
                )
            if now_sim - start_sim >= sim_timeout_s:
                raise RuntimeError("simulation timeout waiting for " + description)
            time.sleep(0.02)
        raise RuntimeError("wall timeout waiting for " + description)

    def sleep_sim(self, duration_s, command=None):
        start = rospy.Time.now().to_sec()
        wall_deadline = time.monotonic() + self.wall_timeout_s
        while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
            now = rospy.Time.now().to_sec()
            if now < start:
                raise RuntimeError("ROS/simulation clock moved backwards")
            if command is not None:
                self.cmd_pub.publish(command)
            if now - start >= duration_s:
                return
            time.sleep(0.02)
        raise RuntimeError("wall timeout while advancing simulation")

    def snapshot(self):
        with self.lock:
            now = rospy.Time.now().to_sec()
            force_fresh = [
                stamp is not None and 0.0 <= now - stamp <= 0.20
                for stamp in self.foot_stamps
            ]
            cmd_fresh = (
                self.final_cmd_stamp is not None
                and 0.0 <= now - self.final_cmd_stamp <= 0.25
            )
            return {
                "controller_ready": self.ready,
                "controller_node": self.controller_node,
                "controller_probe": self.controller_probe_diagnostic,
                "safe_stand_ready_seen": self.safe_stand_edge is not None,
                "safe_stand_edge": self.safe_stand_edge,
                "safe_stand_edge_valid": safe_stand_edge_is_valid(
                    self.safe_stand_edge
                ),
                "foot_forces_n": list(self.foot_forces),
                "foot_force_fresh": force_fresh,
                "final_cmd": self.final_cmd,
                "final_cmd_fresh": cmd_fresh,
                "max_abs_roll_rad": self.max_roll,
                "max_abs_pitch_rad": self.max_pitch,
            }

    def run(self):
        result = {"success": False, "error": ""}
        try:
            self.wait_for(
                lambda: rospy.Time.now().to_sec() > 0.0,
                "non-zero simulation clock",
                10.0,
            )
            self.wait_for(
                lambda: all(value is not None for value in self.foot_forces)
                and self.imu is not None,
                "IMU and four foot-force topics",
                10.0,
            )
            # Bring up fixed stand, let its joint interpolation settle, then
            # enter mode 5 through the ROS Joy path.
            self.wait_for_live_controller_path()
            self.publish_button(1)
            self.sleep_sim(4.0)
            self.publish_button(5)
            self.wait_for(lambda: self.ready, "controller_ready=true", 5.0)

            moving = Twist()
            moving.linear.x = 0.10
            self.sleep_sim(self.command_sim_s, moving)
            self.cmd_pub.publish(Twist())
            self.publish_button(1)
            self.wait_for(
                lambda: self.safe_stand_ready,
                "guarded all-foot stable stand transition",
                8.0,
            )
            self.sleep_sim(0.6, Twist())
            snapshot = self.snapshot()
            values = snapshot["final_cmd"] or (float("nan"),) * 3
            conditions = (
                snapshot["safe_stand_ready_seen"]
                and not snapshot["controller_ready"]
                and snapshot["safe_stand_edge_valid"]
                and snapshot["final_cmd_fresh"]
                and all(abs(value) <= 0.01 for value in values)
                and snapshot["max_abs_roll_rad"] < math.radians(20.0)
                and snapshot["max_abs_pitch_rad"] < math.radians(20.0)
            )
            result.update(snapshot)
            result["success"] = bool(conditions)
            if not conditions:
                result["error"] = "one or more safe-stop acceptance checks failed"
        except Exception as error:
            result.update(self.snapshot())
            result["error"] = str(error)
        finally:
            self.cmd_pub.publish(Twist())
            directory = os.path.dirname(self.output)
            os.makedirs(directory, exist_ok=True)
            descriptor, temporary = tempfile.mkstemp(
                prefix=".safe_stop_", dir=directory, text=True
            )
            with os.fdopen(descriptor, "w") as stream:
                json.dump(result, stream, indent=2, sort_keys=True)
                stream.write("\n")
            os.replace(temporary, self.output)
        return result["success"]


if __name__ == "__main__":
    rospy.init_node("safe_stop_short_test")
    success = SafeStopShortTest().run()
    rospy.signal_shutdown("safe-stop short test complete")
    raise SystemExit(0 if success else 1)
