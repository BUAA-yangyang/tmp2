#!/usr/bin/env python3
"""DEV-ONLY constant-velocity gait stability diagnostic.

This node deliberately does not start move_base or exploration.  It publishes
one constant command through /cmd_vel_nav -> a1_cmd_mux -> /cmd_vel, monitors
truth odometry and IMU only for simulation diagnosis, and fail-closes through
the cmd_mux safety lock.
"""

import json
import math
import os
import tempfile
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


JOINTS = (
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
)
FOOT_FORCE_TOPICS = (
    "/visual/FR_foot_contact/the_force",
    "/visual/FL_foot_contact/the_force",
    "/visual/RR_foot_contact/the_force",
    "/visual/RL_foot_contact/the_force",
)


def quaternion_to_rpy(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw


def angle_error(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


class StabilityDiagnostic:
    def __init__(self):
        self.case_id = rospy.get_param("~case_id")
        self.surface = rospy.get_param("~surface", "unspecified")
        self.livox_enabled = bool(rospy.get_param("~livox_enabled", True))
        self.duration_sim = float(rospy.get_param("~duration_sim_s", 200.0))
        self.command_rate = float(rospy.get_param("~command_rate_hz", 20.0))
        self.vx = float(rospy.get_param("~vx", 0.15))
        self.vy = float(rospy.get_param("~vy", 0.0))
        self.wz = float(rospy.get_param("~wz", 0.0))
        self.tilt_limit = math.radians(float(rospy.get_param("~tilt_limit_deg", 35.0)))
        self.ready_timeout_wall = float(rospy.get_param("~ready_timeout_wall_s", 180.0))
        self.wall_timeout = float(rospy.get_param("~wall_timeout_s", 7200.0))
        self.clock_stall_wall = float(rospy.get_param("~clock_stall_wall_s", 10.0))
        self.arming_wall = float(rospy.get_param("~arming_wall_s", 3.0))
        self.position_tolerance = float(rospy.get_param("~start_position_tolerance_m", 0.75))
        self.yaw_tolerance = math.radians(float(rospy.get_param("~start_yaw_tolerance_deg", 12.0)))
        self.strict_topics = bool(rospy.get_param("~strict_topics", True))
        self.output_path = os.path.abspath(rospy.get_param("~output"))
        self.expected_x = self._optional_param("~expected_x")
        self.expected_y = self._optional_param("~expected_y")
        expected_yaw_deg = self._optional_param("~expected_yaw_deg")
        self.expected_yaw = None if expected_yaw_deg is None else math.radians(expected_yaw_deg)

        if self.duration_sim < 1.0:
            raise ValueError("duration_sim_s must be at least 1.0")
        if self.command_rate <= 0.0:
            raise ValueError("command_rate_hz must be positive")
        if not (0.0 < self.tilt_limit < math.pi / 2.0):
            raise ValueError("tilt_limit_deg must be between 0 and 90")

        self.guard = threading.RLock()
        self.clock = None
        self.clock_wall = None
        self.first_clock = None
        self.ready = False
        self.ready_wall = None
        self.odom = None
        self.odom_wall = None
        self.imu = None
        self.last_output_cmd = None
        self.last_output_cmd_wall = None
        self.start_pose = None
        self.end_pose = None
        self.previous_position = None
        self.distance = 0.0
        self.max_abs_roll = 0.0
        self.max_abs_pitch = 0.0
        self.max_abs_imu_roll = 0.0
        self.max_abs_imu_pitch = 0.0
        self.max_abs_roll_rate = 0.0
        self.previous_roll = None
        self.previous_roll_sim = None
        self.max_cmd = [0.0, 0.0, 0.0]
        self.rtf_samples = []
        self.rtf_window_wall = None
        self.rtf_window_sim = None
        self.started_sim = None
        self.started_wall = None
        self.finished_sim = None
        self.finished_wall = None
        self.stop_reason = None
        self.valid = False
        self.missing_topics = []
        self.final_cmd_zero = False
        self.last_diag_sim = None

        self.command = Twist()
        self.command.linear.x = self.vx
        self.command.linear.y = self.vy
        self.command.angular.z = self.wz
        self.zero = Twist()

        self.command_pub = rospy.Publisher("/cmd_vel_nav", Twist, queue_size=1)
        self.lock_pub = rospy.Publisher(
            "/a1_cmd_mux/safety_lock", Bool, queue_size=1, latch=True
        )
        self.diagnostic_pub = rospy.Publisher(
            "/a1/navigation_tests/stability_diagnostic",
            DiagnosticArray,
            queue_size=2,
        )
        rospy.Subscriber("/clock", Clock, self._clock_callback, queue_size=20)
        rospy.Subscriber(
            "/Odometry_gazebo", Odometry, self._odom_callback, queue_size=20
        )
        rospy.Subscriber("/trunk_imu", Imu, self._imu_callback, queue_size=20)
        rospy.Subscriber(
            "/a1/controller_ready", Bool, self._ready_callback, queue_size=5
        )
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_callback, queue_size=20)
        rospy.on_shutdown(self._shutdown)

    @staticmethod
    def _optional_param(name):
        return float(rospy.get_param(name)) if rospy.has_param(name) else None

    def _clock_callback(self, message):
        wall_now = time.monotonic()
        sim_now = message.clock.to_sec()
        with self.guard:
            self.clock = sim_now
            self.clock_wall = wall_now
            if self.first_clock is None:
                self.first_clock = sim_now
            if self.rtf_window_wall is None:
                self.rtf_window_wall = wall_now
                self.rtf_window_sim = sim_now
            elif wall_now - self.rtf_window_wall >= 1.0:
                wall_delta = wall_now - self.rtf_window_wall
                sim_delta = max(0.0, sim_now - self.rtf_window_sim)
                self.rtf_samples.append(sim_delta / wall_delta)
                self.rtf_window_wall = wall_now
                self.rtf_window_sim = sim_now

    def _odom_callback(self, message):
        wall_now = time.monotonic()
        roll, pitch, yaw = quaternion_to_rpy(message.pose.pose.orientation)
        position = message.pose.pose.position
        should_stop = False
        with self.guard:
            self.odom = (position.x, position.y, position.z, roll, pitch, yaw)
            self.odom_wall = wall_now
            if self.started_sim is None:
                self.end_pose = self.odom
            elif self.stop_reason is None:
                self.end_pose = self.odom
                self.max_abs_roll = max(self.max_abs_roll, abs(roll))
                self.max_abs_pitch = max(self.max_abs_pitch, abs(pitch))
                current = (position.x, position.y, position.z)
                if self.previous_position is not None:
                    self.distance += math.sqrt(
                        sum(
                            (current[index] - self.previous_position[index]) ** 2
                            for index in range(3)
                        )
                    )
                self.previous_position = current
                if (
                    self.previous_roll is not None
                    and self.previous_roll_sim is not None
                    and self.clock is not None
                ):
                    delta_sim = self.clock - self.previous_roll_sim
                    if delta_sim > 1e-4:
                        self.max_abs_roll_rate = max(
                            self.max_abs_roll_rate,
                            abs(angle_error(roll, self.previous_roll)) / delta_sim,
                        )
                self.previous_roll = roll
                self.previous_roll_sim = self.clock
                should_stop = max(abs(roll), abs(pitch)) >= self.tilt_limit
        if should_stop:
            self._request_stop("tilt_limit")

    def _imu_callback(self, message):
        roll, pitch, _ = quaternion_to_rpy(message.orientation)
        with self.guard:
            self.imu = (roll, pitch)
            if self.started_sim is not None and self.stop_reason is None:
                self.max_abs_imu_roll = max(self.max_abs_imu_roll, abs(roll))
                self.max_abs_imu_pitch = max(self.max_abs_imu_pitch, abs(pitch))

    def _ready_callback(self, message):
        with self.guard:
            self.ready = bool(message.data)
            if self.ready and self.ready_wall is None:
                self.ready_wall = time.monotonic()

    def _cmd_callback(self, message):
        with self.guard:
            self.last_output_cmd = (
                message.linear.x,
                message.linear.y,
                message.angular.z,
            )
            self.last_output_cmd_wall = time.monotonic()
            self.max_cmd[0] = max(self.max_cmd[0], abs(message.linear.x))
            self.max_cmd[1] = max(self.max_cmd[1], abs(message.linear.y))
            self.max_cmd[2] = max(self.max_cmd[2], abs(message.angular.z))

    def _required_topics(self):
        topics = {
            "/clock",
            "/Odometry_gazebo",
            "/trunk_imu",
            "/a1/controller_ready",
            "/cmd_vel",
        }
        topics.update(FOOT_FORCE_TOPICS)
        for joint in JOINTS:
            topics.add("/a1_gazebo/{}_controller/command".format(joint))
            topics.add("/a1_gazebo/{}_controller/state".format(joint))
        return topics

    def _topic_check(self):
        published = {name for name, _ in rospy.get_published_topics()}
        return sorted(self._required_topics() - published)

    def _start_pose_error(self):
        if self.odom is None:
            return "odometry unavailable"
        x, y, _, _, _, yaw = self.odom
        if self.expected_x is not None and self.expected_y is not None:
            error = math.hypot(x - self.expected_x, y - self.expected_y)
            if error > self.position_tolerance:
                return "start position error {:.3f} m".format(error)
        if self.expected_yaw is not None:
            error = abs(angle_error(yaw, self.expected_yaw))
            if error > self.yaw_tolerance:
                return "start yaw error {:.1f} deg".format(math.degrees(error))
        return None

    def _request_stop(self, reason):
        with self.guard:
            if self.stop_reason is not None:
                return
            self.stop_reason = reason
            self.finished_sim = self.clock
            self.finished_wall = time.monotonic()
        self.lock_pub.publish(Bool(data=True))
        self.command_pub.publish(self.zero)
        rospy.logwarn("Stability diagnostic stop: %s", reason)

    def _publish_diagnostic(self):
        with self.guard:
            sim_now = self.clock
            if sim_now is None:
                return
            if (
                self.last_diag_sim is not None
                and sim_now - self.last_diag_sim < 0.1
            ):
                return
            self.last_diag_sim = sim_now
            elapsed = (
                0.0 if self.started_sim is None else max(0.0, sim_now - self.started_sim)
            )
            roll = 0.0 if self.odom is None else self.odom[3]
            pitch = 0.0 if self.odom is None else self.odom[4]
            yaw = 0.0 if self.odom is None else self.odom[5]
            rtf = self.rtf_samples[-1] if self.rtf_samples else 0.0
            reason = self.stop_reason
        status = DiagnosticStatus()
        status.name = "a1_constant_velocity_stability"
        status.hardware_id = "gazebo_dev_only"
        status.level = DiagnosticStatus.ERROR if reason not in (None, "completed") else DiagnosticStatus.OK
        status.message = reason or ("running" if self.started_sim is not None else "armed")
        status.values = [
            KeyValue("case_id", self.case_id),
            KeyValue("surface", self.surface),
            KeyValue("livox_enabled", str(self.livox_enabled).lower()),
            KeyValue("elapsed_sim_s", "{:.6f}".format(elapsed)),
            KeyValue("rtf", "{:.6f}".format(rtf)),
            KeyValue("roll_deg", "{:.6f}".format(math.degrees(roll))),
            KeyValue("pitch_deg", "{:.6f}".format(math.degrees(pitch))),
            KeyValue("yaw_deg", "{:.6f}".format(math.degrees(yaw))),
            KeyValue("distance_m", "{:.6f}".format(self.distance)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.from_sec(sim_now)
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def _write_result(self):
        with self.guard:
            sim_duration = (
                0.0
                if self.started_sim is None or self.finished_sim is None
                else max(0.0, self.finished_sim - self.started_sim)
            )
            wall_duration = (
                0.0
                if self.started_wall is None or self.finished_wall is None
                else max(0.0, self.finished_wall - self.started_wall)
            )
            result = {
                "schema_version": 1,
                "dev_only": True,
                "case_id": self.case_id,
                "surface": self.surface,
                "livox_enabled": self.livox_enabled,
                "command": {"vx": self.vx, "vy": self.vy, "wz": self.wz},
                "requested_duration_sim_s": self.duration_sim,
                "outcome": self.stop_reason or "shutdown",
                "valid": self.valid,
                "passed": self.valid
                and self.stop_reason == "completed"
                and self.final_cmd_zero,
                "sim_duration_s": sim_duration,
                "wall_duration_s": wall_duration,
                "overall_rtf": sim_duration / wall_duration if wall_duration > 0.0 else 0.0,
                "rtf_min": min(self.rtf_samples) if self.rtf_samples else 0.0,
                "rtf_max": max(self.rtf_samples) if self.rtf_samples else 0.0,
                "rtf_samples": len(self.rtf_samples),
                "distance_m": self.distance,
                "max_abs_roll_deg": math.degrees(self.max_abs_roll),
                "max_abs_pitch_deg": math.degrees(self.max_abs_pitch),
                "max_abs_imu_roll_deg": math.degrees(self.max_abs_imu_roll),
                "max_abs_imu_pitch_deg": math.degrees(self.max_abs_imu_pitch),
                "max_abs_roll_rate_deg_s": math.degrees(self.max_abs_roll_rate),
                "max_abs_cmd_vel": {
                    "vx": self.max_cmd[0],
                    "vy": self.max_cmd[1],
                    "wz": self.max_cmd[2],
                },
                "start_pose": self.start_pose,
                "end_pose": self.end_pose,
                "missing_topics": self.missing_topics,
                "final_cmd_zero": self.final_cmd_zero,
            }
        directory = os.path.dirname(self.output_path)
        os.makedirs(directory, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            mode="w", dir=directory, delete=False, prefix=".stability_", suffix=".json"
        ) as temporary:
            json.dump(result, temporary, indent=2, sort_keys=True)
            temporary.write("\n")
            temporary_path = temporary.name
        os.replace(temporary_path, self.output_path)
        rospy.loginfo("Wrote stability result to %s", self.output_path)

    def _shutdown(self):
        self.lock_pub.publish(Bool(data=True))
        self.command_pub.publish(self.zero)

    def run(self):
        process_start = time.monotonic()
        last_lock_wall = 0.0
        last_command_sim = None
        rospy.loginfo(
            "Waiting to arm stability case %s; cmd_mux remains safety-locked",
            self.case_id,
        )
        while not rospy.is_shutdown() and self.started_sim is None:
            wall_now = time.monotonic()
            if wall_now - last_lock_wall >= 0.2:
                self.lock_pub.publish(Bool(data=True))
                self.command_pub.publish(self.zero)
                last_lock_wall = wall_now
            with self.guard:
                ready = self.ready
                ready_wall = self.ready_wall
                data_ready = self.clock is not None and self.odom is not None and self.imu is not None
            if ready and data_ready and ready_wall is not None and wall_now - ready_wall >= self.arming_wall:
                self.missing_topics = self._topic_check()
                if self.strict_topics and self.missing_topics:
                    self._request_stop("missing_topics")
                    break
                pose_error = self._start_pose_error()
                if pose_error is not None:
                    rospy.logerr("Refusing to drive: %s", pose_error)
                    self._request_stop("invalid_start_pose")
                    break
                with self.guard:
                    self.started_sim = self.clock
                    self.started_wall = wall_now
                    self.start_pose = self.odom
                    self.previous_position = self.odom[:3]
                    self.previous_roll = self.odom[3]
                    self.previous_roll_sim = self.clock
                    self.distance = 0.0
                    self.max_abs_roll = abs(self.odom[3])
                    self.max_abs_pitch = abs(self.odom[4])
                    self.max_abs_imu_roll = abs(self.imu[0])
                    self.max_abs_imu_pitch = abs(self.imu[1])
                    self.max_abs_roll_rate = 0.0
                    self.max_cmd = [0.0, 0.0, 0.0]
                    self.rtf_samples = []
                    self.rtf_window_wall = wall_now
                    self.rtf_window_sim = self.clock
                    self.valid = True
                self.lock_pub.publish(Bool(data=False))
                rospy.loginfo("Armed case %s at sim time %.3f", self.case_id, self.started_sim)
                break
            if wall_now - process_start > self.ready_timeout_wall:
                self._request_stop("controller_not_ready")
                break
            time.sleep(0.02)

        while not rospy.is_shutdown() and self.stop_reason is None:
            wall_now = time.monotonic()
            with self.guard:
                sim_now = self.clock
                clock_wall = self.clock_wall
                started_sim = self.started_sim
            if started_sim is None:
                break
            if clock_wall is None or wall_now - clock_wall > self.clock_stall_wall:
                self._request_stop("clock_stall")
                break
            if wall_now - self.started_wall > self.wall_timeout:
                self._request_stop("wall_timeout")
                break
            if sim_now - started_sim >= self.duration_sim:
                self._request_stop("completed")
                break
            if last_command_sim is None or sim_now - last_command_sim >= 1.0 / self.command_rate:
                self.command_pub.publish(self.command)
                last_command_sim = sim_now
            self._publish_diagnostic()
            time.sleep(0.01)

        stop_wall = time.monotonic()
        while not rospy.is_shutdown() and time.monotonic() - stop_wall < 2.0:
            self.lock_pub.publish(Bool(data=True))
            self.command_pub.publish(self.zero)
            self._publish_diagnostic()
            with self.guard:
                if self.last_output_cmd is not None:
                    self.final_cmd_zero = all(
                        abs(component) <= 1e-3 for component in self.last_output_cmd
                    )
            time.sleep(0.05)
        if self.finished_wall is None:
            with self.guard:
                self.finished_wall = time.monotonic()
                self.finished_sim = self.clock
        self._write_result()
        rospy.signal_shutdown(self.stop_reason or "finished")


def main():
    rospy.init_node("constant_velocity_stability_diagnostic")
    diagnostic = StabilityDiagnostic()
    diagnostic.run()


if __name__ == "__main__":
    main()
