#!/usr/bin/env python3
"""Own and safely replace the FAST-LIO/localization estimator process group."""
import os
import signal
import subprocess
import threading
import time
from collections import deque
from math import sqrt

import rospy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class LocalizationSupervisor:
    def __init__(self):
        self.lock = threading.RLock()
        self.process = None
        self.generation = 0
        # Initial estimator creation follows the same guarded path as a restart.
        # Starting FAST-LIO while the robot is falling or changing controller
        # modes corrupts its gravity/bias initialization and produces horizontal
        # drift that later LiDAR updates may not remove.
        self.restart_pending = True
        self.restart_reason = "INITIAL_START"
        self.last_fault_reason = None
        self.same_reason_restart_count = 0
        self.restart_requested_at = None
        self.fresh_since = None
        self.last_input_stamp = {"pointcloud": None, "imu": None, "clock": None}
        self.last_input_wall = {"pointcloud": None, "imu": None, "clock": None}
        self.imu_samples = deque()
        self.controller_state = None
        self.launch = rospy.get_param("~managed_launch")
        self.automatic = rospy.get_param("~automatic_reinitialize", True)
        self.monitor_clock = rospy.get_param("~monitor_clock", True)
        self.input_timeout = rospy.get_param("~input_timeout", 3.0)
        self.settle_time = rospy.get_param("~settle_time", 2.0)
        self.imu_stability_window = rospy.get_param("~imu_stability_window", 2.0)
        self.imu_min_samples = rospy.get_param("~imu_min_samples", 50)
        self.max_gyro_std = rospy.get_param("~max_gyro_std", 0.03)
        self.max_accel_std = rospy.get_param("~max_accel_std", 0.20)
        self.max_horizontal_accel_mean = rospy.get_param(
            "~max_horizontal_accel_mean", 1.0)
        self.min_vertical_accel_mean = rospy.get_param("~min_vertical_accel_mean", 8.0)
        self.max_vertical_accel_mean = rospy.get_param("~max_vertical_accel_mean", 11.0)
        self.shutdown_timeout = rospy.get_param("~shutdown_timeout", 8.0)
        self.max_automatic_restarts_per_reason = rospy.get_param(
            "~max_automatic_restarts_per_reason", 1)
        self.require_controller_ready = rospy.get_param(
            "~require_controller_ready", True)
        self.controller_ready_state = rospy.get_param(
            "~controller_ready_state", "fixed stand")
        if not os.path.isfile(self.launch):
            raise RuntimeError("managed launch does not exist: %s" % self.launch)
        self.status_pub = rospy.Publisher(
            rospy.get_param("~supervisor_status_topic"), DiagnosticStatus,
            queue_size=1, latch=True)
        rospy.Subscriber(rospy.get_param("~status_topic"), DiagnosticStatus,
                         self.health_callback, queue_size=10)
        rospy.Subscriber(rospy.get_param("~pointcloud_topic"), PointCloud2,
                         lambda message: self.observe("pointcloud", message.header.stamp), queue_size=1)
        rospy.Subscriber(rospy.get_param("~imu_topic"), Imu,
                         self.imu_callback, queue_size=100)
        if self.monitor_clock:
            rospy.Subscriber(rospy.get_param("~clock_topic"), Clock,
                             lambda message: self.observe("clock", message.clock), queue_size=1)
        if self.require_controller_ready:
            rospy.Subscriber(rospy.get_param("~controller_state_topic"), String,
                             self.controller_state_callback, queue_size=1)
        rospy.Service(rospy.get_param("~reinitialize_service"), Trigger,
                      self.manual_restart)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)
        rospy.on_shutdown(self.shutdown)

    def imu_callback(self, message):
        self.observe("imu", message.header.stamp)
        sample_time = message.header.stamp.to_sec()
        sample = (sample_time,
                  message.angular_velocity.x, message.angular_velocity.y,
                  message.angular_velocity.z, message.linear_acceleration.x,
                  message.linear_acceleration.y, message.linear_acceleration.z)
        with self.lock:
            self.imu_samples.append(sample)
            cutoff = sample_time - self.imu_stability_window
            while self.imu_samples and self.imu_samples[0][0] < cutoff:
                self.imu_samples.popleft()

    def controller_state_callback(self, message):
        with self.lock:
            self.controller_state = message.data

    @staticmethod
    def population_std(values):
        mean = sum(values) / len(values)
        return sqrt(sum((value - mean) ** 2 for value in values) / len(values))

    def imu_stable(self):
        samples = list(self.imu_samples)
        if len(samples) < self.imu_min_samples:
            return False
        if samples[-1][0] - samples[0][0] < 0.9 * self.imu_stability_window:
            return False
        columns = list(zip(*samples))[1:]
        gyro_std = max(self.population_std(column) for column in columns[:3])
        accel_std = max(self.population_std(column) for column in columns[3:])
        accel_mean = [sum(column) / len(column) for column in columns[3:]]
        return (gyro_std <= self.max_gyro_std and accel_std <= self.max_accel_std and
                sqrt(accel_mean[0] ** 2 + accel_mean[1] ** 2) <=
                self.max_horizontal_accel_mean and
                self.min_vertical_accel_mean <= accel_mean[2] <=
                self.max_vertical_accel_mean)

    def observe(self, name, stamp):
        with self.lock:
            self.last_input_stamp[name] = stamp
            self.last_input_wall[name] = time.monotonic()

    def health_callback(self, status):
        values = {item.key: item.value for item in status.values}
        if values.get("reinitialization_required") != "true":
            return
        with self.lock:
            if not self.restart_pending:
                self.restart_pending = True
                fault_reason = values.get("reason", "ESTIMATOR_REQUEST")
                if fault_reason == self.last_fault_reason:
                    self.same_reason_restart_count += 1
                else:
                    self.last_fault_reason = fault_reason
                    self.same_reason_restart_count = 1
                self.restart_reason = fault_reason
                self.restart_requested_at = time.monotonic()
                self.fresh_since = None
                self.clear_recovery_inputs()
                rospy.logerr("estimator reinitialization requested: %s",
                             self.restart_reason)

    def manual_restart(self, _request):
        with self.lock:
            if self.restart_pending:
                self.restart_reason = "MANUAL_REQUEST"
                self.last_fault_reason = None
                self.same_reason_restart_count = 0
                self.restart_requested_at = time.monotonic()
                self.fresh_since = None
                self.clear_recovery_inputs()
                return TriggerResponse(True, "pending reinitialization released manually")
            self.restart_pending = True
            self.restart_reason = "MANUAL_REQUEST"
            self.last_fault_reason = None
            self.same_reason_restart_count = 0
            self.restart_requested_at = time.monotonic()
            self.fresh_since = None
            self.clear_recovery_inputs()
        return TriggerResponse(True, "controlled reinitialization requested")

    def clear_recovery_inputs(self):
        self.imu_samples.clear()
        for name in self.last_input_stamp:
            self.last_input_stamp[name] = None
            self.last_input_wall[name] = None

    def inputs_fresh(self, now_ros):
        names = ["pointcloud", "imu"] + (["clock"] if self.monitor_clock else [])
        return all(self.last_input_stamp[name] is not None and
                   not self.last_input_stamp[name].is_zero() and
                   not now_ros.is_zero() and
                   (self.restart_requested_at is None or
                    self.last_input_wall[name] >= self.restart_requested_at) and
                   0.0 <= (now_ros - self.last_input_stamp[name]).to_sec() <= self.input_timeout
                   for name in names)

    def stop_estimator(self):
        process = self.process
        if process is None:
            return True
        if process.poll() is None:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            deadline = time.monotonic() + self.shutdown_timeout
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if process.poll() is None:
                rospy.logwarn("estimator did not stop after SIGINT; sending SIGTERM")
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    rospy.logerr("estimator process group did not terminate")
                    return False
        self.process = None
        return True

    def start_estimator(self, reason):
        if self.process is not None and self.process.poll() is None:
            raise RuntimeError("refusing to create a duplicate estimator process")
        self.generation += 1
        child_env = os.environ.copy()
        # roslaunch exports the supervisor node namespace. The managed launch
        # already declares its own namespace, so inheriting it would produce
        # /a1_localization/a1_localization/... node names.
        child_env.pop("ROS_NAMESPACE", None)
        self.process = subprocess.Popen(
            ["roslaunch", "--wait", self.launch], start_new_session=True,
            env=child_env)
        rospy.loginfo("started estimator generation=%d pid=%d reason=%s",
                      self.generation, self.process.pid, reason)
        self.publish("STARTING", reason)

    def publish(self, message, reason):
        status = DiagnosticStatus()
        status.name = "a1_localization/supervisor"
        status.hardware_id = "fast_lio_process"
        status.level = (DiagnosticStatus.OK if message == "RUNNING"
                        else DiagnosticStatus.WARN)
        status.message = message
        status.values = [KeyValue("generation", str(self.generation)),
                         KeyValue("reason", reason),
                         KeyValue("restart_pending", str(self.restart_pending).lower()),
                         KeyValue("same_reason_restart_count",
                                  str(self.same_reason_restart_count))]
        self.status_pub.publish(status)

    def tick(self, _event):
        with self.lock:
            now = time.monotonic()
            now_ros = rospy.Time.now()
            if self.process is not None and self.process.poll() is not None:
                code = self.process.returncode
                self.process = None
                if not self.restart_pending:
                    self.restart_pending = True
                    self.restart_reason = "ESTIMATOR_EXIT_%d" % code
                    self.restart_requested_at = now
                    self.fresh_since = None
                    self.clear_recovery_inputs()
            if not self.restart_pending:
                self.publish("RUNNING", "ESTIMATOR_ACTIVE")
                return
            if not self.stop_estimator():
                self.publish("STOP_FAILED", self.restart_reason)
                return
            if not self.automatic and self.restart_reason != "MANUAL_REQUEST":
                self.publish("WAITING_FOR_MANUAL_REINITIALIZE", self.restart_reason)
                return
            if (self.restart_reason != "MANUAL_REQUEST" and
                    self.same_reason_restart_count >
                    self.max_automatic_restarts_per_reason):
                self.publish("RESTART_LIMIT_REACHED", self.restart_reason)
                return
            if not self.inputs_fresh(now_ros):
                self.fresh_since = None
                self.publish("WAITING_FOR_INPUTS", self.restart_reason)
                return
            if (self.require_controller_ready and
                    self.controller_state != self.controller_ready_state):
                self.fresh_since = None
                self.publish("WAITING_FOR_CONTROLLER_READY", self.restart_reason)
                return
            if not self.imu_stable():
                self.fresh_since = None
                self.publish("WAITING_FOR_IMU_STABILITY", self.restart_reason)
                return
            if self.fresh_since is None:
                self.fresh_since = now_ros
            if now_ros < self.fresh_since or (now_ros - self.fresh_since).to_sec() < self.settle_time:
                self.publish("WAITING_FOR_INPUT_SETTLING", self.restart_reason)
                return
            reason = self.restart_reason
            self.restart_pending = False
            self.restart_requested_at = None
            self.fresh_since = None
            self.start_estimator(reason)

    def shutdown(self):
        with self.lock:
            self.stop_estimator()


if __name__ == "__main__":
    rospy.init_node("localization_supervisor")
    LocalizationSupervisor()
    rospy.spin()
