#!/usr/bin/env python3
"""Own and safely replace the FAST-LIO/localization estimator process group."""
import os
import signal
import subprocess
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from std_srvs.srv import Trigger, TriggerResponse


class LocalizationSupervisor:
    def __init__(self):
        self.lock = threading.RLock()
        self.process = None
        self.generation = 0
        self.restart_pending = False
        self.restart_reason = "INITIAL_START"
        self.restart_requested_at = None
        self.fresh_since = None
        self.last_input = {"pointcloud": None, "imu": None, "clock": None}
        self.launch = rospy.get_param("~managed_launch")
        self.automatic = rospy.get_param("~automatic_reinitialize", True)
        self.monitor_clock = rospy.get_param("~monitor_clock", True)
        self.input_timeout = rospy.get_param("~input_timeout", 3.0)
        self.settle_time = rospy.get_param("~settle_time", 2.0)
        self.shutdown_timeout = rospy.get_param("~shutdown_timeout", 8.0)
        if not os.path.isfile(self.launch):
            raise RuntimeError("managed launch does not exist: %s" % self.launch)
        self.status_pub = rospy.Publisher(
            rospy.get_param("~supervisor_status_topic"), DiagnosticStatus,
            queue_size=1, latch=True)
        rospy.Subscriber(rospy.get_param("~status_topic"), DiagnosticStatus,
                         self.health_callback, queue_size=10)
        rospy.Subscriber(rospy.get_param("~pointcloud_topic"), PointCloud2,
                         lambda _: self.observe("pointcloud"), queue_size=1)
        rospy.Subscriber(rospy.get_param("~imu_topic"), Imu,
                         lambda _: self.observe("imu"), queue_size=1)
        if self.monitor_clock:
            rospy.Subscriber(rospy.get_param("~clock_topic"), Clock,
                             lambda _: self.observe("clock"), queue_size=1)
        rospy.Service(rospy.get_param("~reinitialize_service"), Trigger,
                      self.manual_restart)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)
        rospy.on_shutdown(self.shutdown)
        self.start_estimator("INITIAL_START")

    def observe(self, name):
        with self.lock:
            self.last_input[name] = time.monotonic()

    def health_callback(self, status):
        values = {item.key: item.value for item in status.values}
        if values.get("reinitialization_required") != "true":
            return
        with self.lock:
            if not self.restart_pending:
                self.restart_pending = True
                self.restart_reason = values.get("reason", "ESTIMATOR_REQUEST")
                self.restart_requested_at = time.monotonic()
                self.fresh_since = None
                rospy.logerr("estimator reinitialization requested: %s",
                             self.restart_reason)

    def manual_restart(self, _request):
        with self.lock:
            if self.restart_pending:
                self.restart_reason = "MANUAL_REQUEST"
                self.restart_requested_at = time.monotonic()
                self.fresh_since = None
                return TriggerResponse(True, "pending reinitialization released manually")
            self.restart_pending = True
            self.restart_reason = "MANUAL_REQUEST"
            self.restart_requested_at = time.monotonic()
            self.fresh_since = None
        return TriggerResponse(True, "controlled reinitialization requested")

    def inputs_fresh(self, now):
        names = ["pointcloud", "imu"] + (["clock"] if self.monitor_clock else [])
        return all(self.last_input[name] is not None and
                   (self.restart_requested_at is None or
                    self.last_input[name] >= self.restart_requested_at) and
                   now - self.last_input[name] <= self.input_timeout for name in names)

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
                         KeyValue("restart_pending", str(self.restart_pending).lower())]
        self.status_pub.publish(status)

    def tick(self, _event):
        with self.lock:
            now = time.monotonic()
            if self.process is not None and self.process.poll() is not None:
                code = self.process.returncode
                self.process = None
                if not self.restart_pending:
                    self.restart_pending = True
                    self.restart_reason = "ESTIMATOR_EXIT_%d" % code
                    self.restart_requested_at = now
                    self.fresh_since = None
            if not self.restart_pending:
                self.publish("RUNNING", "ESTIMATOR_ACTIVE")
                return
            if not self.stop_estimator():
                self.publish("STOP_FAILED", self.restart_reason)
                return
            if not self.automatic and self.restart_reason != "MANUAL_REQUEST":
                self.publish("WAITING_FOR_MANUAL_REINITIALIZE", self.restart_reason)
                return
            if not self.inputs_fresh(now):
                self.fresh_since = None
                self.publish("WAITING_FOR_INPUTS", self.restart_reason)
                return
            if self.fresh_since is None:
                self.fresh_since = now
            if now - self.fresh_since < self.settle_time:
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
