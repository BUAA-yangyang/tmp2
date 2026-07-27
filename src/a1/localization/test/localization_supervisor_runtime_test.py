#!/usr/bin/env python3
import time
import unittest

import rosnode
import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2


class SupervisorRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("localization_supervisor_runtime_test", anonymous=True)
        cls.health_pub = rospy.Publisher("/test/localization/status", DiagnosticStatus,
                                         queue_size=1)
        cls.cloud_pub = rospy.Publisher("/test/localization/cloud", PointCloud2,
                                        queue_size=1)
        cls.imu_pub = rospy.Publisher("/test/localization/imu", Imu, queue_size=1)
        cls.clock_pub = rospy.Publisher("/test/localization/clock", Clock, queue_size=1)
        cls.statuses = []
        cls.status_sub = rospy.Subscriber("/test/localization/supervisor_status",
                                          DiagnosticStatus, cls.statuses.append)

    @staticmethod
    def values(status):
        return {item.key: item.value for item in status.values}

    def wait_generation(self, generation, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.statuses and int(self.values(self.statuses[-1])["generation"]) >= generation \
                    and self.statuses[-1].message == "RUNNING":
                return
            time.sleep(0.05)
        self.fail("supervisor did not reach running generation %d; last=%s" %
                  (generation, self.statuses[-1].message if self.statuses else "none"))

    def publish_inputs(self):
        stamp = rospy.Time.now()
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        imu = Imu()
        imu.header.stamp = stamp
        self.cloud_pub.publish(cloud)
        self.imu_pub.publish(imu)
        self.clock_pub.publish(Clock(clock=stamp))

    def test_three_controlled_reinitializations(self):
        self.wait_generation(1)
        connection_deadline = time.monotonic() + 5.0
        while time.monotonic() < connection_deadline and not self.health_pub.get_num_connections():
            time.sleep(0.05)
        self.assertTrue(self.health_pub.get_num_connections(),
                        "supervisor did not subscribe to health status")
        for expected_generation in range(2, 5):
            fault = DiagnosticStatus()
            fault.values = [KeyValue("reinitialization_required", "true"),
                            KeyValue("reason", "CLOCK_TIME_ROLLBACK")]
            # Pre-fault samples must never satisfy the recovery gate. This
            # specifically protects reset cases where a sensor plugin stops.
            # The managed roslaunch is allowed to consume shutdown_timeout
            # before the supervisor can publish WAITING_FOR_INPUTS.
            fault_deadline = time.monotonic() + 8.0
            while time.monotonic() < fault_deadline:
                self.health_pub.publish(fault)
                if self.statuses and self.statuses[-1].message == "WAITING_FOR_INPUTS":
                    break
                time.sleep(0.05)
            self.assertLess(int(self.values(self.statuses[-1])["generation"]),
                            expected_generation)
            self.assertEqual(self.statuses[-1].message, "WAITING_FOR_INPUTS")
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline:
                self.publish_inputs()
                if self.statuses and self.statuses[-1].message in (
                        "WAITING_FOR_INPUT_SETTLING", "RUNNING"):
                    if int(self.values(self.statuses[-1])["generation"]) >= expected_generation:
                        break
                time.sleep(0.05)
            self.wait_generation(expected_generation)
            node_deadline = time.monotonic() + 5.0
            estimator_nodes = []
            while time.monotonic() < node_deadline:
                estimator_nodes = [name for name in rosnode.get_node_names()
                                   if name.endswith("/supervisor_test_estimator")]
                if estimator_nodes:
                    break
                time.sleep(0.05)
            self.assertEqual(len(estimator_nodes), 1,
                             "duplicate estimator nodes: %r" % estimator_nodes)


if __name__ == "__main__":
    rostest.rosrun("a1_localization", "localization_supervisor_runtime_test",
                   SupervisorRuntimeTest)
