#!/usr/bin/env python3
import time
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class HealthGateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("floor_mapping_health_gate_test")
        cls.allowed=[];cls.commands=[]
        cls.status_pub=rospy.Publisher("/test/gate/status",DiagnosticStatus,queue_size=1,latch=True)
        rospy.Subscriber("/mapping_gate/mapping_usable",Bool,lambda m:cls.allowed.append(m.data))
        rospy.Subscriber("/test/gate/cmd_vel",Twist,cls.commands.append)
        time.sleep(.5)

    def publish(self,state="MAPPING",valid=True,generation=7):
        message=DiagnosticStatus(message=state,values=[KeyValue("map_valid",str(valid).lower()),KeyValue("obstacle_cloud_valid",str(valid).lower()),KeyValue("localization_generation",str(generation)),KeyValue("floor_session_id","2"),KeyValue("pointcloud_age_sec","0.05"),KeyValue("last_success_tf_age_sec","0.05")])
        self.status_pub.publish(message);time.sleep(.12)

    def test_gate_closes_on_health_generation_and_timeout(self):
        self.publish();self.assertTrue(self.allowed[-1])
        self.publish(generation=8);self.assertFalse(self.allowed[-1]);self.assertTrue(self.commands);self.assertEqual(self.commands[-1],Twist())
        self.publish();self.assertTrue(self.allowed[-1])
        time.sleep(.5);self.assertFalse(self.allowed[-1]);self.assertEqual(self.commands[-1],Twist())
        self.publish(state="LOST",valid=False);self.assertFalse(self.allowed[-1])


if __name__ == "__main__": rostest.rosrun("a1_floor_mapping","floor_mapping_health_gate",HealthGateTest)
