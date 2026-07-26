#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt32

rospy.init_node("supervisor_test_estimator")
publisher = rospy.Publisher("/a1/localization/test_estimator_alive", UInt32,
                            queue_size=1, latch=True)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    publisher.publish(UInt32(rospy.Time.now().to_nsec() & 0xffffffff))
    rate.sleep()
