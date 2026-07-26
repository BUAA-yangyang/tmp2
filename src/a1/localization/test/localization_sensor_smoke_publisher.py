#!/usr/bin/env python3
"""Deterministic sensor source for FAST-LIO integration smoke tests.

This is not an accuracy test and never enters a production launch. It exists so
the complete map path can be exercised when Gazebo is unavailable.
"""
import math
import time

import rospy
from geometry_msgs.msg import Point32
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import ChannelFloat32, Imu, PointCloud


def main():
    rospy.init_node("localization_sensor_smoke_publisher")
    cloud_pub = rospy.Publisher("/scan", PointCloud, queue_size=2)
    imu_pub = rospy.Publisher("/trunk_imu", Imu, queue_size=20)
    clock_pub = rospy.Publisher("/clock", Clock, queue_size=2)
    simulated = 1.0
    tick = 0
    while not rospy.is_shutdown():
        stamp = rospy.Time.from_sec(simulated)
        clock_pub.publish(Clock(stamp))
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.w = 1.0
        imu.linear_acceleration.z = 9.81
        imu_pub.publish(imu)

        if tick % 10 == 0:
            cloud = PointCloud()
            cloud.header.stamp = stamp
            cloud.header.frame_id = "laser_livox"
            intensity = ChannelFloat32(name="intensity")
            for index in range(720):
                angle = 2.0 * math.pi * index / 720.0
                radius = 6.0 + 0.5 * math.sin(3.0 * angle)
                cloud.points.append(Point32(radius * math.cos(angle), radius * math.sin(angle),
                                            -1.0 + 2.0 * (index % 12) / 11.0))
                intensity.values.append(float(index % 100))
            cloud.channels.append(intensity)
            cloud_pub.publish(cloud)
        tick += 1
        simulated += 0.01
        time.sleep(0.01)


if __name__ == "__main__":
    main()
