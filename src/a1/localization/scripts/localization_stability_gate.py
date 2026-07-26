#!/usr/bin/env python3
"""Acceptance-only gate that never republishes truth into localization."""
import argparse
import math
import sys
import time
import rospy
from nav_msgs.msg import Odometry


def rpy(q):
    roll = math.atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y))
    value = 2 * (q.w*q.y - q.z*q.x)
    pitch = math.copysign(math.pi/2, value) if abs(value) >= 1 else math.asin(value)
    return roll, pitch


def main():
    parser = argparse.ArgumentParser()
    # base_w is the valid world-frame p3d output in the A1 model. The
    # base_trunk compatibility topic reports a zero pose in this environment.
    parser.add_argument("--topic", default="/ground_truth/base_w")
    parser.add_argument("--window", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--min-z", type=float, default=0.20)
    parser.add_argument("--max-z", type=float, default=0.75)
    parser.add_argument("--max-tilt-deg", type=float, default=12.0)
    parser.add_argument("--max-speed", type=float, default=0.15)
    parser.add_argument("--max-yaw-rate", type=float, default=0.20)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("localization_stability_gate", anonymous=True)
    state = {"last": None, "since": None}

    def callback(msg):
        roll, pitch = rpy(msg.pose.pose.orientation)
        v = msg.twist.twist
        speed = math.sqrt(v.linear.x**2 + v.linear.y**2 + v.linear.z**2)
        yaw_rate = abs(v.angular.z)
        stable = (args.min_z <= msg.pose.pose.position.z <= args.max_z and
                  max(abs(roll), abs(pitch)) <= math.radians(args.max_tilt_deg) and
                  speed <= args.max_speed and yaw_rate <= args.max_yaw_rate)
        state["last"] = (stable, msg.pose.pose.position.z, math.degrees(roll),
                         math.degrees(pitch), speed, yaw_rate)
        if stable and state["since"] is None:
            state["since"] = time.monotonic()
        elif not stable:
            state["since"] = None

    rospy.Subscriber(args.topic, Odometry, callback, queue_size=1)
    deadline = time.monotonic() + args.timeout
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if state["since"] is not None and time.monotonic() - state["since"] >= args.window:
            rospy.loginfo("localization stability gate passed")
            return 0
        rate.sleep()
    rospy.logerr("localization stability gate failed: no stable %.1fs window", args.window)
    if state["last"]:
        rospy.logerr("last stable=%s z=%.3f roll=%.1f pitch=%.1f speed=%.3f yaw_rate=%.3f",
                     *state["last"])
    return 2


if __name__ == "__main__":
    sys.exit(main())
