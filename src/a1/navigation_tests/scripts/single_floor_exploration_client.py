#!/usr/bin/env python3
"""Headless ExploreFloor client that writes a machine-readable result."""

import json
import math
import os
import time

import actionlib
from a1_navigation_interfaces.msg import ExploreFloorAction, ExploreFloorGoal
from geometry_msgs.msg import Point32
from nav_msgs.msg import OccupancyGrid
import rospy
import tf2_ros


class Client:
    def __init__(self):
        self.feedback = []

    def feedback_callback(self, message):
        row = {
            "wall_time": time.time(),
            "state": int(message.state),
            "coverage_ratio": float(message.coverage_ratio),
            "message": message.message,
            "target": {
                "frame_id": message.current_target.header.frame_id,
                "x": message.current_target.pose.position.x,
                "y": message.current_target.pose.position.y,
            },
        }
        self.feedback.append(row)
        rospy.loginfo(
            "ExploreFloor feedback: state=%d coverage=%.3f %s",
            message.state,
            message.coverage_ratio,
            message.message,
        )


def atomic_json(path, document):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w") as stream:
        json.dump(document, stream, indent=2, sort_keys=True)
    os.replace(temporary, path)


def main():
    rospy.init_node("single_floor_exploration_client")
    action_name = rospy.get_param(
        "~action", "/a1/exploration/explore_floor"
    )
    output = rospy.get_param(
        "~output", "/tmp/a1_single_floor_exploration_result.json"
    )
    server_timeout = float(rospy.get_param("~server_timeout", 60.0))
    wall_timeout = float(rospy.get_param("~wall_timeout", 3600.0))
    harness = Client()
    client = actionlib.SimpleActionClient(action_name, ExploreFloorAction)
    if not client.wait_for_server(rospy.Duration(server_timeout)):
        atomic_json(output, {"success": False, "reason": "server_timeout"})
        return 2

    goal = ExploreFloorGoal()
    goal.floor_id = int(rospy.get_param("~floor_id", -1))
    goal.target_coverage_ratio = float(
        rospy.get_param("~target_coverage_ratio", 0.0)
    )
    goal.timeout_s = float(rospy.get_param("~timeout_s", 0.0))
    goal.entry_door_id = rospy.get_param("~entry_door_id", "")

    map_topic = rospy.get_param(
        "~map_topic", "/a1/floor_mapping/map"
    )
    base_frame = rospy.get_param("~base_frame", "base")
    entry_forward = float(
        rospy.get_param("~entry_forward_offset", 3.5)
    )
    entry_lateral = float(
        rospy.get_param("~entry_lateral_offset", 0.0)
    )
    map_message = rospy.wait_for_message(
        map_topic, OccupancyGrid, timeout=server_timeout
    )
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform(
        map_message.header.frame_id,
        base_frame,
        rospy.Time(0),
        rospy.Duration(server_timeout),
    )
    del tf_listener
    rotation = transform.transform.rotation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
    )
    goal.floor_entry_pose.header.frame_id = map_message.header.frame_id
    goal.floor_entry_pose.header.stamp = rospy.Time.now()
    goal.floor_entry_pose.pose.position.x = (
        transform.transform.translation.x
        + math.cos(yaw) * entry_forward
        - math.sin(yaw) * entry_lateral
    )
    goal.floor_entry_pose.pose.position.y = (
        transform.transform.translation.y
        + math.sin(yaw) * entry_forward
        + math.cos(yaw) * entry_lateral
    )
    goal.floor_entry_pose.pose.orientation.x = 0.0
    goal.floor_entry_pose.pose.orientation.y = 0.0
    goal.floor_entry_pose.pose.orientation.z = math.sin(0.5 * yaw)
    goal.floor_entry_pose.pose.orientation.w = math.cos(0.5 * yaw)

    raw_roi = rospy.get_param(
        "~roi_local",
        [0.0, -8.65, 40.0, -8.65, 40.0, 8.65, 0.0, 8.65],
    )
    if len(raw_roi) < 6 or len(raw_roi) % 2:
        raise ValueError("~roi_local requires at least three x,y pairs")
    goal.roi_local.points = [
        Point32(x=float(raw_roi[index]), y=float(raw_roi[index + 1]))
        for index in range(0, len(raw_roi), 2)
    ]
    client.send_goal(goal, feedback_cb=harness.feedback_callback)
    deadline = time.monotonic() + wall_timeout
    completed = False
    while (
        not rospy.is_shutdown()
        and time.monotonic() < deadline
    ):
        if client.wait_for_result(rospy.Duration(0.2)):
            completed = True
            break
    if not completed:
        client.cancel_goal()
        document = {
            "success": False,
            "reason": "client_wall_timeout",
            "feedback": harness.feedback,
        }
        atomic_json(output, document)
        return 3

    result = client.get_result()
    document = {
        "success": bool(result.success),
        "error_code": int(result.error_code),
        "message": result.message,
        "final_coverage_ratio": float(result.final_coverage_ratio),
        "action_state": int(client.get_state()),
        "feedback": harness.feedback,
    }
    atomic_json(output, document)
    print(json.dumps(document, indent=2, sort_keys=True))
    return 0 if result.success else 4


if __name__ == "__main__":
    raise SystemExit(main())
