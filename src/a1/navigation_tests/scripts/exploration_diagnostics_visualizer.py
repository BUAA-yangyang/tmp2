#!/usr/bin/env python3
"""RViz-only bridge that separates upstream perception from exploration decisions."""

import math

import rospy
from a1_navigation_interfaces.msg import (
    Doorway,
    DoorwayArray,
    ExplorationStatus,
    WallSegmentArray,
)
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def color(r, g, b, a=1.0):
    return ColorRGBA(r=r, g=g, b=b, a=a)


class DiagnosticsVisualizer:
    def __init__(self):
        self.pub = rospy.Publisher(
            "/a1/diagnostics/upstream_markers", MarkerArray,
            queue_size=1, latch=True)
        self.walls = None
        self.doors = None
        self.odom = None
        self.status = None
        rospy.Subscriber("/a1/floor_mapping/walls", WallSegmentArray,
                         self.wall_callback, queue_size=1)
        rospy.Subscriber("/a1/floor_mapping/doorways", DoorwayArray,
                         self.door_callback, queue_size=1)
        rospy.Subscriber("/a1/localization/odom", Odometry,
                         self.odom_callback, queue_size=1)
        rospy.Subscriber("/a1/exploration/status", ExplorationStatus,
                         self.status_callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.publish)

    def wall_callback(self, message):
        self.walls = message

    def door_callback(self, message):
        self.doors = message

    def odom_callback(self, message):
        self.odom = message

    def status_callback(self, message):
        self.status = message

    @staticmethod
    def marker(frame, stamp, namespace, marker_id, marker_type):
        result = Marker()
        result.header.frame_id = frame
        result.header.stamp = stamp
        result.ns = namespace
        result.id = marker_id
        result.type = marker_type
        result.action = Marker.ADD
        result.pose.orientation.w = 1.0
        return result

    def publish(self, _event):
        now = rospy.Time.now()
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)

        if self.walls is not None:
            wall_lines = self.marker(
                self.walls.header.frame_id, now, "upstream_walls", 0,
                Marker.LINE_LIST)
            wall_lines.scale.x = 0.07
            wall_lines.color = color(0.15, 0.65, 1.0, 0.9)
            for wall in self.walls.walls:
                wall_lines.points.extend((wall.start, wall.end))
            output.markers.append(wall_lines)

        if self.doors is not None:
            frame = self.doors.header.frame_id
            for index, door in enumerate(self.doors.doorways):
                opening = self.marker(
                    frame, now, "upstream_door_openings", index,
                    Marker.LINE_STRIP)
                opening.scale.x = 0.12
                opening.points = [door.left_boundary, door.right_boundary]
                if not door.stable:
                    opening.color = color(1.0, 0.65, 0.0, 0.8)
                elif door.state in (Doorway.OPEN, Doorway.PARTIALLY_OPEN):
                    opening.color = color(0.1, 1.0, 0.25, 1.0)
                else:
                    opening.color = color(1.0, 0.1, 0.1, 1.0)
                output.markers.append(opening)

                normal = self.marker(
                    frame, now, "upstream_door_normals", index, Marker.ARROW)
                normal.points = [
                    door.center,
                    Point(x=door.center.x + door.normal.x * 0.8,
                          y=door.center.y + door.normal.y * 0.8,
                          z=door.center.z)]
                normal.scale.x = 0.05
                normal.scale.y = 0.12
                normal.scale.z = 0.16
                normal.color = color(0.8, 0.2, 1.0, 0.9)
                output.markers.append(normal)

                label = self.marker(
                    frame, now, "upstream_door_labels", index,
                    Marker.TEXT_VIEW_FACING)
                label.pose.position = door.center
                label.pose.position.z += 0.55
                label.scale.z = 0.30
                label.color = color(1.0, 1.0, 1.0, 1.0)
                label.text = (
                    "UP door %d state=%d stable=%s\n"
                    "w=%.2f usable=%.2f conf=%.2f obs=%d"
                    % (door.detection_id, door.state, door.stable,
                       door.width, door.usable_width, door.confidence,
                       door.observation_count))
                output.markers.append(label)

        if self.odom is not None:
            pose = self.marker(
                self.odom.header.frame_id, now, "upstream_localization", 0,
                Marker.ARROW)
            pose.pose = self.odom.pose.pose
            pose.scale.x = 0.8
            pose.scale.y = 0.18
            pose.scale.z = 0.18
            pose.color = color(1.0, 0.9, 0.05, 1.0)
            output.markers.append(pose)

        if self.status is not None:
            frame = (self.odom.header.frame_id
                     if self.odom is not None else "odom")
            text = self.marker(
                frame, now, "midstream_decision_status", 0,
                Marker.TEXT_VIEW_FACING)
            if self.odom is not None:
                text.pose.position = self.odom.pose.pose.position
            text.pose.position.z += 1.5
            text.scale.z = 0.32
            text.color = color(1.0, 0.35, 0.85, 1.0)
            target = self.status.current_target.pose.position
            text.text = (
                "MID state=%d coverage=%.1f%%\n"
                "target=(%.2f, %.2f) %s"
                % (self.status.state, 100.0 * self.status.coverage_ratio,
                   target.x, target.y, self.status.message))
            output.markers.append(text)

        self.pub.publish(output)


if __name__ == "__main__":
    rospy.init_node("exploration_diagnostics_visualizer")
    DiagnosticsVisualizer()
    rospy.spin()
