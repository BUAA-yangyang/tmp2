#!/usr/bin/env python3
"""Fixed-building three-floor mission with explicit elevator transactions."""

import copy
import json
import math
import os
import sys
import threading
import time
from types import SimpleNamespace

# align_to_opening and corridor_axis are sibling modules of this script, and
# until they were added to catkin_install_python that was enough: roslaunch ran
# this file straight out of scripts/, so sys.path[0] was scripts/. Once they
# are installed, catkin puts a generated *relay* in devel/lib/a1_mission_manager
# for each one and roslaunch prefers that copy, which makes sys.path[0] the
# devel directory. A relay is executable but not importable -- it exec()s the
# real source into a throwaway dict, so the module it yields exports none of
# the source's names, and "from align_to_opening import opening_bearing" fails
# with ImportError (mf09 died here before the robot ever stood up).
#
# The relay does set __file__ to the real source path, so resolving siblings
# from __file__ is correct under both layouts.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import actionlib
from align_to_opening import opening_bearing
from corridor_axis import (
    estimate_corridor_axis,
    generation_is_new,
    measurement_matches_identity,
    stamped_snapshot_can_bind,
)
from nav_msgs.msg import OccupancyGrid
from a1_navigation_interfaces.msg import (DoorwayArray, ExploreFloorAction,
                                          ExploreFloorGoal, WallSegmentArray)
from building_generator_interfaces.srv import CallElevator
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, ColorRGBA, String
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import Marker, MarkerArray


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def set_yaw(q, yaw):
    q.x = q.y = 0.0
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)


class MissionFailure(RuntimeError):
    pass


BUTTON_FIXED_STAND = 1
BUTTON_RL_CMD_VEL = 3
JOY_BUTTON_COUNT = 11
JOY_AXIS_COUNT = 6


class MultiFloorMission:
    def __init__(self):
        self.lock = threading.RLock()
        self.pose = None
        self.mapping = None
        self.doorways = None
        self.wall_message = None
        self.floor_grid = None
        self.floor_grid_identity = None
        # OccupancyGrid has no generation/session fields.  This ROS-time
        # barrier prevents a delayed latched map from a previous floor from
        # being relabelled with the identity visible when its callback runs.
        self.floor_grid_accept_after_ros = rospy.Time(0)
        # Subscribers are created below before the full parameter block.  Keep
        # this callback dependency initialized so an early latched map/wall
        # message cannot race construction.
        self.perception_max_age_sim = 1.5
        self.floor_entry = None
        self.elevator = {}
        self.elevator_candidate_first_seen = {}
        self.elevator_candidate_log_state = {}
        self.elevator_template_tracks = []
        self.arrived_by_elevator = set()
        self.elevator_return_points = {}
        self.floor = 0
        self.state = "STARTING"
        self.event = 0
        self.controller_ready = False
        self.controller_ready_stamp = None
        self.controller_state = ""
        self.localization_state = ""
        self.localization_values = {}
        self.localization_stamp = 0.0
        self.localization_pose_stamp = 0.0
        self.supervisor_state = ""
        self.supervisor_values = {}
        self.supervisor_stamp = 0.0
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=2)
        self.behavior_cmd_pub = rospy.Publisher(
            rospy.get_param("~elevator/scan_cmd_vel_topic", "/cmd_vel_behavior"),
            Twist, queue_size=1)
        self.status_pub = rospy.Publisher(
            rospy.get_param("~diagnostics/status_topic"), String,
            queue_size=20, latch=True)
        self.marker_pub = rospy.Publisher(
            rospy.get_param("~diagnostics/marker_topic"), MarkerArray,
            queue_size=2, latch=True)
        rospy.Subscriber("/a1/localization/odom", Odometry,
                         self.pose_cb, queue_size=10)
        rospy.Subscriber("/a1/floor_mapping/status", DiagnosticStatus,
                         self.mapping_cb, queue_size=10)
        rospy.Subscriber("/a1/floor_mapping/doorways", DoorwayArray,
                         self.doorways_cb, queue_size=10)
        rospy.Subscriber("/a1/floor_mapping/walls", WallSegmentArray,
                         self.walls_cb, queue_size=10)
        rospy.Subscriber("/a1/controller_ready", Bool,
                         self.controller_ready_cb, queue_size=5)
        rospy.Subscriber("/a1/controller/state", String,
                         self.controller_state_cb, queue_size=5)
        rospy.Subscriber("/a1/localization/status", DiagnosticStatus,
                         self.localization_cb, queue_size=5)
        rospy.Subscriber("/a1/localization/supervisor_status", DiagnosticStatus,
                         self.supervisor_cb, queue_size=5)
        self.explore = actionlib.SimpleActionClient(
            "/a1/exploration/explore_floor", ExploreFloorAction)
        self.move = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.call_elevator = rospy.ServiceProxy(
            "/call_elevator", CallElevator)
        self.reset_mapping = rospy.ServiceProxy(
            "/a1/floor_mapping/reset", Trigger)
        self.reinitialize = rospy.ServiceProxy(
            "/a1/localization/reinitialize", Trigger)
        self.clear_costmaps = rospy.ServiceProxy(
            "/move_base/clear_costmaps", Empty)
        self.nav_timeout = float(rospy.get_param(
            "~mission/navigation_timeout_wall", 180.0))
        self.action_timeout = float(rospy.get_param(
            "~mission/action_timeout_wall", 1800.0))
        self.elevator_id = rospy.get_param(
            "~mission/elevator_id", "elevator_main")
        self.min_width = float(rospy.get_param(
            "~elevator/doorway_min_width", 1.20))
        self.max_width = float(rospy.get_param(
            # The mapper reports the stable wall-edge span (about 1.86-1.93 m
            # in this scene), not the generator's nominal 1.40 m leaf width.
            "~elevator/doorway_max_width", 2.00))
        self.elevator_min_observations = int(rospy.get_param(
            "~elevator/minimum_observations", 5))
        self.elevator_min_confidence = float(rospy.get_param(
            "~elevator/minimum_confidence", 0.65))
        # Fixed-scene elevator template.  These constraints apply only during
        # the explicit post-entrance side-scan transaction; ordinary room-door
        # perception remains completely independent.
        self.template_flank_min = float(rospy.get_param(
            "~elevator/template_flank_min_length", 0.45))
        self.template_flank_max = float(rospy.get_param(
            "~elevator/template_flank_max_length", 4.00))
        self.template_short_flank_max = float(rospy.get_param(
            "~elevator/template_short_flank_max_length", 1.10))
        self.template_gap_min = float(rospy.get_param(
            "~elevator/template_gap_min_width", 1.55))
        self.template_gap_max = float(rospy.get_param(
            "~elevator/template_gap_max_width", 2.10))
        self.template_corner_gap_min = float(rospy.get_param(
            "~elevator/template_corner_gap_min_width", 1.15))
        self.template_corner_gap_max = float(rospy.get_param(
            "~elevator/template_corner_gap_max_width", 1.65))
        self.template_angle_tolerance = float(rospy.get_param(
            "~elevator/template_wall_angle_tolerance", 0.26))
        self.template_plane_tolerance = float(rospy.get_param(
            "~elevator/template_plane_tolerance", 0.30))
        self.template_bearing_tolerance = float(rospy.get_param(
            "~elevator/template_scan_bearing_tolerance", 0.65))
        self.template_center_tolerance = float(rospy.get_param(
            "~elevator/template_center_tolerance", 0.18))
        self.template_width_tolerance = float(rospy.get_param(
            "~elevator/template_width_tolerance", 0.16))
        self.template_min_observations = int(rospy.get_param(
            "~elevator/template_min_observations", 5))
        self.template_min_age = float(rospy.get_param(
            "~elevator/template_min_age_wall", 0.8))
        self.lobby_standoff = float(rospy.get_param(
            "~elevator/lobby_standoff", 0.85))
        self.car_depth = float(rospy.get_param(
            "~elevator/car_depth", 1.45))
        self.floor0_corridor_ingress = float(rospy.get_param(
            "~entry/floor0_corridor_ingress", 7.0))
        self.stand_attempt_timeout = float(rospy.get_param(
            "~startup/stand_attempt_timeout_wall", 120.0))
        self.localization_timeout = float(rospy.get_param(
            "~startup/localization_timeout_wall", 300.0))
        self.mapping_timeout = float(rospy.get_param(
            "~startup/mapping_timeout_wall", 180.0))
        self.fixed_stand_settle_sim = float(rospy.get_param(
            "~startup/fixed_stand_settle_sim_s", 3.0))
        self.special_test_mode = bool(rospy.get_param(
            "~mission/special_test_mode", False))
        self.transfer_turn_max_speed = float(rospy.get_param(
            "~elevator/transfer_turn_max_speed", 1.80))
        self.transfer_turn_min_speed = float(rospy.get_param(
            "~elevator/transfer_turn_min_speed", 0.55))
        self.transfer_turn_gain = float(rospy.get_param(
            "~elevator/transfer_turn_gain", 1.80))
        self.transfer_turn_tolerance = float(rospy.get_param(
            "~elevator/transfer_turn_tolerance", 0.20))
        self.transfer_turn_timeout = float(rospy.get_param(
            "~elevator/transfer_turn_timeout_wall", 90.0))
        self.transfer_turn_timeout_sim = float(rospy.get_param(
            "~elevator/transfer_turn_timeout_sim", 20.0))
        # 出梯对准:只读栅格,不新增任何几何常量。阈值与射线量程独立于
        # transfer_turn_*,因为那组是转向控制参数,这组是感知参数。
        self.align_occupied_threshold = int(rospy.get_param(
            "~elevator/align_occupied_threshold", 65))
        # How long to let the reset floor map accumulate observations before
        # giving up on finding the car opening.
        # The car opening is observed from inside the car, so the doorway sits
        # about car_depth away. The window bounds that, and the exit refuses to
        # run without a measurement inside it.
        # Corridor ingress distance past the elevator lobby standoff. The
        # config carried upper_floor/corridor_forward all along; the old route
        # ignored it and hardcoded 5.0 in the body.
        # The fixed exit route's three calibrated numbers. They were literals
        # in the function body; upper_floor/exit_forward and corridor_forward
        # existed in the config all along and were simply never read.
        self.exit_forward = float(rospy.get_param(
            "~upper_floor/exit_forward", 2.0))
        self.corridor_forward = float(rospy.get_param(
            "~upper_floor/corridor_forward", 5.0))
        self.corridor_turn = float(rospy.get_param(
            "~upper_floor/corridor_turn_rad", math.radians(95.0)))
        self.upper_door_min_distance = float(rospy.get_param(
            "~elevator/upper_door_min_distance", 0.25))
        self.upper_door_max_distance = float(rospy.get_param(
            "~elevator/upper_door_max_distance", 2.20))
        self.upper_exit_detection_timeout = float(rospy.get_param(
            "~elevator/upper_exit_detection_timeout_wall", 90.0))
        self.align_dump_dir = rospy.get_param(
            "~elevator/align_dump_dir", "/tmp")
        self.align_observation_timeout = float(rospy.get_param(
            "~elevator/align_observation_timeout", 20.0))
        self.align_max_range = float(rospy.get_param(
            "~elevator/align_max_range", 3.0))
        self.perception_max_age_sim = float(rospy.get_param(
            "~elevator/perception_max_age_sim", 1.5))
        self.upper_axis_wait_wall = float(rospy.get_param(
            "~upper_floor/corridor_axis/wait_wall", 5.0))
        self.upper_axis_max_age_sim = float(rospy.get_param(
            "~upper_floor/corridor_axis/max_age_sim", 1.5))
        self.upper_axis_maximum_correction = float(rospy.get_param(
            "~upper_floor/corridor_axis/maximum_correction", 0.65))
        self.upper_axis_parallel_tolerance = float(rospy.get_param(
            "~upper_floor/corridor_axis/parallel_tolerance", 0.30))
        self.upper_axis_minimum_wall_length = float(rospy.get_param(
            "~upper_floor/corridor_axis/minimum_wall_length", 0.70))
        self.upper_axis_minimum_width = float(rospy.get_param(
            "~upper_floor/corridor_axis/minimum_width", 1.50))
        self.upper_axis_maximum_width = float(rospy.get_param(
            "~upper_floor/corridor_axis/maximum_width", 3.60))
        self.upper_axis_maximum_midpoint_distance = float(rospy.get_param(
            "~upper_floor/corridor_axis/maximum_midpoint_distance", 8.0))
        # 订阅必须排在 floor_grid / lock 初始化之后。放在构造函数前部时,
        # 第一帧地图可能在这两个属性存在之前就到达,回调直接 AttributeError。
        rospy.Subscriber("/a1/floor_mapping/map", OccupancyGrid,
                         self.on_floor_grid, queue_size=1)
        self.transfer_turn_settle = float(rospy.get_param(
            "~elevator/transfer_turn_settle_wall", 0.5))

    def controller_ready_cb(self, message):
        with self.lock:
            self.controller_ready = bool(message.data)
            self.controller_ready_stamp = rospy.Time.now()

    def controller_state_cb(self, message):
        with self.lock:
            self.controller_state = str(message.data).strip().lower()

    def localization_cb(self, message):
        values = {item.key: item.value for item in message.values}
        with self.lock:
            self.localization_state = str(message.message).strip().upper()
            self.localization_values = values
            self.localization_stamp = time.monotonic()

    def supervisor_cb(self, message):
        values = {item.key: item.value for item in message.values}
        with self.lock:
            self.supervisor_state = str(message.message).strip().upper()
            self.supervisor_values = values
            self.supervisor_stamp = time.monotonic()

    def pose_cb(self, message):
        with self.lock:
            self.pose = copy.deepcopy(message)
            self.localization_pose_stamp = time.monotonic()

    def mapping_cb(self, message):
        values = {item.key: item.value for item in message.values}
        with self.lock:
            self.mapping = (message.message, values, time.monotonic())

    def doorways_cb(self, message):
        with self.lock:
            self.doorways = copy.deepcopy(message)
        self.try_freeze_elevator(message)

    def perception_message_is_fresh(self, message, frame):
        if message is None or message.header.frame_id != frame:
            return False
        now_ros = rospy.Time.now()
        if now_ros < message.header.stamp:
            return False
        return ((now_ros - message.header.stamp).to_sec()
                <= self.perception_max_age_sim)

    @staticmethod
    def undirected_angle(ax, ay, bx, by):
        dot = abs(ax * bx + ay * by)
        return math.acos(max(-1.0, min(1.0, dot)))

    def walls_cb(self, message):
        """Locate the unique F0 elevator as a short-wall/gap template.

        This deliberately does not manufacture a semantic Doorway.  It uses
        only stable wall geometry collected while the robot is stopped and
        looking toward the elevator side of the entrance lobby.
        """
        # Keep the current generation's wall snapshot for the upper-floor
        # entry-axis transaction below.  The F0-only template logic remains
        # gated exactly as before.
        with self.lock:
            self.wall_message = copy.deepcopy(message)
        if (self.floor != 0 or self.floor in self.elevator or
                self.floor in self.arrived_by_elevator or
                not rospy.get_param(
                    "/frontier_explorer/runtime/elevator_scan_active", False)):
            return
        with self.lock:
            pose = copy.deepcopy(self.pose)
            anchor = copy.deepcopy(self.floor_entry)
        if (pose is None or anchor is None or
                not self.perception_message_is_fresh(
                    message, pose.header.frame_id)):
            return
        generation, session = self.current_mapping_identity()
        if generation < 0 or session < 0:
            return
        entry_yaw = yaw_of(anchor.pose.orientation)
        forward = (math.cos(entry_yaw), math.sin(entry_yaw))
        scan_direction = (forward[1], -forward[0])
        robot = (pose.pose.pose.position.x, pose.pose.pose.position.y)
        candidates = []
        walls = [wall for wall in message.walls
                 if measurement_matches_identity(
                     wall, generation, session) and
                 wall.stable and wall.status == "observed" and
                 self.template_flank_min <= wall.length <= self.template_flank_max]
        wall_description = "; ".join(
            "id=%d (%.2f,%.2f)->(%.2f,%.2f) len=%.2f obs=%d" % (
                wall.detection_id, wall.start.x, wall.start.y,
                wall.end.x, wall.end.y, wall.length,
                wall.observation_count) for wall in walls)
        rospy.loginfo_throttle(
            1.0, "ELEVATOR_TEMPLATE_SHORT_WALLS count=%d %s",
            len(walls), wall_description)
        for first in range(len(walls)):
            a = walls[first]
            adx, ady = a.end.x - a.start.x, a.end.y - a.start.y
            alen = math.hypot(adx, ady)
            if alen < 1.0e-4:
                continue
            adx, ady = adx / alen, ady / alen
            if self.undirected_angle(adx, ady, forward[0], forward[1]) > self.template_angle_tolerance:
                continue
            for second in range(first + 1, len(walls)):
                b = walls[second]
                if min(a.length, b.length) > self.template_short_flank_max:
                    continue
                bdx, bdy = b.end.x - b.start.x, b.end.y - b.start.y
                blen = math.hypot(bdx, bdy)
                if blen < 1.0e-4:
                    continue
                bdx, bdy = bdx / blen, bdy / blen
                if self.undirected_angle(adx, ady, bdx, bdy) > self.template_angle_tolerance:
                    continue
                tangent = (adx, ady)
                if tangent[0] * forward[0] + tangent[1] * forward[1] < 0.0:
                    tangent = (-tangent[0], -tangent[1])
                ac = ((a.start.x + a.end.x) * 0.5, (a.start.y + a.end.y) * 0.5)
                bc = ((b.start.x + b.end.x) * 0.5, (b.start.y + b.end.y) * 0.5)
                plane_error = abs((bc[0] - ac[0]) * (-tangent[1]) +
                                  (bc[1] - ac[1]) * tangent[0])
                if plane_error > self.template_plane_tolerance:
                    continue
                a_points = [(a.start.x, a.start.y), (a.end.x, a.end.y)]
                b_points = [(b.start.x, b.start.y), (b.end.x, b.end.y)]
                a_proj = [p[0] * tangent[0] + p[1] * tangent[1] for p in a_points]
                b_proj = [p[0] * tangent[0] + p[1] * tangent[1] for p in b_points]
                if max(a_proj) < min(b_proj):
                    left_point = a_points[a_proj.index(max(a_proj))]
                    right_point = b_points[b_proj.index(min(b_proj))]
                elif max(b_proj) < min(a_proj):
                    left_point = b_points[b_proj.index(max(b_proj))]
                    right_point = a_points[a_proj.index(min(a_proj))]
                else:
                    continue
                width = math.hypot(right_point[0] - left_point[0],
                                   right_point[1] - left_point[1])
                if width < self.template_gap_min or width > self.template_gap_max:
                    continue
                center = ((left_point[0] + right_point[0]) * 0.5,
                          (left_point[1] + right_point[1]) * 0.5)
                to_center = (center[0] - robot[0], center[1] - robot[1])
                distance = math.hypot(*to_center)
                if distance < 0.35 or distance > 4.5:
                    continue
                bearing = math.acos(max(-1.0, min(1.0,
                    (to_center[0] * scan_direction[0] +
                     to_center[1] * scan_direction[1]) / distance)))
                if bearing > self.template_bearing_tolerance:
                    continue
                score = (bearing + plane_error + abs(width - 1.28) * 0.25 -
                         0.02 * min(a.observation_count, b.observation_count))
                candidates.append((score, center, width, plane_error, bearing,
                                   a.detection_id, b.detection_id))
        # In the generated elevator the far opening edge is frequently merged
        # into the perpendicular shaft side wall rather than emitted as a
        # second collinear front-wall flank.  Recognize that measured L-shaped
        # boundary as an equivalent fixed-scene template: one short front
        # flank endpoint plus the tangent projection of the recessed side wall.
        if not candidates:
            short_walls = [wall for wall in walls
                           if wall.length <= self.template_short_flank_max]
            for a in short_walls:
                adx, ady = a.end.x - a.start.x, a.end.y - a.start.y
                alen = math.hypot(adx, ady)
                if alen < 1.0e-4:
                    continue
                tangent = (adx / alen, ady / alen)
                if self.undirected_angle(tangent[0], tangent[1],
                                         forward[0], forward[1]) > self.template_angle_tolerance:
                    continue
                if tangent[0] * forward[0] + tangent[1] * forward[1] < 0.0:
                    tangent = (-tangent[0], -tangent[1])
                a_points = [(a.start.x, a.start.y), (a.end.x, a.end.y)]
                for b in walls:
                    if b.detection_id == a.detection_id:
                        continue
                    bdx, bdy = b.end.x - b.start.x, b.end.y - b.start.y
                    blen = math.hypot(bdx, bdy)
                    if blen < 0.75:
                        continue
                    bdir = (bdx / blen, bdy / blen)
                    perpendicular_error = abs(
                        self.undirected_angle(tangent[0], tangent[1],
                                              bdir[0], bdir[1]) - 0.5 * math.pi)
                    if perpendicular_error > 0.30:
                        continue
                    b_points = [(b.start.x, b.start.y), (b.end.x, b.end.y)]
                    b_projection = sum(p[0] * tangent[0] + p[1] * tangent[1]
                                       for p in b_points) * 0.5
                    for boundary in a_points:
                        boundary_projection = (boundary[0] * tangent[0] +
                                               boundary[1] * tangent[1])
                        signed_width = b_projection - boundary_projection
                        width = abs(signed_width)
                        if (width < self.template_corner_gap_min or
                                width > self.template_corner_gap_max):
                            continue
                        side_sign = 1.0 if signed_width > 0.0 else -1.0
                        center = (boundary[0] + tangent[0] * signed_width * 0.5,
                                  boundary[1] + tangent[1] * signed_width * 0.5)
                        side_center = ((b.start.x + b.end.x) * 0.5,
                                       (b.start.y + b.end.y) * 0.5)
                        setback = ((side_center[0] - center[0]) * scan_direction[0] +
                                   (side_center[1] - center[1]) * scan_direction[1])
                        if setback < 0.20 or setback > 3.0:
                            continue
                        to_center = (center[0] - robot[0], center[1] - robot[1])
                        distance = math.hypot(*to_center)
                        if distance < 0.35 or distance > 4.5:
                            continue
                        bearing = math.acos(max(-1.0, min(1.0,
                            (to_center[0] * scan_direction[0] +
                             to_center[1] * scan_direction[1]) / distance)))
                        if bearing > self.template_bearing_tolerance:
                            continue
                        score = (bearing + 0.2 * perpendicular_error +
                                 0.05 * setback + abs(width - 1.40) * 0.25)
                        candidates.append((score, center, width, setback,
                                           bearing, a.detection_id,
                                           b.detection_id))
        if not candidates:
            rospy.loginfo_throttle(1.0,
                "ELEVATOR_TEMPLATE no_pair walls=%d short=%d thresholds="
                "flank[%.2f,%.2f] gap[%.2f,%.2f] angle<=%.2f plane<=%.2f bearing<=%.2f",
                len(message.walls), len(walls), self.template_flank_min,
                self.template_flank_max, self.template_gap_min,
                self.template_gap_max, self.template_angle_tolerance,
                self.template_plane_tolerance, self.template_bearing_tolerance)
            return
        _score, center, width, plane_error, bearing, wall_a, wall_b = min(candidates)
        now = time.monotonic()
        track = None
        for item in self.elevator_template_tracks:
            if (math.hypot(center[0] - item["center"][0], center[1] - item["center"][1]) <=
                    self.template_center_tolerance and
                    abs(width - item["width"]) <= self.template_width_tolerance):
                track = item
                break
        if track is None:
            track = {"center": center, "width": width, "count": 0,
                     "first": now, "wall_a": wall_a, "wall_b": wall_b}
            self.elevator_template_tracks.append(track)
        count = track["count"] + 1
        alpha = 1.0 / count
        track["center"] = (track["center"][0] * (1.0 - alpha) + center[0] * alpha,
                           track["center"][1] * (1.0 - alpha) + center[1] * alpha)
        track["width"] = track["width"] * (1.0 - alpha) + width * alpha
        track["count"] = count
        age = now - track["first"]
        rospy.loginfo_throttle(0.5,
            "ELEVATOR_TEMPLATE pair=(%d,%d) center=(%.3f,%.3f) width=%.3f "
            "plane_error=%.3f bearing=%.3f observations=%d age=%.2f",
            wall_a, wall_b, track["center"][0], track["center"][1],
            track["width"], plane_error, bearing, count, age)
        if count < self.template_min_observations or age < self.template_min_age:
            return
        cx, cy = track["center"]
        ox, oy = robot[0] - cx, robot[1] - cy
        norm = math.hypot(ox, oy)
        if norm < 1.0e-3:
            return
        outward = (ox / norm, oy / norm)
        landmark = SimpleNamespace(
            header=copy.deepcopy(message.header),
            center=Point(x=cx, y=cy, z=0.0), width=track["width"],
            confidence=min(1.0, 0.70 + 0.05 * count),
            observation_count=count, detection_id=100000 + wall_a,
            floor_session_id=walls[0].floor_session_id,
            localization_generation=walls[0].localization_generation)
        with self.lock:
            if self.floor in self.elevator:
                return
            self.elevator[self.floor] = {
                "door": landmark, "outward": outward,
                "session": int(landmark.floor_session_id),
                "generation": int(landmark.localization_generation)}
        self.emit("ELEVATOR_TEMPLATE_LOCALIZED",
                  "frozen dedicated short-wall gap landmark",
                  x=cx, y=cy, width=track["width"], outward_x=outward[0],
                  outward_y=outward[1], observations=count,
                  wall_a=int(wall_a), wall_b=int(wall_b),
                  plane_error=plane_error, bearing=bearing,
                  frame=message.header.frame_id)
        self.publish_markers()

    def emit(self, state, detail, **extra):
        self.state = state
        self.event += 1
        body = {"event": self.event, "state": state, "floor": self.floor,
                "detail": detail, "wall_time": time.time()}
        body.update(extra)
        self.status_pub.publish(String(data=json.dumps(body, sort_keys=True)))
        rospy.loginfo("MULTIFLOOR[%s] floor=%d %s", state, self.floor, detail)

    def current_pose(self):
        with self.lock:
            if self.pose is None:
                raise MissionFailure("odometry unavailable")
            pose = PoseStamped()
            pose.header = copy.deepcopy(self.pose.header)
            pose.pose = copy.deepcopy(self.pose.pose.pose)
            return pose

    def wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.10)
        raise MissionFailure("timeout waiting for %s" % description)

    def publish_joy(self, button_index=None, wall_seconds=0.6):
        message = Joy(axes=[0.0] * JOY_AXIS_COUNT,
                      buttons=[0] * JOY_BUTTON_COUNT)
        if button_index is not None:
            message.buttons[button_index] = 1
        deadline = time.monotonic() + wall_seconds
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            message.header.stamp = rospy.Time.now()
            self.joy_pub.publish(message)
            time.sleep(0.02)

    def wait_sim(self, seconds):
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            time.sleep(0.02)

    def request_fixed_stand(self):
        self.emit("STARTUP_FIXED_STAND", "requesting quiet fixed stand")
        deadline = time.monotonic() + self.stand_attempt_timeout
        attempt = 0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            attempt += 1
            self.publish_joy(BUTTON_FIXED_STAND)
            self.publish_joy(None, 0.2)
            settle = time.monotonic() + 6.0
            while not rospy.is_shutdown() and time.monotonic() < settle:
                with self.lock:
                    state = self.controller_state
                if "fixed" in state or "stand" in state:
                    self.emit("STARTUP_FIXED_STAND_READY",
                              "fixed stand confirmed", attempt=attempt)
                    self.wait_sim(self.fixed_stand_settle_sim)
                    return
                time.sleep(0.05)
        raise MissionFailure("controller never reached fixed stand")

    def wait_for_localization_tracking(self):
        self.emit("STARTUP_LOCALIZATION", "waiting for TRACKING in fixed stand")
        self.wait_until(lambda: self.localization_state == "TRACKING",
                        self.localization_timeout, "localization TRACKING")
        self.emit("STARTUP_LOCALIZATION_READY", "localization is TRACKING")

    def startup_mapping_ready(self):
        with self.lock:
            if self.mapping is None:
                return False
            message, values, stamp = self.mapping
        return (message == "MAPPING" and
                values.get("map_valid") == "true" and
                time.monotonic() - stamp < 1.5)

    def wait_for_startup_mapping(self):
        self.emit("STARTUP_MAPPING", "waiting for valid floor map")
        self.wait_until(self.startup_mapping_ready, self.mapping_timeout,
                        "floor mapping MAPPING/map_valid")
        self.emit("STARTUP_MAPPING_READY", "floor mapping is valid")

    def enter_learned_controller(self):
        self.emit("STARTUP_RL", "handing locomotion to RL /cmd_vel")
        deadline = time.monotonic() + self.stand_attempt_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.publish_joy(BUTTON_RL_CMD_VEL)
            self.publish_joy(None, 0.2)
            settle = time.monotonic() + 5.0
            while not rospy.is_shutdown() and time.monotonic() < settle:
                with self.lock:
                    state = self.controller_state
                    ready = self.controller_ready
                if state == "rl" and ready:
                    self.emit("STARTUP_RL_READY", "RL controller ready")
                    return
                time.sleep(0.05)
        raise MissionFailure("RL controller never became ready")

    def local_coordinates(self, point, anchor):
        yaw = yaw_of(anchor.pose.orientation)
        dx = point.x - anchor.pose.position.x
        dy = point.y - anchor.pose.position.y
        return (math.cos(yaw) * dx + math.sin(yaw) * dy,
                -math.sin(yaw) * dx + math.cos(yaw) * dy)

    def try_freeze_elevator(self, message):
        # Every early return below is silent, and DOORWAY_RAW is logged only
        # after all of them. mf16 read "zero DOORWAY_RAW on floor 1" as "the
        # detector sees nothing from inside the car" -- that conclusion was not
        # available from the absence of a log line, because a stale pose, an
        # unavailable mapping identity or a per-door identity mismatch produce
        # exactly the same silence. Name the gate instead.
        def gate(reason, **extra):
            key = ("gate", self.floor, reason)
            signature = tuple(sorted(extra.items()))
            if self.elevator_candidate_log_state.get(key) != signature:
                self.elevator_candidate_log_state[key] = signature
                rospy.loginfo(
                    "ELEVATOR_FREEZE_GATE floor=%d reason=%s doorways=%d %s",
                    self.floor, reason, len(message.doorways),
                    " ".join("%s=%s" % kv for kv in signature))
        if self.floor in self.elevator:
            return
        if self.floor == 0 and self.floor not in self.arrived_by_elevator:
            # Initial F0 acquisition is owned exclusively by walls_cb's
            # dedicated short-wall/gap template.  Generic doorways must never
            # race or overwrite that landmark.
            return
        if (self.floor == 0 and self.floor not in self.arrived_by_elevator and
                not rospy.get_param(
                    "/frontier_explorer/runtime/elevator_scan_active", False)):
            # Floor-zero elevator acquisition is an explicit lobby scan
            # transaction.  Never let later corridor/room doorways mutate it.
            return
        with self.lock:
            pose = copy.deepcopy(self.pose)
            anchor = copy.deepcopy(self.floor_entry)
        if (pose is None or not self.perception_message_is_fresh(
                message, pose.header.frame_id)):
            gate("pose_missing_or_stale_perception",
                 pose=pose is not None)
            return
        generation, session = self.current_mapping_identity()
        if generation < 0 or session < 0:
            gate("mapping_identity_unavailable",
                 generation=generation, session=session)
            return
        if not message.doorways:
            gate("detector_published_no_doorways")
            return
        matched = sum(
            1 for door in message.doorways
            if measurement_matches_identity(door, generation, session))
        if matched == 0:
            gate("all_doorways_failed_identity",
                 generation=generation, session=session)
        candidates = []
        for door in message.doorways:
            if not measurement_matches_identity(
                    door, generation, session):
                continue
            raw_key = ("raw", int(door.localization_generation),
                       int(door.floor_session_id), int(door.detection_id))
            raw_signature = (round(float(door.width), 2), bool(door.stable),
                             int(door.state), int(door.observation_count),
                             round(float(door.confidence), 2))
            if self.elevator_candidate_log_state.get(raw_key) != raw_signature:
                self.elevator_candidate_log_state[raw_key] = raw_signature
                rospy.loginfo(
                    "DOORWAY_RAW floor=%d frame=%s id=%d center=(%.3f, %.3f) "
                    "normal=(%.3f, %.3f) width=%.3f usable=%.3f state=%d "
                    "stable=%s confidence=%.3f observations=%d",
                    self.floor, door.header.frame_id, door.detection_id,
                    door.center.x, door.center.y, door.normal.x, door.normal.y,
                    door.width, door.usable_width, door.state, door.stable,
                    door.confidence, door.observation_count)
            # Elevator localization only freezes the measured doorway centre;
            # it does not command traversal yet.  A stable geometric doorway
            # remains a valid landmark while passage-direction resolution is
            # pending, so do not discard it solely on traversable=false.
            if door.width < self.min_width or door.width > self.max_width:
                continue
            if self.floor == 0 and self.floor not in self.arrived_by_elevator:
                # F0 is observed from the entrance lobby.  Freeze the earliest
                # repeatedly observed internal opening after the public and
                # elevator doors open.  This is LiDAR/time evidence, not a
                # guessed XY window.  The broad public entrance is excluded by
                # the width band; room doors only become stable after the 8 m
                # corridor ingress, by which time the elevator is frozen.
                key = (int(door.localization_generation),
                       int(door.floor_session_id), int(door.detection_id))
                first_seen = self.elevator_candidate_first_seen.setdefault(
                    key, time.monotonic())
                reasons = []
                # The public entrance remains in the doorway track list after
                # the robot moves indoors.  Its normal is parallel to the
                # entrance axis; the elevator is on the lobby side wall and
                # therefore has an approximately perpendicular normal.
                if anchor is not None:
                    entry_yaw = yaw_of(anchor.pose.orientation)
                    normal_norm = math.hypot(door.normal.x, door.normal.y)
                    if normal_norm > 1.0e-3:
                        entry_alignment = abs(
                            (door.normal.x * math.cos(entry_yaw) +
                             door.normal.y * math.sin(entry_yaw)) /
                            normal_norm)
                        if entry_alignment > 0.55:
                            reasons.append(
                                "normal_parallel_to_main_entrance=%.2f" %
                                entry_alignment)
                if not door.stable:
                    reasons.append("unstable")
                if door.observation_count < self.elevator_min_observations:
                    reasons.append("observations=%d<%d" % (
                        door.observation_count,
                        self.elevator_min_observations))
                if door.confidence < self.elevator_min_confidence:
                    reasons.append("confidence=%.2f<%.2f" % (
                        door.confidence, self.elevator_min_confidence))
                signature = (bool(door.stable), int(door.state),
                             min(int(door.observation_count),
                                 self.elevator_min_observations),
                             round(float(door.confidence), 2))
                if self.elevator_candidate_log_state.get(key) != signature:
                    self.elevator_candidate_log_state[key] = signature
                    rospy.loginfo(
                        "ELEVATOR_CANDIDATE floor=0 frame=%s id=%d "
                        "generation=%d session=%d center=(%.3f, %.3f) "
                        "normal=(%.3f, %.3f) width=%.3f usable=%.3f "
                        "state=%d stable=%s confidence=%.3f observations=%d "
                        "decision=%s",
                        door.header.frame_id,
                        door.detection_id,
                        door.localization_generation,
                        door.floor_session_id,
                        door.center.x,
                        door.center.y,
                        door.normal.x,
                        door.normal.y,
                        door.width,
                        door.usable_width,
                        door.state,
                        door.stable,
                        door.confidence,
                        door.observation_count,
                        "accept" if not reasons else "reject:" + ",".join(reasons),
                    )
                if reasons:
                    continue
                nx = float(door.normal.x)
                ny = float(door.normal.y)
                norm = math.hypot(nx, ny)
                if norm < 1.0e-3:
                    nx = pose.pose.pose.position.x - door.center.x
                    ny = pose.pose.pose.position.y - door.center.y
                    norm = math.hypot(nx, ny)
                if norm < 1.0e-3:
                    continue
                nx /= norm
                ny /= norm
                # The robot is in the lobby during F0 acquisition.  Pick the
                # sign of the measured wall normal that points to that side.
                to_robot_x = pose.pose.pose.position.x - door.center.x
                to_robot_y = pose.pose.pose.position.y - door.center.y
                if nx * to_robot_x + ny * to_robot_y < 0.0:
                    nx, ny = -nx, -ny
                outward = (nx, ny)
                age = max(0.0, time.monotonic() - first_seen)
                score = (1000.0 - first_seen +
                         door.confidence +
                         0.1 * door.observation_count +
                         min(age, 5.0) * 0.01)
            else:
                # Upper floors are observed from INSIDE the car, so the only
                # doorway within reach is the one we came through. Until now
                # this branch accepted the first opening in the distance
                # window with no evidence test at all -- floor 0's stability,
                # observation-count and confidence gates were inside the
                # `floor == 0` branch and never applied here. Freezing a
                # single noisy detection as the car opening is how the exit
                # route ends up pointing at a wall, so require the same
                # evidence on every floor.
                dx = door.center.x - pose.pose.pose.position.x
                dy = door.center.y - pose.pose.pose.position.y
                distance = math.hypot(dx, dy)
                if (distance > self.upper_door_max_distance
                        or distance < self.upper_door_min_distance):
                    continue
                reasons = []
                if not door.stable:
                    reasons.append("unstable")
                if door.observation_count < self.elevator_min_observations:
                    reasons.append("observations=%d<%d" % (
                        door.observation_count,
                        self.elevator_min_observations))
                if door.confidence < self.elevator_min_confidence:
                    reasons.append("confidence=%.2f<%.2f" % (
                        door.confidence, self.elevator_min_confidence))
                key = ("upper", int(door.localization_generation),
                       int(door.floor_session_id), int(door.detection_id))
                signature = (bool(door.stable), int(door.observation_count),
                             round(float(door.confidence), 2), bool(reasons))
                if self.elevator_candidate_log_state.get(key) != signature:
                    self.elevator_candidate_log_state[key] = signature
                    rospy.loginfo(
                        "ELEVATOR_CANDIDATE floor=%d id=%d center=(%.3f, "
                        "%.3f) width=%.3f distance=%.3f stable=%s "
                        "observations=%d confidence=%.3f decision=%s",
                        self.floor, door.detection_id, door.center.x,
                        door.center.y, door.width, distance, door.stable,
                        door.observation_count, door.confidence,
                        "accept" if not reasons
                        else "reject:" + ",".join(reasons))
                if reasons:
                    continue
                outward = (dx / distance, dy / distance)
                score = door.confidence + 0.1 * door.observation_count - 0.1 * distance
            candidates.append((score, copy.deepcopy(door), outward))
        if not candidates:
            return
        _score, door, outward = max(candidates, key=lambda item: item[0])
        with self.lock:
            if self.floor not in self.elevator:
                self.elevator[self.floor] = {
                    "door": door, "outward": outward,
                    "session": int(door.floor_session_id),
                    "generation": int(door.localization_generation)}
                self.emit("ELEVATOR_LOCALIZED",
                          "frozen earliest stable LiDAR doorway center",
                          x=door.center.x, y=door.center.y,
                          width=door.width, confidence=door.confidence,
                          observations=int(door.observation_count),
                          detection_id=int(door.detection_id),
                          frame=door.header.frame_id,
                          outward_x=outward[0], outward_y=outward[1],
                          session=int(door.floor_session_id))
                self.publish_markers()

    def make_pose(self, x, y, yaw, frame=None):
        result = PoseStamped()
        result.header.frame_id = frame or self.current_pose().header.frame_id
        result.header.stamp = rospy.Time.now()
        result.pose.position.x = x
        result.pose.position.y = y
        set_yaw(result.pose.orientation, yaw)
        return result

    def elevator_poses(self, floor):
        item = self.elevator[floor]
        center = item["door"].center
        ox, oy = item["outward"]
        yaw_out = math.atan2(oy, ox)
        yaw_in = yaw_out + math.pi
        lobby = self.make_pose(center.x + ox * self.lobby_standoff,
                               center.y + oy * self.lobby_standoff, yaw_in)
        threshold = self.make_pose(center.x, center.y, yaw_in)
        car = self.make_pose(center.x - ox * self.car_depth,
                             center.y - oy * self.car_depth, yaw_in)
        return lobby, threshold, car

    def navigate(self, target, label):
        self.emit("NAVIGATING", label, x=target.pose.position.x,
                  y=target.pose.position.y)
        goal = MoveBaseGoal(target_pose=target)
        self.move.send_goal(goal)
        deadline = time.monotonic() + self.nav_timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.move.wait_for_result(rospy.Duration(0.2)):
                if self.move.get_state() == 3:
                    return
                break
        state = self.move.get_state()
        self.move.cancel_goal()
        raise MissionFailure("navigation %s failed state=%d" % (label, state))

    def explore_floor(self, floor, entry, main_entrance):
        goal = ExploreFloorGoal()
        goal.floor_id = floor
        goal.entry_mode = (goal.LEGACY_MAIN_ENTRANCE if main_entrance
                           else goal.ALREADY_AT_FLOOR_ENTRY)
        goal.completion_mode = goal.STAY_ON_FLOOR
        goal.target_coverage_ratio = 0.80
        goal.timeout_s = 0.0
        goal.floor_entry_pose = copy.deepcopy(entry)
        if main_entrance:
            # Fixed-scene safety gate: cross the lobby and reach the main
            # corridor before frontier exhaustion is allowed to complete F0.
            # This target is expressed in the localization/map frame and does
            # not use Gazebo truth coordinates.
            yaw = yaw_of(entry.pose.orientation)
            goal.seed_target = self.make_pose(
                entry.pose.position.x + self.floor0_corridor_ingress * math.cos(yaw),
                entry.pose.position.y + self.floor0_corridor_ingress * math.sin(yaw),
                yaw, entry.header.frame_id)
        self.emit("EXPLORE_FLOOR", "dispatching reusable floor transaction")
        self.explore.send_goal(goal)
        deadline = time.monotonic() + self.action_timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.explore.wait_for_result(rospy.Duration(0.5)):
                result = self.explore.get_result()
                if result and result.success:
                    self.emit("FLOOR_COMPLETE", result.message,
                              coverage=result.final_coverage_ratio)
                    return
                raise MissionFailure("floor %d exploration failed: %s" %
                                     (floor, result.message if result else "no result"))
        self.explore.cancel_goal()
        raise MissionFailure("floor %d exploration wall timeout" % floor)

    def enter_elevator(self, floor):
        self.wait_until(lambda: floor in self.elevator,
                        float(rospy.get_param("~elevator/detection_timeout_wall", 20.0)),
                        "floor %d elevator perception" % floor)
        lobby, threshold, car = self.elevator_poses(floor)
        self.navigate(lobby, "elevator lobby approach")
        self.navigate(threshold, "elevator threshold")
        try:
            self.navigate(car, "elevator car interior")
        except MissionFailure as first_error:
            # The narrow doorway can leave DWA oscillating between equivalent
            # trajectories after the threshold goal.  Recover once from the
            # last confirmed safe pose; never turn this into an unbounded
            # navigation retry or relax obstacle geometry.
            rospy.logwarn("elevator car entry aborted; clearing costmaps and "
                          "retrying once from threshold: %s", first_error)
            try:
                self.clear_costmaps()
            except rospy.ServiceException as error:
                rospy.logwarn("clear_costmaps before car retry failed: %s", error)
            self.navigate(threshold, "elevator threshold retry anchor")
            self.navigate(car, "elevator car interior retry")
        self.emit("INSIDE_ELEVATOR", "car interior pose reached")

    def transfer(self, source, target):
        before = self.current_pose().pose.position.z
        with self.lock:
            previous_session = (-1 if self.mapping is None else
                                int(self.mapping[1].get("floor_session_id", -1)))
            previous_generation = self.localization_values.get(
                "localization_generation")
            previous_supervisor_generation = self.supervisor_values.get(
                "generation")
        # Complete the large turn while the source-floor estimator and map are
        # still stable.  The elevator preserves body yaw during teleport, so
        # the robot arrives already looking through the target-floor doorway.
        self.turn_inside_elevator_before_transfer()
        self.emit("ELEVATOR_CALL", "calling target floor while facing the door",
                  target_floor=target, z_before=before)
        rospy.wait_for_service("/call_elevator", timeout=15.0)
        response = self.call_elevator(self.elevator_id, target, True)
        if not response.accepted or response.current_floor != target:
            raise MissionFailure("elevator rejected target %d: %s" %
                                 (target, response.message))
        # A teleport is a localization discontinuity, not ordinary motion.
        # Restart FAST-LIO deliberately, then clear every floor-scoped map.
        with self.lock:
            # A localization generation owns its coordinates. Never reuse the
            # numeric XY recorded in an earlier visit, even for floor zero.
            self.elevator.pop(target, None)
            self.arrived_by_elevator.add(target)
            self.floor = target
            # A localization generation owns every numeric map/perception
            # sample. Drop queued source-floor data before requesting the new
            # generation; callbacks must repopulate these caches explicitly.
            self.floor_grid = None
            self.floor_grid_identity = None
            self.floor_grid_accept_after_ros = rospy.Time.now()
            self.wall_message = None
            self.doorways = None
        rospy.wait_for_service("/a1/localization/reinitialize", timeout=10.0)
        restart = self.reinitialize()
        if not restart.success:
            raise MissionFailure("localization reinitialize rejected: %s" % restart.message)
        restart_stamp = time.monotonic()
        recovery_timeout = float(rospy.get_param(
            "~mission/localization_recovery_timeout_wall", 45.0))
        self.wait_until(
            lambda: self.supervisor_running_after(
                restart_stamp, previous_supervisor_generation),
            recovery_timeout,
            "localization supervisor generation restart")
        with self.lock:
            supervisor_running_stamp = self.supervisor_stamp
        self.wait_until(
            lambda: self.localization_healthy_after(
                supervisor_running_stamp, previous_generation),
            recovery_timeout,
            "fresh localization tracking after estimator restart")
        rospy.wait_for_service("/a1/floor_mapping/reset", timeout=10.0)
        reset = self.reset_mapping()
        if not reset.success:
            raise MissionFailure("floor mapping reset rejected: %s" % reset.message)
        try:
            self.clear_costmaps()
        except rospy.ServiceException as error:
            rospy.logwarn("clear_costmaps after elevator failed: %s", error)
        old_floor = source
        self.wait_until(lambda: self.mapping_healthy(previous_session),
                        float(rospy.get_param("~mission/mapping_recovery_timeout_wall", 30.0)),
                        "new floor mapping session")
        # Bind subsequent unlabelled grids only after the new mapping session
        # has itself become observable.  A map from before this second barrier
        # is discarded even if its callback was delayed until after reset.
        with self.lock:
            self.floor_grid = None
            self.floor_grid_identity = None
            self.floor_grid_accept_after_ros = rospy.Time.now()
        after = self.current_pose().pose.position.z
        self.emit("FLOOR_SWITCH_VERIFIED",
                  "localization and mapping restarted after elevator discontinuity",
                  source_floor=old_floor, target_floor=target,
                  z_before=before, z_after=after)

    def supervisor_running_after(self, restart_stamp, previous_generation):
        """Observe completion of the supervisor's asynchronous restart."""
        with self.lock:
            state = self.supervisor_state
            values = dict(self.supervisor_values)
            stamp = self.supervisor_stamp
        generation = values.get("generation")
        return (state == "RUNNING" and stamp > restart_stamp and
                generation_is_new(generation, previous_generation))

    def localization_healthy_after(self, restart_stamp, previous_generation):
        """Require a fresh TRACKING diagnostic belonging to the new restart.

        Freshness, not a generation label, is what proves ownership here.
        /a1/localization/status carries no "localization_generation" key --
        nothing in the localization package has ever published one; the only
        generation label in the system is "generation" on the supervisor's own
        status, and supervisor_running_after already gates on that, where
        generation_is_new's fail-closed rule is both meaningful and enforceable.

        Gating this predicate on the absent key made it constant False, so the
        mission could never confirm the restart no matter how healthy the
        estimator was. mf10 measured it exactly: localization reported
        TRACKING/HEALTHY 11.4 s after the estimator restart and the mission
        still aborted at the 45 s bound, 34 s later. That is deterministic, not
        flaky -- it would have failed every transfer of every future run.

        The two stamp comparisons below are the real epoch proof: both the
        diagnostic and the pose must post-date the supervisor's restart.
        """
        with self.lock:
            state = self.localization_state
            values = dict(self.localization_values)
            stamp = self.localization_stamp
            pose_stamp = self.localization_pose_stamp
        if (state != "TRACKING" or stamp <= restart_stamp or
                pose_stamp <= restart_stamp):
            return False
        generation = values.get("localization_generation")
        if generation is None or str(generation).strip() == "":
            return True
        return generation_is_new(generation, previous_generation)

    @staticmethod
    def angle_error(target, current):
        return math.atan2(math.sin(target - current),
                          math.cos(target - current))

    def turn_inside_elevator_before_transfer(self):
        """Face the open door before teleporting to the target floor.

        The robot enters facing the rear wall.  Turning on the source floor
        keeps the maneuver entirely inside a healthy localization generation;
        the target-floor estimator can then initialize while already observing
        the open doorway and corridor.
        """
        start = self.current_pose()
        start_yaw = yaw_of(start.pose.orientation)
        target_yaw = start_yaw + math.pi
        deadline = time.monotonic() + self.transfer_turn_timeout
        start_ros = rospy.Time.now()
        settled_since = None
        self.emit("ELEVATOR_PRETRANSFER_TURN",
                  "turning inside the car to face the door before transfer",
                  start_yaw=start_yaw,
                  target_yaw=self.angle_error(target_yaw, 0.0))
        try:
            while time.monotonic() < deadline and not rospy.is_shutdown():
                now_ros = rospy.Time.now()
                if (now_ros >= start_ros and
                        (now_ros - start_ros).to_sec() >
                        self.transfer_turn_timeout_sim):
                    break
                current_yaw = yaw_of(self.current_pose().pose.orientation)
                error = self.angle_error(target_yaw, current_yaw)
                if abs(error) <= self.transfer_turn_tolerance:
                    if settled_since is None:
                        settled_since = time.monotonic()
                    self.behavior_cmd_pub.publish(Twist())
                    if time.monotonic() - settled_since >= self.transfer_turn_settle:
                        self.emit("ELEVATOR_PRETRANSFER_TURN_READY",
                                  "facing the elevator door before transfer",
                                  yaw_error=error)
                        return
                else:
                    settled_since = None
                    command = Twist()
                    magnitude = min(
                        self.transfer_turn_max_speed,
                        max(self.transfer_turn_min_speed,
                            self.transfer_turn_gain * abs(error)))
                    command.angular.z = math.copysign(magnitude, error)
                    self.behavior_cmd_pub.publish(command)
                rospy.sleep(0.05)
        finally:
            self.behavior_cmd_pub.publish(Twist())
        raise MissionFailure("timeout turning toward the elevator door before transfer")

    def mapping_healthy(self, previous_session=-1):
        with self.lock:
            if self.mapping is None:
                return False
            message, values, stamp = self.mapping
        try:
            session = int(values.get("floor_session_id", -1))
        except ValueError:
            session = -1
        return (session != previous_session and message == "MAPPING"
                and values.get("map_valid") == "true"
                and time.monotonic() - stamp < 1.5)

    def exit_to_corridor(self, floor):
        self.wait_until(lambda: floor in self.elevator, 20.0,
                        "target-floor elevator doorway")
        lobby, threshold, _car = self.elevator_poses(floor)
        ox, oy = self.elevator[floor]["outward"]
        yaw_out = math.atan2(oy, ox)
        current = self.current_pose()
        turn_out = self.make_pose(current.pose.position.x,
                                  current.pose.position.y, yaw_out)
        self.navigate(turn_out, "turn inside elevator to face the open door")
        threshold.pose.orientation = copy.deepcopy(turn_out.pose.orientation)
        lobby.pose.orientation = copy.deepcopy(turn_out.pose.orientation)
        self.navigate(threshold, "exit elevator threshold")
        self.navigate(lobby, "move 1 m clear of elevator door")
        # The requested right turn is a clockwise 90-degree rotation.
        fx, fy = oy, -ox
        yaw = math.atan2(fy, fx)
        entry = self.make_pose(lobby.pose.position.x + 5.0 * fx,
                               lobby.pose.position.y + 5.0 * fy, yaw)
        self.navigate(entry, "right turn and 5 m transit to main corridor")
        with self.lock:
            self.floor_entry = copy.deepcopy(entry)
        return entry

    def on_floor_grid(self, message):
        """Bind an unlabelled grid only to a proven current mapping epoch."""
        with self.lock:
            if self.mapping is None:
                return
            status, values, received_at = self.mapping
            try:
                identity = (
                    int(values.get("localization_generation", -1)),
                    int(values.get("floor_session_id", -1)),
                )
            except (TypeError, ValueError):
                return
            if (status != "MAPPING" or values.get("map_valid") != "true" or
                    time.monotonic() - received_at >= 1.5 or
                    not stamped_snapshot_can_bind(
                        message.header.stamp.to_sec(),
                        self.floor_grid_accept_after_ros.to_sec(),
                        identity[0], identity[1])):
                return
            self.floor_grid = copy.deepcopy(message)
            self.floor_grid_identity = identity

    def dump_align_attempt(self, floor, grid, pose, diagnostics, bearing,
                           attempt):
        """Write one alignment attempt to disk. Diagnostic only, never fatal.

        Records the whole 180-ray profile, the acceptance arithmetic, and a
        crop of the grid around the robot, so the question "why did it choose
        that bearing" can be answered from measurement instead of argument.
        """
        try:
            path = "%s/align_floor%d.jsonl" % (self.align_dump_dir, floor)
            info = grid.info
            half = int(round(4.0 / info.resolution))
            column0 = int((pose.pose.position.x - info.origin.position.x)
                          / info.resolution)
            row0 = int((pose.pose.position.y - info.origin.position.y)
                       / info.resolution)
            crop = []
            for row in range(row0 - half, row0 + half + 1):
                if not (0 <= row < info.height):
                    crop.append(None)
                    continue
                line = []
                for column in range(column0 - half, column0 + half + 1):
                    if not (0 <= column < info.width):
                        line.append(-2)
                    else:
                        line.append(int(grid.data[row * info.width + column]))
                crop.append(line)
            known = sum(1 for v in grid.data if v >= 0)
            record = {
                "floor": floor,
                "attempt": attempt,
                "sim_time": rospy.Time.now().to_sec(),
                "robot_xy": [pose.pose.position.x, pose.pose.position.y],
                "robot_yaw": yaw_of(pose.pose.orientation),
                "bearing": bearing,
                "grid_known_cells": known,
                "grid_total_cells": len(grid.data),
                "grid_known_fraction": known / float(max(1, len(grid.data))),
                "crop_half_cells": half,
                "crop_resolution": info.resolution,
                "crop": crop,
            }
            record.update(diagnostics)
            with open(path, "a") as handle:
                handle.write(json.dumps(record) + "\n")
        except Exception as error:  # noqa: BLE001 - diagnostics must not kill the mission
            rospy.logwarn("align dump failed: %s", error)

    def align_to_car_opening(self, floor):
        """Turn to face the car's opening, using the map rather than history.

        Fail-open by design: when the grid gives no confident opening (no clear
        maximum, or the robot is already in open space), this returns without
        turning and the caller proceeds exactly as before. A wrong alignment
        would be worse than the status quo.
        """
        # Retry while the freshly reset floor map fills in. Straight after
        # FLOOR_SWITCH_VERIFIED the grid is almost entirely unknown, and since
        # unknown cells deliberately stop the rays, every ray comes back short
        # and there is no contrast to find an opening in. mf06 skipped for
        # exactly that reason. "Mapping healthy" is not "the car has been
        # observed" -- the same distinction that made the post-transfer
        # navigation fail earlier.
        bearing = None
        attempt = 0
        deadline = time.monotonic() + self.align_observation_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                grid = copy.deepcopy(self.floor_grid)
                grid_identity = self.floor_grid_identity
            pose = self.current_pose()
            current_identity = self.current_mapping_identity()
            grid_fresh = (
                grid is not None
                and grid_identity == current_identity
                and current_identity[0] >= 0
                and current_identity[1] >= 0
                and self.perception_message_is_fresh(
                    grid, pose.header.frame_id)
            )
            if grid_fresh:
                diagnostics = {}
                bearing = opening_bearing(
                    grid, pose.pose.position.x, pose.pose.position.y,
                    occupied_threshold=self.align_occupied_threshold,
                    max_range=self.align_max_range,
                    diagnostics=diagnostics,
                )
                self.dump_align_attempt(floor, grid, pose, diagnostics,
                                        bearing, attempt)
                attempt += 1
                if bearing is not None:
                    break
            time.sleep(0.5)
        if bearing is None:
            # Fail CLOSED. This used to keep the current heading and carry on,
            # reasoning that a wrong alignment would be worse than the status
            # quo. mf13 measured what "carry on" actually costs: floor 2 skipped
            # the alignment, the fixed 2 m / 95 deg / 5 m route departed on an
            # uncorrected heading, and the robot walked off the floor at truth
            # (1.0-2.0, 7.3-7.7) -- oracle z fell 5.52 -> 3.49 -> 0.06 m in
            # under 10 s of sim time. The fixed route is only meaningful
            # relative to the car opening; without that reference it is a blind
            # 7 m walk next to an open shaft. Refuse it and fail the mission
            # with a diagnosable reason instead of losing the robot.
            self.emit("ELEVATOR_EXIT_ALIGN_FAILED",
                      "no confident car opening after %.0f s of map "
                      "observation; refusing to run the fixed exit route blind"
                      % self.align_observation_timeout)
            raise MissionFailure(
                "elevator exit alignment found no confident car opening on "
                "floor %d after %.0f s of map observation"
                % (floor, self.align_observation_timeout))
        current_yaw = yaw_of(pose.pose.orientation)
        error = self.angle_error(bearing, current_yaw)
        self.emit("ELEVATOR_EXIT_ALIGN",
                  "aligning to car opening: %.1f deg correction"
                  % math.degrees(error))
        if abs(error) <= self.transfer_turn_tolerance:
            self.emit("ELEVATOR_EXIT_ALIGN_READY", "already facing the opening")
            return
        deadline = time.monotonic() + self.transfer_turn_timeout
        settled_since = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            error = self.angle_error(
                bearing, yaw_of(self.current_pose().pose.orientation))
            if abs(error) <= self.transfer_turn_tolerance:
                self.behavior_cmd_pub.publish(Twist())
                if settled_since is None:
                    settled_since = time.monotonic()
                elif time.monotonic() - settled_since >= self.transfer_turn_settle:
                    self.emit("ELEVATOR_EXIT_ALIGN_READY",
                              "facing the car opening")
                    return
            else:
                settled_since = None
                command = Twist()
                speed = min(self.transfer_turn_max_speed,
                            max(self.transfer_turn_min_speed,
                                self.transfer_turn_gain * abs(error)))
                command.angular.z = math.copysign(speed, error)
                self.behavior_cmd_pub.publish(command)
            time.sleep(0.02)
        self.behavior_cmd_pub.publish(Twist())
        self.emit("ELEVATOR_EXIT_ALIGN_TIMEOUT",
                  "alignment did not settle; proceeding on current heading")

    def exit_upper_floor_without_doorway(self, floor):
        """Fixed exit route: the target-floor door cannot be measured in time.

        ⚠️ NEGATIVE RESULT, mf16 (2026-08-04). This was rewritten to derive
        every pose from floor_mapping's measured doorway -- the same perception
        the floor-0 entry trusts -- and it does not work, because the doorway
        detector produces *nothing at all* from inside the car: zero
        DOORWAY_RAW messages on floor 1 across 90 s wall (~25 s sim) while the
        robot stood in the car. That is a property of the detector's geometry,
        not a tuning miss -- it needs two flanking wall segments of at least
        minimum_wall_length 0.80 m either side of the gap, and a 1.45 m deep
        car seen from within, with filter/minimum_range 0.5 m discarding the
        nearest returns, does not present them. The original author's name for
        this function ("without_doorway") records the same finding.

        So the route stays dead-reckoned for now, with the one reference that
        IS sound by construction: turn_inside_elevator_before_transfer() faces
        the door on the *source* floor, where the geometry is exact, and the
        car preserves body yaw through the teleport.

        Still open (do not lose): the 95 degree turn and the 5 m transit are
        calibrated on one building. The measurable fix is to leave the car
        first -- the detector does fire once outside (mf13 t=104.3, mf14
        t=138.7) -- and take the corridor direction from the door measured
        from the lobby side, instead of from a remembered heading.
        """
        self.align_to_car_opening(floor)
        current = self.current_pose()
        yaw_out = yaw_of(current.pose.orientation)

        # Point A belongs to this localization generation.  Store it facing
        # into the car so a later return arrives in the same configuration as
        # an ordinary car entry; transfer() will then turn back toward the door.
        point_a = self.make_pose(current.pose.position.x,
                                 current.pose.position.y,
                                 yaw_out + math.pi,
                                 current.header.frame_id)
        with self.lock:
            self.elevator_return_points[floor] = copy.deepcopy(point_a)
        self.emit("ELEVATOR_RETURN_POINT_LOCKED",
                  "locked generation-local elevator return point A",
                  x=point_a.pose.position.x, y=point_a.pose.position.y)

        exit_pose = self.make_pose(
            current.pose.position.x + self.exit_forward * math.cos(yaw_out),
            current.pose.position.y + self.exit_forward * math.sin(yaw_out),
            yaw_out, current.header.frame_id)
        self.navigate(exit_pose, "fixed-route exit from elevator car")

        yaw_corridor = yaw_out - self.corridor_turn
        turn_pose = self.make_pose(exit_pose.pose.position.x,
                                   exit_pose.pose.position.y,
                                   yaw_corridor, current.header.frame_id)
        self.navigate(turn_pose, "fixed-route turn toward main corridor")

        entry = self.make_pose(
            turn_pose.pose.position.x + self.corridor_forward * math.cos(yaw_corridor),
            turn_pose.pose.position.y + self.corridor_forward * math.sin(yaw_corridor),
            yaw_corridor, current.header.frame_id)
        self.navigate(entry, "fixed-route transit to main corridor")
        entry = self.correct_upper_floor_entry_axis(entry)
        with self.lock:
            self.floor_entry = copy.deepcopy(entry)
        self.emit("UPPER_FLOOR_MAIN_CORRIDOR",
                  "fixed elevator exit route reached the main corridor")
        return entry, point_a

    def current_mapping_identity(self):
        with self.lock:
            values = {} if self.mapping is None else dict(self.mapping[1])
        try:
            generation = int(values.get("localization_generation", -1))
        except (TypeError, ValueError):
            generation = -1
        try:
            session = int(values.get("floor_session_id", -1))
        except (TypeError, ValueError):
            session = -1
        return generation, session

    def correct_upper_floor_entry_axis(self, declared_entry):
        """Re-anchor at the achieved pose and fit the main-corridor direction.

        MoveBase may legally finish inside its XY/yaw tolerances.  The reusable
        ExploreFloor contract, however, uses the entry pose as the origin and
        axis for ROI, room identity and corridor progress.  Freeze the achieved
        position and correct only its planar yaw from a fresh, opposite pair of
        stable LiDAR wall segments in this mapping session.
        """
        reference_yaw = yaw_of(declared_entry.pose.orientation)
        deadline = time.monotonic() + self.upper_axis_wait_wall
        last_reason = "no wall snapshot"
        achieved = self.current_pose()
        estimate = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            achieved = self.current_pose()
            with self.lock:
                message = copy.deepcopy(self.wall_message)
            if message is None:
                last_reason = "no wall snapshot"
                time.sleep(0.05)
                continue
            if message.header.frame_id != achieved.header.frame_id:
                last_reason = "wall frame does not match achieved pose"
                time.sleep(0.05)
                continue
            now_ros = rospy.Time.now()
            if now_ros < message.header.stamp:
                last_reason = "wall timestamp is ahead of ROS time"
                time.sleep(0.05)
                continue
            age = (now_ros - message.header.stamp).to_sec()
            if age > self.upper_axis_max_age_sim:
                last_reason = "wall snapshot age %.2f s" % age
                time.sleep(0.05)
                continue
            generation, session = self.current_mapping_identity()
            if generation < 0 or session < 0:
                last_reason = "mapping identity is unavailable"
                time.sleep(0.05)
                continue
            walls = [
                wall for wall in message.walls
                if measurement_matches_identity(wall, generation, session)
            ]
            estimate = estimate_corridor_axis(
                walls,
                (achieved.pose.position.x, achieved.pose.position.y),
                reference_yaw,
                self.upper_axis_maximum_correction,
                self.upper_axis_parallel_tolerance,
                self.upper_axis_minimum_wall_length,
                self.upper_axis_minimum_width,
                self.upper_axis_maximum_width,
                self.upper_axis_maximum_midpoint_distance,
            )
            if estimate is not None:
                break
            last_reason = "no opposite stable wall pair in current session"
            time.sleep(0.05)

        corrected = copy.deepcopy(achieved)
        corrected.header.stamp = rospy.Time.now()
        if estimate is None:
            set_yaw(corrected.pose.orientation, reference_yaw)
            rospy.logwarn(
                "UPPER_FLOOR_ENTRY_AXIS_UNVERIFIED floor=%d reason=%s "
                "achieved=(%.3f, %.3f) declared_yaw=%.3f",
                self.floor, last_reason, corrected.pose.position.x,
                corrected.pose.position.y, reference_yaw)
            self.emit(
                "UPPER_FLOOR_ENTRY_AXIS_UNVERIFIED",
                "no fresh corridor-wall pair; using declared fixed-route axis",
                reason=last_reason,
                x=corrected.pose.position.x,
                y=corrected.pose.position.y,
                yaw=reference_yaw,
            )
            return corrected

        set_yaw(corrected.pose.orientation, estimate.yaw)
        rospy.loginfo(
            "UPPER_FLOOR_ENTRY_AXIS_LOCKED floor=%d achieved=(%.3f, %.3f) "
            "declared_yaw=%.3f corrected_yaw=%.3f correction=%.3f "
            "width=%.3f walls=%d/%d",
            self.floor, corrected.pose.position.x, corrected.pose.position.y,
            reference_yaw, estimate.yaw, estimate.correction,
            estimate.width, estimate.left_id, estimate.right_id)
        self.emit(
            "UPPER_FLOOR_ENTRY_AXIS_LOCKED",
            "re-anchored at achieved pose with generation-local corridor walls",
            x=corrected.pose.position.x,
            y=corrected.pose.position.y,
            declared_x=declared_entry.pose.position.x,
            declared_y=declared_entry.pose.position.y,
            declared_yaw=reference_yaw,
            corrected_yaw=estimate.yaw,
            correction=estimate.correction,
            width=estimate.width,
            left_wall=estimate.left_id,
            right_wall=estimate.right_id,
        )
        return corrected

    def complete_upper_floor_and_return_to_a(self, floor, special_test):
        """Run the shared F1/F2 fixed exit and generation-local return."""
        entry, point_a = self.exit_upper_floor_without_doorway(floor)
        if special_test:
            self.emit("FLOOR_COMPLETE",
                      "TEST MODE: upper-floor main corridor reached")
        else:
            self.explore_floor(floor, entry, False)
        self.navigate(point_a,
                      "return to generation-local elevator point A")
        self.emit("INSIDE_ELEVATOR",
                  "returned to point A for next transfer")

    def publish_markers(self):
        markers = []
        colors = [ColorRGBA(0.1, 0.5, 1.0, 0.95),
                  ColorRGBA(0.1, 1.0, 0.3, 0.95),
                  ColorRGBA(1.0, 0.4, 0.8, 0.95)]
        for floor, item in self.elevator.items():
            marker = Marker()
            marker.header.frame_id = item["door"].header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "multifloor_elevator_center"
            marker.id = floor
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = copy.deepcopy(item["door"].center)
            marker.pose.position.z = 0.45
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.28
            marker.scale.z = 0.9
            marker.color = colors[floor % len(colors)]
            markers.append(marker)
        self.marker_pub.publish(MarkerArray(markers=markers))

    def run(self):
        self.emit("WAITING", "starting deterministic controller/localization sequence")
        self.request_fixed_stand()
        self.wait_for_localization_tracking()
        self.wait_for_startup_mapping()
        self.enter_learned_controller()
        self.wait_until(lambda: self.pose is not None, 60.0, "odometry")
        if not self.move.wait_for_server(rospy.Duration(30.0)):
            raise MissionFailure("move_base unavailable")
        if not self.explore.wait_for_server(rospy.Duration(30.0)):
            raise MissionFailure("explore_floor unavailable")
        spawn = self.current_pose()
        yaw = yaw_of(spawn.pose.orientation)
        offset = float(rospy.get_param("~entry/floor0_forward_offset", 3.5))
        entry0 = self.make_pose(spawn.pose.position.x + offset * math.cos(yaw),
                                spawn.pose.position.y + offset * math.sin(yaw), yaw,
                                spawn.header.frame_id)
        with self.lock:
            self.floor_entry = copy.deepcopy(entry0)
        self.explore_floor(0, entry0, True)
        self.enter_elevator(0)
        self.transfer(0, 1)
        special_test = self.special_test_mode
        self.complete_upper_floor_and_return_to_a(1, special_test)
        self.transfer(1, 2)
        self.complete_upper_floor_and_return_to_a(2, special_test)
        self.transfer(2, 0)
        if special_test:
            self.emit("SPECIAL_TEST_COMPLETE",
                      "floors 1 and 2 fixed exits, point-A returns, and "
                      "transfer back to floor 0 verified")
            return
        self.wait_until(lambda: 0 in self.elevator, 20.0,
                        "floor-zero elevator after relocalization")
        lobby, threshold, _car = self.elevator_poses(0)
        ox, oy = self.elevator[0]["outward"]
        yaw_out = math.atan2(oy, ox)
        current = self.current_pose()
        turn_out = self.make_pose(current.pose.position.x,
                                  current.pose.position.y, yaw_out)
        self.navigate(turn_out, "turn inside elevator to face final exit")
        threshold.pose.orientation = copy.deepcopy(turn_out.pose.orientation)
        lobby.pose.orientation = copy.deepcopy(turn_out.pose.orientation)
        self.navigate(threshold, "final elevator exit threshold")
        self.navigate(lobby, "final elevator lobby")
        # Recreate the entrance anchor in the new floor-0 localization session
        # from the invariant building geometry: elevator outward then left.
        fx, fy = -oy, ox
        entrance_inside = self.make_pose(lobby.pose.position.x + 2.3 * fx,
                                         lobby.pose.position.y + 2.3 * fy,
                                         math.atan2(-fy, -fx))
        self.navigate(entrance_inside, "floor-zero main entrance inside")
        final = self.make_pose(entrance_inside.pose.position.x - 3.5 * fx,
                               entrance_inside.pose.position.y - 3.5 * fy,
                               math.atan2(-fy, -fx))
        self.navigate(final, "exit building and return to spawn-relative pose")
        self.emit("MISSION_COMPLETE", "three floors explored; returned outside")


if __name__ == "__main__":
    rospy.init_node("a1_multifloor_mission")
    mission = MultiFloorMission()
    try:
        mission.run()
    except Exception as error:
        mission.emit("MISSION_FAILED", str(error))
        rospy.logfatal("multi-floor mission failed: %s", error)
        raise
