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

# Helper modules are siblings of this script, and
# until they were added to catkin_install_python that was enough: roslaunch ran
# this file straight out of scripts/, so sys.path[0] was scripts/. Once they
# are installed, catkin puts a generated *relay* in devel/lib/a1_mission_manager
# for each one and roslaunch prefers that copy, which makes sys.path[0] the
# devel directory. A relay is executable but not importable -- it exec()s the
# real source into a throwaway dict, so the module it yields exports none of
# the source's names, and importing one of those helpers fails
# with ImportError (mf09 died here before the robot ever stood up).
#
# The relay does set __file__ to the real source path, so resolving siblings
# from __file__ is correct under both layouts.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import actionlib
from corridor_axis import (
    estimate_corridor_axis,
    generation_is_new,
    measurement_matches_identity,
    stamped_snapshot_can_bind,
)
from elevator_exit import (
    bounded_exit_step,
    choose_corridor_side,
    known_free_run_in_grid,
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
        # Target-floor coordinates are generation-local.  Record the physical
        # exit heading only after localization has restarted on that floor; it
        # is then the sole directional reference for the short car exit and the
        # later return transaction.
        self.arrival_exit_yaws = {}
        self.elevator_candidate_first_seen = {}
        self.elevator_candidate_log_state = {}
        self.elevator_template_tracks = []
        # The wall template measures its own boundary span (about 1.33 m on
        # mf18), not the 1.99 m DoorwayArray opening width.  Keep the quantity
        # under its truthful name instead of misusing it as a cross-floor door
        # width gate.
        self.elevator_template_width = None
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
        self.known_free_occupied_threshold = int(rospy.get_param(
            "~upper_floor/known_free_occupied_threshold",
            rospy.get_param("~elevator/align_occupied_threshold", 65)))
        # Mission-side no-progress watchdog. Deliberately generous: this is a
        # fault bound to stop an unreachable goal from spinning the robot off a
        # floor, not a performance target.
        self.exit_map_wait = float(rospy.get_param(
            "~upper_floor/exit_map_wait_wall", 45.0))
        self.exit_step_maximum = float(rospy.get_param(
            "~upper_floor/exit_step_maximum", 0.85))
        self.exit_minimum_goal = float(rospy.get_param(
            "~upper_floor/exit_minimum_goal", 0.65))
        self.exit_goal_margin = float(rospy.get_param(
            "~upper_floor/exit_goal_margin", 0.15))
        self.exit_minimum_progress = float(rospy.get_param(
            "~upper_floor/exit_minimum_progress", 0.15))
        self.exit_completion_tolerance = float(rospy.get_param(
            "~upper_floor/exit_completion_tolerance", 0.10))
        self.exit_maximum_steps = int(rospy.get_param(
            "~upper_floor/exit_maximum_steps", 10))
        self.known_free_half_width = float(rospy.get_param(
            "~upper_floor/known_free_half_width", 0.22))
        # try_freeze_elevator's upper-floor branch still reads these three.
        # Their definitions were dropped when the width-based gate was removed,
        # leaving live code referencing missing attributes: any doorway with a
        # matching identity on an upper floor would raise AttributeError inside
        # a subscriber callback. mf22 never reached it because the callback
        # returned at an earlier gate, but mf17 exercised this path at t=124.
        # (The width gate itself stays removed: the wall template measures a
        # boundary span, not the doorway width, and mixing them was wrong.)
        self.upper_door_min_distance = float(rospy.get_param(
            "~elevator/upper_door_min_distance", 0.25))
        self.upper_door_max_distance = float(rospy.get_param(
            "~elevator/upper_door_max_distance", 2.20))
        self.car_opening_max_bearing = float(rospy.get_param(
            "~elevator/car_opening_max_bearing", 1.05))
        self.corridor_minimum_run = float(rospy.get_param(
            "~upper_floor/corridor_minimum_run", 2.0))
        self.corridor_run_margin = float(rospy.get_param(
            "~upper_floor/corridor_run_margin", 0.6))
        self.corridor_minimum_advantage = float(rospy.get_param(
            "~upper_floor/corridor_minimum_advantage", 0.40))
        self.corridor_map_wait = float(rospy.get_param(
            "~upper_floor/corridor_map_wait_wall", 30.0))
        self.corridor_probe_max_range = float(rospy.get_param(
            "~upper_floor/corridor_probe_max_range",
            rospy.get_param("~upper_floor/corridor_forward", 5.0)))
        # Must exceed DWA's xy_goal_tolerance (0.45) so the final in-place yaw
        # settle is never mistaken for a stall.
        # Recentring inside the car. Tolerance is well under the margin a
        # 0.30 m body needs in a ~1.54 m car; the probe range only has to see
        # past the walls.
        self.recenter_tolerance = float(rospy.get_param(
            "~elevator/recenter_tolerance", 0.12))
        self.recenter_probe_range = float(rospy.get_param(
            "~elevator/recenter_probe_range", 2.00))
        self.recenter_speed = float(rospy.get_param(
            "~elevator/recenter_speed", 0.18))
        self.recenter_step_wall = float(rospy.get_param(
            "~elevator/recenter_step_wall", 0.8))
        self.recenter_max_attempts = int(rospy.get_param(
            "~elevator/recenter_max_attempts", 6))
        self.recenter_timeout = float(rospy.get_param(
            "~elevator/recenter_timeout_wall", 40.0))
        self.recenter_minimum_displacement = float(rospy.get_param(
            "~elevator/recenter_minimum_displacement", 0.02))
        self.nav_arrival_band = float(rospy.get_param(
            "~mission/no_progress_arrival_band", 0.70))
        self.nav_progress_timeout = float(rospy.get_param(
            "~mission/no_progress_timeout_wall", 12.0))
        self.nav_progress_distance = float(rospy.get_param(
            "~mission/no_progress_distance", 0.15))
        self.nav_progress_yaw = float(rospy.get_param(
            "~mission/no_progress_yaw", 0.35))
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
            self.elevator_template_width = float(track["width"])
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
        if self.floor in self.arrived_by_elevator:
            # Option A deliberately does not identify a semantic doorway on an
            # arrived floor.  mf16/mf18 saw no in-car doorway, while mf17 later
            # froze a 1.238 m room door as the elevator.  The transfer-preserved
            # arrival heading and generation-local map now own the exit instead.
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
                # The width band alone is far too permissive: mf17 froze a
                # 1.238 m ROOM door at 2.19 m as "the elevator" on floor 1,
                # because 1.238 sits inside [1.20, 2.00] and 2.19 inside the
                # distance window. Two measured facts make the real opening
                # separable, and neither is a scene constant:
                #   * the same shaft is used on every floor, so its opening
                #     width was already measured on floor 0 (1.99 m on mf17);
                #   * the robot arrives facing that opening, because the
                #     pre-transfer turn aims at it and the car preserves yaw.
                bearing = math.atan2(dy, dx)
                ahead = abs(self.angle_error(
                    bearing, yaw_of(pose.pose.pose.orientation)))
                if ahead > self.car_opening_max_bearing:
                    reasons.append("bearing=%.2f rad off the arrival heading"
                                   % ahead)
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
        """Drive to a pose, giving up only when the robot stops closing on it.

        The watchdog measures progress as "did either the distance to the goal
        or the heading error to the goal improve", not "did the robot leave an
        anchor". The anchor form killed a healthy goal on mf23: the return to
        point A has to turn around first, and a turn moves the body very little,
        so twelve seconds elapsed while the robot was in fact executing the task
        -- it covered 4.34 m, commanded 38 of 42 samples, stayed level and drew
        no complaint from move_base.

        Converging-on-goal still catches what the anchor form was written for.
        On mf17 the robot sat 0.73 m from an unreachable goal and rotated for
        three seconds; its heading passed through the goal yaw and kept going to
        164 deg of error while the distance never changed. Neither term
        improved, so this fires -- before wz reaches the guard ceiling and the
        attitude diverges.
        """
        self.emit("NAVIGATING", label, x=target.pose.position.x,
                  y=target.pose.position.y)
        goal = MoveBaseGoal(target_pose=target)
        self.move.send_goal(goal)
        goal_yaw = yaw_of(target.pose.orientation)
        deadline = time.monotonic() + self.nav_timeout
        best_distance = float("inf")
        best_yaw_error = float("inf")
        improved_wall = time.monotonic()
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.move.wait_for_result(rospy.Duration(0.2)):
                if self.move.get_state() == 3:
                    return
                break
            now = self.current_pose()
            distance = math.hypot(
                target.pose.position.x - now.pose.position.x,
                target.pose.position.y - now.pose.position.y)
            yaw_error = abs(self.angle_error(
                goal_yaw, yaw_of(now.pose.orientation)))
            improved = False
            if distance < best_distance - self.nav_progress_distance:
                best_distance = distance
                improved = True
            if yaw_error < best_yaw_error - self.nav_progress_yaw:
                best_yaw_error = yaw_error
                improved = True
            best_distance = min(best_distance, distance)
            best_yaw_error = min(best_yaw_error, yaw_error)
            # Close to the goal there is nothing left to improve BY the step
            # threshold: the remaining error is smaller than the threshold
            # itself, so demanding another nav_progress_distance of gain is
            # unsatisfiable and the timer runs out on a robot that has
            # essentially arrived. mf24 died exactly there -- "distance 0.46 m,
            # yaw error 0.10 rad", one centimetre outside move_base's own
            # 0.45 m xy tolerance, mid final yaw settle. Inside the arrival
            # band the goal belongs to move_base and its own timeout.
            if distance <= self.nav_arrival_band:
                improved = True
            if improved:
                improved_wall = time.monotonic()
            elif time.monotonic() - improved_wall >= self.nav_progress_timeout:
                self.move.cancel_goal()
                self.emit("NAVIGATION_NO_PROGRESS",
                          "%s stopped closing on its goal for %.0f s "
                          "(distance %.2f m, yaw error %.2f rad); abandoning it"
                          % (label, self.nav_progress_timeout, distance,
                             yaw_error),
                          x=now.pose.position.x, y=now.pose.position.y)
                raise MissionFailure(
                    "navigation %s stopped closing on its goal for %.0f wall "
                    "seconds (distance %.2f m, yaw error %.2f rad)"
                    % (label, self.nav_progress_timeout, distance, yaw_error))
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
        self.recenter_in_car("floor %d car entry" % floor)
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
        arrival = self.current_pose()
        arrival_yaw = yaw_of(arrival.pose.orientation)
        with self.lock:
            self.arrival_exit_yaws[target] = arrival_yaw
        after = arrival.pose.position.z
        self.emit("FLOOR_SWITCH_VERIFIED",
                  "localization and mapping restarted after elevator discontinuity",
                  source_floor=old_floor, target_floor=target,
                  z_before=before, z_after=after,
                  arrival_exit_yaw=arrival_yaw)

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
        # Aim at the MEASURED door normal, not at start_yaw + pi.
        #
        # start_yaw is whatever move_base left the robot at when it reached the
        # car interior pose, and its yaw_goal_tolerance is 0.80 rad (46 deg).
        # Adding 180 degrees to a reference that is itself up to 46 deg wrong
        # cannot produce a good heading. Measured on mf17: the floor-0 door
        # normal was -173.0 deg (recovered from the straight push into the car,
        # where body heading and travel direction agreed to 0.1 deg) while the
        # robot arrived on floor 1 facing -158.4 deg -- 14.6 deg of error,
        # injected here, before any upper-floor logic ran.
        #
        # The elevator preserves body yaw through the teleport (measured 0.3
        # deg on mf17), so whatever heading is settled here is the heading the
        # robot arrives with. It is worth getting right.
        with self.lock:
            arrival_yaw = self.arrival_exit_yaws.get(self.floor)
            item = self.elevator.get(self.floor)
        if arrival_yaw is not None:
            target_yaw = arrival_yaw
            rospy.loginfo(
                "pre-transfer turn reuses floor %d generation-local arrival "
                "exit heading %.1f deg", self.floor,
                math.degrees(target_yaw))
        elif item is not None:
            ox, oy = item["outward"]
            target_yaw = math.atan2(oy, ox)
            rospy.loginfo(
                "pre-transfer turn aims at the measured door normal "
                "%.1f deg (geometric fallback would have been %.1f deg)",
                math.degrees(target_yaw),
                math.degrees(self.angle_error(start_yaw + math.pi, 0.0)))
        else:
            raise MissionFailure(
                "no measured source-floor door or generation-local arrival "
                "heading for pre-transfer turn on floor %d" % self.floor)
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

    def fresh_floor_grid(self, pose=None):
        """Return a fresh grid from the current mapping identity, or ``None``."""
        pose = pose or self.current_pose()
        with self.lock:
            # rospy replaces message objects on each callback; it does not
            # mutate an already delivered OccupancyGrid. Holding this reference
            # avoids copying ~1.85 million cells on every 10 Hz safety probe.
            grid = self.floor_grid
            identity = self.floor_grid_identity
        current_identity = self.current_mapping_identity()
        if (grid is None or identity != current_identity or
                current_identity[0] < 0 or current_identity[1] < 0 or
                not self.perception_message_is_fresh(
                    grid, pose.header.frame_id)):
            return None
        return grid

    def car_lateral_clearances(self, pose=None):
        """Free distance to left and right of the body, from the published grid.

        Measured against the car walls, not against a goal. The distinction
        matters: mf25 sat 0.015 m from its move_base goal and 0.41 m from the
        right-hand wall with 1.13 m on the left, because the goal is derived
        from the floor-0 door centre and that centre was itself off. Comparing
        the robot with its own goal proves it arrived; it says nothing about
        where it arrived inside the car.
        """
        here = pose or self.current_pose()
        heading = yaw_of(here.pose.orientation)
        left = self.known_free_run(
            here.pose.position.x, here.pose.position.y,
            heading + 0.5 * math.pi, self.recenter_probe_range, here)
        right = self.known_free_run(
            here.pose.position.x, here.pose.position.y,
            heading - 0.5 * math.pi, self.recenter_probe_range, here)
        return left, right

    def recenter_in_car(self, label):
        """Equalise the side clearances before trusting a forward probe.

        A 0.30 m body in a car measured at 1.54 m of interior width has very
        little to give. mf25 arrived 0.71 m off centre; the centre ray then
        showed 1.95 m of free space ahead while the footprint-width strip was
        blocked at 0.19 m by the robot's own right edge, so the exit never took
        its first step. Centring is what makes a footprint-width strip mean
        anything.

        This needs no knowledge of the real car geometry -- only that the two
        sides should measure the same. Fails OPEN: an off-centre robot is not
        itself dangerous, and the exit's own map proof stops it safely if this
        does not converge. It does report whether a commanded lateral velocity
        produced any displacement, which is the open question the single-floor
        notes raised about a lateral dead zone.
        """
        deadline = time.monotonic() + self.recenter_timeout
        attempts = 0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            left, right = self.car_lateral_clearances()
            offset = 0.5 * (left - right)
            if abs(offset) <= self.recenter_tolerance:
                self.emit("CAR_RECENTERED",
                          "%s centred: left %.2f m right %.2f m (offset %+.2f m)"
                          % (label, left, right, offset),
                          left=left, right=right, offset=offset,
                          attempts=attempts)
                return True
            if attempts >= self.recenter_max_attempts:
                break
            before = self.current_pose()
            heading = yaw_of(before.pose.orientation)
            command = Twist()
            command.linear.y = math.copysign(self.recenter_speed, offset)
            step_deadline = time.monotonic() + self.recenter_step_wall
            while not rospy.is_shutdown() and time.monotonic() < step_deadline:
                self.behavior_cmd_pub.publish(command)
                time.sleep(0.02)
            self.behavior_cmd_pub.publish(Twist())
            time.sleep(0.3)
            after = self.current_pose()
            dx = after.pose.position.x - before.pose.position.x
            dy = after.pose.position.y - before.pose.position.y
            moved = -dx * math.sin(heading) + dy * math.cos(heading)
            attempts += 1
            rospy.loginfo(
                "%s recentre attempt %d: offset %+.2f m, commanded vy %+.2f "
                "for %.1f s, lateral displacement %+.3f m",
                label, attempts, offset, command.linear.y,
                self.recenter_step_wall, moved)
            if abs(moved) < self.recenter_minimum_displacement:
                self.emit("CAR_RECENTER_NO_LATERAL_MOTION",
                          "%s commanded vy %+.2f m/s for %.1f s and moved "
                          "%+.3f m sideways; lateral control is not answering"
                          % (label, command.linear.y, self.recenter_step_wall,
                             moved),
                          commanded=command.linear.y, moved=moved)
                return False
        left, right = self.car_lateral_clearances()
        self.emit("CAR_RECENTER_INCOMPLETE",
                  "%s still %+.2f m off centre after %d attempts "
                  "(left %.2f m right %.2f m)"
                  % (label, 0.5 * (left - right), attempts, left, right),
                  left=left, right=right, attempts=attempts)
        return False

    def known_free_run(self, x, y, bearing, max_range, pose=None):
        """How far the published grid is known-free along a bearing.

        Used instead of a hardcoded transit distance. Unknown cells stop the
        run as firmly as occupied ones: the corridor ingress must not be aimed
        into space the sensor has not seen, which is how mf13/mf17 put a goal
        past the floor edge.
        """
        grid = self.fresh_floor_grid(pose)
        return known_free_run_in_grid(
            grid, x, y, bearing, max_range,
            self.known_free_occupied_threshold,
            self.known_free_half_width)

    def advance_map_checked(self, bearing, distance, label, floor,
                            step_maximum=None):
        """Walk a bearing in steps, re-proving the map before every one.

        The exit-from-car loop already worked this way and mf22 crossed the
        floor-1 and floor-2 sills with it. The corridor ingress did not: it
        probed ONCE at the lobby, then issued a single long goal. mf22 measured
        the consequence -- the probe at the exit point reported 5.00 m free to
        the right, the mission committed to 4.40 m, and 3.94 m later the robot
        stepped off floor 2 (oracle z 5.53 -> 4.00). The grid was not wrong: at
        the moment of the fall it showed free to 1.80 m ahead and UNKNOWN from
        1.95 m. Nobody asked it again after the robot left the vantage point
        the single probe was taken from.

        Returns the distance actually advanced along ``bearing``.
        """
        step_maximum = step_maximum or self.exit_step_maximum
        start = self.current_pose()
        origin_x = start.pose.position.x
        origin_y = start.pose.position.y
        frame = start.header.frame_id
        advanced = 0.0
        steps = 0
        while advanced + self.exit_completion_tolerance < distance:
            if steps >= self.exit_maximum_steps:
                raise MissionFailure(
                    "%s exceeded %d bounded steps after advancing %.2f/%.2f m"
                    % (label, self.exit_maximum_steps, advanced, distance))
            here = self.current_pose()
            remaining = distance - advanced
            deadline = time.monotonic() + self.exit_map_wait
            step = 0.0
            free_run = 0.0
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                probe_range = min(
                    step_maximum + self.exit_goal_margin,
                    max(self.exit_minimum_goal + self.exit_goal_margin,
                        remaining + self.exit_goal_margin))
                free_run = self.known_free_run(
                    here.pose.position.x, here.pose.position.y,
                    bearing, probe_range, here)
                step = bounded_exit_step(
                    remaining, free_run, step_maximum,
                    self.exit_minimum_goal, self.exit_goal_margin)
                if step > 0.0:
                    break
                rospy.loginfo_throttle(
                    1.0, "%s waiting for a known-free strip ahead "
                    "(observed %.2f m, %.2f/%.2f m done)",
                    label, free_run, advanced, distance)
                time.sleep(0.10)
                here = self.current_pose()
            if step <= 0.0:
                # Not a failure when the leg has already covered useful ground:
                # the corridor simply ends, or the map has not seen further.
                # The caller decides whether what was achieved is enough.
                rospy.logwarn(
                    "%s stopped early: no further known-free strip after "
                    "%.2f/%.2f m (observed %.2f m)",
                    label, advanced, distance, free_run)
                return advanced
            target = self.make_pose(
                here.pose.position.x + step * math.cos(bearing),
                here.pose.position.y + step * math.sin(bearing),
                bearing, frame)
            self.emit("MAP_CHECKED_STEP",
                      "%s step %d advances %.2f m inside a %.2f m known-free "
                      "strip" % (label, steps + 1, step, free_run),
                      x=target.pose.position.x, y=target.pose.position.y,
                      advanced=advanced, required=distance, floor=floor)
            self.navigate(target, "%s step %d" % (label, steps + 1))
            achieved = self.current_pose()
            new_advanced = (
                (achieved.pose.position.x - origin_x) * math.cos(bearing) +
                (achieved.pose.position.y - origin_y) * math.sin(bearing))
            if new_advanced - advanced < self.exit_minimum_progress:
                raise MissionFailure(
                    "%s step %d achieved only %.2f m forward progress"
                    % (label, steps + 1, new_advanced - advanced))
            advanced = new_advanced
            steps += 1
        return advanced

    def exit_upper_floor_without_doorway(self, floor):
        """Execute option A: preserved heading exit, then map-probed corridor.

        No upper-floor semantic doorway is required.  The robot arrives facing
        the physical opening, and the freshly restarted localization expresses
        that heading in the new generation.  It advances in short goals only
        when a footprint-width strip is fresh and known-free.  Once clear by
        ``car_depth + lobby_standoff``, perpendicular probes select the corridor
        direction.  Weak, stale or ambiguous evidence fails closed in the car
        or lobby instead of falling back to fixed 2 m / 95 deg / 5 m motion.
        """
        start = self.current_pose()
        frame = start.header.frame_id
        with self.lock:
            yaw_out = self.arrival_exit_yaws.get(floor)
        if yaw_out is None:
            raise MissionFailure(
                "floor %d has no generation-local arrival exit heading" % floor)
        current_yaw = yaw_of(start.pose.orientation)
        yaw_error = abs(self.angle_error(yaw_out, current_yaw))
        if yaw_error > self.transfer_turn_tolerance:
            raise MissionFailure(
                "floor %d arrival heading drifted %.1f deg before exit"
                % (floor, math.degrees(yaw_error)))

        # Point A is the actual post-relocalization car pose, not a door-derived
        # guess.  Its inward orientation makes the later return enter the car;
        # turn_inside_elevator_before_transfer() then reuses yaw_out to face the
        # opening for the next teleport.
        point_a = self.make_pose(
            start.pose.position.x, start.pose.position.y,
            yaw_out + math.pi, frame)
        with self.lock:
            self.elevator_return_points[floor] = copy.deepcopy(point_a)
        self.emit("ELEVATOR_RETURN_POINT_LOCKED",
                  "locked point A at the achieved generation-local arrival pose",
                  x=point_a.pose.position.x, y=point_a.pose.position.y,
                  exit_yaw=yaw_out)

        # Centre before probing: a footprint-width strip measured from against
        # a wall reports the wall, not the way out.
        self.recenter_in_car("floor %d pre-exit" % floor)
        required_clearance = self.car_depth + self.lobby_standoff
        origin_x = start.pose.position.x
        origin_y = start.pose.position.y
        advanced = 0.0
        steps = 0
        while advanced + self.exit_completion_tolerance < required_clearance:
            if steps >= self.exit_maximum_steps:
                raise MissionFailure(
                    "floor %d elevator exit exceeded %d bounded steps after "
                    "advancing %.2f/%.2f m"
                    % (floor, self.exit_maximum_steps, advanced,
                       required_clearance))
            here = self.current_pose()
            remaining = required_clearance - advanced
            deadline = time.monotonic() + self.exit_map_wait
            step = 0.0
            free_run = 0.0
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                probe_range = min(
                    self.exit_step_maximum + self.exit_goal_margin,
                    max(self.exit_minimum_goal + self.exit_goal_margin,
                        remaining + self.exit_goal_margin))
                free_run = self.known_free_run(
                    here.pose.position.x, here.pose.position.y,
                    yaw_out, probe_range, here)
                step = bounded_exit_step(
                    remaining, free_run, self.exit_step_maximum,
                    self.exit_minimum_goal, self.exit_goal_margin)
                if step > 0.0:
                    break
                rospy.loginfo_throttle(
                    1.0,
                    "floor %d elevator exit waiting for %.2f m known-free "
                    "strip ahead (observed %.2f m)",
                    floor,
                    min(self.exit_step_maximum,
                        max(remaining, self.exit_minimum_goal)) +
                    self.exit_goal_margin,
                    free_run)
                time.sleep(0.10)
                here = self.current_pose()
            if step <= 0.0:
                raise MissionFailure(
                    "floor %d elevator exit map never proved a safe next step "
                    "within %.0f wall seconds (free %.2f m, advanced %.2f/%.2f m)"
                    % (floor, self.exit_map_wait, free_run, advanced,
                       required_clearance))
            target = self.make_pose(
                here.pose.position.x + step * math.cos(yaw_out),
                here.pose.position.y + step * math.sin(yaw_out),
                yaw_out, frame)
            self.emit("ELEVATOR_EXIT_STEP",
                      "step %d advances %.2f m inside a %.2f m known-free strip"
                      % (steps + 1, step, free_run),
                      x=target.pose.position.x, y=target.pose.position.y,
                      advanced=advanced, required=required_clearance)
            self.navigate(target, "bounded map-checked elevator exit step %d"
                          % (steps + 1))
            achieved = self.current_pose()
            new_advanced = (
                (achieved.pose.position.x - origin_x) * math.cos(yaw_out) +
                (achieved.pose.position.y - origin_y) * math.sin(yaw_out))
            if new_advanced - advanced < self.exit_minimum_progress:
                raise MissionFailure(
                    "floor %d elevator exit step %d achieved only %.2f m "
                    "forward progress"
                    % (floor, steps + 1, new_advanced - advanced))
            advanced = new_advanced
            steps += 1

        self.emit("ELEVATOR_EXIT_CLEAR",
                  "cleared the car by %.2f m in %d map-checked steps"
                  % (advanced, steps),
                  advanced=advanced, required=required_clearance)

        # Probe both perpendicular directions repeatedly while the lobby map
        # fills.  If both look equally open, direction is not proven and the
        # mission stays put instead of selecting a sign by tuple ordering.
        deadline = time.monotonic() + self.corridor_map_wait
        left_bearing = yaw_out + 0.5 * math.pi
        right_bearing = yaw_out - 0.5 * math.pi
        side = None
        left_run = right_run = 0.0
        here = self.current_pose()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            here = self.current_pose()
            left_run = self.known_free_run(
                here.pose.position.x, here.pose.position.y,
                left_bearing, self.corridor_probe_max_range, here)
            right_run = self.known_free_run(
                here.pose.position.x, here.pose.position.y,
                right_bearing, self.corridor_probe_max_range, here)
            side = choose_corridor_side(
                left_run, right_run, self.corridor_minimum_run,
                self.corridor_minimum_advantage)
            if side is not None:
                break
            rospy.loginfo_throttle(
                1.0,
                "floor %d corridor probe waiting: left=%.2f m right=%.2f m "
                "minimum=%.2f advantage=%.2f",
                floor, left_run, right_run, self.corridor_minimum_run,
                self.corridor_minimum_advantage)
            time.sleep(0.10)
        if side is None:
            raise MissionFailure(
                "floor %d corridor direction remained unavailable or ambiguous "
                "for %.0f wall seconds (left %.2f m, right %.2f m)"
                % (floor, self.corridor_map_wait, left_run, right_run))
        yaw_corridor = left_bearing if side == "left" else right_bearing
        run = left_run if side == "left" else right_run
        # The single probe above only chooses the SIDE. How far to walk is
        # re-proved at every step, because a probe is valid from where it was
        # taken and nowhere else -- see advance_map_checked.
        requested = max(0.0, run - self.corridor_run_margin)
        self.emit("UPPER_FLOOR_CORRIDOR_INGRESS",
                  "walking up to %.2f m along the %s corridor, re-proving the "
                  "map every step (probe: left %.2f, right %.2f m)"
                  % (requested, side, left_run, right_run),
                  requested=requested, side=side)
        achieved = self.advance_map_checked(
            yaw_corridor, requested,
            "floor %d corridor ingress" % floor, floor)
        if achieved < self.corridor_minimum_run:
            raise MissionFailure(
                "floor %d corridor ingress advanced only %.2f m of a requested "
                "%.2f m before the map stopped proving free space"
                % (floor, achieved, requested))
        final = self.current_pose()
        entry = self.make_pose(final.pose.position.x, final.pose.position.y,
                               yaw_corridor, frame)
        entry = self.correct_upper_floor_entry_axis(entry)
        with self.lock:
            self.floor_entry = copy.deepcopy(entry)
        self.emit("UPPER_FLOOR_MAIN_CORRIDOR",
                  "reached the main corridor from the preserved arrival heading")
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
                "no fresh corridor-wall pair; using selected map-probe axis",
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
