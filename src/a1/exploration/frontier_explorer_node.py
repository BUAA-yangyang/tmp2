#!/usr/bin/env python3
"""Single-floor frontier exploration with autonomous return.

Normal exploration and return targets use MoveBaseAction. Explicit room scans
and bounded recovery publish desired body velocity only through the higher
priority behavior input of a1_cmd_mux; State_RL remains the sole joint-level
locomotion controller.
"""

import copy
from collections import deque
import math
import threading
import time
from types import SimpleNamespace

import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from a1_navigation_interfaces.msg import (
    DoorwayArray,
    ExploreFloorAction,
    ExploreFloorFeedback,
    ExploreFloorResult,
    ExplorationStatus,
    SpecialBehaviorAction,
    SpecialBehaviorGoal,
    WallSegmentArray,
)
from diagnostic_msgs.msg import DiagnosticStatus
from dynamic_reconfigure.msg import DoubleParameter
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from geometry_msgs.msg import Point, Point32, PolygonStamped, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from a1_exploration.frontier import (
    GridSpec,
    NoFrontierEvidence,
    NoProgressWatchdog,
    _dilate,
    corridor_gate_decision,
    corridor_probe_goal_state,
    coverage_ratio,
    dominant_axis_correction,
    extract_frontiers,
    failed_goal_state,
    first_near_field_blocker,
    has_pending_retry,
    known_cell_count,
    known_free_path_exists,
    local_plan_is_acceptable,
    map_margin_mask,
    nearest_known_free_anchor,
    occupancy_content_fingerprint,
    point_in_polygon,
    point_near,
    polygon_mask,
    record_failure,
    return_anchor_selection,
    room_axis_bounds,
    room_queue_order,
    segment_corridor_mask,
    transform_local_polygon,
)
from a1_exploration.final_zero import (
    CMD_VEL_NAV_SILENCE_TIMEOUT_S,
    FinalZeroMonitor,
    interstitial_zero_gate,
    wall_backstop_seconds,
)
from a1_exploration.entry_speed_limit import (
    EntrySpeedLimitError,
    EntrySpeedLimiter,
)


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("quaternion is not finite and normalizable")
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    return math.atan2(
        2.0 * (
            w * z
            + x * y
        ),
        1.0 - 2.0 * (
            y * y
            + z * z
        ),
    )


ENTRY_QUATERNION_NORM_TOLERANCE = 1e-3
ENTRY_PLANAR_COMPONENT_TOLERANCE = 1e-6


class InvalidEntryPose(ValueError):
    pass


def validate_floor_entry_pose(pose, expected_frame=None):
    """Validate the public 2-D entry contract without projecting its input."""
    if not pose.header.frame_id:
        raise InvalidEntryPose("frame_id is empty")
    if expected_frame is not None and pose.header.frame_id != expected_frame:
        raise InvalidEntryPose(
            "frame_id %r does not match current map frame %r"
            % (pose.header.frame_id, expected_frame)
        )
    position = pose.pose.position
    position_values = (position.x, position.y, position.z)
    if not all(math.isfinite(value) for value in position_values):
        raise InvalidEntryPose("position contains a non-finite value")
    quaternion = pose.pose.orientation
    quaternion_values = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    if not all(math.isfinite(value) for value in quaternion_values):
        raise InvalidEntryPose("orientation contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in quaternion_values))
    if abs(norm - 1.0) > ENTRY_QUATERNION_NORM_TOLERANCE:
        raise InvalidEntryPose(
            "orientation is not a unit quaternion: norm=%.9f tolerance=%.1e"
            % (norm, ENTRY_QUATERNION_NORM_TOLERANCE)
        )
    if (
            abs(quaternion.x) > ENTRY_PLANAR_COMPONENT_TOLERANCE
            or abs(quaternion.y) > ENTRY_PLANAR_COMPONENT_TOLERANCE):
        raise InvalidEntryPose(
            "orientation is not planar: |x|=%.9g |y|=%.9g tolerance=%.1e"
            % (
                abs(quaternion.x),
                abs(quaternion.y),
                ENTRY_PLANAR_COMPONENT_TOLERANCE,
            )
        )
    return yaw_from_quaternion(quaternion)


def angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def diagnostic_values(message):
    return {item.key: item.value for item in message.values}


class ExplorationFailure(RuntimeError):
    def __init__(self, code, message, preempted=False):
        super().__init__(message)
        self.code = code
        self.preempted = preempted


class FrontierExplorer:
    INTERNAL_STATES = (
        "IDLE",
        "RECORD_START",
        "REQUEST_ENTRY_DOOR_OPEN",
        "TRANSIT_TO_ENTRY",
        "ENTERED_FLOOR",
        "SELECT_FRONTIER",
        "NAVIGATING",
        "UPDATE_COVERAGE",
        "EXPLORATION_DONE",
        "RETURNING",
        "RETURNED",
        "FAILED",
        "CANCELLED",
    )

    STATUS_CODES = {
        "IDLE": ExplorationStatus.IDLE,
        "RECORD_START": ExplorationStatus.RECORD_START,
        "REQUEST_ENTRY_DOOR_OPEN":
            ExplorationStatus.REQUEST_ENTRY_DOOR_OPEN,
        "TRANSIT_TO_ENTRY": ExplorationStatus.TRANSIT_TO_ENTRY,
        "ENTERED_FLOOR": ExplorationStatus.ENTERED_FLOOR,
        "SELECT_FRONTIER": ExplorationStatus.SELECTING_TARGET,
        "NAVIGATING": ExplorationStatus.NAVIGATING,
        "UPDATE_COVERAGE": ExplorationStatus.UPDATE_COVERAGE,
        "EXPLORATION_DONE": ExplorationStatus.EXPLORATION_DONE,
        "RETURNING": ExplorationStatus.RETURNING,
        "RETURNED": ExplorationStatus.RETURNED,
        "FAILED": ExplorationStatus.FAILED,
        "CANCELLED": ExplorationStatus.CANCELLED,
    }

    FEEDBACK_CODES = {
        "IDLE": ExploreFloorFeedback.IDLE,
        "RECORD_START": ExploreFloorFeedback.RECORD_START,
        "REQUEST_ENTRY_DOOR_OPEN":
            ExploreFloorFeedback.REQUEST_ENTRY_DOOR_OPEN,
        "TRANSIT_TO_ENTRY": ExploreFloorFeedback.TRANSIT_TO_ENTRY,
        "ENTERED_FLOOR": ExploreFloorFeedback.ENTERED_FLOOR,
        "SELECT_FRONTIER": ExploreFloorFeedback.SELECTING_TARGET,
        "NAVIGATING": ExploreFloorFeedback.NAVIGATING,
        "UPDATE_COVERAGE": ExploreFloorFeedback.UPDATE_COVERAGE,
        "EXPLORATION_DONE": ExploreFloorFeedback.EXPLORATION_DONE,
        "RETURNING": ExploreFloorFeedback.RETURNING,
        "RETURNED": ExploreFloorFeedback.RETURNED,
        "FAILED": ExploreFloorFeedback.FAILED,
        "CANCELLED": ExploreFloorFeedback.CANCELLED,
    }

    def __init__(self):
        self.lock = threading.RLock()
        self.map_message = None
        # Set before any subscriber exists: map_callback reaches it on the
        # very first grid, and a missing attribute there is an AttributeError
        # inside a callback, not a startup error.
        self.map_margin_cache = None
        self.doorway_message = None
        self.wall_message = None
        self.corridor_model = None
        self.remembered_room_doorways = {}
        self.mapping_status = None
        self.last_mapping_healthy_wall = 0.0
        self.final_command = None
        self.safety_locked = False
        self.controller_ready = False
        self.controller_ready_stamp = None

        # Explicit integration-test hook.  It is off in every normal launch;
        # when enabled it exercises entrance -> mandatory corridor ingress ->
        # elevator transfer without conflating the result with room coverage.
        self.force_floor_complete_after_ingress = bool(self.param(
            "testing/force_floor_complete_after_mandatory_ingress", False
        ))
        # The competition building has a fixed, known per-floor room count.
        # Complete only after that many distinct corridor room transactions
        # have fully entered, scanned, and exited; merely approaching a door
        # or reaching an interior goal is deliberately insufficient evidence.
        self.floor_completion_room_count = int(self.param(
            "floor_completion/completed_room_count", 0
        ))
        if self.floor_completion_room_count < 0:
            raise ValueError(
                "floor_completion/completed_room_count must be non-negative"
            )

        self.state = "IDLE"
        self.state_message = "waiting for ExploreFloor goal"
        self.floor_id = -1
        self.coverage = 0.0
        self.current_target = PoseStamped()
        self.action_active = False
        self.action_identity = None
        self.action_ros_deadline = None
        self.action_wall_deadline = None
        self.start_pose = None
        self.floor_entry_pose = None
        self.roi_local = ()
        self.roi_polygon_map = ()
        self.trajectory = Path()
        self.visited_goals = []
        self.failed_goals = []
        self.completed_room_branches = set()
        self.floor_completion_target = 0
        self.approached_room_branches = set()
        self.room_branch_entry_poses = {}
        self.room_branch_interior_poses = {}
        self.active_room_branch = None
        self.selected_room_branch = None
        self.selected_room_stage = None
        self.selected_corridor_center_target = False
        self.maximum_corridor_progress = 0.0
        self.no_goal_evidence = None
        self.make_plan_failure_since_wall = None

        self.base_frame = self.param("frames/base", "base")
        self.map_topic = self.param(
            "topics/map", "/a1/floor_mapping/map"
        )
        self.mapping_status_topic = self.param(
            "topics/mapping_status", "/a1/floor_mapping/status"
        )
        self.doorways_topic = self.param(
            "topics/doorways", "/a1/floor_mapping/doorways"
        )
        self.walls_topic = self.param(
            "topics/walls", "/a1/floor_mapping/walls"
        )
        self.final_cmd_topic = self.param(
            "topics/final_cmd_vel", "/cmd_vel"
        )
        self.nav_cmd_topic = self.param(
            "topics/cmd_vel_nav", "/cmd_vel_nav"
        )
        self.recovery_cmd_topic = self.param(
            "topics/recovery_cmd_vel", "/cmd_vel_behavior"
        )
        self.safety_lock_topic = self.param(
            "topics/safety_lock", "/a1_cmd_mux/safety_lock"
        )
        self.controller_ready_topic = self.param(
            "topics/controller_ready", "/a1/controller_ready"
        )
        self.status_topic = self.param(
            "topics/status", "/a1/exploration/status"
        )
        self.frontier_topic = self.param(
            "topics/frontiers", "/a1/exploration/frontiers"
        )
        self.target_topic = self.param(
            "topics/selected_target", "/a1/exploration/selected_target"
        )
        self.failed_topic = self.param(
            "topics/failed_targets", "/a1/exploration/failed_targets"
        )
        self.trajectory_topic = self.param(
            "topics/trajectory", "/a1/exploration/trajectory"
        )
        self.scope_topic = self.param(
            "topics/scope", "/a1/exploration/scope"
        )
        self.roi_topic = self.param(
            "topics/roi", "/a1/exploration/roi"
        )
        self.corridor_model_topic = self.param(
            "topics/corridor_model", "/a1/exploration/corridor_model"
        )
        self.explore_action_name = self.param(
            "actions/explore_floor", "/a1/exploration/explore_floor"
        )
        self.move_action_name = self.param(
            "actions/move_base", "/move_base"
        )
        self.building_behavior_action_name = self.param(
            "actions/building_behavior", "/a1/building_behavior/special"
        )
        self.make_plan_name = self.param(
            "services/make_plan", "/move_base/make_plan"
        )
        self.dwa_reconfigure_name = self.param(
            "services/dwa_reconfigure",
            "/move_base/DWAPlannerROS/set_parameters",
        )

        self.min_frontier_length = float(
            self.param("frontier/min_length", 0.35)
        )
        self.obstacle_clearance = float(
            self.param("frontier/obstacle_clearance", 0.30)
        )
        self.goal_standoff = float(
            self.param("frontier/goal_standoff", 0.35)
        )
        self.goal_search_radius = float(
            self.param("frontier/goal_search_radius", 0.60)
        )
        self.min_goal_distance = float(
            self.param("frontier/min_goal_distance", 0.45)
        )
        self.navigation_xy_goal_tolerance = float(
            self.param("navigation/xy_goal_tolerance", 0.35)
        )
        self.effective_min_goal_distance = max(
            self.min_goal_distance,
            self.goal_standoff + self.navigation_xy_goal_tolerance,
        )
        self.max_goal_distance = float(
            self.param("frontier/max_goal_distance", 9.0)
        )
        self.free_threshold = int(
            self.param("frontier/free_threshold", 20)
        )
        self.occupied_threshold = int(
            self.param("frontier/occupied_threshold", 65)
        )
        self.information_gain_weight = float(
            self.param("frontier/information_gain_weight", 1.0)
        )
        self.distance_weight = float(
            self.param("frontier/distance_weight", 0.25)
        )
        self.minimum_frontier_score = float(
            self.param("frontier/minimum_score", 0.0)
        )
        self.room_priority_enabled = bool(
            self.param("frontier/room_priority/enabled", True)
        )
        self.room_lateral_threshold = float(
            self.param("frontier/room_priority/lateral_threshold", 1.0)
        )
        self.room_minimum_door_longitudinal = float(
            self.param(
                "frontier/room_priority/minimum_door_longitudinal", 3.0
            )
        )
        self.room_door_minimum_width = float(
            self.param("frontier/room_priority/door_minimum_width", 1.0)
        )
        self.room_door_maximum_width = float(
            self.param("frontier/room_priority/door_maximum_width", 1.6)
        )
        self.room_door_station_tolerance = float(
            self.param("frontier/room_priority/door_station_tolerance", 1.25)
        )
        self.room_door_maximum_lateral = float(
            self.param(
                "frontier/room_priority/door_maximum_lateral", 2.20
            )
        )
        self.room_lookahead = float(
            self.param("frontier/room_priority/lookahead", 5.0)
        )
        self.room_backtrack = float(
            self.param("frontier/room_priority/backtrack", 2.0)
        )
        self.room_station_width = float(
            self.param("frontier/room_priority/station_width", 1.5)
        )
        self.room_identity_longitudinal_tolerance = float(
            self.param(
                "frontier/room_priority/identity_longitudinal_tolerance",
                1.0,
            )
        )
        self.room_identity_lateral_tolerance = float(
            self.param(
                "frontier/room_priority/identity_lateral_tolerance", 0.75
            )
        )
        self.room_completion_depth = float(
            self.param("frontier/room_priority/completion_depth", 2.0)
        )
        self.room_goal_extension = float(
            self.param("frontier/room_priority/goal_extension", 1.2)
        )
        self.room_scan_angular_speed = float(
            self.param("frontier/room_priority/scan_angular_speed", 0.50)
        )
        self.room_exit_align_tolerance = float(
            self.param(
                "frontier/room_priority/exit_align_tolerance", 0.12
            )
        )
        self.room_exit_align_timeout = float(
            self.param(
                "frontier/room_priority/exit_align_timeout", 10.0
            )
        )
        self.room_exit_reobserve_time = float(
            self.param(
                "frontier/room_priority/exit_reobserve_time", 1.0
            )
        )
        self.room_exit_center_clearance = float(
            self.param(
                "frontier/room_priority/exit_center_clearance", 0.55
            )
        )
        self.room_scan_clearance = float(
            self.param("frontier/room_priority/scan_clearance", 0.78)
        )
        self.room_scan_search_distance = float(
            self.param("frontier/room_priority/scan_search_distance", 2.5)
        )
        self.corridor_probe_enabled = bool(
            self.param("frontier/corridor_probe/enabled", True)
        )
        self.corridor_probe_step = float(
            self.param("frontier/corridor_probe/step", 3.0)
        )
        self.corridor_probe_clearance = float(
            self.param("frontier/corridor_probe/clearance", 0.24)
        )
        self.corridor_probe_minimum_completed_rooms = int(
            self.param(
                "frontier/corridor_probe/minimum_completed_rooms", 2
            )
        )
        self.corridor_wall_angle_tolerance = float(
            self.param("frontier/corridor_model/wall_angle_tolerance", 0.30)
        )
        self.corridor_minimum_width = float(
            self.param("frontier/corridor_model/minimum_width", 1.50)
        )
        self.corridor_maximum_width = float(
            self.param("frontier/corridor_model/maximum_width", 3.60)
        )
        self.corridor_minimum_wall_length = float(
            self.param("frontier/corridor_model/minimum_wall_length", 0.70)
        )
        self.corridor_model_lookahead = float(
            self.param("frontier/corridor_model/lookahead", 4.0)
        )
        self.corridor_model_lookbehind = float(
            self.param("frontier/corridor_model/lookbehind", 2.0)
        )
        self.corridor_model_max_age = float(
            self.param("frontier/corridor_model/max_age", 1.5)
        )
        self.corridor_model_minimum_longitudinal = float(
            self.param("frontier/corridor_model/minimum_longitudinal", 7.0)
        )
        self.active_corridor_model_minimum_longitudinal = (
            self.corridor_model_minimum_longitudinal
        )
        self.corridor_center_tolerance = float(
            self.param("frontier/corridor_model/center_tolerance", 0.18)
        )
        self.corridor_center_filter_gain = float(
            self.param("frontier/corridor_model/filter_gain", 0.45)
        )
        self.entry_frontier_exclusion_depth = float(
            self.param("frontier/entry_exclusion/depth", 1.0)
        )
        self.entry_frontier_exclusion_half_width = float(
            self.param("frontier/entry_exclusion/half_width", 1.25)
        )
        self.visited_radius = float(
            self.param("frontier/visited_radius", 0.70)
        )
        self.failed_radius = float(
            self.param("frontier/failed_radius", 0.75)
        )
        self.failure_cooldown = float(
            self.param("frontier/failure_cooldown", 4.0)
        )
        self.maximum_failures = int(
            self.param("frontier/maximum_failures", 2)
        )
        self.empty_confirmations = int(
            self.param("frontier/empty_confirmations", 3)
        )
        self.stable_no_frontier_duration = float(
            self.param("frontier/stable_no_frontier_duration", 2.0)
        )
        self.minimum_free_cells = int(
            self.param("frontier/minimum_free_cells", 100)
        )
        self.make_plan_retry_attempts = int(
            self.param("planning/make_plan_retry_attempts", 3)
        )
        self.make_plan_retry_delay = float(
            self.param("planning/make_plan_retry_delay_wall", 0.15)
        )
        self.make_plan_unavailable_timeout = float(
            self.param("planning/make_plan_unavailable_timeout_wall", 10.0)
        )

        self.roi_enabled = bool(self.param("roi/enabled", True))
        self.require_explicit_entry_pose = bool(
            self.param("entry/require_explicit_pose", True)
        )
        self.entry_position_tolerance = float(
            self.param("entry/position_tolerance", 0.40)
        )
        # The elevator anchor is reached by an ordinary move_base goal, so the
        # arrival error is bounded by DWA's own xy_goal_tolerance (0.45 m), not
        # by the explorer. Checking it against the same 0.45 m makes the gate a
        # coin flip on a legal arrival; this is a sanity bound on the exit
        # route, and a genuinely failed exit misses by metres, not centimetres.
        self.elevator_anchor_tolerance = float(
            self.param("entry/elevator_anchor_tolerance", 0.75)
        )
        self.entry_centerline_enabled = bool(
            self.param("entry/door_centerline/enabled", True)
        )
        self.entry_centerline_required = bool(
            self.param("entry/door_centerline/required", True)
        )
        self.entry_centerline_timeout = float(
            self.param("entry/door_centerline/timeout_wall", 5.0)
        )
        self.entry_centerline_min_width = float(
            self.param("entry/door_centerline/minimum_width", 1.20)
        )
        self.entry_centerline_max_width = float(
            self.param("entry/door_centerline/maximum_width", 2.40)
        )
        self.entry_centerline_max_lateral = float(
            self.param("entry/door_centerline/maximum_lateral", 1.20)
        )
        self.entry_centerline_min_forward = float(
            self.param("entry/door_centerline/minimum_forward", 0.40)
        )
        self.entry_centerline_target_margin = float(
            self.param("entry/door_centerline/target_forward_margin", 0.60)
        )
        self.entry_centerline_min_inside_depth = float(
            self.param("entry/door_centerline/minimum_inside_depth", 0.80)
        )
        self.entry_centerline_max_inside_depth = float(
            self.param("entry/door_centerline/maximum_inside_depth", 2.50)
        )
        self.entry_yaw_tolerance = float(
            self.param("entry/yaw_tolerance", 0.65)
        )
        self.entry_inside_probe_distance = float(
            self.param("entry/inside_probe_distance", 0.75)
        )
        self.entry_anchor_search_radius = float(
            self.param("entry/anchor_search_radius", 0.60)
        )
        self.entry_minimum_new_known_cells = int(
            self.param("entry/minimum_new_known_cells", 20)
        )
        self.entry_corridor_half_width = float(
            self.param("entry/corridor_half_width", 0.75)
        )
        self.entry_transit_speed = float(
            self.param("entry/transit_speed", 0.80)
        )
        self.entry_near_field_distance = float(
            self.param("entry/near_field_distance", 0.70)
        )
        self.entry_near_field_half_width = float(
            self.param("entry/near_field_half_width", 0.30)
        )
        self.entry_obstacle_hold_timeout = float(
            self.param("entry/obstacle_hold_timeout", 2.0)
        )
        self.entry_plan_endpoint_tolerance = float(
            self.param("entry/plan_endpoint_tolerance", 0.60)
        )
        self.entry_plan_maximum_length_ratio = float(
            self.param("entry/plan_maximum_length_ratio", 1.50)
        )
        self.entry_plan_maximum_length_slack = float(
            self.param("entry/plan_maximum_length_slack", 0.75)
        )
        self.entry_speed_limit_enabled = bool(
            self.param("entry/speed_limit/enabled", True)
        )
        self.entry_speed_service_wait = float(
            self.param("entry/speed_limit/service_wait_wall", 3.0)
        )
        self.entry_speed_limits = {
            "max_vel_x": float(
                self.param("entry/speed_limit/max_vel_x", 0.15)
            ),
            "max_vel_y": float(
                self.param("entry/speed_limit/max_vel_y", 0.0)
            ),
            "max_vel_trans": float(
                self.param("entry/speed_limit/max_vel_trans", 0.15)
            ),
            "max_vel_theta": float(
                self.param("entry/speed_limit/max_vel_theta", 0.01)
            ),
            "min_vel_x": float(
                self.param("entry/speed_limit/min_vel_x", 0.15)
            ),
            "min_vel_trans": float(
                self.param("entry/speed_limit/min_vel_trans", 0.15)
            ),
            "min_vel_theta": float(
                self.param("entry/speed_limit/min_vel_theta", 0.0)
            ),
            "sim_time": float(
                self.param("entry/speed_limit/sim_time", 0.5)
            ),
        }
        self.roi_entry_forward_offset = float(
            self.param("roi/default_entry_forward_offset", 3.5)
        )
        raw_roi_polygon = self.param(
            "roi/default_local_polygon",
            [
                0.0, -8.65, 40.0, -8.65,
                40.0, 8.65, 0.0, 8.65,
            ],
        )
        if len(raw_roi_polygon) % 2 != 0:
            raise ValueError("roi/default_local_polygon requires x,y pairs")
        self.default_roi_local = tuple(
            (float(raw_roi_polygon[index]), float(raw_roi_polygon[index + 1]))
            for index in range(0, len(raw_roi_polygon), 2)
        )
        # An elevator entry stands part-way down the floor, so unlike the
        # public entrance it has building behind it as well as in front.
        raw_elevator_polygon = self.param(
            "roi/elevator_entry_local_polygon",
            [
                -13.0, -10.5, 32.0, -10.5,
                32.0, 10.5, -13.0, 10.5,
            ],
        )
        if len(raw_elevator_polygon) % 2 != 0:
            raise ValueError(
                "roi/elevator_entry_local_polygon requires x,y pairs"
            )
        self.elevator_roi_local = tuple(
            (
                float(raw_elevator_polygon[index]),
                float(raw_elevator_polygon[index + 1]),
            )
            for index in range(0, len(raw_elevator_polygon), 2)
        )
        self.roi_boundary_margin = float(
            self.param("roi/boundary_margin", 0.35)
        )
        self.roi_map_boundary_margin = float(
            self.param("roi/map_boundary_margin", 8.0)
        )
        self.entry_axis_alignment_enabled = bool(
            self.param("entry/axis_alignment/enabled", True)
        )
        self.entry_axis_maximum_correction = float(
            self.param("entry/axis_alignment/maximum_correction_rad", 0.61)
        )
        self.entry_axis_minimum_wall_length = float(
            self.param("entry/axis_alignment/minimum_wall_length", 1.00)
        )
        self.entry_axis_radius = float(
            self.param("entry/axis_alignment/radius", 12.0)
        )
        self.entry_axis_minimum_weight = float(
            self.param("entry/axis_alignment/minimum_weight", 2.50)
        )
        self.entry_axis_minimum_walls = int(
            self.param("entry/axis_alignment/minimum_walls", 2)
        )
        self.entry_axis_timeout = float(
            self.param("entry/axis_alignment/timeout_wall", 8.0)
        )
        self.controller_ready_freshness = float(
            self.param("controller/ready_freshness", 0.35)
        )

        self.prerequisite_timeout = float(
            self.param("timeouts/prerequisites", 30.0)
        )
        self.navigation_timeout = float(
            self.param("timeouts/navigation_goal", 45.0)
        )
        self.backout_speed = float(
            self.param("navigation/backout/speed", 0.35)
        )
        self.backout_step_distance = float(
            self.param("navigation/backout/step_distance", 0.35)
        )
        self.backout_max_sim_time = float(
            self.param("navigation/backout/max_sim_time", 2.0)
        )
        self.backout_max_steps = int(
            self.param("navigation/backout/max_steps", 3)
        )
        self.entry_door_timeout = float(
            self.param("timeouts/entry_door_wall", 10.0)
        )
        self.entry_transit_timeout = float(
            self.param("timeouts/entry_transit", 90.0)
        )
        self.entry_map_timeout = float(
            self.param("timeouts/entry_map", 15.0)
        )
        self.entry_plan_timeout = float(
            self.param("timeouts/entry_plan", 15.0)
        )
        self.return_timeout = float(
            self.param("timeouts/return_goal", 60.0)
        )
        self.map_update_wait = float(
            self.param("timeouts/map_update_wait", 2.0)
        )
        self.mapping_health_grace = float(
            self.param("timeouts/mapping_health_grace", 1.0)
        )
        self.final_zero_timeout = float(
            self.param("timeouts/final_zero", 3.0)
        )
        self.wall_factor = float(
            self.param("timeouts/wall_factor", 20.0)
        )

        self.return_position_tolerance = float(
            self.param("return/position_tolerance", 0.40)
        )
        self.return_yaw_tolerance = float(
            self.param("return/yaw_tolerance", 0.65)
        )
        self.return_attempts = int(
            self.param("return/maximum_attempts", 2)
        )
        self.zero_epsilon = float(
            self.param("return/zero_velocity_epsilon", 0.01)
        )
        self.zero_settle = float(
            self.param("return/zero_settle_time", 0.50)
        )
        self.command_freshness = float(
            self.param("return/command_freshness", 0.25)
        )
        self.final_zero_monitor = FinalZeroMonitor(
            self.zero_epsilon, self.command_freshness, self.zero_settle
        )

        # --- ported from the single-floor tree with the room transaction ---
        # No-progress watchdog for navigate(). move_base can sit on an
        # unreachable goal publishing nothing until the goal timeout expires:
        # measured 2026-08-01, the robot stood at world (2.240, 1.011) for more
        # than 24 s with cmd_vel identically zero while the status still read
        # NAVIGATING. Rotating on the spot is legitimate progress, so yaw counts.
        self.no_progress_timeout = float(
            self.param("navigation/no_progress/timeout", 20.0)
        )
        self.no_progress_distance = float(
            self.param("navigation/no_progress/distance", 0.20)
        )
        self.no_progress_yaw = float(
            self.param("navigation/no_progress/yaw", 0.35)
        )
        # B2: how long commanded velocity must be continuously zero,
        # on ROS/sim time, after ANY non-success navigation outcome
        # (aborted/cancelled/preempted/recovery/timeout) before the
        # node may select or send the next goal.
        self.interstitial_zero_settle_s = float(
            self.param("frontier/interstitial_zero_settle_s", 0.5)
        )
        self.room_goal_clearance = float(
            self.param("frontier/room_priority/goal_clearance", 0.40)
        )
        # How many of the nearest gap cells to try as coverage goals before
        # giving up this cycle.
        self.camera_coverage_candidates = int(
            self.param("room/camera_coverage/candidates", 12)
        )
        self.camera_coverage_enabled = bool(
            self.param("room/camera_coverage/enabled", False)
        )
        self.camera_coverage_half_angle = float(
            self.param("room/camera_coverage/half_angle", math.radians(30.0))
        )
        self.camera_coverage_radius = float(
            self.param("room/camera_coverage/radius", 4.5)
        )
        # Fraction of the room's free area that must be camera-covered before
        # the transaction may report proven coverage.
        self.camera_coverage_required = float(
            self.param("room/camera_coverage/required_fraction", 0.90)
        )
        self.camera_covered = None
        self.corridor_minimum_frontier_score = float(
            self.param("frontier/corridor_minimum_score", 1.5)
        )
        self.corridor_probe_barren = 0
        self.corridor_probe_exhausted = False
        self.corridor_probe_known_before = None
        self.corridor_probe_maximum_barren = int(
            self.param("frontier/corridor_probe/maximum_barren", 2)
        )
        self.corridor_probe_minimum_new_cells = int(
            self.param("frontier/corridor_probe/minimum_new_cells", 40)
        )
        # B2: separate monitors gate the NEXT goal after any
        # non-success navigation outcome. Distinct from
        # final_zero_monitor above (which only gates the end-of-action
        # return-to-start verification) so the two settle windows can
        # be tuned independently even though their defaults coincide.
        self.interstitial_cmd_vel_monitor = FinalZeroMonitor(
            self.zero_epsilon,
            self.command_freshness,
            self.interstitial_zero_settle_s,
        )
        self.interstitial_cmd_vel_nav_monitor = FinalZeroMonitor(
            self.zero_epsilon,
            self.command_freshness,
            self.interstitial_zero_settle_s,
        )
        self.last_room_transaction_proven = False
        # Corridor band used by the relaxed selection pass only. Must exceed the
        # largest plausible offset between the corridor centreline and the spawn
        # line (measured 1.27 m) so a corridor frontier is never mistaken for an
        # unmatched room mouth. CORRIDOR_WIDTH is 2.2 m, so 1.6 m still leaves
        # genuine room mouths outside the band.
        self.relaxed_lateral_threshold = float(
            self.param("frontier/relaxed_lateral_threshold", 1.6)
        )
        # Room transaction: once the robot is inside a room it must cover that
        # room before the mission may leave it. See explore_room_transaction.
        self.room_door_plane_margin = float(
            self.param("frontier/room_priority/door_plane_margin", 0.25)
        )
        self.room_frontier_min_distance = float(
            self.param("frontier/room_priority/frontier_min_distance", 0.55)
        )
        self.room_frontier_minimum_score = float(
            self.param("frontier/room_priority/frontier_minimum_score", -0.5)
        )
        self.room_goal_timeout = float(
            self.param("frontier/room_priority/goal_timeout", 25.0)
        )
        # Camera coverage of a room, as opposed to LiDAR frontier coverage.
        #
        # The two are not the same thing and the difference costs danger
        # sources. LiDAR maps a room's far side from the doorway, so that space
        # becomes known-free and stops producing frontiers; the transaction then
        # reports "no reachable frontier remains inside the room" and leaves,
        # even though the camera never got near enough to recognise anything
        # there. Measured across run14/run18 against the two floor-0 truth
        # sources, with the source inside the D415's 60 degree HFOV:
        #
        #   detected  at 2.10 m, 3.01 m, 1.49 m closest in-FOV range
        #   MISSED    at 5.55 m (run18, source B, 13 in-FOV samples)
        #
        # The danger-perception owner states the reliable recognition range is
        # within 5 m, and the measurements agree: the miss was at 5.55 m, just
        # outside it, and every detection was well inside. 4.5 m keeps a small
        # margin for pose error, motion blur and partial occlusion while
        # staying as large as possible -- a larger radius covers a room from
        # fewer viewpoints, and with 600 s of budget for three floors against
        # ~250 s already spent on one, travel is the scarce resource.
        # Explicitly OFF for the single-floor stabilisation phase. An explicit
        # flag rather than required_fraction=0, because 0 would read as
        # "coverage requirement satisfied" and silently look like a pass. With
        # this false the mask is never built, coverage never gates room or
        # floor completion, and no gap-closing goal is ever issued. The code
        # stays for the phase after multi-floor is reproducible.
        # Two doorways closer than this along the corridor axis are treated as
        # the same door station rather than as two rooms to cut between.
        self.room_station_separation = float(
            self.param("room/station_separation", 3.0)
        )
        self.room_transaction_max_goals = int(
            self.param("frontier/room_priority/transaction_max_goals", 8)
        )
        self.room_transaction_timeout = float(
            self.param("frontier/room_priority/transaction_timeout", 90.0)
        )
        # Rooms left on budget rather than on proof, with the number of
        # attempts already spent. They are marked complete so selection does
        # not loop on them, but get one more chance before the floor is
        # declared finished.
        self.unproven_room_branches = {}
        self.trajectory_period = float(
            self.param("trajectory/sample_period", 0.20)
        )
        self.trajectory_minimum_step = float(
            self.param("trajectory/minimum_step", 0.05)
        )
        self.trajectory_maximum_poses = int(
            self.param("trajectory/maximum_poses", 10000)
        )
        self.validate_parameters()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.move_client = actionlib.SimpleActionClient(
            self.move_action_name, MoveBaseAction
        )
        self.building_behavior_client = actionlib.SimpleActionClient(
            self.building_behavior_action_name, SpecialBehaviorAction
        )
        self.make_plan = rospy.ServiceProxy(self.make_plan_name, GetPlan)
        self.floor_mapping_reset = rospy.ServiceProxy(
            "/a1/floor_mapping/reset", Trigger
        )
        self.dwa_reconfigure = rospy.ServiceProxy(
            self.dwa_reconfigure_name, Reconfigure
        )
        self.entry_speed_limiter = EntrySpeedLimiter(
            self.dwa_reconfigure,
            ReconfigureRequest,
            DoubleParameter,
            self.entry_speed_limits,
        )

        self.status_pub = rospy.Publisher(
            self.status_topic, ExplorationStatus, queue_size=1, latch=True
        )
        self.frontier_pub = rospy.Publisher(
            self.frontier_topic, MarkerArray, queue_size=1, latch=True
        )
        self.target_pub = rospy.Publisher(
            self.target_topic, Marker, queue_size=1, latch=True
        )
        self.failed_pub = rospy.Publisher(
            self.failed_topic, Marker, queue_size=1, latch=True
        )
        self.trajectory_pub = rospy.Publisher(
            self.trajectory_topic, Path, queue_size=1, latch=True
        )
        self.scope_pub = rospy.Publisher(
            self.scope_topic, Marker, queue_size=1, latch=True
        )
        self.roi_pub = rospy.Publisher(
            self.roi_topic, PolygonStamped, queue_size=1, latch=True
        )
        self.corridor_model_pub = rospy.Publisher(
            self.corridor_model_topic, MarkerArray, queue_size=1, latch=True
        )
        self.recovery_cmd_pub = rospy.Publisher(
            self.recovery_cmd_topic, Twist, queue_size=1
        )

        rospy.Subscriber(
            self.map_topic, OccupancyGrid, self.map_callback, queue_size=1
        )
        rospy.Subscriber(
            self.doorways_topic,
            DoorwayArray,
            self.doorways_callback,
            queue_size=2,
        )
        rospy.Subscriber(
            self.walls_topic,
            WallSegmentArray,
            self.walls_callback,
            queue_size=2,
        )
        rospy.Subscriber(
            self.mapping_status_topic,
            DiagnosticStatus,
            self.mapping_status_callback,
            queue_size=2,
        )
        rospy.Subscriber(
            self.final_cmd_topic, Twist, self.final_command_callback,
            queue_size=20,
        )
        rospy.Subscriber(
            self.nav_cmd_topic, Twist, self.nav_command_callback,
            queue_size=20,
        )
        rospy.Subscriber(
            self.safety_lock_topic, Bool, self.safety_callback, queue_size=1
        )
        rospy.Subscriber(
            self.controller_ready_topic,
            Bool,
            self.controller_ready_callback,
            queue_size=5,
        )

        self.server = actionlib.SimpleActionServer(
            self.explore_action_name,
            ExploreFloorAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.server.start()
        self.status_timer = rospy.Timer(
            rospy.Duration(0.5), self.status_timer_callback
        )
        self.trajectory_timer = rospy.Timer(
            rospy.Duration(self.trajectory_period),
            self.trajectory_timer_callback,
        )
        self.publish_status()
        rospy.loginfo(
            "a1_exploration ready: map=%s, move_base=%s, action=%s",
            self.map_topic,
            self.move_action_name,
            self.explore_action_name,
        )

    @staticmethod
    def param(name, fallback):
        return rospy.get_param("~" + name, fallback)

    def configure_floor_completion(self):
        """Freeze the room target for one ExploreFloor transaction.

        The development launch stack may reload the explorer's private YAML
        after this long-lived node was constructed.  Reading the target here
        prevents the constructor fallback (zero/disabled) from leaking into a
        subsequently dispatched floor action.
        """
        target = int(self.param(
            "floor_completion/completed_room_count",
            self.floor_completion_room_count,
        ))
        if target < 0:
            raise ValueError(
                "floor_completion/completed_room_count must be non-negative"
            )
        self.floor_completion_target = target
        rospy.loginfo(
            "fixed-layout floor completion armed: target=%d distinct rooms",
            target,
        )

    def complete_room_branch(self, branch):
        """Atomically record a completed room and report floor completion."""
        was_new = branch not in self.completed_room_branches
        self.completed_room_branches.add(branch)
        completed_count = len(self.completed_room_branches)
        target = self.floor_completion_target
        rospy.loginfo(
            "fixed-layout room completion: %d/%d distinct rooms; "
            "station=%d side=%s new=%s",
            completed_count,
            target,
            branch[0],
            "left" if branch[1] > 0 else "right",
            was_new,
        )
        return target > 0 and completed_count >= target

    def validate_parameters(self):
        positive = {
            "frontier/min_length": self.min_frontier_length,
            "frontier/obstacle_clearance": self.obstacle_clearance,
            "frontier/max_goal_distance": self.max_goal_distance,
            "frontier/room_priority/scan_clearance":
                self.room_scan_clearance,
            "frontier/room_priority/scan_search_distance":
                self.room_scan_search_distance,
            "frontier/room_priority/minimum_door_longitudinal":
                self.room_minimum_door_longitudinal,
            "frontier/room_priority/door_minimum_width":
                self.room_door_minimum_width,
            "frontier/room_priority/door_maximum_width":
                self.room_door_maximum_width,
            "frontier/room_priority/door_station_tolerance":
                self.room_door_station_tolerance,
            "frontier/corridor_model/minimum_width":
                self.corridor_minimum_width,
            "frontier/corridor_model/maximum_width":
                self.corridor_maximum_width,
            "frontier/corridor_model/minimum_wall_length":
                self.corridor_minimum_wall_length,
            "frontier/corridor_model/lookahead":
                self.corridor_model_lookahead,
            "frontier/corridor_model/lookbehind":
                self.corridor_model_lookbehind,
            "frontier/corridor_model/max_age":
                self.corridor_model_max_age,
            "frontier/corridor_model/minimum_longitudinal":
                self.corridor_model_minimum_longitudinal,
            "frontier/corridor_model/center_tolerance":
                self.corridor_center_tolerance,
            "frontier/entry_exclusion/depth":
                self.entry_frontier_exclusion_depth,
            "frontier/entry_exclusion/half_width":
                self.entry_frontier_exclusion_half_width,
            "navigation/xy_goal_tolerance":
                self.navigation_xy_goal_tolerance,
            "frontier/stable_no_frontier_duration":
                self.stable_no_frontier_duration,
            "entry/position_tolerance": self.entry_position_tolerance,
            "entry/yaw_tolerance": self.entry_yaw_tolerance,
            "entry/inside_probe_distance":
                self.entry_inside_probe_distance,
            "entry/anchor_search_radius":
                self.entry_anchor_search_radius,
            "entry/corridor_half_width": self.entry_corridor_half_width,
            "entry/plan_endpoint_tolerance":
                self.entry_plan_endpoint_tolerance,
            "entry/plan_maximum_length_ratio":
                self.entry_plan_maximum_length_ratio,
            "timeouts/prerequisites": self.prerequisite_timeout,
            "timeouts/navigation_goal": self.navigation_timeout,
            "timeouts/entry_door_wall": self.entry_door_timeout,
            "timeouts/entry_transit": self.entry_transit_timeout,
            "timeouts/entry_map": self.entry_map_timeout,
            "timeouts/entry_plan": self.entry_plan_timeout,
            "timeouts/return_goal": self.return_timeout,
            "return/position_tolerance": self.return_position_tolerance,
            "return/yaw_tolerance": self.return_yaw_tolerance,
            "return/zero_settle_time": self.zero_settle,
            "return/command_freshness": self.command_freshness,
            "controller/ready_freshness": self.controller_ready_freshness,
            "planning/make_plan_retry_delay_wall":
                self.make_plan_retry_delay,
            "planning/make_plan_unavailable_timeout_wall":
                self.make_plan_unavailable_timeout,
        }
        if self.entry_speed_limit_enabled:
            positive.update({
                "entry/speed_limit/service_wait_wall":
                    self.entry_speed_service_wait,
                "entry/speed_limit/max_vel_x":
                    self.entry_speed_limits["max_vel_x"],
                "entry/speed_limit/max_vel_trans":
                    self.entry_speed_limits["max_vel_trans"],
                "entry/speed_limit/max_vel_theta":
                    self.entry_speed_limits["max_vel_theta"],
                "entry/speed_limit/min_vel_trans":
                    self.entry_speed_limits["min_vel_trans"],
            })
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("%s must be positive" % name)
        if self.corridor_maximum_width <= self.corridor_minimum_width:
            raise ValueError(
                "frontier/corridor_model maximum_width must exceed minimum_width"
            )
        if not 0.0 <= self.corridor_center_filter_gain <= 1.0:
            raise ValueError(
                "frontier/corridor_model/filter_gain must be in [0, 1]"
            )
        if (
            self.maximum_failures < 1
            or self.empty_confirmations < 1
            or self.minimum_free_cells < 1
            or self.entry_minimum_new_known_cells < 1
            or self.make_plan_retry_attempts < 1
            or self.return_attempts < 1
        ):
            raise ValueError("integer exploration limits must be positive")
        if not math.isfinite(self.minimum_frontier_score):
            raise ValueError("frontier/minimum_score must be finite")
        if self.entry_plan_maximum_length_ratio < 1.0:
            raise ValueError(
                "entry/plan_maximum_length_ratio must be at least 1.0"
            )
        if (
                not math.isfinite(self.entry_plan_maximum_length_slack)
                or self.entry_plan_maximum_length_slack < 0.0):
            raise ValueError(
                "entry/plan_maximum_length_slack must be finite and nonnegative"
            )
        if (
            not math.isfinite(self.entry_speed_limits["max_vel_y"])
            or self.entry_speed_limits["max_vel_y"] < 0.0):
            raise ValueError(
                "entry/speed_limit/max_vel_y must be finite and nonnegative"
            )
        if (
            not math.isfinite(self.entry_speed_limits["min_vel_theta"])
            or self.entry_speed_limits["min_vel_theta"] < 0.0):
            raise ValueError(
                "entry/speed_limit/min_vel_theta must be finite and "
                "nonnegative"
            )
        if self.roi_enabled:
            if (
                not math.isfinite(self.roi_entry_forward_offset)
                or not math.isfinite(self.roi_boundary_margin)
                or not math.isfinite(self.roi_map_boundary_margin)
                or self.roi_boundary_margin < 0.0
                or self.roi_map_boundary_margin < 0.0
            ):
                raise ValueError(
                    "ROI fallback and map/boundary margins are invalid"
                )
            if not (
                0.0 < self.entry_axis_maximum_correction < 0.5 * math.pi
                and self.entry_axis_minimum_wall_length > 0.0
                and self.entry_axis_radius > 0.0
                and self.entry_axis_minimum_weight > 0.0
                and self.entry_axis_minimum_walls >= 1
                and self.entry_axis_timeout >= 0.0
                and self.elevator_anchor_tolerance > 0.0
            ):
                raise ValueError(
                    "entry axis alignment / elevator anchor parameters are "
                    "invalid"
                )
            # Validation and zero-area rejection are shared with runtime goals.
            transform_local_polygon(self.default_roi_local, (0.0, 0.0), 0.0)
            transform_local_polygon(self.elevator_roi_local, (0.0, 0.0), 0.0)

    def map_callback(self, message):
        try:
            self.grid_spec(message)
            if len(message.data) != message.info.width * message.info.height:
                raise ValueError(
                    "occupancy data size does not match grid geometry"
                )
        except ValueError as error:
            with self.lock:
                self.map_message = None
                self.coverage = 0.0
            rospy.logerr_throttle(
                1.0, "rejected invalid floor map: %s", error
            )
            return
        coverage = coverage_ratio(message.data)
        with self.lock:
            roi_ready = bool(self.roi_polygon_map)
        if self.roi_enabled and roi_ready:
            try:
                allowed = self.build_roi_mask(message)
                coverage = coverage_ratio(message.data, allowed)
            except ValueError as error:
                coverage = 0.0
                rospy.logerr_throttle(
                    1.0, "invalid active exploration ROI: %s", error
                )
        with self.lock:
            self.map_message = message
            self.coverage = coverage

    def mapping_status_callback(self, message):
        with self.lock:
            self.mapping_status = message
            if self.mapping_usable(message):
                self.last_mapping_healthy_wall = time.monotonic()

    def doorways_callback(self, message):
        """Keep LiDAR-derived room doors even after they leave the local view."""
        with self.lock:
            self.doorway_message = message
            if (
                    self.floor_entry_pose is None
                    or not self.room_priority_enabled):
                return
            for doorway in message.doorways:
                longitudinal, lateral = self.entry_coordinates(
                    doorway.center.x, doorway.center.y
                )
                if (
                        not doorway.stable
                        or doorway.state in (doorway.CLOSED, doorway.BLOCKED)
                        or doorway.width < self.room_door_minimum_width
                        or doorway.width > self.room_door_maximum_width
                        or abs(lateral) > self.room_door_maximum_lateral
                        or longitudinal
                        < self.room_minimum_door_longitudinal):
                    continue
                branch = self.room_branch_key(longitudinal, lateral)
                self.remembered_room_doorways[branch] = copy.deepcopy(doorway)

    def walls_callback(self, message):
        """Retain only the latest wall snapshot; modelling uses the live pose."""
        with self.lock:
            self.wall_message = message

    def final_command_callback(self, message):
        stamp = rospy.Time.now().to_sec()
        with self.lock:
            self.final_command = message
            # All six components, matching the single-floor tree: a residual
            # linear.z or angular.x/y is still a non-zero command, and the
            # three-component check could not see it.
            values = (
                message.linear.x,
                message.linear.y,
                message.linear.z,
                message.angular.x,
                message.angular.y,
                message.angular.z,
            )
            self.final_zero_monitor.observe(stamp, values)
            self.interstitial_cmd_vel_monitor.observe(stamp, values)

    def nav_command_callback(self, message):
        # /cmd_vel_nav is move_base's raw local-planner output. It is only used
        # by the interstitial gate (see wait_for_interstitial_zero_settle); it
        # legitimately stops publishing once a goal is cancelled/aborted, which
        # is why cmd_vel_nav_satisfied() tolerates sustained silence on it.
        stamp = rospy.Time.now().to_sec()
        with self.lock:
            self.interstitial_cmd_vel_nav_monitor.observe(
                stamp,
                (
                    message.linear.x,
                    message.linear.y,
                    message.linear.z,
                    message.angular.x,
                    message.angular.y,
                    message.angular.z,
                ),
            )

    def safety_callback(self, message):
        with self.lock:
            self.safety_locked = bool(message.data)
            active = self.action_active
        if message.data and active:
            self.move_client.cancel_goal()
            rospy.logerr("exploration cancelled by a1_cmd_mux safety lock")

    def controller_ready_callback(self, message):
        stamp = rospy.Time.now()
        with self.lock:
            was_ready = self.controller_ready
            self.controller_ready = bool(message.data)
            self.controller_ready_stamp = stamp
            active = self.action_active
        if not message.data and active and was_ready:
            self.move_client.cancel_goal()
            rospy.logerr("exploration stopped because controller_ready is false")

    def controller_is_ready(self):
        now = rospy.Time.now()
        with self.lock:
            ready = self.controller_ready
            stamp = self.controller_ready_stamp
        if not ready or stamp is None or now < stamp:
            return False
        return (now - stamp).to_sec() <= self.controller_ready_freshness

    @staticmethod
    def mapping_usable(message):
        if message is None:
            return False
        values = diagnostic_values(message)
        return (
            message.message == "MAPPING"
            and values.get("state", "MAPPING") == "MAPPING"
            and values.get("map_valid") == "true"
            and values.get("obstacle_cloud_valid") == "true"
        )

    @staticmethod
    def mapping_identity(message, map_message):
        if message is None or map_message is None:
            return None
        values = diagnostic_values(message)
        return (
            map_message.header.frame_id,
            values.get("localization_generation", "unknown"),
            values.get("floor_session_id", "unknown"),
            values.get("floor_id", "unassigned"),
        )

    @classmethod
    def map_version(cls, message):
        return occupancy_content_fingerprint(
            message.data, cls.grid_spec(message)
        )

    @staticmethod
    def grid_spec(message):
        values = (
            message.info.resolution,
            message.info.origin.position.x,
            message.info.origin.position.y,
            message.info.origin.orientation.x,
            message.info.origin.orientation.y,
            message.info.origin.orientation.z,
            message.info.origin.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("occupancy grid geometry is not finite")
        orientation = message.info.origin.orientation
        norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if (
            message.info.width <= 0
            or message.info.height <= 0
            or message.info.resolution <= 0.0
            or norm < 1e-9
        ):
            raise ValueError("occupancy grid geometry is invalid")
        if (
            abs(orientation.x / norm) > 1e-6
            or abs(orientation.y / norm) > 1e-6
            or abs(orientation.z / norm) > 1e-6
        ):
            raise ValueError(
                "rotated OccupancyGrid origins are not supported"
            )
        return GridSpec(
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
        )

    def build_roi_mask(self, map_message):
        if not self.roi_enabled:
            return None
        if not self.roi_polygon_map:
            raise ValueError("ROI has not been established")
        if (
            self.floor_entry_pose is None
            or not map_message.header.frame_id
            or self.floor_entry_pose.header.frame_id
            != map_message.header.frame_id
        ):
            raise ValueError(
                "ROI entry frame does not match the current floor map"
            )
        spec = self.grid_spec(map_message)
        allowed = polygon_mask(
            spec,
            self.roi_polygon_map,
            self.roi_boundary_margin,
        )
        if not allowed.any():
            raise ValueError("ROI does not overlap the current floor map")
        outside = [
            (x, y)
            for x, y in self.roi_polygon_map
            if not self.within_map_margin(spec, x, y)
        ]
        if outside:
            # The ROI polygon is a generous rectangle sized to be certain it
            # covers the floor; its far corners deliberately sit past the far
            # wall, in space no sensor will ever reach. Refusing the whole
            # transaction because such a corner pokes out of the grid throws
            # away an entire 6-8 minute run over cells that could not have
            # mattered (mf08 died exactly this way, 0.15 s into F1). Clip to
            # the sensor margin instead: the "no ROI cell within
            # map_boundary_margin of the grid edge" invariant is preserved
            # exactly, and the retained region is still every cell the grid
            # can represent. Loud, because a large clip means the grid is
            # genuinely mis-sized for this floor frame.
            clipped = allowed & self.cached_map_margin_mask(spec)
            retained = int(clipped.sum())
            requested = int(allowed.sum())
            if retained <= 0:
                raise ValueError(
                    "ROI lies entirely within the %.2f m sensor margin of the "
                    "OccupancyGrid edge; map=[%.2f, %.2f]x[%.2f, %.2f], "
                    "outside_vertices=%r"
                    % (
                        self.roi_map_boundary_margin,
                        spec.origin_x,
                        spec.origin_x + spec.width * spec.resolution,
                        spec.origin_y,
                        spec.origin_y + spec.height * spec.resolution,
                        outside,
                    )
                )
            # map_callback rebuilds this mask on every published grid, so
            # throttle: the first report is what matters, not 1 Hz of it.
            rospy.logwarn_throttle(
                10.0,
                "ROI clipped to the %.2f m OccupancyGrid sensor margin: "
                "%d of %d cells retained (%.1f%% dropped); "
                "map=[%.2f, %.2f]x[%.2f, %.2f], outside_vertices=%r",
                self.roi_map_boundary_margin,
                retained,
                requested,
                100.0 * (requested - retained) / max(requested, 1),
                spec.origin_x,
                spec.origin_x + spec.width * spec.resolution,
                spec.origin_y,
                spec.origin_y + spec.height * spec.resolution,
                outside,
            )
            return clipped
        return allowed

    def cached_map_margin_mask(self, spec):
        """The margin mask for one grid geometry, built at most once."""
        cached = self.map_margin_cache
        if cached is not None and cached[0] == spec:
            return cached[1]
        mask = map_margin_mask(spec, self.roi_map_boundary_margin)
        self.map_margin_cache = (spec, mask)
        return mask

    def within_map_margin(self, spec, x, y):
        margin = self.roi_map_boundary_margin
        return (
            spec.origin_x + margin <= x
            <= spec.origin_x + spec.width * spec.resolution - margin
            and spec.origin_y + margin <= y
            <= spec.origin_y + spec.height * spec.resolution - margin
        )

    def target_in_roi(self, target):
        if not self.roi_enabled:
            return True
        if self.floor_entry_pose is None or not self.roi_polygon_map:
            return False
        if target.header.frame_id != self.floor_entry_pose.header.frame_id:
            return False
        return point_in_polygon(
            target.pose.position.x,
            target.pose.position.y,
            self.roi_polygon_map,
            self.roi_boundary_margin,
        )

    def establish_roi(self, goal, map_frame):
        if not self.roi_enabled:
            self.floor_entry_pose = None
            self.roi_local = ()
            self.roi_polygon_map = ()
            return

        if goal.floor_entry_pose.header.frame_id:
            validate_floor_entry_pose(goal.floor_entry_pose, map_frame)
            entry = copy.deepcopy(goal.floor_entry_pose)
            entry.header.stamp = rospy.Time.now()
        else:
            if self.require_explicit_entry_pose:
                raise InvalidEntryPose(
                    "floor_entry_pose is required; RECORD_START is only the "
                    "outdoor return pose"
                )
            entry = copy.deepcopy(self.start_pose)
            yaw = yaw_from_quaternion(entry.pose.orientation)
            entry.pose.position.x += (
                math.cos(yaw) * self.roi_entry_forward_offset
            )
            entry.pose.position.y += (
                math.sin(yaw) * self.roi_entry_forward_offset
            )
            entry.header.stamp = rospy.Time.now()
            rospy.logwarn(
                "ExploreFloor omitted floor_entry_pose; using the configured "
                "RECORD_START-relative compatibility fallback"
            )

        local_points = tuple(
            (point.x, point.y) for point in goal.roi_local.points
        )
        if not local_points:
            local_points = (
                self.elevator_roi_local
                if goal.entry_mode == goal.ALREADY_AT_FLOOR_ENTRY
                else self.default_roi_local
            )
        yaw = yaw_from_quaternion(entry.pose.orientation)
        world_points = transform_local_polygon(
            local_points,
            (entry.pose.position.x, entry.pose.position.y),
            yaw,
        )
        self.floor_entry_pose = entry
        self.roi_local = local_points
        self.roi_polygon_map = world_points

    def rebuild_roi_from_entry_pose(self):
        """Keep the exploration ROI rigidly attached to a corrected entry axis."""
        yaw = yaw_from_quaternion(self.floor_entry_pose.pose.orientation)
        self.roi_polygon_map = transform_local_polygon(
            self.roi_local,
            (
                self.floor_entry_pose.pose.position.x,
                self.floor_entry_pose.pose.position.y,
            ),
            yaw,
        )

    def align_entry_axis_to_walls(self, map_frame):
        """Snap an elevator-delivered entry axis onto the measured corridor.

        The floor-0 entry axis is exact by construction: the mission derives it
        from the spawn pose, which the scene places square with the entrance.
        An upper-floor anchor instead arrives through the elevator, and every
        link in that chain leaks yaw -- the car pose at relocalization, the
        opening alignment (0.20 rad settle tolerance), and the fixed 95 degree
        turn, which DWA is entitled to call reached at yaw_goal_tolerance 0.80
        rad. Measured on mf08/seed 382835531, the declared F1 axis sat 17 deg
        off the real corridor.

        That is not cosmetic. Every room decision is taken in entry-frame
        coordinates, so at 17 deg a door 20 m down the corridor picks up 6.8 m
        of apparent lateral offset and is discarded by room_priority's 2.2 m
        door band; the ROI rectangle likewise shears off the far corner of the
        floor. Walls are the only floor-fixed direction available without
        simulator truth, so measure the axis from them.

        Fails open: an unconvincing wall set leaves the declared axis alone.
        """
        if not self.entry_axis_alignment_enabled:
            return
        deadline = time.monotonic() + self.entry_axis_timeout
        declared_yaw = yaw_from_quaternion(
            self.floor_entry_pose.pose.orientation
        )
        result = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.check_cancel_safety_and_deadline(check_controller=False)
            result = self.entry_axis_correction(map_frame, declared_yaw)
            if result is not None:
                break
            time.sleep(0.20)
        if result is None:
            rospy.logwarn(
                "upper-floor entry axis kept as declared: no wall family "
                "within %.2f rad carried %.1f m of evidence in %.0f s",
                self.entry_axis_maximum_correction,
                self.entry_axis_minimum_weight,
                self.entry_axis_timeout,
            )
            return
        correction, weight, walls = result
        corrected = normalize_angle(declared_yaw + correction)
        (self.floor_entry_pose.pose.orientation.x,
         self.floor_entry_pose.pose.orientation.y,
         self.floor_entry_pose.pose.orientation.z,
         self.floor_entry_pose.pose.orientation.w) = quaternion_from_yaw(
            corrected)
        self.rebuild_roi_from_entry_pose()
        rospy.loginfo(
            "upper-floor entry axis aligned to measured walls: "
            "declared=%.1f deg corrected=%.1f deg (%.1f deg), "
            "%d walls, %.1f m of wall evidence",
            math.degrees(declared_yaw),
            math.degrees(corrected),
            math.degrees(correction),
            walls,
            weight,
        )

    def entry_axis_correction(self, map_frame, declared_yaw):
        """One attempt at a wall-measured correction, or None if unconvincing."""
        with self.lock:
            message = copy.deepcopy(self.wall_message)
        if message is None or message.header.frame_id != map_frame:
            return None
        anchor = self.floor_entry_pose.pose.position
        segments = []
        for wall in message.walls:
            if (
                    not wall.stable
                    or wall.status != "observed"
                    or wall.length < self.entry_axis_minimum_wall_length):
                continue
            midpoint_x = 0.5 * (wall.start.x + wall.end.x)
            midpoint_y = 0.5 * (wall.start.y + wall.end.y)
            if math.hypot(
                    midpoint_x - anchor.x,
                    midpoint_y - anchor.y) > self.entry_axis_radius:
                continue
            wall_yaw = math.atan2(
                wall.end.y - wall.start.y, wall.end.x - wall.start.x
            )
            # Length is the evidence; confidence only discounts it.
            segments.append(
                (wall_yaw, float(wall.length) * max(0.05, float(wall.confidence)))
            )
        if len(segments) < self.entry_axis_minimum_walls:
            return None
        result = dominant_axis_correction(
            declared_yaw, segments, self.entry_axis_maximum_correction
        )
        if result is None:
            return None
        _correction, weight, walls = result
        if (
                walls < self.entry_axis_minimum_walls
                or weight < self.entry_axis_minimum_weight):
            return None
        return result

    def entrance_door_candidate(self, doorway_message, goal):
        """Select the scanned public entrance without using simulator truth."""
        if (
                doorway_message is None
                or doorway_message.header.frame_id
                != self.floor_entry_pose.header.frame_id):
            return None
        start = self.start_pose.pose.position
        target = self.floor_entry_pose.pose.position
        intended_yaw = yaw_from_quaternion(
            self.floor_entry_pose.pose.orientation
        )
        forward_x = math.cos(intended_yaw)
        forward_y = math.sin(intended_yaw)
        intended_distance = max(
            0.0,
            (target.x - start.x) * forward_x
            + (target.y - start.y) * forward_y,
        )
        best = None
        best_score = float("inf")
        for doorway in doorway_message.doorways:
            dx = doorway.center.x - start.x
            dy = doorway.center.y - start.y
            longitudinal = dx * forward_x + dy * forward_y
            lateral = -dx * forward_y + dy * forward_x
            if (
                    not doorway.stable
                    or doorway.width < self.entry_centerline_min_width
                    or doorway.width > self.entry_centerline_max_width
                    or longitudinal < self.entry_centerline_min_forward
                    or longitudinal > intended_distance
                    + self.entry_centerline_target_margin
                    or abs(lateral) > self.entry_centerline_max_lateral):
                continue
            control_match = bool(
                doorway.control_id_matched
                and (
                    not goal.entry_door_id
                    or doorway.control_door_id == goal.entry_door_id
                )
            )
            # A control-ID match dominates; geometry then favors the entrance
            # nearest the declared transit axis and its expected depth.
            score = (
                (0.0 if control_match else 100.0)
                + abs(lateral) * 10.0
                + abs(longitudinal - 0.5 * intended_distance)
                - min(float(doorway.observation_count), 20.0) * 0.02
                - float(doorway.confidence) * 0.10
            )
            if score < best_score:
                best = doorway
                best_score = score
        return best

    def align_entry_to_scanned_door(self, goal):
        """Recenter entry pose and ROI on stable LiDAR doorway geometry."""
        if not self.entry_centerline_enabled:
            return
        deadline = time.monotonic() + self.entry_centerline_timeout
        doorway = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.check_cancel_safety_and_deadline(check_controller=False)
            with self.lock:
                message = copy.deepcopy(self.doorway_message)
            doorway = self.entrance_door_candidate(message, goal)
            if doorway is not None:
                break
            time.sleep(0.05)
        if doorway is None:
            detail = (
                "no stable LiDAR doorway matched the public entrance axis "
                "within %.1f wall seconds" % self.entry_centerline_timeout
            )
            if self.entry_centerline_required:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_ENTRY_MAP, detail
                )
            rospy.logwarn("%s; retaining declared entry pose", detail)
            return

        old_x = self.floor_entry_pose.pose.position.x
        old_y = self.floor_entry_pose.pose.position.y
        old_yaw = yaw_from_quaternion(
            self.floor_entry_pose.pose.orientation
        )
        forward_x = math.cos(old_yaw)
        forward_y = math.sin(old_yaw)
        normal_norm = math.hypot(doorway.normal.x, doorway.normal.y)
        if normal_norm > 0.50:
            normal_x = doorway.normal.x / normal_norm
            normal_y = doorway.normal.y / normal_norm
            alignment = normal_x * forward_x + normal_y * forward_y
            if abs(alignment) >= 0.60:
                if alignment < 0.0:
                    normal_x = -normal_x
                    normal_y = -normal_y
                forward_x, forward_y = normal_x, normal_y
        inside_depth = (
            (old_x - doorway.center.x) * forward_x
            + (old_y - doorway.center.y) * forward_y
        )
        inside_depth = min(
            self.entry_centerline_max_inside_depth,
            max(self.entry_centerline_min_inside_depth, inside_depth),
        )
        new_x = doorway.center.x + forward_x * inside_depth
        new_y = doorway.center.y + forward_y * inside_depth
        new_yaw = math.atan2(forward_y, forward_x)
        lateral_correction = (
            -(new_x - old_x) * math.sin(old_yaw)
            + (new_y - old_y) * math.cos(old_yaw)
        )
        self.floor_entry_pose.pose.position.x = new_x
        self.floor_entry_pose.pose.position.y = new_y
        self.floor_entry_pose.pose.orientation.x, \
            self.floor_entry_pose.pose.orientation.y, \
            self.floor_entry_pose.pose.orientation.z, \
            self.floor_entry_pose.pose.orientation.w = \
            quaternion_from_yaw(new_yaw)
        self.floor_entry_pose.header.stamp = rospy.Time.now()
        self.rebuild_roi_from_entry_pose()
        rospy.loginfo(
            "LiDAR entrance centerline locked: door_center=(%.2f, %.2f) "
            "width=%.2f confidence=%.2f observations=%d control_match=%s; "
            "entry_target (%.2f, %.2f)->(%.2f, %.2f), "
            "lateral_correction=%.2f m yaw=%.2f rad inside_depth=%.2f m",
            doorway.center.x, doorway.center.y, doorway.width,
            doorway.confidence, doorway.observation_count,
            doorway.control_id_matched, old_x, old_y, new_x, new_y,
            lateral_correction, new_yaw, inside_depth,
        )

    def pose_in_frame(self, frame):
        transform = self.tf_buffer.lookup_transform(
            frame, self.base_frame, rospy.Time(0), rospy.Duration(0.5)
        )
        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def entry_probe_xy(self):
        yaw = yaw_from_quaternion(self.floor_entry_pose.pose.orientation)
        return (
            self.floor_entry_pose.pose.position.x
            + math.cos(yaw) * self.entry_inside_probe_distance,
            self.floor_entry_pose.pose.position.y
            + math.sin(yaw) * self.entry_inside_probe_distance,
        )

    @staticmethod
    def pose_errors(current, target):
        position_error = math.hypot(
            current.pose.position.x - target.pose.position.x,
            current.pose.position.y - target.pose.position.y,
        )
        yaw_error = abs(
            angle_difference(
                yaw_from_quaternion(current.pose.orientation),
                yaw_from_quaternion(target.pose.orientation),
            )
        )
        return position_error, yaw_error

    def refine_scan_heading(self, frame, target_yaw):
        """Hold a localization-confirmed heading with enough elevator view.

        move_base may report an orientation-only goal inside its comparatively
        broad yaw tolerance.  The dedicated L-shaped template remains stable
        with roughly 76--80 degrees of side rotation, so this loop enforces a
        repeatable *view* rather than requiring a strict 90-degree pose.
        """
        view_tolerance = float(self.param(
            "entry/elevator_scan/view_yaw_tolerance", 0.25))
        timeout = float(self.param("entry/elevator_scan/yaw_timeout_wall", 30.0))
        maximum = float(self.param("entry/elevator_scan/max_angular_speed", 0.45))
        minimum = float(self.param("entry/elevator_scan/min_angular_speed", 0.12))
        deadline = time.monotonic() + timeout
        stable_since = None
        zero = Twist()
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                self.check_cancel_safety_and_deadline()
                pose = self.pose_in_frame(frame)
                current_yaw = yaw_from_quaternion(pose.pose.orientation)
                error = normalize_angle(target_yaw - current_yaw)
                rospy.loginfo_throttle(
                    0.5,
                    "ELEVATOR_SCAN_HEADING target=%.3f current=%.3f "
                    "error=%.3f tolerance=%.3f",
                    target_yaw, current_yaw, error, view_tolerance,
                )
                if abs(error) <= view_tolerance:
                    if stable_since is None:
                        stable_since = time.monotonic()
                    self.recovery_cmd_pub.publish(zero)
                    if time.monotonic() - stable_since >= 0.5:
                        return
                else:
                    stable_since = None
                    speed = max(minimum, min(maximum, 1.2 * abs(error)))
                    command = Twist()
                    command.angular.z = math.copysign(speed, error)
                    self.recovery_cmd_pub.publish(command)
                time.sleep(0.02)
        finally:
            for _unused in range(10):
                self.recovery_cmd_pub.publish(zero)
                time.sleep(0.02)
        raise ExplorationFailure(
            ExploreFloorResult.ERROR_ENTRY_TRANSIT,
            "elevator side-scan heading did not converge to %.2f rad within "
            "%.1f wall seconds" % (target_yaw, timeout),
        )

    def request_entry_door_open(self, goal):
        deadline = time.monotonic() + self.entry_door_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.check_cancel_safety_and_deadline(check_controller=False)
            if self.building_behavior_client.wait_for_server(
                rospy.Duration(0.05)
            ):
                break
        else:
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_ENTRY_DOOR,
                "entry door behavior action is unavailable",
            )

        behavior = SpecialBehaviorGoal()
        behavior.behavior_type = SpecialBehaviorGoal.OPEN_DOOR
        behavior.target_id = goal.entry_door_id
        behavior.target_floor_id = goal.floor_id
        behavior.timeout_s = self.entry_door_timeout
        self.building_behavior_client.send_goal(behavior)
        while not rospy.is_shutdown():
            try:
                self.check_cancel_safety_and_deadline(
                    check_controller=False
                )
            except ExplorationFailure:
                self.building_behavior_client.cancel_goal()
                raise
            state = self.building_behavior_client.get_state()
            if state in (
                GoalStatus.SUCCEEDED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.PREEMPTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST,
            ):
                result = self.building_behavior_client.get_result()
                if (
                    state == GoalStatus.SUCCEEDED
                    and result is not None
                    and result.success
                ):
                    rospy.loginfo("entry door response accepted: %s", result.message)
                    return
                detail = (
                    result.message if result is not None
                    else "no SpecialBehavior result"
                )
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_ENTRY_DOOR,
                    "entry door open failed: action_state=%d %s"
                    % (state, detail),
                )
            if time.monotonic() >= deadline:
                self.building_behavior_client.cancel_goal()
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_ENTRY_DOOR,
                    "entry door behavior exceeded %.1f wall seconds"
                    % self.entry_door_timeout,
                )
            time.sleep(0.05)

    def wait_for_entry_passage(self, baseline_message, outside_pose):
        """Wait for a fresh post-door local map, without demanding unseen free space.

        Requiring a complete known-free path to an indoor pose deadlocks the
        mapper outside the doorway: the robot must advance before Livox can
        observe the far side. Entry motion therefore uses the continuously
        refreshed near-field obstacle gate below.
        """
        baseline_version = self.map_version(baseline_message)
        start_xy = (
            outside_pose.pose.position.x,
            outside_pose.pose.position.y,
        )
        started_ros = rospy.Time.now()
        started_wall = time.monotonic()
        while not rospy.is_shutdown():
            self.check_cancel_safety_and_deadline(check_controller=False)
            now_ros = rospy.Time.now()
            if now_ros < started_ros:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_ENTRY_MAP,
                    "ROS/simulation clock moved backwards while verifying door",
                )
            if (
                (now_ros - started_ros).to_sec() >= self.entry_map_timeout
                or time.monotonic() - started_wall
                >= self.entry_map_timeout * self.wall_factor
            ):
                break
            with self.lock:
                message = copy.deepcopy(self.map_message)
            spec = self.grid_spec(message) if message is not None else None
            passage_anchor = (
                nearest_known_free_anchor(
                    message.data,
                    spec,
                    start_xy,
                    self.entry_anchor_search_radius,
                    self.free_threshold,
                    None,
                )
                if spec is not None else None
            )
            if (
                message is not None
                and message.header.frame_id
                == self.floor_entry_pose.header.frame_id
                and self.map_version(message) != baseline_version
                and passage_anchor is not None
            ):
                rospy.loginfo(
                    "fresh post-door OccupancyGrid accepted for short-horizon "
                    "entry; unknown space remains fail-safe behind a moving "
                    "explicit-obstacle gate, "
                    "outside_anchor_offset=%.3f m",
                    math.hypot(
                        passage_anchor[0] - start_xy[0],
                        passage_anchor[1] - start_xy[1],
                    ),
                )
                return message
            time.sleep(0.05)
        raise ExplorationFailure(
            ExploreFloorResult.ERROR_ENTRY_MAP,
            "door service succeeded but no fresh post-door OccupancyGrid "
            "with a local free anchor appeared",
        )

    def entry_near_field_clear(self, pose, map_message):
        """Reject only explicit occupied cells in the next short body corridor."""
        if map_message is None:
            return False
        spec = self.grid_spec(map_message)
        yaw = yaw_from_quaternion(pose.pose.orientation)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        step = max(0.04, spec.resolution)
        longitudinal = 0.18
        while longitudinal <= self.entry_near_field_distance + 1e-6:
            lateral = -self.entry_near_field_half_width
            while lateral <= self.entry_near_field_half_width + 1e-6:
                x = (pose.pose.position.x + longitudinal * cos_yaw
                     - lateral * sin_yaw)
                y = (pose.pose.position.y + longitudinal * sin_yaw
                     + lateral * cos_yaw)
                cell_x = int(math.floor(
                    (x - spec.origin_x) / spec.resolution
                ))
                cell_y = int(math.floor(
                    (y - spec.origin_y) / spec.resolution
                ))
                if (
                        0 <= cell_x < spec.width
                        and 0 <= cell_y < spec.height):
                    value = map_message.data[cell_y * spec.width + cell_x]
                    if value >= self.occupied_threshold:
                        return False
                lateral += step
            longitudinal += step
        return True

    def controlled_entry_transit(self):
        """Cross the apron and doorway with State_RL at 0.8 m/s.

        This is deliberately not a global-planner problem. The explorer emits
        only a body velocity; State_RL remains the sole joint-level controller.
        Localization closes heading/position error and the live floor map gates
        the next 0.7 m for explicit obstacles.
        """
        frame = self.floor_entry_pose.header.frame_id
        target_x = self.floor_entry_pose.pose.position.x
        target_y = self.floor_entry_pose.pose.position.y
        started_ros = rospy.Time.now()
        started_wall = time.monotonic()
        blocked_since = None
        blocked_since_ros = None
        zero = Twist()
        try:
            while not rospy.is_shutdown():
                self.check_cancel_safety_and_deadline()
                pose = self.pose_in_frame(frame)
                dx = target_x - pose.pose.position.x
                dy = target_y - pose.pose.position.y
                distance = math.hypot(dx, dy)
                if distance <= self.entry_position_tolerance:
                    rospy.loginfo(
                        "controlled entry reached indoor anchor: error=%.3f m",
                        distance,
                    )
                    return
                ros_elapsed = (rospy.Time.now() - started_ros).to_sec()
                wall_elapsed = time.monotonic() - started_wall
                if (
                        ros_elapsed >= self.entry_transit_timeout
                        or wall_elapsed
                        >= self.entry_transit_timeout * self.wall_factor):
                    raise ExplorationFailure(
                        ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                        "controlled entry timed out: remaining %.2f m"
                        % distance,
                    )
                with self.lock:
                    map_message = copy.deepcopy(self.map_message)
                if not self.entry_near_field_clear(pose, map_message):
                    self.recovery_cmd_pub.publish(zero)
                    if blocked_since is None:
                        blocked_since = time.monotonic()
                        blocked_since_ros = rospy.Time.now()
                        rospy.logwarn(
                            "entry near-field explicit obstacle; holding for "
                            "a fresh map"
                        )
                    elif (
                            (
                                rospy.Time.now() - blocked_since_ros
                            ).to_sec() >= self.entry_obstacle_hold_timeout
                            or time.monotonic() - blocked_since
                            >= (
                                self.entry_obstacle_hold_timeout
                                * self.wall_factor
                            )):
                        raise ExplorationFailure(
                            ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                            "entry remained explicitly occupied for %.1f sim "
                            "s (wall fallback %.1f s)"
                            % (
                                self.entry_obstacle_hold_timeout,
                                self.entry_obstacle_hold_timeout
                                * self.wall_factor,
                            ),
                        )
                    time.sleep(0.02)
                    continue
                blocked_since = None
                blocked_since_ros = None
                current_yaw = yaw_from_quaternion(pose.pose.orientation)
                desired_yaw = math.atan2(dy, dx)
                heading_error = normalize_angle(desired_yaw - current_yaw)
                if abs(heading_error) > 0.55:
                    raise ExplorationFailure(
                        ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                        "entry heading diverged by %.2f rad; refusing a turn "
                        "on the step" % heading_error,
                    )
                command = Twist()
                command.linear.x = min(
                    self.entry_transit_speed,
                    max(0.35, distance * 0.9),
                )
                command.angular.z = max(
                    -0.40, min(0.40, 1.4 * heading_error)
                )
                self.recovery_cmd_pub.publish(command)
                time.sleep(0.02)
        finally:
            for _unused in range(15):
                self.recovery_cmd_pub.publish(zero)
                time.sleep(0.02)

    def reset_map_after_entry_door_opens(self):
        """Discard occupancy evidence collected while the public door was shut."""
        with self.lock:
            identity_before = self.action_identity
        try:
            rospy.wait_for_service("/a1/floor_mapping/reset", timeout=3.0)
            response = self.floor_mapping_reset()
            if not response.success:
                raise RuntimeError(response.message)
            rospy.loginfo(
                "floor map reset after entry door opened: %s",
                response.message,
            )
        except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_ENTRY_MAP,
                "entry door opened but floor map reset failed: %s" % error,
            )
        # A commanded reset intentionally advances the mapper generation.
        # Adopt exactly that next healthy identity here; identity changes at
        # every other point in the action remain fatal.
        ros_deadline = (
            rospy.Time.now() + rospy.Duration(self.entry_map_timeout)
        )
        wall_deadline = time.monotonic() + min(
            60.0,
            max(8.0, self.entry_map_timeout * self.wall_factor),
        )
        while (
                not rospy.is_shutdown()
                and rospy.Time.now() < ros_deadline
                and time.monotonic() < wall_deadline):
            with self.lock:
                identity_after = self.mapping_identity(
                    self.mapping_status, self.map_message
                )
                usable = self.mapping_usable(self.mapping_status)
            if usable and identity_after != identity_before:
                with self.lock:
                    self.action_identity = identity_after
                rospy.loginfo(
                    "adopted intentional post-door map identity: %r -> %r",
                    identity_before,
                    identity_after,
                )
                return
            time.sleep(0.05)
        raise ExplorationFailure(
            ExploreFloorResult.ERROR_ENTRY_MAP,
            "floor mapping reset did not publish a new healthy generation",
        )

    def apply_entry_speed_limit(self):
        """Apply the live, reversible DWA entry profile before any motion."""
        if not self.entry_speed_limit_enabled:
            return
        self.check_cancel_safety_and_deadline()
        try:
            rospy.wait_for_service(
                self.dwa_reconfigure_name,
                timeout=self.entry_speed_service_wait,
            )
        except rospy.ROSException as error:
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_NAVIGATION_UNAVAILABLE,
                "entry speed limiter service %s is unavailable: %s"
                % (self.dwa_reconfigure_name, error),
            )
        self.check_cancel_safety_and_deadline()
        try:
            self.entry_speed_limiter.apply()
        except EntrySpeedLimitError as error:
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                "entry speed limiter failed closed before MoveBaseAction: %s"
                % error,
            )
        limits = self.entry_speed_limiter.limits
        rospy.loginfo(
            "entry State_RL speed profile active: vx=[%.3f, %.3f] vy=%.3f "
            "trans=[%.3f, %.3f] theta=[%.3f, %.3f] sim_time=%.2f",
            limits["min_vel_x"],
            limits["max_vel_x"],
            limits["max_vel_y"],
            limits["min_vel_trans"],
            limits["max_vel_trans"],
            limits["min_vel_theta"],
            limits["max_vel_theta"],
            limits["sim_time"],
        )

    def restore_entry_speed_limit(self):
        """Restore the exact live DWA snapshot before frontier selection."""
        if (
                not self.entry_speed_limit_enabled
                or not self.entry_speed_limiter.active):
            return
        try:
            self.entry_speed_limiter.restore()
        except EntrySpeedLimitError as error:
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                "entry DWA speed profile restore failed closed: %s" % error,
            )
        rospy.loginfo(
            "entry DWA speed profile restored before leaving TRANSIT_TO_ENTRY"
        )

    def wait_for_local_entry_plan(self):
        """Require a finite near-direct entry plan before sending MoveBaseAction."""
        started_ros = rospy.Time.now()
        started_wall = time.monotonic()
        last_reason = "no make_plan response"
        while not rospy.is_shutdown():
            self.check_cancel_safety_and_deadline()
            now_ros = rospy.Time.now()
            if now_ros < started_ros:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                    "ROS/simulation clock moved backwards while validating "
                    "entry plan",
                )
            if (
                (now_ros - started_ros).to_sec() >= self.entry_plan_timeout
                or time.monotonic() - started_wall
                >= self.entry_plan_timeout * self.wall_factor
            ):
                break

            start = self.pose_in_frame(
                self.floor_entry_pose.header.frame_id
            )
            target = copy.deepcopy(self.floor_entry_pose)
            target.header.stamp = now_ros
            try:
                response = self.make_plan(
                    start=start, goal=target, tolerance=0.20
                )
                self.make_plan_failure_since_wall = None
            except rospy.ServiceException as error:
                if self.make_plan_failure_since_wall is None:
                    self.make_plan_failure_since_wall = time.monotonic()
                last_reason = "make_plan service error: %s" % error
                if (
                    time.monotonic() - self.make_plan_failure_since_wall
                    >= self.make_plan_unavailable_timeout
                ):
                    raise ExplorationFailure(
                        ExploreFloorResult.ERROR_NAVIGATION_UNAVAILABLE,
                        last_reason,
                    )
                time.sleep(self.make_plan_retry_delay)
                continue

            plan = response.plan
            expected_frame = self.floor_entry_pose.header.frame_id
            if (
                    plan.header.frame_id
                    and plan.header.frame_id != expected_frame):
                last_reason = (
                    "plan frame %r does not match entry frame %r"
                    % (plan.header.frame_id, expected_frame)
                )
            elif any(
                    pose.header.frame_id != expected_frame
                    for pose in plan.poses):
                last_reason = (
                    "plan header may be empty, but every pose must explicitly "
                    "use entry frame %r" % expected_frame
                )
            else:
                points = [
                    (pose.pose.position.x, pose.pose.position.y)
                    for pose in plan.poses
                ]
                start_xy = (
                    start.pose.position.x,
                    start.pose.position.y,
                )
                goal_xy = (
                    target.pose.position.x,
                    target.pose.position.y,
                )
                if local_plan_is_acceptable(
                    points,
                    start_xy,
                    goal_xy,
                    self.entry_corridor_half_width,
                    self.entry_plan_endpoint_tolerance,
                    self.entry_plan_maximum_length_ratio,
                    self.entry_plan_maximum_length_slack,
                ):
                    path_length = sum(
                        math.hypot(
                            points[index][0] - points[index - 1][0],
                            points[index][1] - points[index - 1][1],
                        )
                        for index in range(1, len(points))
                    )
                    rospy.loginfo(
                        "entry make_plan accepted before motion: "
                        "poses=%d length=%.3f m direct=%.3f m",
                        len(points),
                        path_length,
                        math.hypot(
                            goal_xy[0] - start_xy[0],
                            goal_xy[1] - start_xy[1],
                        ),
                    )
                    return
                last_reason = (
                    "plan leaves %.2f m entry corridor, has invalid endpoints, "
                    "or exceeds ratio %.2f + %.2f m"
                    % (
                        self.entry_corridor_half_width,
                        self.entry_plan_maximum_length_ratio,
                        self.entry_plan_maximum_length_slack,
                    )
                )
            rospy.logwarn_throttle(
                2.0,
                "entry plan rejected before motion: %s",
                last_reason,
            )
            time.sleep(0.10)

        raise ExplorationFailure(
            ExploreFloorResult.ERROR_ENTRY_TRANSIT,
            "no safe local entry plan within %.2f ROS s: %s"
            % (self.entry_plan_timeout, last_reason),
        )

    def wait_for_entered_floor(self, baseline_message):
        baseline_version = self.map_version(baseline_message)
        allowed = self.build_roi_mask(baseline_message)
        baseline_known = known_cell_count(baseline_message.data, allowed)
        probe_xy = self.entry_probe_xy()
        started_ros = rospy.Time.now()
        started_wall = time.monotonic()
        last_position_error = float("inf")
        last_yaw_error = float("inf")
        last_known_gain = 0
        while not rospy.is_shutdown():
            self.check_cancel_safety_and_deadline()
            now_ros = rospy.Time.now()
            if now_ros < started_ros:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_ENTRY_MAP,
                    "ROS/simulation clock moved backwards after entry transit",
                )
            if (
                (now_ros - started_ros).to_sec() >= self.entry_map_timeout
                or time.monotonic() - started_wall
                >= self.entry_map_timeout * self.wall_factor
            ):
                break
            current = self.pose_in_frame(
                self.floor_entry_pose.header.frame_id
            )
            last_position_error, last_yaw_error = self.pose_errors(
                current, self.floor_entry_pose
            )
            with self.lock:
                message = copy.deepcopy(self.map_message)
            if message is not None:
                current_allowed = self.build_roi_mask(message)
                current_known = known_cell_count(
                    message.data, current_allowed
                )
                last_known_gain = current_known - baseline_known
                current_xy = (
                    current.pose.position.x,
                    current.pose.position.y,
                )
                current_anchor = nearest_known_free_anchor(
                    message.data,
                    self.grid_spec(message),
                    current_xy,
                    self.entry_anchor_search_radius,
                    self.free_threshold,
                )
                if (
                    self.map_version(message) != baseline_version
                    and last_known_gain
                    >= self.entry_minimum_new_known_cells
                    and last_position_error
                    <= self.entry_position_tolerance
                    and last_yaw_error <= self.entry_yaw_tolerance
                    and current_anchor is not None
                    and known_free_path_exists(
                        message.data,
                        self.grid_spec(message),
                        current_anchor,
                        probe_xy,
                        self.free_threshold,
                    )
                ):
                    with self.lock:
                        self.coverage = coverage_ratio(
                            message.data, current_allowed
                        )
                    rospy.loginfo(
                        "entered floor confirmed without truth: "
                        "pose_error=%.3f m/%.3f rad, ROI known gain=%d cells",
                        last_position_error,
                        last_yaw_error,
                        last_known_gain,
                    )
                    return
            time.sleep(0.05)
        raise ExplorationFailure(
            ExploreFloorResult.ERROR_ENTRY_MAP,
            "entry transit ended but floor entry was not confirmed: "
            "pose_error=%.3f m/%.3f rad, ROI known gain=%d/%d cells"
            % (
                last_position_error,
                last_yaw_error,
                last_known_gain,
                self.entry_minimum_new_known_cells,
            ),
        )

    def wait_for_prerequisites(self):
        deadline = time.monotonic() + self.prerequisite_timeout
        service_ready = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.check_cancel_safety_and_deadline(
                check_mapping=False, check_controller=False
            )
            with self.lock:
                map_message = self.map_message
                status = self.mapping_status
                healthy = (
                    self.mapping_usable(status)
                    and map_message is not None
                    and map_message.header.frame_id
                    and map_message.info.resolution > 0.0
                    and sum(
                        1
                        for value in map_message.data
                        if 0 <= value <= self.free_threshold
                    ) >= self.minimum_free_cells
                )
            move_ready = self.move_client.wait_for_server(
                rospy.Duration(0.05)
            )
            if not service_ready:
                try:
                    rospy.wait_for_service(self.make_plan_name, timeout=0.05)
                    service_ready = True
                except rospy.ROSException:
                    pass
            controller_ready = self.controller_is_ready()
            if healthy and move_ready and service_ready and controller_ready:
                try:
                    self.pose_in_frame(map_message.header.frame_id)
                    return
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                ):
                    pass
            time.sleep(0.05)
        raise ExplorationFailure(
            ExploreFloorResult.ERROR_PRECONDITION,
            "prerequisite timeout: require healthy floor map, TF, move_base, "
            "make_plan, and a fresh controller_ready=true heartbeat",
        )

    def check_cancel_safety_and_deadline(
        self, check_mapping=True, check_controller=True
    ):
        if self.server.is_preempt_requested():
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_CANCELLED,
                "exploration goal cancelled; move_base goal stopped",
                preempted=True,
            )
        with self.lock:
            safety_locked = self.safety_locked
            identity = self.mapping_identity(
                self.mapping_status, self.map_message
            )
            healthy_age = time.monotonic() - self.last_mapping_healthy_wall
        if safety_locked:
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_SAFETY_STOP,
                "a1_cmd_mux safety lock active",
            )
        if check_controller and not self.controller_is_ready():
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_PRECONDITION,
                "controller_ready is false, stale, or clock-invalid; "
                "MoveBaseAction goal is not permitted",
            )
        if (
            self.action_ros_deadline is not None
            and rospy.Time.now() >= self.action_ros_deadline
        ) or (
            self.action_wall_deadline is not None
            and time.monotonic() >= self.action_wall_deadline
        ):
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_TIMEOUT,
                "ExploreFloor overall timeout",
            )
        if check_mapping and self.action_identity is not None:
            if identity != self.action_identity:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_MAP_LOST,
                    "floor map identity changed during action: %r -> %r"
                    % (self.action_identity, identity),
                )
            if healthy_age > self.mapping_health_grace:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_MAP_LOST,
                    "floor mapping health lost for %.2f s" % healthy_age,
                )

    def transition(self, state, message, target=None):
        if state not in self.INTERNAL_STATES:
            raise ValueError("unknown exploration state %s" % state)
        with self.lock:
            self.state = state
            self.state_message = message
            if target is not None:
                self.current_target = copy.deepcopy(target)
            elif state not in (
                "NAVIGATING", "TRANSIT_TO_ENTRY", "RETURNING"
            ):
                self.current_target = PoseStamped()
        self.publish_status()
        if self.server.is_active():
            feedback = ExploreFloorFeedback()
            feedback.state = self.FEEDBACK_CODES[state]
            feedback.coverage_ratio = self.coverage
            feedback.current_target = copy.deepcopy(self.current_target)
            feedback.message = "%s: %s" % (state, message)
            self.server.publish_feedback(feedback)
        rospy.loginfo("exploration state %s: %s", state, message)

    def publish_status(self):
        with self.lock:
            message = ExplorationStatus()
            message.header.stamp = rospy.Time.now()
            if self.map_message is not None:
                message.header.frame_id = self.map_message.header.frame_id
            message.state = self.STATUS_CODES[self.state]
            message.floor_id = self.floor_id
            message.coverage_ratio = self.coverage
            message.current_target = copy.deepcopy(self.current_target)
            message.message = "%s: %s" % (
                self.state, self.state_message
            )
        self.status_pub.publish(message)

    def status_timer_callback(self, _event):
        self.publish_status()

    def trajectory_timer_callback(self, _event):
        with self.lock:
            if (
                not self.action_active
                or self.map_message is None
                or not self.map_message.header.frame_id
            ):
                return
            frame = self.map_message.header.frame_id
        try:
            pose = self.pose_in_frame(frame)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return
        with self.lock:
            if self.trajectory.poses:
                previous = self.trajectory.poses[-1].pose.position
                if math.hypot(
                    pose.pose.position.x - previous.x,
                    pose.pose.position.y - previous.y,
                ) < self.trajectory_minimum_step:
                    return
            self.trajectory.header.frame_id = frame
            self.trajectory.header.stamp = rospy.Time.now()
            pose.header = self.trajectory.header
            self.trajectory.poses.append(pose)
            if len(self.trajectory.poses) > self.trajectory_maximum_poses:
                self.trajectory.poses = self.trajectory.poses[
                    -self.trajectory_maximum_poses:
                ]
            path = copy.deepcopy(self.trajectory)
        self.trajectory_pub.publish(path)

    def frontier_snapshot(self):
        with self.lock:
            message = copy.deepcopy(self.map_message)
        pose = self.pose_in_frame(message.header.frame_id)
        try:
            allowed = self.build_roi_mask(message)
        except ValueError as error:
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_PRECONDITION,
                "invalid single-floor exploration ROI: %s" % error,
            )
        scoped_coverage = coverage_ratio(message.data, allowed)
        with self.lock:
            self.coverage = scoped_coverage
        frontiers = extract_frontiers(
            message.data,
            self.grid_spec(message),
            robot_xy=(pose.pose.position.x, pose.pose.position.y),
            min_frontier_length_m=self.min_frontier_length,
            obstacle_clearance_m=self.obstacle_clearance,
            goal_standoff_m=self.goal_standoff,
            goal_search_radius_m=self.goal_search_radius,
            minimum_goal_distance_m=self.effective_min_goal_distance,
            maximum_goal_distance_m=self.max_goal_distance,
            free_threshold=self.free_threshold,
            occupied_threshold=self.occupied_threshold,
            information_gain_weight=self.information_gain_weight,
            distance_weight=self.distance_weight,
            allowed_mask=allowed,
        )
        self.publish_roi(message)
        return message, pose, frontiers

    def pose_for_frontier(self, frame, frontier):
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = frame
        target.pose.position.x = frontier.goal_x
        target.pose.position.y = frontier.goal_y
        target.pose.orientation.x, target.pose.orientation.y, \
            target.pose.orientation.z, target.pose.orientation.w = \
            quaternion_from_yaw(frontier.yaw)
        return target

    def entry_coordinates(self, x, y):
        """Return longitudinal/lateral coordinates relative to floor entry."""
        if self.floor_entry_pose is None:
            return None
        entry = self.floor_entry_pose.pose
        entry_yaw = yaw_from_quaternion(entry.orientation)
        cosine = math.cos(entry_yaw)
        sine = math.sin(entry_yaw)
        dx = x - entry.position.x
        dy = y - entry.position.y
        return (
            dx * cosine + dy * sine,
            -dx * sine + dy * cosine,
        )

    def corridor_point(self, longitudinal, lateral, frame):
        """Construct a pose from entry-axis coordinates without using a lane."""
        entry = self.floor_entry_pose.pose
        yaw = yaw_from_quaternion(entry.orientation)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = (
            entry.position.x + longitudinal * cosine - lateral * sine
        )
        pose.pose.position.y = (
            entry.position.y + longitudinal * sine + lateral * cosine
        )
        pose.pose.orientation.x, pose.pose.orientation.y, \
            pose.pose.orientation.z, pose.pose.orientation.w = \
            quaternion_from_yaw(yaw)
        return pose

    def estimate_corridor_model(self, robot_pose):
        """Fit the local corridor from a fresh pair of opposite stable walls.

        Door openings are deliberately excluded from this estimate.  Each wall
        is reduced to its signed entry-axis offset; a valid pair must bracket
        the robot and have a plausible corridor width.  This makes centering a
        closed-loop measurement instead of a remembered-doorway guess.
        """
        if self.floor_entry_pose is None:
            return None
        with self.lock:
            message = copy.deepcopy(self.wall_message)
            previous = copy.deepcopy(self.corridor_model)
        if (
                message is None
                or message.header.frame_id != robot_pose.header.frame_id):
            return None
        now = rospy.Time.now()
        if now < message.header.stamp:
            return None
        age = (now - message.header.stamp).to_sec()
        if age > self.corridor_model_max_age:
            rospy.logwarn_throttle(
                2.0, "corridor model unavailable: wall snapshot age %.2f s", age
            )
            return None
        robot_longitudinal, robot_lateral = self.entry_coordinates(
            robot_pose.pose.position.x, robot_pose.pose.position.y
        )
        if robot_longitudinal < self.active_corridor_model_minimum_longitudinal:
            rospy.loginfo_throttle(
                2.0,
                "corridor centering intentionally inactive before main "
                "corridor: longitudinal=%.2f m < %.2f m",
                robot_longitudinal,
                self.active_corridor_model_minimum_longitudinal,
            )
            return None
        corridor_yaw = yaw_from_quaternion(
            self.floor_entry_pose.pose.orientation
        )
        candidates = []
        window_min = robot_longitudinal - self.corridor_model_lookbehind
        window_max = robot_longitudinal + self.corridor_model_lookahead
        for wall in message.walls:
            if (
                    not wall.stable
                    or wall.status != "observed"
                    or wall.length < self.corridor_minimum_wall_length):
                continue
            dx = wall.end.x - wall.start.x
            dy = wall.end.y - wall.start.y
            wall_yaw = math.atan2(dy, dx)
            parallel_error = abs(normalize_angle(wall_yaw - corridor_yaw))
            parallel_error = min(parallel_error, abs(math.pi - parallel_error))
            if parallel_error > self.corridor_wall_angle_tolerance:
                continue
            start_long, start_lat = self.entry_coordinates(
                wall.start.x, wall.start.y
            )
            end_long, end_lat = self.entry_coordinates(wall.end.x, wall.end.y)
            segment_min = min(start_long, end_long)
            segment_max = max(start_long, end_long)
            if segment_max < window_min or segment_min > window_max:
                continue
            lateral = 0.5 * (start_lat + end_lat)
            distance_to_window = max(
                0.0,
                segment_min - robot_longitudinal,
                robot_longitudinal - segment_max,
            )
            quality = (
                max(0.05, float(wall.confidence))
                * min(float(wall.length), 5.0)
                / (1.0 + distance_to_window + 3.0 * parallel_error)
            )
            candidates.append((lateral, quality, wall))
        left = [item for item in candidates if item[0] > robot_lateral + 0.05]
        right = [item for item in candidates if item[0] < robot_lateral - 0.05]
        pairs = []
        for left_item in left:
            for right_item in right:
                width = left_item[0] - right_item[0]
                if not (
                        self.corridor_minimum_width <= width
                        <= self.corridor_maximum_width):
                    continue
                center = 0.5 * (left_item[0] + right_item[0])
                centering = abs(robot_lateral - center)
                score = (
                    left_item[1] + right_item[1]
                    - 0.20 * centering
                    - 0.10 * abs(width - 2.2)
                )
                pairs.append((score, center, width, left_item, right_item))
        if not pairs:
            rospy.logwarn_throttle(
                2.0,
                "corridor model unavailable: no fresh opposite stable-wall pair "
                "brackets robot lateral=%.2f (%d candidates)",
                robot_lateral,
                len(candidates),
            )
            return None
        _score, measured_center, width, left_item, right_item = max(
            pairs, key=lambda item: item[0]
        )
        center = measured_center
        if (
                previous is not None
                and previous.frame == message.header.frame_id
                and abs(previous.center_lateral - measured_center) < 0.80):
            gain = max(0.0, min(1.0, self.corridor_center_filter_gain))
            center = (
                (1.0 - gain) * previous.center_lateral
                + gain * measured_center
            )
        confidence = min(
            1.0,
            0.5 * (
                float(left_item[2].confidence)
                + float(right_item[2].confidence)
            ),
        )
        model = SimpleNamespace(
            frame=message.header.frame_id,
            stamp=message.header.stamp,
            center_lateral=center,
            measured_center_lateral=measured_center,
            width=width,
            left_lateral=left_item[0],
            right_lateral=right_item[0],
            left_id=left_item[2].detection_id,
            right_id=right_item[2].detection_id,
            confidence=confidence,
            robot_longitudinal=robot_longitudinal,
            robot_lateral=robot_lateral,
        )
        with self.lock:
            self.corridor_model = model
        self.publish_corridor_model(model)
        return model

    def publish_corridor_model(self, model):
        markers = MarkerArray()
        for marker_id, lateral, color in (
                (0, model.left_lateral, (0.1, 0.8, 1.0)),
                (1, model.right_lateral, (0.1, 0.8, 1.0)),
                (2, model.center_lateral, (1.0, 0.9, 0.1))):
            marker = Marker()
            marker.header.frame_id = model.frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "corridor_model"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.06 if marker_id < 2 else 0.10
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.9
            for longitudinal in (
                    model.robot_longitudinal - 3.0,
                    model.robot_longitudinal + 6.0):
                pose = self.corridor_point(longitudinal, lateral, model.frame)
                marker.points.append(copy.deepcopy(pose.pose.position))
            markers.markers.append(marker)
        text_marker = Marker()
        text_marker.header.frame_id = model.frame
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "corridor_model"
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        label_pose = self.corridor_point(
            model.robot_longitudinal + 1.0,
            model.center_lateral,
            model.frame,
        )
        text_marker.pose.position = label_pose.pose.position
        text_marker.pose.position.z = 0.8
        text_marker.scale.z = 0.30
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = (
            "WALL CENTER err=%+.2fm width=%.2fm conf=%.2f ids=%d/%d"
            % (
                model.robot_lateral - model.center_lateral,
                model.width,
                model.confidence,
                model.left_id,
                model.right_id,
            )
        )
        markers.markers.append(text_marker)
        self.corridor_model_pub.publish(markers)

    def corridor_probe_target(self, map_message, robot_pose):
        """Return a short, map-verified goal farther down the main corridor.

        A 360-degree room scan can turn the visible corridor into known free
        space without leaving a free/unknown boundary large enough to survive
        frontier filtering.  That does not mean the corridor was traversed.
        After a room pair, advance through already observed free space and
        collect a new forward scan before permitting no-frontier completion.
        This uses only localization, the OccupancyGrid and make_plan.
        """
        if (
                not self.corridor_probe_enabled
                or self.floor_entry_pose is None
                or len(self.completed_room_branches)
                < self.corridor_probe_minimum_completed_rooms):
            return None, False
        robot_longitudinal, robot_lateral = self.entry_coordinates(
            robot_pose.pose.position.x, robot_pose.pose.position.y
        )
        corridor_model = self.estimate_corridor_model(robot_pose)
        if corridor_model is None:
            rospy.logwarn_throttle(
                2.0,
                "corridor advance deferred: no fresh dual-wall centre model",
            )
            return None, False
        center_lateral = corridor_model.center_lateral
        base = max(robot_longitudinal, self.maximum_corridor_progress)
        # A probe is an observation stride, not a request to approach the end
        # wall as closely as possible.  If one complete stride plus the robot
        # clearance cannot fit, exploration has reached the corridor end.
        steps = [self.corridor_probe_step]
        for distance in steps:
            if distance < 0.70:
                continue
            longitudinal = base + distance
            target = self.corridor_point(
                longitudinal, center_lateral, map_message.header.frame_id
            )
            if not self.target_in_roi(target):
                continue
            if not self.known_free_clearance(
                    map_message,
                    target.pose.position.x,
                    target.pose.position.y,
                    self.corridor_probe_clearance):
                continue
            if not self.known_free_segment(
                    map_message,
                    robot_pose,
                    target,
                    self.corridor_probe_clearance):
                rospy.loginfo(
                    "corridor end reached: the next %.2f m centreline stride "
                    "does not have continuous known-free clearance; no probe "
                    "goal will be sent",
                    distance,
                )
                continue
            if point_near(
                    self.visited_goals,
                    target.pose.position.x,
                    target.pose.position.y,
                    self.visited_radius):
                continue
            probe_state = failed_goal_state(
                self.failed_goals,
                target.pose.position.x,
                target.pose.position.y,
                self.failed_radius,
                time.monotonic(),
                self.maximum_failures,
            )
            if probe_state in ("permanent", "cooldown"):
                rospy.loginfo(
                    "corridor probe suppressed by navigation history: "
                    "longitudinal=%.2f state=%s",
                    longitudinal,
                    probe_state,
                )
                continue
            reachable = self.path_exists(robot_pose, target)
            if reachable is None:
                return None, True
            if reachable:
                rospy.loginfo(
                    "no eligible frontier after completed rooms; advancing "
                    "%.2f m along live dual-wall centre to longitudinal %.2f m; "
                    "lateral %.2f -> %.2f m width=%.2f m conf=%.2f",
                    distance,
                    longitudinal,
                    robot_lateral,
                    center_lateral,
                    corridor_model.width,
                    corridor_model.confidence,
                )
                return target, False
        return None, False

    def room_branch_key(self, longitudinal, lateral):
        """Associate a noisy door observation with a persistent room identity."""
        station = int(round(longitudinal / self.room_station_width))
        side = 1 if lateral > 0.0 else -1
        best_branch = None
        best_error = float("inf")
        for branch, doorway in self.remembered_room_doorways.items():
            remembered_longitudinal, remembered_lateral = (
                self.entry_coordinates(doorway.center.x, doorway.center.y)
            )
            remembered_side = 1 if remembered_lateral > 0.0 else -1
            longitudinal_error = abs(
                remembered_longitudinal - longitudinal
            )
            lateral_error = abs(remembered_lateral - lateral)
            error = longitudinal_error + lateral_error
            if (
                remembered_side == side
                and longitudinal_error
                <= self.room_identity_longitudinal_tolerance
                and lateral_error <= self.room_identity_lateral_tolerance
                and error < best_error
            ):
                best_branch = branch
                best_error = error
        if best_branch is not None:
            return best_branch
        return station, side

    def matching_room_doorway(self, frontier):
        """Match a lateral frontier to a stable, traversable narrow doorway."""
        if self.floor_entry_pose is None:
            return None
        frontier_longitudinal, frontier_lateral = self.entry_coordinates(
            frontier.goal_x, frontier.goal_y
        )
        if (
                frontier_longitudinal
                < self.room_minimum_door_longitudinal
                or abs(frontier_lateral) < self.room_lateral_threshold):
            return None
        with self.lock:
            doorway_message = copy.deepcopy(self.doorway_message)
        if (
                doorway_message is None
                or doorway_message.header.frame_id
                != self.floor_entry_pose.header.frame_id):
            return None
        best = None
        best_error = float("inf")
        frontier_side = 1 if frontier_lateral > 0.0 else -1
        for doorway in doorway_message.doorways:
            door_longitudinal, door_lateral = self.entry_coordinates(
                doorway.center.x, doorway.center.y
            )
            if (
                    not doorway.stable
                    # The structural detector can establish a precise open
                    # wall gap before its ray-based state estimator has enough
                    # bins to leave UNKNOWN. Reject only explicit negative
                    # states here; make_plan validates physical reachability
                    # before the room target is ever dispatched.
                    or doorway.state in (doorway.CLOSED, doorway.BLOCKED)
                    or doorway.width < self.room_door_minimum_width
                    or doorway.width > self.room_door_maximum_width
                    or abs(door_lateral)
                    > self.room_door_maximum_lateral
                    or door_longitudinal
                    < self.room_minimum_door_longitudinal
                    or (1 if door_lateral > 0.0 else -1) != frontier_side):
                continue
            error = abs(door_longitudinal - frontier_longitudinal)
            if (
                    error <= self.room_door_station_tolerance
                    and error < best_error):
                best = doorway
                best_error = error
        return best

    def room_target_from_doorway(self, frame, doorway):
        """Build corridor-side and room-side poses from perceived geometry."""
        candidates = []
        for raw_pose in (doorway.entry_pose, doorway.exit_pose):
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp = rospy.Time.now()
            pose.pose = copy.deepcopy(raw_pose)
            _longitudinal, lateral = self.entry_coordinates(
                pose.pose.position.x, pose.pose.position.y
            )
            candidates.append((abs(lateral), pose))
        candidates.sort(key=lambda item: item[0])
        corridor_pose = candidates[0][1]
        room_pose = candidates[-1][1]
        door_longitudinal, door_lateral = self.entry_coordinates(
            doorway.center.x, doorway.center.y
        )
        side = 1.0 if door_lateral > 0.0 else -1.0
        entry_yaw = yaw_from_quaternion(
            self.floor_entry_pose.pose.orientation
        )
        room_pose.pose.position.x += (
            -math.sin(entry_yaw) * side * self.room_goal_extension
        )
        room_pose.pose.position.y += (
            math.cos(entry_yaw) * side * self.room_goal_extension
        )
        inward_yaw = math.atan2(
            room_pose.pose.position.y - corridor_pose.pose.position.y,
            room_pose.pose.position.x - corridor_pose.pose.position.x,
        )
        room_pose.pose.orientation.x, room_pose.pose.orientation.y, \
            room_pose.pose.orientation.z, room_pose.pose.orientation.w = \
            quaternion_from_yaw(inward_yaw)
        branch = self.room_branch_key(door_longitudinal, door_lateral)
        return branch, corridor_pose, room_pose

    def freeze_room_branch_geometry(
            self, branch, corridor_pose, room_pose):
        """Freeze both sides of a doorway for one room-exploration transaction.

        Structural doorway tracks can be re-associated while the robot rotates
        at a door.  Mixing the corridor pose from one detection with the room
        pose from a later detection can send the second stage across a
        different opening.  Once a branch is first approached, its two poses
        therefore remain immutable until the branch is completed.
        """
        if branch not in self.room_branch_entry_poses:
            self.room_branch_entry_poses[branch] = copy.deepcopy(
                corridor_pose
            )
        if branch not in self.room_branch_interior_poses:
            self.room_branch_interior_poses[branch] = copy.deepcopy(
                room_pose
            )

    def frozen_room_branch_geometry(
            self, branch, corridor_pose, room_pose):
        self.freeze_room_branch_geometry(
            branch, corridor_pose, room_pose
        )
        return (
            copy.deepcopy(self.room_branch_entry_poses[branch]),
            copy.deepcopy(self.room_branch_interior_poses[branch]),
        )

    def room_interior_progress(self, branch, pose):
        """Return signed distance crossed from the frozen corridor-side pose."""
        corridor_pose = self.room_branch_entry_poses.get(branch)
        room_pose = self.room_branch_interior_poses.get(branch)
        if corridor_pose is None or room_pose is None:
            return None
        start = corridor_pose.pose.position
        finish = room_pose.pose.position
        dx = finish.x - start.x
        dy = finish.y - start.y
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            return None
        current = pose.pose.position
        return (
            (current.x - start.x) * dx
            + (current.y - start.y) * dy
        ) / length

    def is_entry_transit_frontier(self, x, y):
        """Exclude the already traversed public doorway from exploration."""
        longitudinal, lateral = self.entry_coordinates(x, y)
        return (
            longitudinal <= self.entry_frontier_exclusion_depth
            and abs(lateral) <= self.entry_frontier_exclusion_half_width
        )

    def known_free_clearance(self, map_message, x, y, radius):
        """Require a known-free disc so a quadruped can rotate without a wall."""
        spec = self.grid_spec(map_message)
        minimum_x = int(math.floor(
            (x - radius - spec.origin_x) / spec.resolution
        ))
        maximum_x = int(math.floor(
            (x + radius - spec.origin_x) / spec.resolution
        ))
        minimum_y = int(math.floor(
            (y - radius - spec.origin_y) / spec.resolution
        ))
        maximum_y = int(math.floor(
            (y + radius - spec.origin_y) / spec.resolution
        ))
        radius_squared = radius * radius
        for cell_y in range(minimum_y, maximum_y + 1):
            for cell_x in range(minimum_x, maximum_x + 1):
                world_x = spec.origin_x + (cell_x + 0.5) * spec.resolution
                world_y = spec.origin_y + (cell_y + 0.5) * spec.resolution
                if (world_x - x) ** 2 + (world_y - y) ** 2 > radius_squared:
                    continue
                if (
                        cell_x < 0 or cell_x >= spec.width
                        or cell_y < 0 or cell_y >= spec.height):
                    return False
                value = map_message.data[cell_y * spec.width + cell_x]
                if value < 0 or value > self.free_threshold:
                    return False
        return True

    def known_free_segment(self, map_message, start, target, radius):
        """Require the complete commanded segment to remain known and free.

        Checking only the endpoint can miss a thin end wall or accept a goal
        from a stale/open cell on the far side of that wall.  Sample no farther
        apart than two grid cells and apply the robot clearance disc at every
        sample, including the endpoint.
        """
        dx = target.pose.position.x - start.pose.position.x
        dy = target.pose.position.y - start.pose.position.y
        distance = math.hypot(dx, dy)
        if distance <= 1.0e-6:
            return self.known_free_clearance(
                map_message,
                target.pose.position.x,
                target.pose.position.y,
                radius,
            )
        resolution = self.grid_spec(map_message).resolution
        sample_spacing = max(0.05, min(0.10, 2.0 * resolution))
        samples = max(1, int(math.ceil(distance / sample_spacing)))
        for index in range(1, samples + 1):
            ratio = float(index) / float(samples)
            x = start.pose.position.x + ratio * dx
            y = start.pose.position.y + ratio * dy
            if not self.known_free_clearance(map_message, x, y, radius):
                return False
        return True

    def open_room_scan_pose(self, branch, current_pose):
        """Find a reachable observation point deeper in the room with clearance."""
        with self.lock:
            map_message = copy.deepcopy(self.map_message)
        if map_message is None:
            return None
        frame = map_message.header.frame_id
        if current_pose.header.frame_id != frame:
            return None
        entry_yaw = yaw_from_quaternion(
            self.floor_entry_pose.pose.orientation
        )
        side = 1.0 if branch[1] > 0 else -1.0
        deeper_x = -math.sin(entry_yaw) * side
        deeper_y = math.cos(entry_yaw) * side
        axial_x = math.cos(entry_yaw)
        axial_y = math.sin(entry_yaw)
        # Prefer moving farther into the open area. Small axial alternatives
        # avoid selecting a point on the same wall when the branch is offset.
        for distance in [
                0.0, 0.5, 1.0, 1.5, 2.0, self.room_scan_search_distance]:
            for axial_offset in (0.0, 0.45, -0.45):
                candidate = copy.deepcopy(current_pose)
                candidate.header.stamp = rospy.Time.now()
                candidate.pose.position.x += (
                    deeper_x * distance + axial_x * axial_offset
                )
                candidate.pose.position.y += (
                    deeper_y * distance + axial_y * axial_offset
                )
                if not self.known_free_clearance(
                        map_message,
                        candidate.pose.position.x,
                        candidate.pose.position.y,
                        self.room_scan_clearance):
                    continue
                if self.path_exists(current_pose, candidate):
                    return candidate
        return None

    def path_exists(self, start, target):
        last_error = None
        for attempt in range(1, self.make_plan_retry_attempts + 1):
            self.check_cancel_safety_and_deadline()
            try:
                response = self.make_plan(
                    start=start, goal=target, tolerance=0.20
                )
                self.make_plan_failure_since_wall = None
                return len(response.plan.poses) >= 2
            except rospy.ServiceException as error:
                last_error = error
                now = time.monotonic()
                if self.make_plan_failure_since_wall is None:
                    self.make_plan_failure_since_wall = now
                unavailable_for = now - self.make_plan_failure_since_wall
                if unavailable_for >= self.make_plan_unavailable_timeout:
                    raise ExplorationFailure(
                        ExploreFloorResult.ERROR_NAVIGATION_UNAVAILABLE,
                        "make_plan unavailable for %.1f wall seconds: %r"
                        % (unavailable_for, error),
                    )
                rospy.logwarn(
                    "transient make_plan error %d/%d (unavailable %.1fs): %r",
                    attempt,
                    self.make_plan_retry_attempts,
                    unavailable_for,
                    error,
                )
                if attempt < self.make_plan_retry_attempts:
                    self.check_cancel_safety_and_deadline()
                    time.sleep(self.make_plan_retry_delay)
        rospy.logwarn(
            "make_plan still transiently unavailable after %d attempts: %r",
            self.make_plan_retry_attempts,
            last_error,
        )
        return None

    def choose_frontier(self, map_message, robot_pose, frontiers):
        self.selected_room_branch = None
        self.selected_room_stage = None
        self.selected_corridor_center_target = False
        now = time.monotonic()
        cooling = has_pending_retry(
            self.failed_goals, now, self.maximum_failures
        )
        frame = map_message.header.frame_id
        if self.room_priority_enabled and self.floor_entry_pose is not None:
            with self.lock:
                remembered_doorways = copy.deepcopy(
                    self.remembered_room_doorways
                )
                active_room_branch = self.active_room_branch
            robot_longitudinal, _robot_lateral = self.entry_coordinates(
                robot_pose.pose.position.x, robot_pose.pose.position.y
            )
            self.maximum_corridor_progress = max(
                self.maximum_corridor_progress, robot_longitudinal
            )
            doorway_candidates = []
            if map_message.header.frame_id == frame:
                for branch, doorway in remembered_doorways.items():
                    longitudinal, lateral = self.entry_coordinates(
                        doorway.center.x, doorway.center.y
                    )
                    if branch in self.completed_room_branches:
                        continue
                    if (
                            active_room_branch is not None
                            and branch != active_room_branch):
                        continue
                    if (
                            active_room_branch is None
                            and (
                                longitudinal
                                > robot_longitudinal + self.room_lookahead
                                or longitudinal
                                < self.maximum_corridor_progress
                                - self.room_backtrack)):
                        continue
                    retry_state = failed_goal_state(
                        self.failed_goals,
                        doorway.center.x,
                        doorway.center.y,
                        self.failed_radius,
                        now,
                        self.maximum_failures,
                    )
                    if retry_state == "permanent":
                        if active_room_branch == branch:
                            rospy.logwarn(
                                "room transaction released after permanent "
                                "doorway failure: station=%d side=%s",
                                branch[0],
                                "left" if branch[1] > 0 else "right",
                            )
                            self.active_room_branch = None
                            active_room_branch = None
                        continue
                    if retry_state == "cooldown":
                        cooling = True
                        continue
                    doorway_candidates.append(
                        (
                            branch[0],
                            0 if lateral > 0.0 else 1,
                            longitudinal,
                            doorway,
                        )
                    )
            doorway_candidates.sort(
                key=lambda item: (item[0], item[1], item[2])
            )
            for _station, _side_order, _longitudinal, doorway \
                    in doorway_candidates:
                branch, corridor_pose, room_target = \
                    self.room_target_from_doorway(frame, doorway)
                corridor_pose, room_target = \
                    self.frozen_room_branch_geometry(
                        branch, corridor_pose, room_target
                    )
                if branch not in self.approached_room_branches:
                    target = PoseStamped()
                    target.header.frame_id = frame
                    target.header.stamp = rospy.Time.now()
                    target.pose.position = copy.deepcopy(doorway.center)
                    center_yaw = math.atan2(
                        room_target.pose.position.y - doorway.center.y,
                        room_target.pose.position.x - doorway.center.x,
                    )
                    target.pose.orientation.x, \
                        target.pose.orientation.y, \
                        target.pose.orientation.z, \
                        target.pose.orientation.w = \
                        quaternion_from_yaw(center_yaw)
                    stage = "door_center"
                else:
                    target = room_target
                    stage = "room_interior"
                reachable = self.path_exists(robot_pose, target)
                if reachable is None:
                    return None, None, cooling, True
                if reachable:
                    if self.active_room_branch is None:
                        self.active_room_branch = branch
                        rospy.loginfo(
                            "room transaction acquired: station=%d side=%s; "
                            "other doorways are deferred until scan, exit, "
                            "recenter and alignment complete",
                            branch[0],
                            "left" if branch[1] > 0 else "right",
                        )
                    self.selected_room_branch = branch
                    self.selected_room_stage = stage
                    rospy.loginfo(
                        "selected structural doorway: id=%d width=%.2f m "
                        "state=%d station=%d side=%s stage=%s "
                        "target=(%.2f, %.2f)",
                        doorway.detection_id,
                        doorway.width,
                        doorway.state,
                        branch[0],
                        "left" if branch[1] > 0 else "right",
                        stage,
                        target.pose.position.x,
                        target.pose.position.y,
                    )
                    synthetic = SimpleNamespace(
                        goal_x=doorway.center.x,
                        goal_y=doorway.center.y,
                        length_m=doorway.width,
                        score=100.0,
                    )
                    return target, synthetic, cooling, False
                failure = record_failure(
                    self.failed_goals,
                    doorway.center.x,
                    doorway.center.y,
                    self.failed_radius,
                    now,
                    self.failure_cooldown,
                )
                rospy.logwarn(
                    "structural doorway %d has no known-space plan, "
                    "failure %d/%d",
                    doorway.detection_id,
                    failure.failures,
                    self.maximum_failures,
                )
                if failure.failures < self.maximum_failures:
                    cooling = True
            if active_room_branch is not None:
                rospy.logwarn_throttle(
                    2.0,
                    "active room transaction station=%d side=%s has no "
                    "dispatchable target; waiting for retry/map evidence",
                    active_room_branch[0],
                    "left" if active_room_branch[1] > 0 else "right",
                )
                return None, None, True, False
        ordered_frontiers = list(frontiers)
        if (
                self.room_priority_enabled
                and self.floor_entry_pose is not None):
            robot_longitudinal, _robot_lateral = self.entry_coordinates(
                robot_pose.pose.position.x, robot_pose.pose.position.y
            )

            def room_priority(frontier):
                longitudinal, lateral = self.entry_coordinates(
                    frontier.goal_x, frontier.goal_y
                )
                doorway = self.matching_room_doorway(frontier)
                nearby_room = (
                    doorway is not None
                    and longitudinal
                    <= robot_longitudinal + self.room_lookahead
                    and longitudinal
                    >= robot_longitudinal - self.room_backtrack
                )
                if nearby_room:
                    branch = self.room_branch_key(longitudinal, lateral)
                    if branch in self.completed_room_branches:
                        return (2, 0, 0, 0)
                    # Progress door station by door station. At the same
                    # station positive lateral is robot-left, so left rooms
                    # are deliberately exhausted before right rooms. Within
                    # one branch, choose the deepest reachable frontier first
                    # instead of repeatedly nibbling at its doorway.
                    return (
                        0,
                        branch[0],
                        0 if lateral > 0.0 else 1,
                        -abs(lateral),
                    )
                return (1, 0, 0, -frontier.score)

            ordered_frontiers.sort(key=room_priority)

        for frontier in ordered_frontiers:
            # Tiny negative-utility fragments arise next to the robot after a
            # room scan.  They add no observable area, but can require a
            # collision-invalid turn and trigger repeated backout recovery.
            # Wait for the next map/frontier instead of moving backwards for
            # a target that cannot advance exploration.
            if frontier.score < self.minimum_frontier_score:
                continue
            if self.room_priority_enabled and self.floor_entry_pose is not None:
                longitudinal, lateral = self.entry_coordinates(
                    frontier.goal_x, frontier.goal_y
                )
                # The two large side openings at the public entrance are
                # branch corridors, not room doors. They occupy the entrance
                # junction station, whereas real room doors occur farther
                # down the main corridor. Skip those junction branches in
                # this simplified observation mode and keep advancing axially.
                if (
                        abs(lateral) >= self.room_lateral_threshold
                        and longitudinal
                        < self.room_minimum_door_longitudinal):
                    continue
                if (
                        abs(lateral) < self.room_lateral_threshold
                        and longitudinal
                        < self.maximum_corridor_progress - 0.75):
                    continue
                doorway = None
                if abs(lateral) >= self.room_lateral_threshold:
                    doorway = self.matching_room_doorway(frontier)
                    if doorway is None:
                        continue
                if self.is_entry_transit_frontier(
                        frontier.goal_x, frontier.goal_y):
                    continue
                if (
                        abs(lateral) >= self.room_lateral_threshold
                        and self.room_branch_key(longitudinal, lateral)
                        in self.completed_room_branches):
                    continue
            if point_near(
                self.visited_goals,
                frontier.goal_x,
                frontier.goal_y,
                self.visited_radius,
            ):
                continue
            state = failed_goal_state(
                self.failed_goals,
                frontier.goal_x,
                frontier.goal_y,
                self.failed_radius,
                now,
                self.maximum_failures,
            )
            if state == "permanent":
                continue
            if state == "cooldown":
                cooling = True
                continue
            target = self.pose_for_frontier(frame, frontier)
            branch = None
            corridor_pose = None
            if self.room_priority_enabled and self.floor_entry_pose is not None:
                longitudinal, lateral = self.entry_coordinates(
                    frontier.goal_x, frontier.goal_y
                )
                doorway = (
                    self.matching_room_doorway(frontier)
                    if abs(lateral) >= self.room_lateral_threshold
                    else None
                )
                if doorway is not None:
                    branch, corridor_pose, target = \
                        self.room_target_from_doorway(frame, doorway)
                    corridor_pose, target = \
                        self.frozen_room_branch_geometry(
                            branch, corridor_pose, target
                        )
                elif (
                        longitudinal
                        >= self.active_corridor_model_minimum_longitudinal
                        and abs(lateral) < self.room_lateral_threshold):
                    model = self.estimate_corridor_model(robot_pose)
                    if model is not None:
                        robot_longitudinal, robot_lateral = \
                            self.entry_coordinates(
                                robot_pose.pose.position.x,
                                robot_pose.pose.position.y,
                            )
                        rolling_longitudinal = min(
                            longitudinal,
                            robot_longitudinal + self.corridor_probe_step,
                        )
                        target = self.corridor_point(
                            rolling_longitudinal,
                            model.center_lateral,
                            frame,
                        )
                        self.selected_corridor_center_target = True
                        rospy.loginfo(
                            "main-corridor frontier projected onto live wall "
                            "centre: raw=(%.2f, %.2f) rolling_long=%.2f "
                            "lateral %.2f -> %.2f width=%.2f conf=%.2f",
                            frontier.goal_x,
                            frontier.goal_y,
                            rolling_longitudinal,
                            robot_lateral,
                            model.center_lateral,
                            model.width,
                            model.confidence,
                        )
            # Room geometry can replace the raw frontier with a frozen door or
            # interior pose.  Navigation failures are recorded against that
            # dispatched pose, so enforce cooldown/permanent history against
            # the same coordinates rather than only the raw frontier point.
            dispatched_state = failed_goal_state(
                self.failed_goals,
                target.pose.position.x,
                target.pose.position.y,
                self.failed_radius,
                now,
                self.maximum_failures,
            )
            if dispatched_state == "permanent":
                if branch is not None and self.active_room_branch == branch:
                    rospy.logwarn(
                        "releasing permanently unreachable room transaction: "
                        "station=%d side=%s target=(%.2f, %.2f)",
                        branch[0],
                        "left" if branch[1] > 0 else "right",
                        target.pose.position.x,
                        target.pose.position.y,
                    )
                    self.active_room_branch = None
                continue
            if dispatched_state == "cooldown":
                cooling = True
                continue
            reachable = self.path_exists(robot_pose, target)
            if reachable is None:
                return None, None, cooling, True
            if reachable:
                if branch is not None and self.active_room_branch is None:
                    self.active_room_branch = branch
                    rospy.loginfo(
                        "room transaction acquired from frontier match: "
                        "station=%d side=%s",
                        branch[0],
                        "left" if branch[1] > 0 else "right",
                    )
                self.selected_room_branch = branch
                self.selected_room_stage = (
                    "room_interior" if branch is not None else None
                )
                return target, frontier, cooling, False
            failure = record_failure(
                self.failed_goals,
                frontier.goal_x,
                frontier.goal_y,
                self.failed_radius,
                now,
                self.failure_cooldown,
            )
            rospy.logwarn(
                "frontier make_plan rejected (%.2f, %.2f), failure %d/%d",
                frontier.goal_x,
                frontier.goal_y,
                failure.failures,
                self.maximum_failures,
            )
            if failure.failures < self.maximum_failures:
                cooling = True
        corridor_target, planner_degraded = self.corridor_probe_target(
            map_message, robot_pose
        )
        if planner_degraded:
            return None, None, cooling, True
        if corridor_target is not None:
            synthetic = SimpleNamespace(
                goal_x=corridor_target.pose.position.x,
                goal_y=corridor_target.pose.position.y,
                length_m=self.corridor_probe_step,
                score=1.0,
            )
            return corridor_target, synthetic, cooling, False
        return None, None, cooling, False

    def publish_frontiers(self, map_message, frontiers):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        marker = Marker()
        marker.header = copy.deepcopy(map_message.header)
        marker.header.stamp = rospy.Time.now()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = max(0.04, map_message.info.resolution * 0.8)
        marker.scale.y = marker.scale.x
        marker.color.r = 0.10
        marker.color.g = 1.00
        marker.color.b = 0.20
        marker.color.a = 0.90
        spec = self.grid_spec(map_message)
        total = sum(len(frontier.cells) for frontier in frontiers)
        stride = max(1, int(math.ceil(total / 20000.0)))
        index = 0
        for frontier in frontiers:
            for cell in frontier.cells:
                if index % stride == 0:
                    x, y = spec.cell_to_world(cell)
                    marker.points.append(Point(x=x, y=y, z=0.05))
                index += 1
        array.markers.append(marker)
        self.frontier_pub.publish(array)

        failed = Marker()
        failed.header = marker.header
        failed.ns = "failed_frontiers"
        failed.id = 0
        failed.type = Marker.POINTS
        failed.action = Marker.ADD
        failed.scale.x = 0.18
        failed.scale.y = 0.18
        failed.color.r = 1.0
        failed.color.g = 0.1
        failed.color.b = 0.1
        failed.color.a = 0.9
        failed.points = [
            Point(x=item.x, y=item.y, z=0.10)
            for item in self.failed_goals
        ]
        self.failed_pub.publish(failed)

    def publish_roi(self, map_message):
        polygon = PolygonStamped()
        polygon.header = copy.deepcopy(map_message.header)
        polygon.header.stamp = rospy.Time.now()
        polygon.polygon.points = [
            Point32(x=x, y=y, z=0.0) for x, y in self.roi_polygon_map
        ]
        self.roi_pub.publish(polygon)

        marker = Marker()
        marker.header = copy.deepcopy(map_message.header)
        marker.header.stamp = rospy.Time.now()
        marker.ns = "single_floor_roi"
        marker.id = 0
        if not self.roi_enabled or not self.roi_polygon_map:
            marker.action = Marker.DELETE
            self.scope_pub.publish(marker)
            return

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.color.r = 0.10
        marker.color.g = 0.45
        marker.color.b = 1.00
        marker.color.a = 0.95

        marker.points = [
            Point(x=x, y=y, z=0.04)
            for x, y in self.roi_polygon_map + self.roi_polygon_map[:1]
        ]
        self.scope_pub.publish(marker)

    def publish_target(self, target, namespace):
        marker = Marker()
        marker.header = target.header
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = target.pose
        marker.scale.x = 0.55
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = 1.0 if namespace == "return_target" else 0.1
        marker.color.g = 0.7
        marker.color.b = 1.0 if namespace != "return_target" else 0.1
        marker.color.a = 1.0
        self.target_pub.publish(marker)

    def clear_floor_visualization(self):
        """Remove latched markers before a localization generation changes."""
        self.trajectory_pub.publish(Path())
        self.roi_pub.publish(PolygonStamped())
        for publisher in (self.target_pub, self.failed_pub, self.scope_pub):
            clear = Marker()
            clear.action = Marker.DELETEALL
            publisher.publish(clear)
        clear_array = Marker()
        clear_array.action = Marker.DELETEALL
        self.frontier_pub.publish(MarkerArray(markers=[clear_array]))
        self.corridor_model_pub.publish(MarkerArray(markers=[clear_array]))

    def navigate(self, target, timeout, returning=False, allow_backout=True):
        # This check is intentionally adjacent to send_goal: frontier
        # extraction and make_plan must never create a race that bypasses the
        # controller-ready gate.
        self.check_cancel_safety_and_deadline()
        # move_base runs with respawn:=true after the integration_fix19
        # segfault, so a goal can be dispatched while the action server is
        # still coming back. Sending into a dead server silently loses the
        # goal and it surfaces later as a bogus "unreachable" outcome.
        if not self.move_client.wait_for_server(
                rospy.Duration(self.make_plan_unavailable_timeout)):
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_NAVIGATION_UNAVAILABLE,
                "move_base action server did not return within %.1f wall "
                "seconds" % self.make_plan_unavailable_timeout,
            )
        move_goal = MoveBaseGoal(target_pose=target)
        self.move_client.send_goal(move_goal)
        started_ros = rospy.Time.now()
        started_wall = time.monotonic()
        backout_steps = 0
        # No-progress watchdog. move_base can sit on an unreachable goal
        # publishing nothing until navigation_goal (75 s) expires: measured on
        # 2026-08-01, the robot stood at world (2.240, 1.011) for more than 24 s
        # with cmd_vel identically zero while the status still read NAVIGATING.
        # Rotating on the spot is legitimate progress towards a goal pose, so
        # yaw counts too; only when neither position nor heading moves is the
        # goal declared stuck.
        progress_watchdog = (
            NoProgressWatchdog(
                self.no_progress_timeout,
                self.no_progress_distance,
                self.no_progress_yaw,
            )
            if self.no_progress_timeout > 0.0 else None
        )
        while not rospy.is_shutdown():
            try:
                self.check_cancel_safety_and_deadline()
            except ExplorationFailure:
                self.cancel_move_goal()
                raise
            state = self.move_client.get_state()
            if state in (
                GoalStatus.SUCCEEDED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.PREEMPTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST,
            ):
                if (
                        state == GoalStatus.ABORTED
                        and allow_backout
                        and backout_steps < self.backout_max_steps
                        and self.bounded_backout(target.header.frame_id)):
                    backout_steps += 1
                    rospy.logwarn(
                        "move_base could not turn; completed bounded backout "
                        "step %d/%d and retrying the same goal",
                        backout_steps,
                        self.backout_max_steps,
                    )
                    self.check_cancel_safety_and_deadline()
                    self.wait_for_interstitial_zero_settle()
                    move_goal.target_pose.header.stamp = rospy.Time.now()
                    self.move_client.send_goal(move_goal)
                    if progress_watchdog is not None:
                        progress_watchdog.reset()
                    continue
                recordable_failure = state in (
                    GoalStatus.ABORTED,
                    GoalStatus.REJECTED,
                    GoalStatus.LOST,
                )
                return (
                    state == GoalStatus.SUCCEEDED,
                    state,
                    "unreachable" if recordable_failure else None,
                )
            if progress_watchdog is not None:
                try:
                    current = self.pose_in_frame(target.header.frame_id)
                except Exception:  # noqa: BLE001 - TF hiccup is not a failure
                    current = None
                if current is not None:
                    progress = progress_watchdog.observe(
                        rospy.Time.now().to_sec(),
                        current.pose.position.x,
                        current.pose.position.y,
                        yaw_from_quaternion(current.pose.orientation),
                    )
                    if progress.stalled:
                        # A stall usually means the robot is physically
                        # wedged, not that the goal is wrong. Measured in
                        # run15: commanded vx averaged 0.374 m/s for 385
                        # sim s at world (4.51, 14.58) -- 0.84 m from the
                        # centre of a 2.2 x 1.0 x 0.75 m piece of furniture
                        # -- with the body 8 cm below standing height and a
                        # steady 4 deg nose-down pitch. Pushing harder or
                        # picking a new goal cannot help while the robot is
                        # leaning on an obstacle; backing off can. The
                        # ABORTED path already does this, and the watchdog
                        # used to return without ever attempting it.
                        if (
                                allow_backout
                                and backout_steps < self.backout_max_steps
                                and self.bounded_backout(
                                    target.header.frame_id)):
                            backout_steps += 1
                            rospy.logwarn(
                                "no progress for %.1f sim s at (%.2f, "
                                "%.2f); backed out (step %d/%d) and "
                                "retrying the goal",
                                self.no_progress_timeout,
                                current.pose.position.x,
                                current.pose.position.y,
                                backout_steps, self.backout_max_steps,
                            )
                            self.check_cancel_safety_and_deadline()
                            self.wait_for_interstitial_zero_settle()
                            move_goal.target_pose.header.stamp = \
                                rospy.Time.now()
                            self.move_client.send_goal(move_goal)
                            progress_watchdog.reset()
                            continue
                        cancelled_state = self.cancel_move_goal()
                        rospy.logwarn(
                            "%s goal made no progress: %.3f m / %.3f rad "
                            "in %.1f sim s at (%.2f, %.2f); abandoning it "
                            "instead of standing until the %.0f s goal "
                            "timeout",
                            "return" if returning else "frontier",
                            progress.moved_m, progress.turned_rad,
                            self.no_progress_timeout,
                            current.pose.position.x,
                            current.pose.position.y, timeout,
                        )
                        # "transient", never "unreachable": the watchdog
                        # cancelled our own goal, it did not prove move_base
                        # cannot reach it. record_failure()'s contract is
                        # explicit that only outcomes proving
                        # unreachability may count toward the permanent
                        # exclusion budget; a cancellation must merely
                        # cool the target down. Recording it as unreachable
                        # would let three slow approaches permanently
                        # blacklist a perfectly reachable room.
                        return False, cancelled_state, "transient"
            ros_elapsed = (rospy.Time.now() - started_ros).to_sec()
            wall_elapsed = time.monotonic() - started_wall
            if ros_elapsed >= timeout or wall_elapsed >= timeout * self.wall_factor:
                cancelled_state = self.cancel_move_goal()
                rospy.logwarn(
                    "%s move_base goal timeout after %.2f ROS s / %.2f wall s; "
                    "cancel settled in state %d",
                    "return" if returning else "frontier",
                    ros_elapsed,
                    wall_elapsed,
                    cancelled_state,
                )
                # A navigation timeout does NOT prove the target is unreachable.
                return False, cancelled_state, "transient"
            time.sleep(0.05)
        self.cancel_move_goal()
        return False, GoalStatus.LOST, False

    def bounded_backout(self, frame):
        """Back up one short step, then hand control back to turn-first DWA.

        The behavior source has higher mux priority than navigation, but is
        published only for this bounded maneuver. After every step the same
        move_base goal is retried with min_vel_x=0, so the robot tests for a
        collision-free turn again instead of reversing continuously.
        """
        try:
            start = self.pose_in_frame(frame)
        except Exception as error:
            rospy.logwarn("bounded backout pose unavailable: %s", error)
            return False
        started_ros = rospy.Time.now()
        started_wall = time.monotonic()
        command = Twist()
        command.linear.x = -abs(self.backout_speed)
        moved = 0.0
        while not rospy.is_shutdown():
            self.check_cancel_safety_and_deadline()
            try:
                current = self.pose_in_frame(frame)
                moved = math.hypot(
                    current.pose.position.x - start.pose.position.x,
                    current.pose.position.y - start.pose.position.y,
                )
            except Exception:
                current = None
            ros_elapsed = (rospy.Time.now() - started_ros).to_sec()
            wall_elapsed = time.monotonic() - started_wall
            if (
                    moved >= self.backout_step_distance
                    or ros_elapsed >= self.backout_max_sim_time
                    or wall_elapsed
                    >= self.backout_max_sim_time * self.wall_factor):
                break
            self.recovery_cmd_pub.publish(command)
            time.sleep(0.02)
        zero = Twist()
        for _unused in range(15):
            self.recovery_cmd_pub.publish(zero)
            time.sleep(0.02)
        rospy.loginfo(
            "bounded backout finished: distance=%.3f m sim_time=%.3f s",
            moved,
            (rospy.Time.now() - started_ros).to_sec(),
        )
        return moved >= min(0.15, self.backout_step_distance * 0.5)

    # ------------------------------------------------------------------
    # Room transaction, ported verbatim from the single-floor tree
    # (integration_20260730 / codex_single_floor_clean_20260803, 27 rounds
    # of measured evidence + 102 unit tests). It replaces the debug
    # scan_room_and_exit below, which entered a room, span 360 degrees on
    # the spot and left: mf11 measured that version livelocking at a door
    # mouth for 486 s of sim time, moving 1.1 m, burning the whole floor
    # budget. A room is finished when no reachable frontier remains inside
    # it, not when a rotation completes.
    # ------------------------------------------------------------------
    def explore_room_transaction(self, branch, frame):
        """Cover one room before allowing the mission to leave it.

        Replaces the debug-era single-point 360-degree scan.  Measured in the
        first competition run: the robot entered the room holding the floor's
        only danger source, came within 2.98 m of it, and left without ever
        putting it in the camera -- a 8.4 m deep room with furniture has large
        blind sectors from any single viewpoint.  Here the room's own free
        component is the exploration region, so space hidden behind furniture
        stays unknown and keeps producing frontiers until the robot has moved
        somewhere that observes it.
        """
        started_ros = rospy.Time.now()
        wall_budget = wall_backstop_seconds(
            self.room_transaction_timeout, self.wall_factor
        )
        started_wall = time.monotonic()
        goals = 0
        self.last_room_transaction_proven = False
        # Coverage is per room: a previous room's covered cells must not make
        # this one look already seen.
        self.camera_covered = None
        while not rospy.is_shutdown():
            self.check_cancel_safety_and_deadline()
            if (
                    (rospy.Time.now() - started_ros).to_sec()
                    >= self.room_transaction_timeout
                    or time.monotonic() - started_wall >= wall_budget
                    or goals >= self.room_transaction_max_goals):
                rospy.logwarn(
                    "room transaction budget spent after %d goals; leaving "
                    "the room with coverage unproven", goals,
                )
                self.last_room_transaction_proven = False
                break
            with self.lock:
                map_message = copy.deepcopy(self.map_message)
            if map_message is None:
                time.sleep(0.05)
                continue
            robot_pose = self.pose_in_frame(frame)
            self.mark_camera_coverage(map_message, robot_pose)
            frontiers = self.room_transaction_frontiers(
                map_message, robot_pose, branch
            )
            if frontiers is None:
                rospy.logwarn(
                    "room transaction could not bound the room; leaving"
                )
                self.last_room_transaction_proven = False
                break
            now = time.monotonic()
            target = None
            chosen = None
            for frontier in frontiers:
                if frontier.score < self.room_frontier_minimum_score:
                    continue
                if point_near(
                        self.visited_goals, frontier.goal_x, frontier.goal_y,
                        self.visited_radius):
                    continue
                if failed_goal_state(
                        self.failed_goals, frontier.goal_x, frontier.goal_y,
                        self.failed_radius, now,
                        self.maximum_failures) != "available":
                    continue
                candidate = self.pose_for_frontier(frame, frontier)
                reachable = self.path_exists(robot_pose, candidate)
                if reachable is None:
                    time.sleep(0.1)
                    target = None
                    break
                if reachable:
                    target = candidate
                    chosen = frontier
                    break
                record_failure(
                    self.failed_goals, frontier.goal_x, frontier.goal_y,
                    self.failed_radius, now, self.failure_cooldown,
                )
            if target is None:
                if chosen is None and frontiers is not None:
                    # "No frontier left" is not "the camera has seen this
                    # room". LiDAR maps the far side of a room from the
                    # doorway, so that area becomes known-free and stops
                    # emitting frontiers while the camera is still 5 m away and
                    # cannot recognise a 0.15 m source there. run18 left this
                    # room on exactly that reasoning: source B was in the FOV
                    # 13 times but never closer than 5.55 m, and was missed;
                    # run14 reached 1.49 m on the same source and found it.
                    # Require camera coverage before declaring the room proven,
                    # and spend any remaining goals closing the largest gap.
                    gap = (
                        self.room_camera_gap(map_message, robot_pose, branch)
                        if self.camera_coverage_enabled else None
                    )
                    if gap is not None:
                        uncovered, covered_fraction = gap
                        if covered_fraction < self.camera_coverage_required:
                            coverage_target = self.camera_coverage_target(
                                map_message, robot_pose, uncovered, frame
                            )
                            if coverage_target is not None:
                                rospy.loginfo(
                                    "room has no frontier left but only %.0f%% "
                                    "of its free area is camera-covered; "
                                    "closing the gap", 100.0 * covered_fraction,
                                )
                                self.transition(
                                    "NAVIGATING",
                                    "room camera coverage %.0f%%"
                                    % (100.0 * covered_fraction),
                                    coverage_target,
                                )
                                goals += 1
                                self.navigate(
                                    coverage_target,
                                    self.room_goal_timeout,
                                    allow_backout=False,
                                )
                                continue
                        if covered_fraction < self.camera_coverage_required:
                            # Coverage was NOT achieved -- there simply was no
                            # reachable way to improve it this cycle. Reporting
                            # that as proven is what let run19 close two rooms
                            # at 17% and 25% camera coverage. Leave it unproven
                            # so the existing unproven-room revival can come
                            # back, and so the floor is not declared finished
                            # on the strength of a room nobody looked at.
                            rospy.logwarn(
                                "room transaction leaving after %d goals with "
                                "only %.0f%% camera coverage and no reachable "
                                "way to improve it; NOT marking it proven",
                                goals, 100.0 * covered_fraction,
                            )
                            self.last_room_transaction_proven = False
                            break
                        rospy.loginfo(
                            "room transaction complete after %d goals: no "
                            "reachable frontier remains and %.0f%% of the "
                            "room is camera-covered",
                            goals, 100.0 * covered_fraction,
                        )
                    else:
                        rospy.loginfo(
                            "room transaction complete after %d goals: no "
                            "reachable frontier remains inside the room", goals,
                        )
                    self.last_room_transaction_proven = True
                    break
                continue
            goals += 1
            self.transition(
                "NAVIGATING",
                "room transaction goal %d: length=%.2f m score=%.2f"
                % (goals, chosen.length_m, chosen.score),
                target,
            )
            # A room goal is worth far less than a corridor goal and is far
            # more likely to be an unreachable sliver beside furniture, so it
            # gets a short leash and no backout retry. Measured in competition
            # run10: three fragments scoring 0.42-1.13 each consumed about two
            # minutes of wall time in abort/backout/retry cycles and the room
            # still ended with "coverage unproven". Failing fast and trying the
            # next fragment covers more of the room in the same budget.
            succeeded, action_state, recordable = self.navigate(
                target, self.room_goal_timeout, allow_backout=False
            )
            if succeeded:
                self.visited_goals.append(
                    (target.pose.position.x, target.pose.position.y)
                )
            else:
                self.wait_for_interstitial_zero_settle()
                if recordable:
                    record_failure(
                        self.failed_goals,
                        target.pose.position.x, target.pose.position.y,
                        self.failed_radius, time.monotonic(),
                        self.failure_cooldown,
                        kind=(recordable if isinstance(recordable, str)
                              else "unreachable"),
                    )
                rospy.logwarn(
                    "room transaction goal %d failed: state=%d",
                    goals, action_state,
                )
        return self.exit_room_through_mouth(branch, frame)

    def room_transaction_frontiers(self, map_message, robot_pose, branch):
        """Frontiers restricted to the room the transaction is open on."""
        component = self.room_free_component_mask(
            map_message, branch, robot_pose
        )
        if component is None:
            return None
        allowed = component
        if self.roi_enabled:
            try:
                allowed = component & self.build_roi_mask(map_message)
            except ValueError:
                return None
        if not allowed.any():
            return []
        # The room component is dilated by one cell so a frontier sitting on
        # its rim is not clipped away before it can be scored.
        candidates = extract_frontiers(
            map_message.data,
            self.grid_spec(map_message),
            robot_xy=(
                robot_pose.pose.position.x,
                robot_pose.pose.position.y,
            ),
            min_frontier_length_m=self.min_frontier_length,
            obstacle_clearance_m=self.obstacle_clearance,
            goal_standoff_m=self.goal_standoff,
            goal_search_radius_m=self.goal_search_radius,
            minimum_goal_distance_m=self.room_frontier_min_distance,
            maximum_goal_distance_m=self.max_goal_distance,
            free_threshold=self.free_threshold,
            occupied_threshold=self.occupied_threshold,
            information_gain_weight=self.information_gain_weight,
            distance_weight=self.distance_weight,
            allowed_mask=_dilate(allowed, 1),
        )
        # The transaction path used to apply the ROI and nothing else, so the
        # zone rules that keep every other path out of the lobby did not run
        # here.  History bookkeeping stays with the transaction itself.
        now = time.monotonic()
        admitted = []
        for candidate in candidates:
            ok, _cooling = self.frontier_is_admissible(
                candidate.goal_x, candidate.goal_y, now, check_history=False
            )
            if ok:
                admitted.append(candidate)
        if len(admitted) != len(candidates):
            rospy.loginfo(
                "room transaction %r: %d of %d frontiers rejected by the "
                "shared admissibility rules",
                branch, len(candidates) - len(admitted), len(candidates),
            )
        return admitted

    def room_free_component_mask(self, map_message, branch, robot_pose):
        """Known-free cells of the room the robot is standing in.

        The room is bounded by its own walls on three sides and by the doorway
        on the fourth, so a flood fill from the robot that refuses to cross the
        frozen door plane yields exactly the room.  This is derived from the
        live OccupancyGrid and the perceived doorway; no layout metadata and no
        generator constant is consulted, so it holds for any floor.
        """
        corridor_pose = self.room_branch_entry_poses.get(branch)
        room_pose = self.room_branch_interior_poses.get(branch)
        if corridor_pose is None or room_pose is None:
            return None
        spec = self.grid_spec(map_message)
        grid = np.asarray(map_message.data, dtype=np.int16).reshape(
            (spec.height, spec.width)
        )
        free = (grid >= 0) & (grid <= self.free_threshold)

        start = corridor_pose.pose.position
        finish = room_pose.pose.position
        inward_x = finish.x - start.x
        inward_y = finish.y - start.y
        inward_norm = math.hypot(inward_x, inward_y)
        if inward_norm <= 1e-6:
            return None
        inward_x /= inward_norm
        inward_y /= inward_norm

        rows, cols = np.indices((spec.height, spec.width), dtype=np.float64)
        world_x = spec.origin_x + (cols + 0.5) * spec.resolution
        world_y = spec.origin_y + (rows + 0.5) * spec.resolution
        # Distance past the door plane, measured from the corridor-side pose.
        past_door = (
            (world_x - start.x) * inward_x + (world_y - start.y) * inward_y
        )
        free &= past_door >= self.room_door_plane_margin

        # The door plane is perpendicular to the corridor, so "past the door"
        # is a half-plane that is UNBOUNDED along the corridor axis: every
        # known-free cell on the room's side, all the way back to the entrance,
        # qualifies as long as it is 4-connected to the robot.  The docstring
        # above assumes the room is walled on three sides, but §2 of the
        # handoff is explicit that this building has no room doors and the
        # "doorway" is an entrance-plane abstraction, so an open-plan side of
        # the floor lets the fill run the whole length of the building.
        #
        # The exclusions in select_from_frontiers were never applied on this
        # path -- room_transaction_frontiers intersects with the ROI and
        # nothing else -- so a transaction could legally target the lobby.
        # Measured live in return_harness01: while the transaction on the
        # station-9 right room was open, the robot was driven to world
        # (2.240, 1.011), i.e. entry-frame longitudinal 4.2 m / lateral 2.2 m,
        # which is inside the elevator-shaft footprint x in [1.65, 4.05],
        # y in [1.25, 3.95], where the global planner then could not produce
        # any plan and the robot stood still.  §5 records a 20->60 degree fall
        # in that same zone, and run13 fell there with peak roll 42.9 deg.
        # Apply the lobby rule to the component itself.
        if self.floor_entry_pose is not None:
            entry_pose = self.floor_entry_pose.pose
            entry_yaw = yaw_from_quaternion(entry_pose.orientation)
            entry_cos = math.cos(entry_yaw)
            entry_sin = math.sin(entry_yaw)
            delta_x = world_x - entry_pose.position.x
            delta_y = world_y - entry_pose.position.y
            entry_longitudinal = delta_x * entry_cos + delta_y * entry_sin
            entry_lateral = -delta_x * entry_sin + delta_y * entry_cos
            free &= ~(
                (np.abs(entry_lateral) >= self.room_lateral_threshold)
                & (entry_longitudinal < self.room_minimum_door_longitudinal)
            )
            # The decisive cut. The door plane is placed at the corridor-side
            # pose, which sits INSIDE the 2.2 m corridor, so the far half of the
            # corridor survives it and becomes a highway that connects every
            # room on that side of the floor. Measured by the diagnostic below
            # in return_harness02: branch (10, 1) produced a 15678-cell
            # "room" spanning longitudinal -0.22..22.05 m -- the whole
            # building -- entered through the corridor sliver at lateral
            # 0.36..1.1 m. A room begins past the corridor band, so remove the
            # band itself; room_lateral_threshold is the same corridor-band
            # parameter the selection filters already use, not a new constant.
            free &= np.abs(entry_lateral) >= self.room_lateral_threshold
            # Bound the room along the corridor axis using the neighbouring
            # door stations on the SAME side. The door-plane cut is
            # perpendicular to the corridor and therefore says nothing about
            # how far the room runs along it, so a component could swallow the
            # whole side of the floor. Measured: branch (19,-1) spanned
            # longitudinal 7.57..35.25 m -- both right-hand rooms at once --
            # while (10,1) and (19,1) came out correctly separated at 7.8..21.3
            # and 21.8..35.3, i.e. the leak appears exactly where a dividing
            # wall is momentarily missing from the map. Perceived doorways are
            # the only room delimiters available, so cut halfway to the nearest
            # detected doorway on each side along the axis; with no neighbour
            # the bound stays open and behaviour is unchanged.
            own_longitudinal, own_lateral = self.entry_coordinates(
                start.x, start.y
            )
            with self.lock:
                neighbours = list(self.remembered_room_doorways.values())
            neighbour_coordinates = [
                self.entry_coordinates(other.center.x, other.center.y)
                for other in neighbours
            ]
            lower, upper = room_axis_bounds(
                own_longitudinal,
                own_lateral,
                neighbour_coordinates,
                self.room_station_separation,
            )
            if lower is not None:
                free &= entry_longitudinal >= lower
            if upper is not None:
                free &= entry_longitudinal <= upper
            if lower is not None or upper is not None:
                rospy.loginfo(
                    "room %r bounded along the corridor by neighbouring door "
                    "stations: longitudinal %s..%s m",
                    branch,
                    "open" if lower is None else "%.2f" % lower,
                    "open" if upper is None else "%.2f" % upper,
                )

        seed = spec.world_to_cell(
            robot_pose.pose.position.x, robot_pose.pose.position.y
        )
        if seed is None or not free[seed]:
            seed = None
            anchor = nearest_known_free_anchor(
                map_message.data,
                spec,
                (
                    robot_pose.pose.position.x,
                    robot_pose.pose.position.y,
                ),
                self.entry_anchor_search_radius,
                self.free_threshold,
                free,
            )
            if anchor is not None:
                seed = spec.world_to_cell(*anchor)
        if seed is None or not free[seed]:
            return None

        component = np.zeros(free.shape, dtype=bool)
        remaining = free.copy()
        queue = deque([seed])
        remaining[seed] = False
        while queue:
            row, col = queue.popleft()
            component[row, col] = True
            for delta_row, delta_col in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                next_row = row + delta_row
                next_col = col + delta_col
                if (
                        0 <= next_row < spec.height
                        and 0 <= next_col < spec.width
                        and remaining[next_row, next_col]):
                    remaining[next_row, next_col] = False
                    queue.append((next_row, next_col))
        # The component is what the transaction treats as "this room". Report
        # its entry-frame extent so a leak past the room is visible in the log
        # instead of having to be inferred from where the robot ended up.
        if self.floor_entry_pose is not None and component.any():
            component_longitudinal = entry_longitudinal[component]
            component_lateral = entry_lateral[component]
            rospy.loginfo(
                "room component %r: %d cells, longitudinal %.2f..%.2f m, "
                "lateral %.2f..%.2f m",
                branch,
                int(component.sum()),
                float(component_longitudinal.min()),
                float(component_longitudinal.max()),
                float(component_lateral.min()),
                float(component_lateral.max()),
            )
        return component

    def exit_room_through_mouth(self, branch, frame):
        """Leave through the doorway this transaction was opened on."""
        doorway = self.room_branch_entry_poses.get(branch)
        if doorway is None:
            rospy.logwarn("room branch %r has no recorded entry pose", branch)
            return False
        current_pose = self.pose_in_frame(frame)
        exit_pose = copy.deepcopy(doorway)
        exit_pose.header.stamp = rospy.Time.now()
        exit_yaw = math.atan2(
            exit_pose.pose.position.y - current_pose.pose.position.y,
            exit_pose.pose.position.x - current_pose.pose.position.x,
        )
        exit_pose.pose.orientation.x, exit_pose.pose.orientation.y, \
            exit_pose.pose.orientation.z, exit_pose.pose.orientation.w = \
            quaternion_from_yaw(exit_yaw)
        succeeded, state, _recordable = self.navigate(
            exit_pose, self.navigation_timeout
        )
        if not succeeded:
            rospy.logwarn(
                "room branch exit failed: branch=%r move_base state=%d",
                branch, state,
            )
            return False
        return self.face_corridor_forward(frame)

    def frontier_is_admissible(self, x, y, now, check_history=True):
        """The single admissibility gate every goal-producing path must use.

        Four defects of the same shape were fixed one at a time before this
        existed, all of them "a guard exists but another path does not run it":
        the relaxed selection pass dropped the doorway requirement, the mapping
        recovery bypassed its own identity guard, and the room transaction ran
        neither the lobby rule nor the corridor-band cut -- which is how a
        transaction on a room 13 m away drove the robot to the entrance-side
        lobby at world (2.240, 1.011) and left it there.  The exclusions used to
        be inline filters inside select_from_frontiers, so every new path
        started out with none of them and only got them when somebody
        remembered.  Routing all paths through here makes forgetting impossible.

        Zone rules always apply.  ``check_history`` additionally applies the
        visited/failed-goal bookkeeping, which the room transaction keeps its
        own version of and therefore opts out of.

        Returns ``(admissible, cooling)``; ``cooling`` means the goal is only
        temporarily barred and the caller should not declare the floor finished.

        Deliberately NOT routed through here: corridor_probe_target().  A probe
        target is a point on the traversal spine rather than a frontier, it sits
        at essentially zero lateral offset so the lobby rule can never fire on
        it, and the entry-transit exclusion would reject the first few steps out
        of the entrance and stall the traversal before it starts.  It keeps its
        own guards (ROI, known-free clearance, visited, failed/attempt history,
        and make_plan reachability).
        """
        if self.is_entry_transit_frontier(x, y):
            return False, False
        if self.floor_entry_pose is not None:
            longitudinal, lateral = self.entry_coordinates(x, y)
            lateral_room = abs(lateral) >= self.room_lateral_threshold
            # The lobby holds the stair core and the elevator shaft and the
            # generator places no source outside a room, so a lateral opening
            # this close to the entrance can only cost travel -- and it is where
            # the 20->60 degree tilt excursions happened.
            if (
                    lateral_room
                    and longitudinal < self.room_minimum_door_longitudinal):
                return False, False
            if (
                    lateral_room
                    and self.room_branch_key(longitudinal, lateral)
                    in self.completed_room_branches):
                return False, False
        if not check_history:
            return True, False
        if point_near(self.visited_goals, x, y, self.visited_radius):
            return False, False
        state = failed_goal_state(
            self.failed_goals, x, y, self.failed_radius, now,
            self.maximum_failures,
        )
        if state == "permanent":
            return False, False
        if state == "cooldown":
            return False, True
        return True, False

    def select_from_frontiers(
            self, frame, robot_pose, ordered_frontiers, now, cooling,
            strict_room_filters):
        """Pick the first eligible frontier from an already ordered list.

        ``strict_room_filters`` applies the room-priority classification: side
        openings near the entrance junction are skipped, axial progress is
        monotonic, and a lateral frontier must match a perceived doorway.  With
        it disabled only the evidence-based exclusions remain, which is the
        fallback used before declaring the floor finished.
        """
        for frontier in ordered_frontiers:
            # Tiny negative-utility fragments arise next to the robot after a
            # room scan.  They add no observable area, but can require a
            # collision-invalid turn and trigger repeated backout recovery.
            # Wait for the next map/frontier instead of moving backwards for
            # a target that cannot advance exploration.
            if frontier.score < self.minimum_frontier_score:
                continue
            # Furniture is mapped as a thin shell -- the occupancy grid only
            # marks surfaces the LiDAR struck, so every object leaves a small
            # unknown pocket that keeps emitting low-value frontiers. Chasing
            # those matters inside a room, where a danger source can be hiding
            # behind the furniture, and is pure travel in the corridor, where
            # the generator places no sources at all. Hold corridor-band
            # fragments to a higher bar; lateral frontiers are left alone
            # because an undetected room mouth appears exactly there.
            if self.floor_entry_pose is not None:
                _longitudinal, band_lateral = self.entry_coordinates(
                    frontier.goal_x, frontier.goal_y
                )
                if (
                        abs(band_lateral) < self.room_lateral_threshold
                        and frontier.score
                        < self.corridor_minimum_frontier_score):
                    continue
            if self.room_priority_enabled and self.floor_entry_pose is not None:
                longitudinal, lateral = self.entry_coordinates(
                    frontier.goal_x, frontier.goal_y
                )
                # The relaxed pass exists because room_lateral_threshold (1.0 m)
                # is narrower than the main corridor's own half-width: a
                # corridor whose centreline is offset from the spawn line --
                # 1.27 m in integration_fix18 -- had its frontiers classified as
                # room frontiers, found no doorway, and was dropped, which is
                # what made exploration stop at 55.6%.  It used to fix that by
                # dropping the doorway requirement altogether, and that is what
                # sent run11 at unconfirmed lateral openings twice
                # ("no frontier survived the room-priority filters" at
                # (10.56, -2.56) and (17.09, -4.59)) instead of at the second
                # room.  Widen the corridor band instead: the misclassified
                # corridor frontier is then simply corridor, and a frontier
                # still outside the widened band is a genuine room mouth that
                # must match a perceived doorway in either pass.
                lateral_threshold = (
                    self.room_lateral_threshold if strict_room_filters
                    else self.relaxed_lateral_threshold
                )
                if (
                        strict_room_filters
                        and abs(lateral) < lateral_threshold
                        and longitudinal
                        < self.maximum_corridor_progress - 0.75):
                    continue
                if (
                        abs(lateral) >= lateral_threshold
                        and self.matching_room_doorway(frontier) is None):
                    continue
            # Every evidence-based exclusion now lives in one predicate that
            # all goal-producing paths share; see frontier_is_admissible().
            admissible, goal_cooling = self.frontier_is_admissible(
                frontier.goal_x, frontier.goal_y, now
            )
            if goal_cooling:
                cooling = True
            if not admissible:
                continue
            target = self.pose_for_frontier(frame, frontier)
            branch = None
            corridor_pose = None
            if self.room_priority_enabled and self.floor_entry_pose is not None:
                longitudinal, lateral = self.entry_coordinates(
                    frontier.goal_x, frontier.goal_y
                )
                doorway = (
                    self.matching_room_doorway(frontier)
                    if abs(lateral) >= self.room_lateral_threshold
                    else None
                )
                if doorway is not None:
                    branch, corridor_pose, target = \
                        self.room_target_from_doorway(frame, doorway)
                    corridor_pose, target = \
                        self.frozen_room_branch_geometry(
                            branch, corridor_pose, target
                        )
            reachable = self.path_exists(robot_pose, target)
            if reachable is None:
                return None, None, cooling, True
            if reachable:
                self.selected_room_branch = branch
                self.selected_room_stage = (
                    "room_interior" if branch is not None else None
                )
                return target, frontier, cooling, False
            failure = record_failure(
                self.failed_goals,
                frontier.goal_x,
                frontier.goal_y,
                self.failed_radius,
                now,
                self.failure_cooldown,
            )
            rospy.logwarn(
                "frontier make_plan rejected (%.2f, %.2f), failure %d/%d",
                frontier.goal_x,
                frontier.goal_y,
                failure.failures,
                self.maximum_failures,
            )
            if failure.failures < self.maximum_failures:
                cooling = True
        return None, None, cooling, False

    def camera_coverage_target(self, map_message, robot_pose, uncovered, frame):
        """A reachable standing pose that closes the nearest camera gap.

        Nearest-first rather than largest-first on purpose: the floor budget is
        600 s for three floors and one floor already costs ~250 s, so coverage
        has to be bought with the least travel that still gets the camera onto
        the gap. Standing one radius short of the gap centre means the wedge
        covers it without walking all the way in.
        """
        spec = self.grid_spec(map_message)
        rows, cols = np.nonzero(uncovered)
        if rows.size == 0:
            return None
        origin = robot_pose.pose.position
        world_x = spec.origin_x + (cols + 0.5) * spec.resolution
        world_y = spec.origin_y + (rows + 0.5) * spec.resolution
        distances = np.hypot(world_x - origin.x, world_y - origin.y)
        for index in np.argsort(distances)[:self.camera_coverage_candidates]:
            gap_x = float(world_x[index])
            gap_y = float(world_y[index])
            span = float(distances[index])
            if span <= 1e-3:
                continue
            # Stop short of the gap by one coverage radius, but never behind
            # the robot's current position.
            standoff = max(0.0, span - 0.75 * self.camera_coverage_radius)
            stand_x = origin.x + (gap_x - origin.x) * (standoff / span)
            stand_y = origin.y + (gap_y - origin.y) * (standoff / span)
            anchor = nearest_known_free_anchor(
                map_message.data, spec, (stand_x, stand_y),
                self.goal_search_radius, self.free_threshold,
            )
            if anchor is None:
                continue
            target = PoseStamped()
            target.header.frame_id = frame
            target.header.stamp = rospy.Time.now()
            target.pose.position.x = anchor[0]
            target.pose.position.y = anchor[1]
            heading = math.atan2(gap_y - anchor[1], gap_x - anchor[0])
            (target.pose.orientation.x, target.pose.orientation.y,
             target.pose.orientation.z, target.pose.orientation.w) = \
                quaternion_from_yaw(heading)
            if point_near(self.visited_goals, anchor[0], anchor[1],
                          self.visited_radius * 0.5):
                continue
            reachable = self.path_exists(robot_pose, target)
            if reachable:
                return target
        return None

    def mark_camera_coverage(self, map_message, robot_pose):
        """Mark grid cells the camera can currently recognise a source in.

        Marched as rays across the horizontal FOV rather than tested per cell:
        a per-cell test with a per-cell raycast is O(r^2 * r) and at a 4.5 m
        radius that is millions of Python operations per call, which would
        stall the transaction loop. Rays stop at the first occupied cell, so
        the blind pocket behind furniture stays uncovered -- which is exactly
        where the generator hides sources, and the whole point of this mask.
        """
        if not self.camera_coverage_enabled:
            return
        spec = self.grid_spec(map_message)
        if (
                self.camera_covered is None
                or self.camera_covered.shape != (spec.height, spec.width)):
            self.camera_covered = np.zeros(
                (spec.height, spec.width), dtype=bool)
        grid = np.asarray(map_message.data, dtype=np.int16).reshape(
            (spec.height, spec.width)
        )
        origin = robot_pose.pose.position
        yaw = yaw_from_quaternion(robot_pose.pose.orientation)
        step = spec.resolution * 0.5
        steps = max(1, int(self.camera_coverage_radius / step))
        # Angular spacing fine enough that neighbouring rays stay within half a
        # cell of each other at maximum range, so the wedge has no gaps.
        angular_step = spec.resolution / (2.0 * self.camera_coverage_radius)
        count = max(1, int(2.0 * self.camera_coverage_half_angle / angular_step))
        for index in range(count + 1):
            angle = (
                yaw - self.camera_coverage_half_angle
                + index * (2.0 * self.camera_coverage_half_angle / count)
            )
            cosine, sine = math.cos(angle), math.sin(angle)
            for distance_index in range(1, steps + 1):
                distance = distance_index * step
                cell = spec.world_to_cell(
                    origin.x + distance * cosine, origin.y + distance * sine
                )
                if cell is None:
                    break
                if grid[cell] >= self.occupied_threshold:
                    break
                self.camera_covered[cell] = True

    def room_camera_gap(self, map_message, robot_pose, branch):
        """Free cells of the room the camera has not yet been close enough to.

        Returns ``(uncovered_mask, covered_fraction)`` or ``None``.
        """
        component = self.room_free_component_mask(
            map_message, branch, robot_pose
        )
        if component is None:
            return None
        spec = self.grid_spec(map_message)
        grid = np.asarray(map_message.data, dtype=np.int16).reshape(
            (spec.height, spec.width)
        )
        free_room = component & (grid >= 0) & (grid <= self.free_threshold)
        if self.roi_enabled:
            try:
                free_room &= self.build_roi_mask(map_message)
            except ValueError:
                return None
        total = int(free_room.sum())
        if total == 0:
            return None
        if self.camera_covered is None or self.camera_covered.shape != free_room.shape:
            return free_room, 0.0
        uncovered = free_room & ~self.camera_covered
        return uncovered, 1.0 - float(uncovered.sum()) / float(total)

    def record_corridor_probe_outcome(self):
        """Retire the corridor probe once it stops revealing new ROI cells."""
        before = self.corridor_probe_known_before
        self.corridor_probe_known_before = None
        if before is None:
            return
        with self.lock:
            map_message = copy.deepcopy(self.map_message)
        if map_message is None:
            return
        try:
            gained = known_cell_count(
                map_message.data, self.build_roi_mask(map_message)
            ) - before
        except ValueError:
            return
        if gained >= self.corridor_probe_minimum_new_cells:
            self.corridor_probe_barren = 0
            return
        self.corridor_probe_barren += 1
        rospy.logwarn(
            "corridor probe revealed only %d new ROI cells (need %d); "
            "barren probe %d/%d",
            gained,
            self.corridor_probe_minimum_new_cells,
            self.corridor_probe_barren,
            self.corridor_probe_maximum_barren,
        )
        if self.corridor_probe_barren >= self.corridor_probe_maximum_barren:
            self.corridor_probe_exhausted = True
            rospy.logwarn(
                "corridor probing retired: %d consecutive probes revealed no "
                "new ROI space", self.corridor_probe_barren,
            )

    def flattened_goal_pose(pose):
        """Strip roll/pitch so move_base will accept the pose as a goal.

        MoveBase::isQuaternionValid rejects any goal whose quaternion z-axis is
        more than about 2.6 degrees off vertical, and poses taken straight from
        TF carry the standing robot's real attitude -- roughly 8-14 degrees of
        roll and pitch on this platform. Every other goal in this node is built
        with quaternion_from_yaw and is therefore already flat; the return
        anchors were copied from TF instead, so move_base rejected them before
        planning ever started. Observed in competition return_test02 as four
        "Quaternion is invalid" errors and a return that failed without the
        robot moving.
        """
        flattened = copy.deepcopy(pose)
        yaw = yaw_from_quaternion(pose.pose.orientation)
        flattened.pose.orientation.x, flattened.pose.orientation.y, \
            flattened.pose.orientation.z, flattened.pose.orientation.w = \
            quaternion_from_yaw(yaw)
        flattened.pose.position.z = 0.0
        return flattened

    def wait_for_interstitial_zero_settle(self):
        """Block goal selection/sending until commanded velocity has
        really settled at zero after a non-success navigation outcome
        (aborted, cancelled, preempted, a bounded-backout recovery
        retry, or a timeout). See
        a1_exploration.final_zero.interstitial_zero_gate for the pure
        decision rule; this method only supplies live ROS/sim-time
        samples and fails closed through the existing
        ExplorationFailure error codes -- it never invents a new one.

        This wait must never touch failed_goals or the no-frontier
        completion evidence: it is a timing gate, not a frontier
        outcome.
        """
        while not rospy.is_shutdown():
            self.check_cancel_safety_and_deadline()
            now_ros = rospy.Time.now().to_sec()
            with self.lock:
                cmd_vel_result = (
                    self.interstitial_cmd_vel_monitor.evaluate(now_ros)
                )
                nav_result = (
                    self.interstitial_cmd_vel_nav_monitor.evaluate(
                        now_ros
                    )
                )
            decision = interstitial_zero_gate(
                cmd_vel_result, nav_result,
                CMD_VEL_NAV_SILENCE_TIMEOUT_S,
            )
            if decision["allowed"]:
                rospy.loginfo(
                    "interstitial zero-settle satisfied before next "
                    "goal: %s", decision["reason"],
                )
                return
            if decision["fail_closed"]:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_PRECONDITION,
                    "interstitial zero-settle failed closed before "
                    "next goal: %s" % decision["reason"],
                )
            time.sleep(0.02)
        raise ExplorationFailure(
            ExploreFloorResult.ERROR_CANCELLED,
            "exploration goal cancelled; move_base goal stopped",
            preempted=True,
        )

    @staticmethod

    def scan_room_and_exit(self, branch, frame):
        """Scan once in the open area, then drive forward back to its mouth."""
        doorway = self.room_branch_entry_poses.get(branch)
        if doorway is None:
            rospy.logwarn("room branch %r has no recorded entry pose", branch)
            return False
        try:
            scan_pose = self.pose_in_frame(frame)
        except Exception as error:
            rospy.logwarn("room scan pose unavailable: %s", error)
            return False
        open_pose = self.open_room_scan_pose(branch, scan_pose)
        if open_pose is None:
            rospy.logwarn(
                "room scan deferred: no reachable %.2f m-clear observation "
                "point found within %.2f m",
                self.room_scan_clearance,
                self.room_scan_search_distance,
            )
            return False
        relocation = math.hypot(
            open_pose.pose.position.x - scan_pose.pose.position.x,
            open_pose.pose.position.y - scan_pose.pose.position.y,
        )
        if relocation > 0.10:
            rospy.loginfo(
                "room wall too close for rotation; moving %.2f m deeper to "
                "a clearance-verified observation point",
                relocation,
            )
            succeeded, state, _recordable = self.navigate(
                open_pose, self.navigation_timeout
            )
            if not succeeded:
                rospy.logwarn(
                    "room observation-point relocation failed: state=%d",
                    state,
                )
                return False
            scan_pose = self.pose_in_frame(frame)
        previous = yaw_from_quaternion(scan_pose.pose.orientation)
        accumulated = 0.0
        started_ros = rospy.Time.now()
        started_wall = time.monotonic()
        command = Twist()
        command.angular.z = abs(self.room_scan_angular_speed)
        while not rospy.is_shutdown() and accumulated < 2.0 * math.pi:
            self.check_cancel_safety_and_deadline()
            try:
                current = yaw_from_quaternion(
                    self.pose_in_frame(frame).pose.orientation
                )
                accumulated += abs(normalize_angle(current - previous))
                previous = current
            except Exception:
                pass
            if (
                    (rospy.Time.now() - started_ros).to_sec() >= 25.0
                    or time.monotonic() - started_wall
                    >= 25.0 * self.wall_factor):
                break
            self.recovery_cmd_pub.publish(command)
            time.sleep(0.02)
        zero = Twist()
        for _unused in range(15):
            self.recovery_cmd_pub.publish(zero)
            time.sleep(0.02)
        if accumulated < 1.75 * math.pi:
            rospy.logwarn(
                "room scan incomplete: accumulated %.2f rad", accumulated
            )
            return False

        current_pose = self.pose_in_frame(frame)
        exit_pose = copy.deepcopy(doorway)
        exit_pose.header.stamp = rospy.Time.now()
        exit_yaw = math.atan2(
            exit_pose.pose.position.y - current_pose.pose.position.y,
            exit_pose.pose.position.x - current_pose.pose.position.x,
        )
        exit_pose.pose.orientation.x, exit_pose.pose.orientation.y, \
            exit_pose.pose.orientation.z, exit_pose.pose.orientation.w = \
            quaternion_from_yaw(exit_yaw)
        rospy.loginfo(
            "room scan complete: %.2f rad; room exit phase 1/3 navigating "
            "forward to branch mouth",
            accumulated,
        )
        succeeded, state, _recordable = self.navigate(
            exit_pose, self.navigation_timeout
        )
        if not succeeded:
            rospy.logwarn(
                "room branch exit failed: branch=%r move_base state=%d",
                branch,
                state,
            )
            return False
        if not self.recenter_in_corridor(branch, frame):
            return False
        return self.face_corridor_forward(frame)

    def recenter_in_corridor(self, branch, frame):
        """Move from the room-side wall band to the main-corridor centreline.

        Turning toward the corridor axis immediately at a doorway can leave the
        quadruped travelling parallel to, and inside the inflation band of, the
        wall.  Complete the lateral transaction first; only then restore the
        remembered forward heading.
        """
        doorway = self.room_branch_entry_poses.get(branch)
        if doorway is None or self.floor_entry_pose is None:
            rospy.logwarn(
                "room exit recenter unavailable: branch=%r has no geometry",
                branch,
            )
            return False
        longitudinal, doorway_lateral = self.entry_coordinates(
            doorway.pose.position.x, doorway.pose.position.y
        )
        entry = self.floor_entry_pose.pose
        corridor_yaw = yaw_from_quaternion(entry.orientation)
        target = PoseStamped()
        target.header.frame_id = frame
        target.header.stamp = rospy.Time.now()
        current = self.pose_in_frame(frame)
        corridor_model = self.estimate_corridor_model(current)
        with self.lock:
            source_door = copy.deepcopy(
                self.remembered_room_doorways.get(branch)
            )
            remembered_doorways = copy.deepcopy(
                self.remembered_room_doorways
            )
        opposite_door = None
        opposite_branch = None
        if source_door is not None:
            source_longitudinal, _source_lateral = self.entry_coordinates(
                source_door.center.x, source_door.center.y
            )
            pair_candidates = []
            for candidate_branch, candidate_door \
                    in remembered_doorways.items():
                if candidate_branch[1] == branch[1]:
                    continue
                candidate_longitudinal, _candidate_lateral = \
                    self.entry_coordinates(
                        candidate_door.center.x, candidate_door.center.y
                    )
                error = abs(
                    candidate_longitudinal - source_longitudinal
                )
                if error <= self.room_door_station_tolerance:
                    pair_candidates.append(
                        (error, candidate_branch, candidate_door)
                    )
            if pair_candidates:
                _error, opposite_branch, opposite_door = min(
                    pair_candidates, key=lambda item: item[0]
                )
        if corridor_model is not None:
            current_longitudinal, current_lateral = self.entry_coordinates(
                current.pose.position.x, current.pose.position.y
            )
            travel_sign = (
                1.0
                if corridor_model.center_lateral >= current_lateral
                else -1.0
            )
            # move_base reports success anywhere inside xy_goal_tolerance.  A
            # goal exactly on the centreline therefore stopped the robot about
            # 0.35 m short in testing. Command a bounded point beyond the
            # centre so the tolerance band is centred on the measured line.
            available_overshoot = max(
                0.0,
                0.5 * corridor_model.width
                - self.room_exit_center_clearance
                - 0.05,
            )
            overshoot = min(
                self.navigation_xy_goal_tolerance,
                available_overshoot,
            )
            commanded_lateral = (
                corridor_model.center_lateral + travel_sign * overshoot
            )
            target = self.corridor_point(
                current_longitudinal,
                commanded_lateral,
                frame,
            )
            center_source = (
                "live dual-wall model ids=%d/%d width=%.2f conf=%.2f "
                "measured_lateral=%.2f commanded_lateral=%.2f overshoot=%.2f"
                % (
                    corridor_model.left_id,
                    corridor_model.right_id,
                    corridor_model.width,
                    corridor_model.confidence,
                    corridor_model.center_lateral,
                    commanded_lateral,
                    overshoot,
                )
            )
        elif source_door is not None and opposite_door is not None:
            target.pose.position.x = 0.5 * (
                source_door.center.x + opposite_door.center.x
            )
            target.pose.position.y = 0.5 * (
                source_door.center.y + opposite_door.center.y
            )
            center_source = (
                "paired-door midpoint ids=%d/%d branches=%r/%r"
            ) % (
                source_door.detection_id,
                opposite_door.detection_id,
                branch,
                opposite_branch,
            )
        else:
            # A single doorway cannot determine the transverse corridor
            # midpoint. The frozen corridor-side observation pose is still
            # local measured geometry and is safer than projecting the remote
            # public-entry axis through a possibly offset main corridor.
            target.pose.position = copy.deepcopy(doorway.pose.position)
            center_source = "degraded frozen-corridor-pose fallback"

        lateral_yaw = math.atan2(
            target.pose.position.y - current.pose.position.y,
            target.pose.position.x - current.pose.position.x,
        )
        target.pose.orientation.x, target.pose.orientation.y, \
            target.pose.orientation.z, target.pose.orientation.w = \
            quaternion_from_yaw(lateral_yaw)
        distance = math.hypot(
            target.pose.position.x - current.pose.position.x,
            target.pose.position.y - current.pose.position.y,
        )
        with self.lock:
            map_message = copy.deepcopy(self.map_message)
        if (
                map_message is None
                or map_message.header.frame_id != frame
                or not self.known_free_clearance(
                    map_message,
                    target.pose.position.x,
                    target.pose.position.y,
                    self.room_exit_center_clearance,
                )):
            rospy.logwarn(
                "room exit recenter rejected: station=%d side=%s "
                "centre=(%.2f, %.2f) lacks %.2f m known-free clearance",
                branch[0],
                "left" if branch[1] > 0 else "right",
                target.pose.position.x,
                target.pose.position.y,
                self.room_exit_center_clearance,
            )
            return False
        reachable = self.path_exists(current, target)
        if not reachable:
            rospy.logwarn(
                "room exit recenter rejected: station=%d side=%s "
                "no known-space plan from doorway lateral=%.2f m",
                branch[0],
                "left" if branch[1] > 0 else "right",
                doorway_lateral,
            )
            return False
        rospy.loginfo(
            "room exit phase 2/3: recentering %.2f m from doorway band "
            "to corridor centre=(%.2f, %.2f), station=%d side=%s source=%s",
            distance,
            target.pose.position.x,
            target.pose.position.y,
            branch[0],
            "left" if branch[1] > 0 else "right",
            center_source,
        )
        self.publish_target(target, "room_exit_center_target")
        succeeded, state, _recordable = self.navigate(
            target, self.navigation_timeout
        )
        if not succeeded:
            rospy.logwarn(
                "room exit recenter failed: branch=%r move_base state=%d",
                branch,
                state,
            )
            return False
        reached = self.pose_in_frame(frame)
        _reached_longitudinal, reached_lateral = self.entry_coordinates(
            reached.pose.position.x, reached.pose.position.y
        )
        verified_model = self.estimate_corridor_model(reached)
        if verified_model is None:
            rospy.logwarn(
                "room exit recenter cannot be verified: fresh dual-wall model "
                "is unavailable after motion"
            )
            return False
        center_error = reached_lateral - verified_model.center_lateral
        left_clearance = verified_model.left_lateral - reached_lateral
        right_clearance = reached_lateral - verified_model.right_lateral
        if abs(center_error) > self.corridor_center_tolerance:
            rospy.logwarn(
                "room exit recenter rejected by wall verification: "
                "center_error=%+.3f m tolerance=%.3f m left=%.3f m "
                "right=%.3f m width=%.3f m conf=%.2f",
                center_error,
                self.corridor_center_tolerance,
                left_clearance,
                right_clearance,
                verified_model.width,
                verified_model.confidence,
            )
            return False
        rospy.loginfo(
            "room exit phase 2/3 wall-verified: center_error=%+.3f m "
            "left=%.3f m right=%.3f m width=%.3f m conf=%.2f; phase 3/3 "
            "restoring forward heading",
            center_error,
            left_clearance,
            right_clearance,
            verified_model.width,
            verified_model.confidence,
        )
        return True

    def settle_corridor_center_and_heading(self, frame):
        """Wall-verify a corridor target, correct residual offset, then face forward.

        move_base may report success anywhere inside xy_goal_tolerance.  When a
        robot approaches the centre laterally this can leave it short of the
        measured centre and still facing the opposite wall.  Use fresh dual-wall
        geometry to close the remaining lateral error before collecting the next
        forward observation.
        """
        maximum_attempts = 3
        for attempt in range(1, maximum_attempts + 1):
            current = self.pose_in_frame(frame)
            model = self.estimate_corridor_model(current)
            if model is None:
                rospy.logwarn(
                    "corridor target settle deferred: no fresh dual-wall model"
                )
                return False
            current_longitudinal, current_lateral = self.entry_coordinates(
                current.pose.position.x, current.pose.position.y
            )
            center_error = current_lateral - model.center_lateral
            if abs(center_error) <= self.corridor_center_tolerance:
                rospy.loginfo(
                    "corridor target wall-verified: center_error=%+.3f m "
                    "tolerance=%.3f m width=%.3f m conf=%.2f",
                    center_error,
                    self.corridor_center_tolerance,
                    model.width,
                    model.confidence,
                )
                return self.face_corridor_forward(frame)

            travel_sign = 1.0 if model.center_lateral >= current_lateral else -1.0
            available_overshoot = max(
                0.0,
                0.5 * model.width - self.room_exit_center_clearance - 0.05,
            )
            overshoot = min(
                self.navigation_xy_goal_tolerance,
                available_overshoot,
            )
            commanded_lateral = model.center_lateral + travel_sign * overshoot
            correction = self.corridor_point(
                current_longitudinal, commanded_lateral, frame
            )
            lateral_yaw = math.atan2(
                correction.pose.position.y - current.pose.position.y,
                correction.pose.position.x - current.pose.position.x,
            )
            correction.pose.orientation.x, correction.pose.orientation.y, \
                correction.pose.orientation.z, correction.pose.orientation.w = \
                quaternion_from_yaw(lateral_yaw)
            with self.lock:
                map_message = copy.deepcopy(self.map_message)
            if (
                    map_message is None
                    or map_message.header.frame_id != frame
                    or not self.known_free_clearance(
                        map_message,
                        correction.pose.position.x,
                        correction.pose.position.y,
                        self.room_exit_center_clearance,
                    )
                    or not self.path_exists(current, correction)):
                rospy.logwarn(
                    "corridor target correction rejected: attempt=%d/%d "
                    "center_error=%+.3f m target_lateral=%.3f m",
                    attempt,
                    maximum_attempts,
                    center_error,
                    commanded_lateral,
                )
                return False
            rospy.loginfo(
                "corridor target correction %d/%d: center_error=%+.3f m; "
                "measured_lateral=%.3f commanded_lateral=%.3f overshoot=%.3f",
                attempt,
                maximum_attempts,
                center_error,
                model.center_lateral,
                commanded_lateral,
                overshoot,
            )
            self.publish_target(correction, "corridor_center_correction")
            succeeded, state, _recordable = self.navigate(
                correction, self.navigation_timeout
            )
            if not succeeded:
                rospy.logwarn(
                    "corridor target correction failed: attempt=%d state=%d",
                    attempt,
                    state,
                )
                return False
        rospy.logwarn(
            "corridor target remained outside center tolerance after %d attempts",
            maximum_attempts,
        )
        return False

    def face_corridor_forward(self, frame):
        """Restore the remembered main-corridor heading after leaving a room."""
        corridor_yaw = yaw_from_quaternion(
            self.floor_entry_pose.pose.orientation
        )
        started_ros = rospy.Time.now()
        started_wall = time.monotonic()
        zero = Twist()
        aligned = False
        try:
            while not rospy.is_shutdown():
                self.check_cancel_safety_and_deadline()
                current = self.pose_in_frame(frame)
                current_yaw = yaw_from_quaternion(
                    current.pose.orientation
                )
                error = normalize_angle(corridor_yaw - current_yaw)
                if abs(error) <= self.room_exit_align_tolerance:
                    aligned = True
                    break
                ros_elapsed = (rospy.Time.now() - started_ros).to_sec()
                wall_elapsed = time.monotonic() - started_wall
                if (
                        ros_elapsed >= self.room_exit_align_timeout
                        or wall_elapsed >=
                        self.room_exit_align_timeout * self.wall_factor):
                    rospy.logwarn(
                        "corridor-forward alignment timed out: error=%.2f rad",
                        error,
                    )
                    return False
                command = Twist()
                magnitude = min(
                    self.room_scan_angular_speed,
                    max(0.55, 1.8 * abs(error)),
                )
                command.angular.z = math.copysign(magnitude, error)
                self.recovery_cmd_pub.publish(command)
                time.sleep(0.02)
        finally:
            for _unused in range(15):
                self.recovery_cmd_pub.publish(zero)
                time.sleep(0.02)
        if not aligned:
            return False

        observation_start_ros = rospy.Time.now()
        observation_start_wall = time.monotonic()
        while not rospy.is_shutdown():
            self.check_cancel_safety_and_deadline()
            if (
                    (rospy.Time.now() - observation_start_ros).to_sec()
                    >= self.room_exit_reobserve_time):
                break
            if (
                    time.monotonic() - observation_start_wall
                    >= self.room_exit_reobserve_time * self.wall_factor):
                break
            self.recovery_cmd_pub.publish(zero)
            time.sleep(0.02)
        rospy.loginfo(
            "room exit aligned with remembered corridor heading: yaw=%.2f "
            "rad; fresh forward Livox observation collected",
            corridor_yaw,
        )
        return True

    def cancel_move_goal(self, wait_wall=2.0):
        """Cancel this client's goal and briefly wait for a terminal state."""
        terminal_states = (
            GoalStatus.PREEMPTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        )
        self.move_client.cancel_goal()
        deadline = time.monotonic() + wait_wall
        state = self.move_client.get_state()
        while (
            not rospy.is_shutdown()
            and state not in terminal_states
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)
            state = self.move_client.get_state()
        return state

    def wait_for_map_update(self, previous_version):
        deadline = time.monotonic() + self.map_update_wait
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.check_cancel_safety_and_deadline()
            with self.lock:
                if (
                    self.map_message is not None
                    and self.map_version(self.map_message) != previous_version
                ):
                    return
            time.sleep(0.05)

    def register_no_goal_confirmation(self, version):
        return self.no_goal_evidence.observe(
            version, rospy.Time.now().to_sec()
        )

    def reset_no_goal_confirmations(self):
        self.no_goal_evidence.reset()

    def verify_return_pose(self):
        current = self.pose_in_frame(self.start_pose.header.frame_id)
        position_error = math.hypot(
            current.pose.position.x - self.start_pose.pose.position.x,
            current.pose.position.y - self.start_pose.pose.position.y,
        )
        yaw_error = abs(
            angle_difference(
                yaw_from_quaternion(current.pose.orientation),
                yaw_from_quaternion(self.start_pose.pose.orientation),
            )
        )
        return position_error, yaw_error

    def wait_for_final_zero(self):
        deadline = time.monotonic() + self.final_zero_timeout
        last_result = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.check_cancel_safety_and_deadline()
            now_ros = rospy.Time.now().to_sec()
            with self.lock:
                last_result = self.final_zero_monitor.evaluate(now_ros)
            if last_result["ready"]:
                rospy.loginfo(
                    "final /cmd_vel is fresh (age %.3f ROS s) and has been "
                    "zero for %.3f ROS s",
                    last_result["message_age"],
                    last_result["zero_duration"],
                )
                return True
            if "clock" in last_result["reason"]:
                rospy.logerr(
                    "final /cmd_vel verification failed closed: %s",
                    last_result["reason"],
                )
                return False
            time.sleep(0.02)
        if last_result is None:
            last_result = {
                "reason": "no verification sample",
                "message_age": float("inf"),
                "zero_duration": 0.0,
                "values": (float("nan"),) * 3,
            }
        rospy.logwarn(
            "final /cmd_vel did not settle in ROS time: reason=%s, "
            "zero_duration=%.3f, last=(%.3f, %.3f, %.3f), age=%.3f, "
            "required=%.2f",
            last_result["reason"],
            last_result["zero_duration"],
            last_result["values"][0],
            last_result["values"][1],
            last_result["values"][2],
            last_result["message_age"],
            self.zero_settle,
        )
        return False

    def execute_return(self):
        self.transition(
            "RETURNING",
            "returning through the verified floor-entry corridor",
            self.floor_entry_pose,
        )
        self.publish_target(self.start_pose, "return_target")
        last_errors = (float("inf"), float("inf"))
        # A single plan from the back of the building to the outdoor spawn is
        # brittle with an incrementally built map: the entrance threshold can
        # remain an unknown/disconnected seam even though the robot traversed
        # it successfully on entry.  Return first to the already validated
        # indoor entry anchor, then reverse the same short corridor to spawn.
        stages = (
            ("indoor entry anchor", self.floor_entry_pose, False),
            ("outdoor start pose", self.start_pose, True),
        )
        for stage_name, target, use_entry_profile in stages:
            self.transition(
                "RETURNING", "returning to %s" % stage_name, target
            )
            stage_succeeded = False
            for attempt in range(1, self.return_attempts + 1):
                if use_entry_profile:
                    self.apply_entry_speed_limit()
                try:
                    succeeded, action_state, _recordable = self.navigate(
                        target, self.return_timeout, returning=True
                    )
                finally:
                    if use_entry_profile:
                        self.restore_entry_speed_limit()
                if succeeded:
                    stage_succeeded = True
                    break
                rospy.logwarn(
                    "return %s attempt %d/%d failed: action_state=%d",
                    stage_name, attempt, self.return_attempts, action_state,
                )
            if not stage_succeeded:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_RETURN_FAILED,
                    "return failed before reaching %s: move_base state=%d"
                    % (stage_name, action_state),
                )

        last_errors = self.verify_return_pose()
        if (
                last_errors[0] <= self.return_position_tolerance
                and last_errors[1] <= self.return_yaw_tolerance
                and self.wait_for_final_zero()):
            return last_errors
        raise ExplorationFailure(
            ExploreFloorResult.ERROR_RETURN_FAILED,
            "return failed: position_error=%.3f m, yaw_error=%.3f rad, "
            "or final /cmd_vel did not settle to zero" % last_errors,
        )

    def reset_action_state(self, goal):
        with self.lock:
            self.floor_id = goal.floor_id
            self.active_corridor_model_minimum_longitudinal = (
                0.0 if goal.entry_mode == goal.ALREADY_AT_FLOOR_ENTRY
                else self.corridor_model_minimum_longitudinal
            )
            self.action_active = True
            self.action_identity = None
            self.start_pose = None
            self.floor_entry_pose = None
            self.roi_local = ()
            self.roi_polygon_map = ()
            self.visited_goals = []
            self.failed_goals = []
            self.completed_room_branches = set()
            self.configure_floor_completion()
            self.approached_room_branches = set()
            self.room_branch_entry_poses = {}
            self.room_branch_interior_poses = {}
            self.active_room_branch = None
            self.selected_room_branch = None
            self.selected_room_stage = None
            self.selected_corridor_center_target = False
            self.remembered_room_doorways = {}
            self.corridor_model = None
            self.maximum_corridor_progress = 0.0
            self.no_goal_evidence = NoFrontierEvidence(
                self.empty_confirmations,
                self.stable_no_frontier_duration,
            )
            self.make_plan_failure_since_wall = None
            self.current_target = PoseStamped()
            self.trajectory = Path()
            self.coverage = (
                coverage_ratio(self.map_message.data)
                if self.map_message is not None else 0.0
            )
            self.final_zero_monitor.reset()
            self.interstitial_cmd_vel_monitor.reset()
            self.interstitial_cmd_vel_nav_monitor.reset()
        if goal.timeout_s > 0.0:
            self.action_ros_deadline = (
                rospy.Time.now() + rospy.Duration(goal.timeout_s)
            )
            self.action_wall_deadline = (
                time.monotonic() + goal.timeout_s * self.wall_factor
            )
        else:
            self.action_ros_deadline = None
            self.action_wall_deadline = None
        self.trajectory_pub.publish(Path())
        empty_scope = Marker()
        empty_scope.action = Marker.DELETE
        empty_scope.ns = "single_floor_roi"
        empty_scope.id = 0
        self.scope_pub.publish(empty_scope)
        self.roi_pub.publish(PolygonStamped())
        clear_corridor = Marker()
        clear_corridor.action = Marker.DELETEALL
        self.corridor_model_pub.publish(MarkerArray(markers=[clear_corridor]))

    def execute(self, goal):
        self.reset_action_state(goal)
        result = ExploreFloorResult()
        try:
            try:
                if goal.floor_entry_pose.header.frame_id:
                    validate_floor_entry_pose(goal.floor_entry_pose)
                elif self.require_explicit_entry_pose:
                    raise InvalidEntryPose(
                        "floor_entry_pose is required; RECORD_START is "
                        "only the outdoor return pose"
                    )
            except InvalidEntryPose as error:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_INVALID_ENTRY_POSE,
                    "invalid floor_entry_pose: %s" % error,
                )
            self.transition(
                "RECORD_START",
                "waiting for healthy map and recording start pose",
            )
            self.wait_for_prerequisites()
            with self.lock:
                self.action_identity = self.mapping_identity(
                    self.mapping_status, self.map_message
                )
                map_frame = self.map_message.header.frame_id
            self.start_pose = self.pose_in_frame(map_frame)
            self.start_pose.header.stamp = rospy.Time.now()
            try:
                self.establish_roi(goal, map_frame)
                with self.lock:
                    initial_map = copy.deepcopy(self.map_message)
                allowed = self.build_roi_mask(initial_map)
            except InvalidEntryPose as error:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_INVALID_ENTRY_POSE,
                    "invalid floor_entry_pose: %s" % error,
                )
            except ValueError as error:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_PRECONDITION,
                    "invalid single-floor exploration ROI: %s" % error,
                )
            with self.lock:
                self.coverage = coverage_ratio(
                    initial_map.data, allowed
                )
            self.trajectory.header.frame_id = map_frame
            self.trajectory.header.stamp = rospy.Time.now()
            self.trajectory.poses = [copy.deepcopy(self.start_pose)]
            self.trajectory_pub.publish(self.trajectory)
            rospy.loginfo(
                "recorded exploration start: frame=%s x=%.3f y=%.3f "
                "identity=%r",
                map_frame,
                self.start_pose.pose.position.x,
                self.start_pose.pose.position.y,
                self.action_identity,
            )
            self.publish_roi(initial_map)
            if self.roi_enabled:
                rospy.loginfo(
                    "single-floor ROI: entry frame=%s x=%.2f y=%.2f, "
                    "%d local vertices, boundary_margin=%.2f m; "
                    "coverage denominator=%d OccupancyGrid cell centers",
                    self.floor_entry_pose.header.frame_id,
                    self.floor_entry_pose.pose.position.x,
                    self.floor_entry_pose.pose.position.y,
                    len(self.roi_local),
                    self.roi_boundary_margin,
                    int(allowed.sum()),
                )

            if goal.entry_mode == goal.LEGACY_MAIN_ENTRANCE:
                self.transition(
                    "REQUEST_ENTRY_DOOR_OPEN",
                    "requesting the public main entrance door and checking response",
                )
                self.request_entry_door_open(goal)
                self.reset_map_after_entry_door_opens()
                door_open_map = self.wait_for_entry_passage(
                    initial_map, self.start_pose
                )

                # A closed public door fills the wall gap, so the structural
                # detector cannot measure a doorway center before the control
                # service opens it.  Lock the LiDAR geometry after opening but
                # before any entry motion, then rebuild both the ROI mask and
                # coverage denominator around the corrected axis.
                rospy.loginfo(
                    "entry preflight: locking the open public entrance center "
                    "and inward axis from stable LiDAR doorway geometry"
                )
                self.align_entry_to_scanned_door(goal)
                with self.lock:
                    aligned_map = copy.deepcopy(self.map_message)
                allowed = self.build_roi_mask(aligned_map)
                with self.lock:
                    self.coverage = coverage_ratio(
                        aligned_map.data, allowed
                    )
                self.publish_roi(aligned_map)

                self.floor_entry_pose.header.stamp = rospy.Time.now()
                self.publish_target(self.floor_entry_pose, "floor_entry_target")
                self.transition(
                    "TRANSIT_TO_ENTRY",
                    "crossing the entrance with localization and a continuously "
                    "refreshed short-horizon obstacle gate",
                    self.floor_entry_pose,
                )
                self.controlled_entry_transit()

                self.transition(
                    "ENTERED_FLOOR",
                    "checking entry pose and new post-entry OccupancyGrid evidence",
                    self.floor_entry_pose,
                )
                self.wait_for_entered_floor(door_open_map)

                # The elevator is beside the entrance lobby.  Give the
                # structural doorway detector a deliberate side-looking scan
                # before the mandatory corridor ingress, and publish an
                # explicit acquisition gate so the mission manager can never
                # confuse later room doors with the elevator.
                entered_pose = self.pose_in_frame(map_frame)
                entrance_yaw = yaw_from_quaternion(
                    self.floor_entry_pose.pose.orientation)
                scan_pose = copy.deepcopy(entered_pose)
                scan_pose.header.stamp = rospy.Time.now()
                scan_pose.pose.position.x += math.cos(entrance_yaw) * 2.5
                scan_pose.pose.position.y += math.sin(entrance_yaw) * 2.5
                (scan_pose.pose.orientation.x,
                 scan_pose.pose.orientation.y,
                 scan_pose.pose.orientation.z,
                 scan_pose.pose.orientation.w) = quaternion_from_yaw(
                    entrance_yaw)
                self.publish_target(scan_pose, "elevator_scan_waiting_point")
                self.transition(
                    "NAVIGATING",
                    "moving 2.5 m beyond the entrance before elevator scan",
                    scan_pose,
                )
                succeeded, state, _recordable = self.navigate(
                    scan_pose, self.navigation_timeout)
                if not succeeded:
                    raise ExplorationFailure(
                        ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                        "failed to reach 2.5 m post-entry elevator scan point "
                        "state=%d" % state,
                    )

                # Only observations acquired after the robot is fully clear
                # of the public door are eligible elevator landmarks.
                scan_pose = self.pose_in_frame(map_frame)
                scan_yaw = entrance_yaw - 0.5 * math.pi
                side_scan = copy.deepcopy(scan_pose)
                side_scan.header.stamp = rospy.Time.now()
                (side_scan.pose.orientation.x,
                 side_scan.pose.orientation.y,
                 side_scan.pose.orientation.z,
                 side_scan.pose.orientation.w) = quaternion_from_yaw(scan_yaw)
                rospy.set_param(
                    "/frontier_explorer/runtime/elevator_scan_active", True)
                try:
                    self.transition(
                        "NAVIGATING",
                        "active entrance-lobby scan toward elevator side",
                        side_scan,
                    )
                    succeeded, state, _recordable = self.navigate(
                        side_scan, self.navigation_timeout)
                    if not succeeded:
                        raise ExplorationFailure(
                            ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                            "entrance-lobby elevator scan rotation failed "
                            "state=%d" % state,
                        )
                    self.refine_scan_heading(map_frame, scan_yaw)
                    rospy.sleep(3.0)
                    forward_scan = copy.deepcopy(scan_pose)
                    forward_scan.header.stamp = rospy.Time.now()
                    self.transition(
                        "NAVIGATING",
                        "restoring entrance heading after elevator scan",
                        forward_scan,
                    )
                    succeeded, state, _recordable = self.navigate(
                        forward_scan, self.navigation_timeout)
                    if not succeeded:
                        raise ExplorationFailure(
                            ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                            "entrance heading restore failed state=%d" % state,
                        )
                finally:
                    rospy.set_param(
                        "/frontier_explorer/runtime/elevator_scan_active", False)
            elif goal.entry_mode == goal.ALREADY_AT_FLOOR_ENTRY:
                self.floor_entry_pose.header.stamp = rospy.Time.now()
                self.publish_target(self.floor_entry_pose, "floor_entry_target")
                current = self.pose_in_frame(map_frame)
                distance = math.hypot(
                    current.pose.position.x - self.floor_entry_pose.pose.position.x,
                    current.pose.position.y - self.floor_entry_pose.pose.position.y,
                )
                if distance > self.elevator_anchor_tolerance:
                    raise ExplorationFailure(
                        ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                        "elevator entry transaction ended %.2f m from the "
                        "declared main-corridor anchor" % distance,
                    )
                # The declared anchor is where the elevator exit route aimed;
                # the robot is wherever move_base stopped, up to its own
                # xy_goal_tolerance. Anchor the ROI on the measured pose so
                # entry-frame coordinates describe the robot's real position,
                # then correct the axis, which the elevator chain cannot carry.
                self.floor_entry_pose.pose.position.x = current.pose.position.x
                self.floor_entry_pose.pose.position.y = current.pose.position.y
                self.rebuild_roi_from_entry_pose()
                self.align_entry_axis_to_walls(map_frame)
                self.floor_entry_pose.header.stamp = rospy.Time.now()
                self.publish_target(self.floor_entry_pose, "floor_entry_target")
                self.transition(
                    "ENTERED_FLOOR",
                    "elevator exit verified; starting exploration at the "
                    "measured main-corridor anchor",
                    self.floor_entry_pose,
                )
            else:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_PRECONDITION,
                    "unsupported entry_mode=%d" % goal.entry_mode,
                )
            with self.lock:
                entered_map = copy.deepcopy(self.map_message)
            allowed = self.build_roi_mask(entered_map)
            with self.lock:
                self.coverage = coverage_ratio(
                    entered_map.data, allowed
                )
            self.publish_roi(entered_map)

            mandatory_ingress_completed = False
            if goal.seed_target.header.frame_id:
                if goal.seed_target.header.frame_id != map_frame:
                    raise ExplorationFailure(
                        ExploreFloorResult.ERROR_PRECONDITION,
                        "seed_target must use current map frame %s"
                        % map_frame,
                    )
                seed = copy.deepcopy(goal.seed_target)
                seed.header.stamp = rospy.Time.now()
                if not self.target_in_roi(seed):
                    raise ExplorationFailure(
                        ExploreFloorResult.ERROR_PRECONDITION,
                        "seed_target lies outside the current single-floor "
                        "exploration ROI",
                    )
                current_pose = self.pose_in_frame(map_frame)
                if self.path_exists(current_pose, seed):
                    self.publish_target(seed, "seed_target")
                    mandatory_ingress = (
                        goal.entry_mode == goal.LEGACY_MAIN_ENTRANCE
                    )
                    self.transition(
                        "NAVIGATING",
                        ("executing mandatory post-entrance main-corridor ingress"
                         if mandatory_ingress else
                         "executing optional caller-provided seed target"),
                        seed,
                    )
                    succeeded, _state, _recordable = self.navigate(
                        seed, self.navigation_timeout
                    )
                    if succeeded:
                        self.visited_goals.append(
                            (seed.pose.position.x, seed.pose.position.y)
                        )
                        mandatory_ingress_completed = mandatory_ingress
                    elif mandatory_ingress:
                        raise ExplorationFailure(
                            ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                            "mandatory post-entrance main-corridor ingress "
                            "navigation failed",
                        )
                else:
                    if goal.entry_mode == goal.LEGACY_MAIN_ENTRANCE:
                        raise ExplorationFailure(
                            ExploreFloorResult.ERROR_ENTRY_TRANSIT,
                            "mandatory post-entrance main-corridor ingress "
                            "has no known-space plan",
                        )
                    rospy.logwarn(
                        "optional seed_target has no known-space plan; "
                        "continuing automatic frontier selection"
                    )

            completion_reason = ""
            force_ingress_completion = (
                self.force_floor_complete_after_ingress
                and mandatory_ingress_completed
            )
            if force_ingress_completion:
                completion_reason = (
                    "TEST MODE: mandatory post-entrance corridor ingress "
                    "completed; bypassing room/frontier exploration to "
                    "exercise elevator perception and transfer"
                )
                rospy.logwarn(completion_reason)
            if goal.target_coverage_ratio > 0.0:
                rospy.logwarn(
                    "target_coverage_ratio=%.3f is diagnostic only; "
                    "completion still requires no reachable frontier in ROI",
                    goal.target_coverage_ratio,
                )
            while not rospy.is_shutdown() and not force_ingress_completion:
                self.check_cancel_safety_and_deadline()
                self.transition(
                    "SELECT_FRONTIER",
                    "extracting and checking reachable frontiers",
                )
                map_message, robot_pose, frontiers = self.frontier_snapshot()
                self.publish_frontiers(map_message, frontiers)
                version = self.map_version(map_message)
                target, frontier, cooling, planner_degraded = \
                    self.choose_frontier(
                    map_message, robot_pose, frontiers
                )
                if target is None:
                    if planner_degraded:
                        self.transition(
                            "UPDATE_COVERAGE",
                            "make_plan transiently unavailable; waiting for "
                            "a newer map before retry",
                        )
                        self.wait_for_map_update(version)
                        continue
                    if cooling:
                        self.transition(
                            "UPDATE_COVERAGE",
                            "frontiers are in retry cooldown; waiting for "
                            "new map evidence",
                        )
                        self.wait_for_map_update(version)
                        continue
                    evidence = self.register_no_goal_confirmation(version)
                    if evidence["complete"]:
                        completion_reason = (
                            "%s; remaining targets are visited or permanently "
                            "unreachable after valid navigation attempts"
                            % evidence["reason"]
                        )
                        break
                    self.transition(
                        "UPDATE_COVERAGE",
                        evidence["reason"],
                    )
                    self.wait_for_map_update(version)
                    continue

                self.reset_no_goal_confirmations()
                self.publish_target(target, "frontier_target")
                self.transition(
                    "NAVIGATING",
                    "frontier length=%.2f m score=%.2f"
                    % (frontier.length_m, frontier.score),
                    target,
                )
                succeeded, action_state, recordable_failure = self.navigate(
                    target, self.navigation_timeout
                )
                if succeeded:
                    completed_branch = None
                    if self.selected_corridor_center_target:
                        self.transition(
                            "NAVIGATING",
                            "frontier position reached; wall-verifying corridor "
                            "centre and restoring longitudinal heading",
                            target,
                        )
                        if not self.settle_corridor_center_and_heading(
                                target.header.frame_id):
                            self.transition(
                                "UPDATE_COVERAGE",
                                "corridor centre/heading verification failed; "
                                "waiting for fresh wall and map evidence",
                            )
                            self.wait_for_map_update(version)
                            continue
                    if (
                            self.room_priority_enabled
                            and self.floor_entry_pose is not None):
                        branch = self.selected_room_branch
                        if (
                                branch is not None
                                and self.selected_room_stage
                                == "door_center"):
                            self.approached_room_branches.add(branch)
                            rospy.loginfo(
                                "door center reached: station=%d side=%s; "
                                "next stage is room interior",
                                branch[0],
                                "left" if branch[1] > 0 else "right",
                            )
                        if (
                                branch is not None
                                and self.selected_room_stage
                                == "room_interior"):
                            actual_pose = self.pose_in_frame(
                                target.header.frame_id
                            )
                            actual_depth = self.room_interior_progress(
                                branch, actual_pose
                            )
                            planned_depth = self.room_interior_progress(
                                branch,
                                self.room_branch_interior_poses[branch],
                            )
                            required_depth = min(
                                self.room_completion_depth,
                                max(0.60, planned_depth - 0.45),
                            )
                            if (
                                    actual_depth is None
                                    or actual_depth < required_depth):
                                failure = record_failure(
                                    self.failed_goals,
                                    target.pose.position.x,
                                    target.pose.position.y,
                                    self.failed_radius,
                                    time.monotonic(),
                                    self.failure_cooldown,
                                )
                                self.transition(
                                    "UPDATE_COVERAGE",
                                    "room target reported reached before "
                                    "crossing its frozen door plane: "
                                    "depth=%.2f required=%.2f; failure %d/%d"
                                    % (
                                        -1.0 if actual_depth is None
                                        else actual_depth,
                                        required_depth,
                                        failure.failures,
                                        self.maximum_failures,
                                    ),
                                )
                                self.wait_for_map_update(version)
                                continue
                            self.transition(
                                "NAVIGATING",
                                "room interior reached through frozen door "
                                "geometry (depth=%.2f m); opening the room "
                                "transaction" % actual_depth,
                                target,
                            )
                            if not self.explore_room_transaction(
                                    branch, target.header.frame_id):
                                failure = record_failure(
                                    self.failed_goals,
                                    target.pose.position.x,
                                    target.pose.position.y,
                                    self.failed_radius,
                                    time.monotonic(),
                                    self.failure_cooldown,
                                )
                                self.transition(
                                    "UPDATE_COVERAGE",
                                    "room transaction/exit failed; failure "
                                    "%d/%d"
                                    % (
                                        failure.failures,
                                        self.maximum_failures,
                                    ),
                                )
                                self.wait_for_map_update(version)
                                continue
                            floor_complete = self.complete_room_branch(branch)
                            if self.active_room_branch == branch:
                                rospy.loginfo(
                                    "room transaction released: station=%d "
                                    "side=%s completed",
                                    branch[0],
                                    "left" if branch[1] > 0 else "right",
                                )
                                self.active_room_branch = None
                            completed_branch = branch
                            rospy.loginfo(
                                "room branch covered and exited: station=%d "
                                "side=%s depth=%.2f m",
                                branch[0],
                                "left" if branch[1] > 0 else "right",
                                actual_depth,
                            )
                            completed_count = len(self.completed_room_branches)
                            if floor_complete:
                                completion_reason = (
                                    "completed %d/%d distinct rooms on floor "
                                    "%d; fixed-layout floor completion "
                                    "criterion satisfied"
                                    % (
                                        completed_count,
                                        self.floor_completion_target,
                                        self.floor_id,
                                    )
                                )
                                rospy.loginfo(completion_reason)
                                break
                    self.visited_goals.append(
                        (target.pose.position.x, target.pose.position.y)
                    )
                    self.transition(
                        "UPDATE_COVERAGE",
                        (
                            "room branch scanned, exited, and marked complete"
                            if completed_branch is not None
                            else "frontier reached; waiting for a newer "
                            "floor map"
                        ),
                    )
                elif recordable_failure:
                    failure = record_failure(
                        self.failed_goals,
                        target.pose.position.x,
                        target.pose.position.y,
                        self.failed_radius,
                        time.monotonic(),
                        self.failure_cooldown,
                        kind=(recordable_failure
                              if isinstance(recordable_failure, str)
                              else "unreachable"),
                    )
                    self.transition(
                        "UPDATE_COVERAGE",
                        "move_base state=%d; frontier failure %d/%d"
                        % (
                            action_state,
                            failure.failures,
                            self.maximum_failures,
                        ),
                    )
                else:
                    self.transition(
                        "UPDATE_COVERAGE",
                        "move_base state=%d was cancelled/preempted; target "
                        "was not added to unreachable history" % action_state,
                    )
                self.wait_for_map_update(version)

            self.transition(
                "EXPLORATION_DONE",
                completion_reason,
            )
            if goal.completion_mode == goal.LEGACY_RETURN_TO_START:
                position_error, yaw_error = self.execute_return()
                self.transition(
                    "RETURNED",
                    "returned within %.3f m / %.3f rad and final /cmd_vel is zero"
                    % (position_error, yaw_error),
                )
            elif goal.completion_mode == goal.STAY_ON_FLOOR:
                self.move_client.cancel_goal()
                self.recovery_cmd_pub.publish(Twist())
                if not self.wait_for_final_zero():
                    raise ExplorationFailure(
                        ExploreFloorResult.ERROR_SAFETY_STOP,
                        "floor explored but final /cmd_vel did not settle",
                    )
                self.transition(
                    "RETURNED",
                    "floor explored; holding endpoint for elevator transfer",
                )
            else:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_PRECONDITION,
                    "unsupported completion_mode=%d" % goal.completion_mode,
                )
            result.success = True
            result.error_code = ExploreFloorResult.ERROR_NONE
            result.message = self.state_message
            result.final_coverage_ratio = self.coverage
            self.clear_floor_visualization()
            self.server.set_succeeded(result)
        except ExplorationFailure as failure:
            self.move_client.cancel_goal()
            self.building_behavior_client.cancel_goal()
            state = "CANCELLED" if failure.preempted else "FAILED"
            self.transition(state, str(failure))
            result.success = False
            result.error_code = failure.code
            result.message = "%s: %s" % (state, failure)
            result.final_coverage_ratio = self.coverage
            if failure.preempted:
                self.server.set_preempted(result)
            else:
                self.server.set_aborted(result)
        except Exception as error:
            self.move_client.cancel_goal()
            self.building_behavior_client.cancel_goal()
            rospy.logerr("unhandled exploration error: %s", error)
            self.transition("FAILED", "internal error: %s" % error)
            result.success = False
            result.error_code = ExploreFloorResult.ERROR_INTERNAL
            result.message = self.state_message
            result.final_coverage_ratio = self.coverage
            self.server.set_aborted(result)
        finally:
            if self.entry_speed_limiter.active:
                try:
                    self.restore_entry_speed_limit()
                except ExplorationFailure as restore_failure:
                    rospy.logfatal(
                        "ExploreFloor exit left the conservative entry speed "
                        "profile active because exact restore failed: %s",
                        restore_failure,
                    )
            with self.lock:
                self.action_active = False
                self.action_identity = None
            self.publish_status()


if __name__ == "__main__":
    rospy.init_node("a1_frontier_explorer")
    try:
        FrontierExplorer()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("a1_exploration startup failed: %s", error)
        raise
