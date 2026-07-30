#!/usr/bin/env python3
"""Single-floor frontier exploration with autonomous return.

Normal exploration and return targets use MoveBaseAction. Explicit room scans
and bounded recovery publish desired body velocity only through the higher
priority behavior input of a1_cmd_mux; State_RL remains the sole joint-level
locomotion controller.
"""

import copy
import math
import threading
import time
from types import SimpleNamespace

import actionlib
from actionlib_msgs.msg import GoalStatus
from a1_navigation_interfaces.msg import (
    DoorwayArray,
    ExploreFloorAction,
    ExploreFloorFeedback,
    ExploreFloorResult,
    ExplorationStatus,
    SpecialBehaviorAction,
    SpecialBehaviorGoal,
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
    coverage_ratio,
    extract_frontiers,
    failed_goal_state,
    has_pending_retry,
    known_cell_count,
    known_free_path_exists,
    local_plan_is_acceptable,
    nearest_known_free_anchor,
    occupancy_content_fingerprint,
    point_in_polygon,
    point_near,
    polygon_mask,
    record_failure,
    segment_corridor_mask,
    transform_local_polygon,
)
from a1_exploration.final_zero import FinalZeroMonitor
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
        self.doorway_message = None
        self.remembered_room_doorways = {}
        self.mapping_status = None
        self.last_mapping_healthy_wall = 0.0
        self.final_command = None
        self.safety_locked = False
        self.controller_ready = False
        self.controller_ready_stamp = None

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
        self.approached_room_branches = set()
        self.room_branch_entry_poses = {}
        self.room_branch_interior_poses = {}
        self.selected_room_branch = None
        self.selected_room_stage = None
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
        self.final_cmd_topic = self.param(
            "topics/final_cmd_vel", "/cmd_vel"
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
        self.roi_boundary_margin = float(
            self.param("roi/boundary_margin", 0.35)
        )
        self.roi_map_boundary_margin = float(
            self.param("roi/map_boundary_margin", 8.0)
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
            # Validation and zero-area rejection are shared with runtime goals.
            transform_local_polygon(self.default_roi_local, (0.0, 0.0), 0.0)

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

    def final_command_callback(self, message):
        stamp = rospy.Time.now().to_sec()
        with self.lock:
            self.final_command = message
            self.final_zero_monitor.observe(
                stamp,
                (message.linear.x, message.linear.y, message.angular.z),
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
        minimum_x = spec.origin_x + self.roi_map_boundary_margin
        minimum_y = spec.origin_y + self.roi_map_boundary_margin
        maximum_x = (
            spec.origin_x + spec.width * spec.resolution
            - self.roi_map_boundary_margin
        )
        maximum_y = (
            spec.origin_y + spec.height * spec.resolution
            - self.roi_map_boundary_margin
        )
        outside = [
            (x, y)
            for x, y in self.roi_polygon_map
            if (
                x < minimum_x
                or x > maximum_x
                or y < minimum_y
                or y > maximum_y
            )
        ]
        if outside:
            raise ValueError(
                "ROI is not fully contained in OccupancyGrid with %.2f m "
                "sensor margin; map=[%.2f, %.2f]x[%.2f, %.2f], "
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
        return allowed

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
            local_points = self.default_roi_local
        yaw = yaw_from_quaternion(entry.pose.orientation)
        world_points = transform_local_polygon(
            local_points,
            (entry.pose.position.x, entry.pose.position.y),
            yaw,
        )
        self.floor_entry_pose = entry
        self.roi_local = local_points
        self.roi_polygon_map = world_points

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
        base = max(robot_longitudinal, self.maximum_corridor_progress)
        entry = self.floor_entry_pose.pose
        yaw = yaw_from_quaternion(entry.orientation)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        steps = [
            self.corridor_probe_step,
            self.corridor_probe_step * 0.75,
            self.corridor_probe_step * 0.50,
            self.corridor_probe_step * 0.35,
            self.corridor_probe_step * 0.25,
        ]
        for distance in steps:
            if distance < 0.70:
                continue
            longitudinal = base + distance
            target = PoseStamped()
            target.header.frame_id = map_message.header.frame_id
            target.header.stamp = rospy.Time.now()
            # Preserve the currently occupied, map-verified corridor lane for
            # the first forward observation.  The entry axis is not guaranteed
            # to coincide with the corridor centreline; snapping lateral=0
            # after leaving a room can point the robot diagonally into a wall.
            target.pose.position.x = (
                entry.position.x
                + longitudinal * cosine
                - robot_lateral * sine
            )
            target.pose.position.y = (
                entry.position.y
                + longitudinal * sine
                + robot_lateral * cosine
            )
            target.pose.orientation.x, target.pose.orientation.y, \
                target.pose.orientation.z, target.pose.orientation.w = \
                quaternion_from_yaw(yaw)
            if not self.target_in_roi(target):
                continue
            if not self.known_free_clearance(
                    map_message,
                    target.pose.position.x,
                    target.pose.position.y,
                    self.corridor_probe_clearance):
                continue
            if point_near(
                    self.visited_goals,
                    target.pose.position.x,
                    target.pose.position.y,
                    self.visited_radius):
                continue
            reachable = self.path_exists(robot_pose, target)
            if reachable is None:
                return None, True
            if reachable:
                rospy.loginfo(
                    "no eligible frontier after completed rooms; advancing "
                    "through map-verified main-corridor free space by %.2f m "
                    "to longitudinal %.2f m",
                    distance,
                    longitudinal,
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
                    if (
                            longitudinal
                            > robot_longitudinal + self.room_lookahead
                            or longitudinal
                            < self.maximum_corridor_progress
                            - self.room_backtrack
                            or branch in self.completed_room_branches):
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
                        continue
                    if retry_state == "cooldown":
                        cooling = True
                        continue
                    doorway_candidates.append(
                        (
                            longitudinal,
                            0 if lateral > 0.0 else 1,
                            doorway,
                        )
                    )
            doorway_candidates.sort(key=lambda item: (item[0], item[1]))
            for _longitudinal, _side_order, doorway in doorway_candidates:
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

    def navigate(self, target, timeout, returning=False):
        # This check is intentionally adjacent to send_goal: frontier
        # extraction and make_plan must never create a race that bypasses the
        # controller-ready gate.
        self.check_cancel_safety_and_deadline()
        move_goal = MoveBaseGoal(target_pose=target)
        self.move_client.send_goal(move_goal)
        started_ros = rospy.Time.now()
        started_wall = time.monotonic()
        backout_steps = 0
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
                    move_goal.target_pose.header.stamp = rospy.Time.now()
                    self.move_client.send_goal(move_goal)
                    continue
                recordable_failure = state in (
                    GoalStatus.ABORTED,
                    GoalStatus.REJECTED,
                    GoalStatus.LOST,
                )
                return (
                    state == GoalStatus.SUCCEEDED,
                    state,
                    recordable_failure,
                )
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
                return False, cancelled_state, True
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
            "room scan complete: %.2f rad; navigating forward to branch mouth",
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
        return self.face_corridor_forward(frame)

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
            self.action_active = True
            self.action_identity = None
            self.start_pose = None
            self.floor_entry_pose = None
            self.roi_local = ()
            self.roi_polygon_map = ()
            self.visited_goals = []
            self.failed_goals = []
            self.completed_room_branches = set()
            self.approached_room_branches = set()
            self.room_branch_entry_poses = {}
            self.room_branch_interior_poses = {}
            self.selected_room_branch = None
            self.selected_room_stage = None
            self.remembered_room_doorways = {}
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

            self.transition(
                "REQUEST_ENTRY_DOOR_OPEN",
                "requesting the public main entrance door and checking response",
            )
            self.request_entry_door_open(goal)
            self.reset_map_after_entry_door_opens()
            door_open_map = self.wait_for_entry_passage(
                initial_map, self.start_pose
            )

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
            with self.lock:
                entered_map = copy.deepcopy(self.map_message)
            allowed = self.build_roi_mask(entered_map)
            with self.lock:
                self.coverage = coverage_ratio(
                    entered_map.data, allowed
                )
            self.publish_roi(entered_map)

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
                    self.transition(
                        "NAVIGATING",
                        "executing optional caller-provided seed target",
                        seed,
                    )
                    succeeded, _state, _recordable = self.navigate(
                        seed, self.navigation_timeout
                    )
                    if succeeded:
                        self.visited_goals.append(
                            (seed.pose.position.x, seed.pose.position.y)
                        )
                else:
                    rospy.logwarn(
                        "optional seed_target has no known-space plan; "
                        "continuing automatic frontier selection"
                    )

            completion_reason = ""
            if goal.target_coverage_ratio > 0.0:
                rospy.logwarn(
                    "target_coverage_ratio=%.3f is diagnostic only; "
                    "completion still requires no reachable frontier in ROI",
                    goal.target_coverage_ratio,
                )
            while not rospy.is_shutdown():
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
                                "geometry (depth=%.2f m); scanning 360 "
                                "degrees then returning through its branch "
                                "mouth" % actual_depth,
                                target,
                            )
                            if not self.scan_room_and_exit(
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
                                    "room scan/forward exit failed; failure "
                                    "%d/%d"
                                    % (
                                        failure.failures,
                                        self.maximum_failures,
                                    ),
                                )
                                self.wait_for_map_update(version)
                                continue
                            self.completed_room_branches.add(branch)
                            completed_branch = branch
                            rospy.loginfo(
                                "room branch scanned and exited: station=%d "
                                "side=%s depth=%.2f m",
                                branch[0],
                                "left" if branch[1] > 0 else "right",
                                actual_depth,
                            )
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
            position_error, yaw_error = self.execute_return()
            self.transition(
                "RETURNED",
                "returned within %.3f m / %.3f rad and final /cmd_vel is zero"
                % (position_error, yaw_error),
            )
            result.success = True
            result.error_code = ExploreFloorResult.ERROR_NONE
            result.message = self.state_message
            result.final_coverage_ratio = self.coverage
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
