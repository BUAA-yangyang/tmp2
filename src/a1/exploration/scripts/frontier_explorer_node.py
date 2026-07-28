#!/usr/bin/env python3
"""Single-floor frontier exploration with autonomous return.

The node never publishes velocity.  Every exploration and return target is sent
through move_base_msgs/MoveBaseAction, so the existing
move_base -> /cmd_vel_nav -> a1_cmd_mux -> /cmd_vel chain remains the only
motion path.
"""

import copy
import math
import threading
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from a1_navigation_interfaces.msg import (
    ExploreFloorAction,
    ExploreFloorFeedback,
    ExploreFloorResult,
    ExplorationStatus,
)
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
import rospy
from std_msgs.msg import Bool
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from a1_exploration.frontier import (
    GridSpec,
    coverage_ratio,
    extract_frontiers,
    failed_goal_state,
    point_in_start_aligned_scope,
    point_near,
    record_failure,
    start_aligned_scope_mask,
)


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


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
        self.mapping_status = None
        self.last_mapping_healthy_wall = 0.0
        self.final_command = None
        self.final_command_wall = 0.0
        self.safety_locked = False

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
        self.trajectory = Path()
        self.visited_goals = []
        self.failed_goals = []
        self.no_goal_versions = []
        self.make_plan_failure_since_wall = None

        self.base_frame = self.param("frames/base", "base")
        self.map_topic = self.param(
            "topics/map", "/a1/floor_mapping/map"
        )
        self.mapping_status_topic = self.param(
            "topics/mapping_status", "/a1/floor_mapping/status"
        )
        self.final_cmd_topic = self.param(
            "topics/final_cmd_vel", "/cmd_vel"
        )
        self.safety_lock_topic = self.param(
            "topics/safety_lock", "/a1_cmd_mux/safety_lock"
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
        self.explore_action_name = self.param(
            "actions/explore_floor", "/a1/exploration/explore_floor"
        )
        self.move_action_name = self.param(
            "actions/move_base", "/move_base"
        )
        self.make_plan_name = self.param(
            "services/make_plan", "/move_base/make_plan"
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

        self.scope_enabled = bool(self.param("scope/enabled", True))
        self.scope_forward_distance = float(
            self.param("scope/forward_distance", 40.0)
        )
        self.scope_rear_distance = float(
            self.param("scope/rear_distance", 1.0)
        )
        self.scope_lateral_half_width = float(
            self.param("scope/lateral_half_width", 9.0)
        )
        self.scope_boundary_margin = float(
            self.param("scope/boundary_margin", 0.35)
        )

        self.prerequisite_timeout = float(
            self.param("timeouts/prerequisites", 30.0)
        )
        self.navigation_timeout = float(
            self.param("timeouts/navigation_goal", 45.0)
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
        self.make_plan = rospy.ServiceProxy(self.make_plan_name, GetPlan)

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

        rospy.Subscriber(
            self.map_topic, OccupancyGrid, self.map_callback, queue_size=1
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
            "timeouts/prerequisites": self.prerequisite_timeout,
            "timeouts/navigation_goal": self.navigation_timeout,
            "timeouts/return_goal": self.return_timeout,
            "return/position_tolerance": self.return_position_tolerance,
            "return/yaw_tolerance": self.return_yaw_tolerance,
            "return/zero_settle_time": self.zero_settle,
            "planning/make_plan_retry_delay_wall":
                self.make_plan_retry_delay,
            "planning/make_plan_unavailable_timeout_wall":
                self.make_plan_unavailable_timeout,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("%s must be positive" % name)
        if (
            self.maximum_failures < 1
            or self.empty_confirmations < 1
            or self.minimum_free_cells < 1
            or self.make_plan_retry_attempts < 1
            or self.return_attempts < 1
        ):
            raise ValueError("integer exploration limits must be positive")
        if self.scope_enabled:
            scope_values = (
                self.scope_forward_distance,
                self.scope_rear_distance,
                self.scope_lateral_half_width,
                self.scope_boundary_margin,
            )
            if not all(math.isfinite(value) for value in scope_values):
                raise ValueError("scope geometry must be finite")
            if (
                self.scope_forward_distance <= self.scope_boundary_margin
                or self.scope_rear_distance < self.scope_boundary_margin
                or self.scope_lateral_half_width
                <= self.scope_boundary_margin
                or self.scope_boundary_margin < 0.0
            ):
                raise ValueError(
                    "scope extents must remain positive after boundary margin"
                )

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
            start_pose = copy.deepcopy(self.start_pose)
        if self.scope_enabled and start_pose is not None:
            try:
                allowed = self.build_scope_mask(message, start_pose)
                coverage = coverage_ratio(message.data, allowed)
            except ValueError as error:
                coverage = 0.0
                rospy.logerr_throttle(
                    1.0, "invalid active exploration scope: %s", error
                )
        with self.lock:
            self.map_message = message
            self.coverage = coverage

    def mapping_status_callback(self, message):
        with self.lock:
            self.mapping_status = message
            if self.mapping_usable(message):
                self.last_mapping_healthy_wall = time.monotonic()

    def final_command_callback(self, message):
        with self.lock:
            self.final_command = message
            self.final_command_wall = time.monotonic()

    def safety_callback(self, message):
        with self.lock:
            self.safety_locked = bool(message.data)
            active = self.action_active
        if message.data and active:
            self.move_client.cancel_goal()
            rospy.logerr("exploration cancelled by a1_cmd_mux safety lock")

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

    @staticmethod
    def map_version(message):
        return (
            message.header.stamp.to_nsec(),
            message.info.width,
            message.info.height,
            sum(1 for value in message.data if value >= 0),
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

    def build_scope_mask(self, map_message, start_pose):
        if not self.scope_enabled:
            return None
        if start_pose is None:
            raise ValueError("scope requires the recorded start pose")
        if (
            not map_message.header.frame_id
            or start_pose.header.frame_id != map_message.header.frame_id
        ):
            raise ValueError(
                "scope anchor frame does not match the current map frame"
            )
        position = start_pose.pose.position
        orientation = start_pose.pose.orientation
        values = (
            position.x,
            position.y,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("scope anchor pose is not finite")
        orientation_norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if orientation_norm < 1e-6:
            raise ValueError("scope anchor orientation is invalid")
        allowed = start_aligned_scope_mask(
            self.grid_spec(map_message),
            (position.x, position.y),
            yaw_from_quaternion(orientation),
            self.scope_forward_distance,
            self.scope_rear_distance,
            self.scope_lateral_half_width,
            self.scope_boundary_margin,
        )
        if not allowed.any():
            raise ValueError("scope does not overlap the current floor map")
        return allowed

    def target_in_scope(self, target):
        if not self.scope_enabled:
            return True
        if self.start_pose is None:
            return False
        if target.header.frame_id != self.start_pose.header.frame_id:
            return False
        return point_in_start_aligned_scope(
            target.pose.position.x,
            target.pose.position.y,
            (
                self.start_pose.pose.position.x,
                self.start_pose.pose.position.y,
            ),
            yaw_from_quaternion(self.start_pose.pose.orientation),
            self.scope_forward_distance,
            self.scope_rear_distance,
            self.scope_lateral_half_width,
            self.scope_boundary_margin,
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

    def wait_for_prerequisites(self):
        deadline = time.monotonic() + self.prerequisite_timeout
        service_ready = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.check_cancel_safety_and_deadline(check_mapping=False)
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
            if healthy and move_ready and service_ready:
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
            "and make_plan",
        )

    def check_cancel_safety_and_deadline(self, check_mapping=True):
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
            elif state not in ("NAVIGATING", "RETURNING"):
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
            start_pose = copy.deepcopy(self.start_pose)
        pose = self.pose_in_frame(message.header.frame_id)
        try:
            allowed = self.build_scope_mask(message, start_pose)
        except ValueError as error:
            raise ExplorationFailure(
                ExploreFloorResult.ERROR_PRECONDITION,
                "invalid single-floor exploration scope: %s" % error,
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
            minimum_goal_distance_m=self.min_goal_distance,
            maximum_goal_distance_m=self.max_goal_distance,
            free_threshold=self.free_threshold,
            occupied_threshold=self.occupied_threshold,
            information_gain_weight=self.information_gain_weight,
            distance_weight=self.distance_weight,
            allowed_mask=allowed,
        )
        self.publish_scope(message)
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

    def path_exists(self, start, target):
        last_error = None
        for attempt in range(1, self.make_plan_retry_attempts + 1):
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
        now = time.monotonic()
        cooling = False
        frame = map_message.header.frame_id
        for frontier in frontiers:
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
            reachable = self.path_exists(robot_pose, target)
            if reachable is None:
                return None, None, cooling, True
            if reachable:
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

    def publish_scope(self, map_message):
        marker = Marker()
        marker.header = copy.deepcopy(map_message.header)
        marker.header.stamp = rospy.Time.now()
        marker.ns = "single_floor_scope"
        marker.id = 0
        if not self.scope_enabled or self.start_pose is None:
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

        start_x = self.start_pose.pose.position.x
        start_y = self.start_pose.pose.position.y
        yaw = yaw_from_quaternion(self.start_pose.pose.orientation)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        rear = -self.scope_rear_distance + self.scope_boundary_margin
        forward = (
            self.scope_forward_distance - self.scope_boundary_margin
        )
        lateral = (
            self.scope_lateral_half_width - self.scope_boundary_margin
        )
        local_corners = (
            (rear, -lateral),
            (forward, -lateral),
            (forward, lateral),
            (rear, lateral),
            (rear, -lateral),
        )
        marker.points = [
            Point(
                x=start_x + cosine * longitudinal - sine * sideways,
                y=start_y + sine * longitudinal + cosine * sideways,
                z=0.04,
            )
            for longitudinal, sideways in local_corners
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
        self.check_cancel_safety_and_deadline()
        move_goal = MoveBaseGoal(target_pose=target)
        self.move_client.send_goal(move_goal)
        started_ros = rospy.Time.now()
        started_wall = time.monotonic()
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
                return state == GoalStatus.SUCCEEDED, state
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
                return False, cancelled_state
            time.sleep(0.05)
        self.cancel_move_goal()
        return False, GoalStatus.LOST

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
        if version not in self.no_goal_versions:
            self.no_goal_versions.append(version)
        return len(self.no_goal_versions) >= self.empty_confirmations

    def reset_no_goal_confirmations(self):
        self.no_goal_versions = []

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
        zero_since = None
        longest_zero = 0.0
        last_values = (float("nan"),) * 3
        last_age = float("inf")
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.check_cancel_safety_and_deadline()
            now = time.monotonic()
            with self.lock:
                command = copy.deepcopy(self.final_command)
                command_age = now - self.final_command_wall
            last_age = command_age
            if command is not None:
                last_values = (
                    command.linear.x,
                    command.linear.y,
                    command.angular.z,
                )
            fresh = command is not None and command_age <= self.command_freshness
            zero = fresh and all(
                abs(value) <= self.zero_epsilon
                for value in last_values
            )
            if zero:
                zero_since = now if zero_since is None else zero_since
                longest_zero = max(longest_zero, now - zero_since)
                if longest_zero >= self.zero_settle:
                    rospy.loginfo(
                        "final /cmd_vel settled to zero for %.2f wall seconds",
                        longest_zero,
                    )
                    return True
            else:
                zero_since = None
            time.sleep(0.02)
        rospy.logwarn(
            "final /cmd_vel did not settle: longest_zero=%.2f s, "
            "last=(%.3f, %.3f, %.3f), age=%.3f s, required=%.2f s",
            longest_zero,
            last_values[0],
            last_values[1],
            last_values[2],
            last_age,
            self.zero_settle,
        )
        return False

    def execute_return(self):
        self.transition(
            "RETURNING",
            "returning to recorded start pose",
            self.start_pose,
        )
        self.publish_target(self.start_pose, "return_target")
        last_errors = (float("inf"), float("inf"))
        for attempt in range(1, self.return_attempts + 1):
            succeeded, action_state = self.navigate(
                self.start_pose, self.return_timeout, returning=True
            )
            if succeeded:
                last_errors = self.verify_return_pose()
                if (
                    last_errors[0] <= self.return_position_tolerance
                    and last_errors[1] <= self.return_yaw_tolerance
                    and self.wait_for_final_zero()
                ):
                    return last_errors
            rospy.logwarn(
                "return attempt %d/%d failed: action_state=%d, "
                "position_error=%.3f, yaw_error=%.3f",
                attempt,
                self.return_attempts,
                action_state,
                last_errors[0],
                last_errors[1],
            )
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
            self.visited_goals = []
            self.failed_goals = []
            self.no_goal_versions = []
            self.make_plan_failure_since_wall = None
            self.current_target = PoseStamped()
            self.trajectory = Path()
            self.coverage = (
                coverage_ratio(self.map_message.data)
                if self.map_message is not None else 0.0
            )
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
        empty_scope.ns = "single_floor_scope"
        empty_scope.id = 0
        self.scope_pub.publish(empty_scope)

    def execute(self, goal):
        self.reset_action_state(goal)
        result = ExploreFloorResult()
        try:
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
                allowed = self.build_scope_mask(
                    self.map_message, self.start_pose
                )
            except ValueError as error:
                raise ExplorationFailure(
                    ExploreFloorResult.ERROR_PRECONDITION,
                    "invalid single-floor exploration scope: %s" % error,
                )
            with self.lock:
                self.coverage = coverage_ratio(
                    self.map_message.data, allowed
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
            self.publish_scope(self.map_message)
            if self.scope_enabled:
                rospy.loginfo(
                    "single-floor scope anchored at start yaw: "
                    "forward=%.2f m rear=%.2f m half_width=%.2f m "
                    "boundary_margin=%.2f m",
                    self.scope_forward_distance,
                    self.scope_rear_distance,
                    self.scope_lateral_half_width,
                    self.scope_boundary_margin,
                )

            if goal.seed_target.header.frame_id:
                if goal.seed_target.header.frame_id != map_frame:
                    raise ExplorationFailure(
                        ExploreFloorResult.ERROR_PRECONDITION,
                        "seed_target must use current map frame %s"
                        % map_frame,
                    )
                seed = copy.deepcopy(goal.seed_target)
                seed.header.stamp = rospy.Time.now()
                if not self.target_in_scope(seed):
                    raise ExplorationFailure(
                        ExploreFloorResult.ERROR_PRECONDITION,
                        "seed_target lies outside the current single-floor "
                        "exploration scope",
                    )
                if self.path_exists(self.start_pose, seed):
                    self.publish_target(seed, "seed_target")
                    self.transition(
                        "NAVIGATING",
                        "executing optional caller-provided seed target",
                        seed,
                    )
                    succeeded, _state = self.navigate(
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
            while not rospy.is_shutdown():
                self.check_cancel_safety_and_deadline()
                if (
                    goal.target_coverage_ratio > 0.0
                    and self.coverage >= goal.target_coverage_ratio
                ):
                    completion_reason = (
                        "explicit known-grid coverage target %.3f reached"
                        % goal.target_coverage_ratio
                    )
                    break
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
                    confirmed = self.register_no_goal_confirmation(version)
                    if confirmed:
                        completion_reason = (
                            "no eligible frontier on %d distinct map updates; "
                            "remaining targets are visited or unreachable"
                            % self.empty_confirmations
                        )
                        break
                    self.transition(
                        "UPDATE_COVERAGE",
                        "no eligible frontier on map update %d/%d"
                        % (
                            len(self.no_goal_versions),
                            self.empty_confirmations,
                        ),
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
                succeeded, action_state = self.navigate(
                    target, self.navigation_timeout
                )
                if succeeded:
                    self.visited_goals.append(
                        (target.pose.position.x, target.pose.position.y)
                    )
                    self.transition(
                        "UPDATE_COVERAGE",
                        "frontier reached; waiting for a newer floor map",
                    )
                else:
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
            rospy.logerr("unhandled exploration error: %s", error)
            self.transition("FAILED", "internal error: %s" % error)
            result.success = False
            result.error_code = ExploreFloorResult.ERROR_INTERNAL
            result.message = self.state_message
            result.final_coverage_ratio = self.coverage
            self.server.set_aborted(result)
        finally:
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
