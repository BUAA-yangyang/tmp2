#!/usr/bin/env python3
"""Bounded/full Gazebo acceptance for the production single-floor mission.

The harness is DEV-ONLY. It uses Gazebo truth only through the existing,
clearly-labelled localization substitute. Door and entry evidence come
exclusively from the public door action and the Livox-derived OccupancyGrid.
"""

import copy
import json
import math
import os
import tempfile
import threading
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from a1_building_behavior.public_scene import (
    load_public_scene,
    resolve_entry_door,
)
from a1_exploration.frontier import (
    GridSpec,
    coverage_ratio,
    known_cell_count,
    known_free_path_exists,
    nearest_known_free_anchor,
    point_in_polygon,
    polygon_mask,
    transform_local_polygon,
)
from a1_navigation_interfaces.msg import (
    ExploreFloorAction,
    ExploreFloorFeedback,
    ExploreFloorGoal,
    ExploreFloorResult,
    SpecialBehaviorActionResult,
)
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Point32, PoseStamped, Twist, WrenchStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rosgraph
import rospy
from sensor_msgs.msg import Imu, JointState, Joy
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import tf2_ros


FOOT_TOPICS = (
    "/visual/FR_foot_contact/the_force",
    "/visual/FL_foot_contact/the_force",
    "/visual/RR_foot_contact/the_force",
    "/visual/RL_foot_contact/the_force",
)

JOY_TOPIC = "/joy"
MOTOR_COMMAND_TOPICS = tuple(
    "/a1_gazebo/%s_%s_controller/command" % (leg, joint)
    for leg in ("FR", "FL", "RR", "RL")
    for joint in ("hip", "thigh", "calf")
)
CONTROLLER_MESSAGE_MAX_AGE = 0.35
DEV_TF_MAX_AGE = 0.25
DEFAULT_BAG_RECORDER_NODE = "/single_floor_acceptance_bag"
REQUIRED_BAG_TOPICS = (
    "/clock",
    "/a1/exploration/explore_floor/goal",
    "/move_base/goal",
)
SAFE_STAND_FOOT_FORCE_MIN_N = 5.0
SAFE_STAND_FOOT_FORCE_MAX_AGE = 0.20
SAFE_STAND_MAX_TILT_RAD = 0.20
SAFE_STAND_MAX_GYRO_RAD_S = 0.15
SAFE_STAND_GYRO_FILTER_TAU_S = 0.10
SAFE_STAND_GYRO_FILTER_MAX_STEP_S = 0.05
# The explicit FixedStand target settles at about 0.256 m in the low-RTF A1
# simulation. Keep a clear gap above the folded-on-ground height (~0.12 m)
# without requiring the taller dynamic-trotting stance (~0.35 m).
FIXED_STAND_MIN_BASE_HEIGHT_M = 0.24
FIXED_STAND_MAX_TILT_RAD = 0.10
FIXED_STAND_MAX_GYRO_RAD_S = 0.15
FIXED_STAND_MAX_JOINT_SPEED_RAD_S = 0.50
FIXED_STAND_STATE_MAX_AGE_S = 0.20
FIXED_STAND_STABLE_DURATION_S = 0.50


class AcceptanceFailure(RuntimeError):
    pass


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def quaternion_roll_pitch(quaternion):
    sin_roll = 2.0 * (
        quaternion.w * quaternion.x
        + quaternion.y * quaternion.z
    )
    cos_roll = 1.0 - 2.0 * (
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
    )
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (
        quaternion.w * quaternion.y
        - quaternion.z * quaternion.x
    )
    pitch = (
        math.copysign(math.pi / 2.0, sin_pitch)
        if abs(sin_pitch) >= 1.0
        else math.asin(sin_pitch)
    )
    return roll, pitch


def angle_error(lhs, rhs):
    return abs(math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs)))


def controller_graph_diagnostic(system_state, resolve_name=None):
    """Return the node-name-independent A1 controller candidate sets."""
    resolve = resolve_name or (lambda name: name)
    publishers, subscribers, _services = system_state

    def topic_nodes(entries):
        result = {}
        for topic, nodes in entries:
            resolved_topic = resolve(topic)
            result.setdefault(resolved_topic, set()).update(
                resolve(node) for node in nodes
            )
        return result

    publisher_map = topic_nodes(publishers)
    subscriber_map = topic_nodes(subscribers)
    joy_subscribers = subscriber_map.get(resolve(JOY_TOPIC), set())
    motor_topic_publishers = {
        resolve(topic): publisher_map.get(resolve(topic), set())
        for topic in MOTOR_COMMAND_TOPICS
    }
    motor_publishers = set().union(
        *motor_topic_publishers.values()
    )
    candidates = joy_subscribers & motor_publishers
    coverage = {
        node: sorted(
            topic for topic, nodes in motor_topic_publishers.items()
            if node in nodes
        )
        for node in motor_publishers
    }
    return {
        "joy_subscribers": sorted(joy_subscribers),
        "motor_publishers": sorted(motor_publishers),
        "intersection": sorted(candidates),
        "motor_topic_publishers": {
            topic: sorted(nodes)
            for topic, nodes in motor_topic_publishers.items()
        },
        "motor_topics_by_publisher": coverage,
    }


def bag_recorder_graph_diagnostic(
        system_state, recorder_node=DEFAULT_BAG_RECORDER_NODE,
        required_topics=REQUIRED_BAG_TOPICS, resolve_name=None):
    """Prove that the named recorder is subscribed before any goal is sent."""
    resolve = resolve_name or (lambda name: name)
    _publishers, subscribers, _services = system_state
    recorder = resolve(recorder_node)
    subscribers_by_topic = {}
    for topic, nodes in subscribers:
        resolved_topic = resolve(topic)
        subscribers_by_topic.setdefault(resolved_topic, set()).update(
            resolve(node) for node in nodes
        )
    observed = {
        resolve(topic): sorted(
            subscribers_by_topic.get(resolve(topic), set())
        )
        for topic in required_topics
    }
    missing = sorted(
        topic for topic, nodes in observed.items()
        if recorder not in nodes
    )
    return {
        "ready": not missing,
        "recorder_node": recorder,
        "required_topics": sorted(observed),
        "subscribers_by_topic": observed,
        "missing_topics": missing,
    }


def message_is_fresh(now, stamp, maximum_age):
    return (
        stamp is not None
        and 0.0 <= now - stamp <= maximum_age
    )


def finite_vector(values):
    return all(math.isfinite(value) for value in values)


def quaternion_from_yaw(yaw):
    if not math.isfinite(yaw):
        raise ValueError("yaw must be finite")
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class SafeStandGyroFilter:
    """Python mirror of the production safe-stand gyro evidence filter."""

    def __init__(
            self, time_constant=SAFE_STAND_GYRO_FILTER_TAU_S,
            maximum_step=SAFE_STAND_GYRO_FILTER_MAX_STEP_S):
        self.time_constant = float(time_constant)
        self.maximum_step = float(maximum_step)
        self.reset()

    def reset(self):
        self.initialized = False
        self.stamp = 0.0
        self.value = [0.0, 0.0, 0.0]

    def update(self, stamp, sample):
        sample = [float(value) for value in sample]
        if (
                not math.isfinite(stamp)
                or not math.isfinite(self.time_constant)
                or not math.isfinite(self.maximum_step)
                or self.time_constant <= 0.0
                or self.maximum_step <= 0.0
                or not finite_vector(sample)):
            self.reset()
            return {
                "valid": False,
                "discontinuity": True,
                "norm": float("inf"),
            }
        if not self.initialized:
            self.value = sample
            self.stamp = stamp
            self.initialized = True
            return {
                "valid": True,
                "discontinuity": False,
                "norm": math.sqrt(sum(value * value for value in self.value)),
            }
        step = stamp - self.stamp
        if step < 0.0 or step > self.maximum_step:
            self.value = sample
            self.stamp = stamp
            return {
                "valid": True,
                "discontinuity": True,
                "norm": math.sqrt(sum(value * value for value in self.value)),
            }
        if step > 0.0:
            alpha = -math.expm1(-step / self.time_constant)
            self.value = [
                value + alpha * (target - value)
                for value, target in zip(self.value, sample)
            ]
            self.stamp = stamp
        return {
            "valid": True,
            "discontinuity": False,
            "norm": math.sqrt(sum(value * value for value in self.value)),
        }


def dev_tf_sample_evidence(
        now, stamp, translation, rotation,
        maximum_age=DEV_TF_MAX_AGE):
    quaternion_norm = math.sqrt(sum(value * value for value in rotation))
    finite = finite_vector(translation) and finite_vector(rotation)
    fresh = message_is_fresh(now, stamp, maximum_age)
    return {
        "ready": bool(finite and fresh and quaternion_norm > 1e-6),
        "stamp": stamp,
        "age_sim_s": None if stamp is None else now - stamp,
        "fresh": fresh,
        "finite": finite,
        "quaternion_norm": quaternion_norm,
        "translation": list(translation),
        "rotation": list(rotation),
    }


def safe_stand_edge_evidence(
        now, foot_forces, foot_stamps, imu, filtered_gyro):
    force_ages = [
        None if stamp is None else now - stamp for stamp in foot_stamps
    ]
    force_fresh = [
        message_is_fresh(now, stamp, SAFE_STAND_FOOT_FORCE_MAX_AGE)
        for stamp in foot_stamps
    ]
    raw_gyro_norm = (
        None if imu is None
        else math.sqrt(sum(value * value for value in imu[2]))
    )
    return {
        "sim_time": now,
        "foot_forces_n": list(foot_forces),
        "foot_force_age_sim_s": force_ages,
        "foot_force_fresh": force_fresh,
        "roll_rad": None if imu is None else imu[0],
        "pitch_rad": None if imu is None else imu[1],
        "raw_gyro_norm_rad_s": raw_gyro_norm,
        "filtered_gyro": copy.deepcopy(filtered_gyro),
    }


def safe_stand_edge_is_valid(evidence):
    if evidence is None:
        return False
    forces = evidence["foot_forces_n"]
    filtered = evidence["filtered_gyro"]
    roll = evidence["roll_rad"]
    pitch = evidence["pitch_rad"]
    return (
        all(evidence["foot_force_fresh"])
        and all(
            force is not None and math.isfinite(force)
            and force >= SAFE_STAND_FOOT_FORCE_MIN_N
            for force in forces
        )
        and roll is not None
        and pitch is not None
        and math.isfinite(roll)
        and math.isfinite(pitch)
        and abs(roll) <= SAFE_STAND_MAX_TILT_RAD
        and abs(pitch) <= SAFE_STAND_MAX_TILT_RAD
        and filtered is not None
        and filtered["valid"]
        and not filtered["discontinuity"]
        and math.isfinite(filtered["norm"])
        and filtered["norm"] <= SAFE_STAND_MAX_GYRO_RAD_S
    )


def fixed_stand_evidence(
        now, odom, odom_stamp, joint_velocity, joint_stamp,
        foot_forces, foot_stamps, imu, imu_stamp):
    """Build a DEV-ONLY physical readiness sample before entering mode 5."""
    state_stamps = {
        "odom": odom_stamp,
        "joint_state": joint_stamp,
        "imu": imu_stamp,
    }
    state_fresh = {
        name: message_is_fresh(
            now, stamp, FIXED_STAND_STATE_MAX_AGE_S
        )
        for name, stamp in state_stamps.items()
    }
    foot_fresh = [
        message_is_fresh(
            now, stamp, SAFE_STAND_FOOT_FORCE_MAX_AGE
        )
        for stamp in foot_stamps
    ]
    gyro_norm = (
        None
        if imu is None
        else math.sqrt(sum(value * value for value in imu[2]))
    )
    maximum_joint_speed = (
        None
        if joint_velocity is None or not joint_velocity
        else max(abs(value) for value in joint_velocity)
    )
    return {
        "sim_time": now,
        "dev_only_odom_height_m": (
            None if odom is None else odom[0]
        ),
        "odom_roll_rad": None if odom is None else odom[1],
        "odom_pitch_rad": None if odom is None else odom[2],
        "imu_roll_rad": None if imu is None else imu[0],
        "imu_pitch_rad": None if imu is None else imu[1],
        "raw_gyro_norm_rad_s": gyro_norm,
        "maximum_joint_speed_rad_s": maximum_joint_speed,
        "foot_forces_z_n": list(foot_forces),
        "foot_force_fresh": foot_fresh,
        "state_fresh": state_fresh,
        "state_age_sim_s": {
            name: None if stamp is None else now - stamp
            for name, stamp in state_stamps.items()
        },
    }


def fixed_stand_sample_is_ready(evidence):
    if evidence is None:
        return False
    height = evidence["dev_only_odom_height_m"]
    roll = evidence["imu_roll_rad"]
    pitch = evidence["imu_pitch_rad"]
    gyro = evidence["raw_gyro_norm_rad_s"]
    joint_speed = evidence["maximum_joint_speed_rad_s"]
    forces = evidence["foot_forces_z_n"]
    return (
        all(evidence["state_fresh"].values())
        and all(evidence["foot_force_fresh"])
        and height is not None
        and math.isfinite(height)
        and height >= FIXED_STAND_MIN_BASE_HEIGHT_M
        and roll is not None
        and pitch is not None
        and math.isfinite(roll)
        and math.isfinite(pitch)
        and abs(roll) <= FIXED_STAND_MAX_TILT_RAD
        and abs(pitch) <= FIXED_STAND_MAX_TILT_RAD
        and gyro is not None
        and math.isfinite(gyro)
        and gyro <= FIXED_STAND_MAX_GYRO_RAD_S
        and joint_speed is not None
        and math.isfinite(joint_speed)
        and joint_speed <= FIXED_STAND_MAX_JOINT_SPEED_RAD_S
        and all(
            force is not None
            and math.isfinite(force)
            and force >= SAFE_STAND_FOOT_FORCE_MIN_N
            for force in forces
        )
    )


def entry_micro_action_is_expected(state, result):
    return (
        state == GoalStatus.PREEMPTED
        and result is not None
        and not result.success
        and result.error_code == ExploreFloorResult.ERROR_CANCELLED
    )


def evaluate_controller_probe(
        diagnostic, now, joy_stamp, motor_stamps,
        maximum_age=CONTROLLER_MESSAGE_MAX_AGE):
    """Evaluate graph uniqueness, full motor coverage, and live messages."""
    result = copy.deepcopy(diagnostic)
    candidates = result["intersection"]
    result["selected_node"] = (
        candidates[0] if len(candidates) == 1 else None
    )
    result["candidate_count"] = len(candidates)
    result["joy_message_fresh"] = message_is_fresh(
        now, joy_stamp, maximum_age
    )
    result["joy_message_age_sim_s"] = (
        None if joy_stamp is None else now - joy_stamp
    )

    selected = result["selected_node"]
    covered = set(
        result["motor_topics_by_publisher"].get(selected, [])
    )
    expected = set(MOTOR_COMMAND_TOPICS)
    missing_topics = sorted(expected - covered)
    ambiguous_topics = sorted(
        topic for topic in expected
        if set(result["motor_topic_publishers"].get(topic, []))
        != ({selected} if selected is not None else set())
    )
    stale_topics = sorted(
        topic for topic in expected
        if not message_is_fresh(
            now, motor_stamps.get(topic), maximum_age
        )
    )
    result["missing_motor_topics"] = missing_topics
    result["ambiguous_motor_topics"] = ambiguous_topics
    result["stale_motor_topics"] = stale_topics
    result["ready"] = (
        selected is not None
        and not missing_topics
        and not ambiguous_topics
        and not stale_topics
        and result["joy_message_fresh"]
    )
    if len(candidates) == 0:
        result["reason"] = "zero controller candidates"
    elif len(candidates) > 1:
        result["reason"] = "multiple controller candidates"
    elif missing_topics:
        result["reason"] = "selected node does not publish all A1 motor topics"
    elif ambiguous_topics:
        result["reason"] = "A1 motor topics do not have one attributable publisher"
    elif stale_topics:
        result["reason"] = "A1 motor command messages are stale or missing"
    elif not result["joy_message_fresh"]:
        result["reason"] = "Joy message is stale or missing"
    else:
        result["reason"] = "unique live controller path"
    return result


def safe_stop_policy(mode5_entered):
    return (
        "FULL_GAIT_SAFE_STOP"
        if mode5_entered else "PRE_MOTION_ABORT"
    )


def grid_spec(message):
    return GridSpec(
        message.info.width,
        message.info.height,
        message.info.resolution,
        message.info.origin.position.x,
        message.info.origin.position.y,
    )


def corridor_evidence(
        message, start_xy, finish_xy, half_width=0.45,
        anchor_search_radius=0.60):
    """Summarize only OccupancyGrid cells in an entry-centred corridor."""
    spec = grid_spec(message)
    dx = finish_xy[0] - start_xy[0]
    dy = finish_xy[1] - start_xy[1]
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        raise ValueError("entry evidence corridor has zero length")
    ux = dx / length
    uy = dy / length
    px = -uy
    py = ux
    step = max(0.05, spec.resolution * 0.5)
    along_count = max(2, int(math.ceil(length / step)) + 1)
    across_count = max(
        3, int(math.ceil((2.0 * half_width) / step)) + 1
    )
    cells = set()
    for along_index in range(along_count):
        along = length * along_index / float(along_count - 1)
        for across_index in range(across_count):
            offset = (
                -half_width
                + 2.0 * half_width
                * across_index / float(across_count - 1)
            )
            cell = spec.world_to_cell(
                start_xy[0] + ux * along + px * offset,
                start_xy[1] + uy * along + py * offset,
            )
            if cell is not None:
                cells.add(cell)
    data = list(message.data)
    values = [
        int(data[row * spec.width + column])
        for row, column in cells
    ]
    anchor = nearest_known_free_anchor(
        message.data,
        spec,
        start_xy,
        anchor_search_radius,
        free_threshold=20,
    )
    return {
        "sampled_cells": len(values),
        "free_cells": sum(0 <= value <= 20 for value in values),
        "occupied_cells": sum(value >= 65 for value in values),
        "unknown_cells": sum(value < 0 for value in values),
        "maximum_occupancy": max(values) if values else None,
        "path_anchor": (
            None if anchor is None else [anchor[0], anchor[1]]
        ),
        "path_anchor_offset_m": (
            None if anchor is None else math.hypot(
                anchor[0] - start_xy[0],
                anchor[1] - start_xy[1],
            )
        ),
        "known_free_path": bool(
            anchor is not None
            and known_free_path_exists(
                message.data,
                spec,
                anchor,
                finish_xy,
                free_threshold=20,
            )
        ),
    }


def atomic_json(path, document):
    target = os.path.abspath(path)
    directory = os.path.dirname(target)
    os.makedirs(directory, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=".single_floor_acceptance_", dir=directory, text=True
    )
    try:
        with os.fdopen(descriptor, "w") as stream:
            json.dump(document, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, target)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


class GazeboAcceptance:
    def __init__(self):
        self.lock = threading.RLock()
        self.output = os.path.abspath(rospy.get_param("~output"))
        self.run_id = rospy.get_param("~run_id", "bounded")
        self.action_name = rospy.get_param(
            "~action", "/a1/exploration/explore_floor"
        )
        self.map_topic = rospy.get_param(
            "~map_topic", "/a1/floor_mapping/map"
        )
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.dev_global_frame = rospy.get_param(
            "~dev_global_frame", "odom"
        )
        self.floor_id = int(rospy.get_param("~floor_id", 0))
        self.entry_forward = float(
            rospy.get_param("~entry_forward_offset", 3.5)
        )
        self.entry_probe = float(
            rospy.get_param("~entry_probe_distance", 0.75)
        )
        self.roi_depth = float(rospy.get_param("~roi_depth", 12.0))
        self.roi_half_width = float(
            rospy.get_param("~roi_half_width", 6.0)
        )
        self.roi_boundary_margin = float(
            rospy.get_param("~roi_boundary_margin", 0.35)
        )
        self.minimum_frontier_successes = int(
            rospy.get_param("~minimum_frontier_successes", 2)
        )
        self.action_timeout_sim = float(
            rospy.get_param("~action_timeout_sim", 600.0)
        )
        self.action_wall_timeout = float(
            rospy.get_param("~action_wall_timeout", 7200.0)
        )
        self.prerequisite_sim_timeout = float(
            rospy.get_param("~prerequisite_sim_timeout", 40.0)
        )
        self.wall_timeout = float(
            rospy.get_param("~wall_timeout", 240.0)
        )
        self.tilt_abort_rad = math.radians(
            float(rospy.get_param("~tilt_abort_deg", 10.0))
        )
        self.team_scene_info = rospy.get_param(
            "~team_scene_info",
            "/workspace/SimEnv/generated_building/team_scene_info.json",
        )
        self.manage_physics_unpause = bool(
            rospy.get_param("~manage_physics_unpause", True)
        )
        self.bag_recorder_node = rospy.get_param(
            "~bag_recorder_node", DEFAULT_BAG_RECORDER_NODE
        )
        self.entry_motion_only = bool(
            rospy.get_param("~entry_motion_only", False)
        )

        self.master = rosgraph.Master(rospy.get_name())
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.action_client = actionlib.SimpleActionClient(
            self.action_name, ExploreFloorAction
        )
        self.joy_pub = rospy.Publisher(JOY_TOPIC, Joy, queue_size=2)
        self.zero_pub = rospy.Publisher(
            "/cmd_vel_nav", Twist, queue_size=2
        )
        self.safety_lock_pub = rospy.Publisher(
            "/a1_cmd_mux/safety_lock", Bool, queue_size=1, latch=True
        )

        self.map_message = None
        self.mapping_status = None
        self.controller_ready = False
        self.controller_ready_stamp = None
        self.mode5_entered = False
        self.mode5_entered_sim_time = None
        self.controller_node = None
        self.controller_probe_diagnostic = None
        self.joy_message_stamp = None
        self.motor_command_stamps = {
            topic: None for topic in MOTOR_COMMAND_TOPICS
        }
        self.safe_stand_ready = False
        self.safe_stand_seen = False
        self.safe_stand_edge = None
        self.safe_stand_gyro_filter = SafeStandGyroFilter()
        self.filtered_gyro = None
        self.dev_tf_probe_evidence = None
        self.bag_recorder_probe_diagnostic = None
        self.physics_unpause_evidence = None
        self.safety_locked = False
        self.final_cmd = None
        self.final_cmd_stamp = None
        self.last_nonzero_cmd_stamp = None
        self.foot_forces = [None] * 4
        self.foot_stamps = [None] * 4
        self.imu = None
        self.imu_stamp = None
        self.odom = None
        self.odom_stamp = None
        self.joint_velocity = None
        self.joint_stamp = None
        self.fixed_stand_candidate_since = None
        self.fixed_stand_preflight = None
        self.max_roll = 0.0
        self.max_pitch = 0.0
        self.trajectory = None
        self.feedback = []
        self.feedback_states = set()
        self.frontier_targets = []
        self.frontier_successes = 0
        self.frontier_failures = 0
        self.frontier_cancelled = 0
        self.entry_state_seen = False
        self.entry_cancel_sent = False
        self.entry_observation = None
        self.post_open_map = None
        self.entered_map = None
        self.completion_reason = ""
        self.door_result = None
        self.action_start_sim = None
        self.action_end_sim = None
        self.test_start_sim = None
        self.start_pose = None
        self.entry_pose = None
        self.roi_local = [
            (0.0, -self.roi_half_width),
            (self.roi_depth, -self.roi_half_width),
            (self.roi_depth, self.roi_half_width),
            (0.0, self.roi_half_width),
        ]
        self.subscribers = [
            rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1,
            ),
            rospy.Subscriber(
                "/a1/floor_mapping/status",
                DiagnosticStatus,
                self.mapping_status_callback,
                queue_size=2,
            ),
            rospy.Subscriber(
                "/a1/controller_ready",
                Bool,
                self.controller_ready_callback,
                queue_size=20,
            ),
            rospy.Subscriber(
                "/a1/safe_stand_ready",
                Bool,
                self.safe_stand_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/a1_cmd_mux/safety_lock",
                Bool,
                self.safety_lock_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/cmd_vel",
                Twist,
                self.cmd_callback,
                queue_size=50,
            ),
            rospy.Subscriber(
                "/trunk_imu",
                Imu,
                self.imu_callback,
                queue_size=50,
            ),
            rospy.Subscriber(
                "/Odometry_gazebo",
                Odometry,
                self.odom_callback,
                queue_size=50,
            ),
            rospy.Subscriber(
                "/a1_gazebo/joint_states",
                JointState,
                self.joint_state_callback,
                queue_size=50,
            ),
            rospy.Subscriber(
                JOY_TOPIC,
                Joy,
                self.joy_callback,
                queue_size=2,
            ),
            rospy.Subscriber(
                "/a1/exploration/trajectory",
                Path,
                self.trajectory_callback,
                queue_size=1,
            ),
            rospy.Subscriber(
                "/a1/building_behavior/special/result",
                SpecialBehaviorActionResult,
                self.door_result_callback,
                queue_size=2,
            ),
        ]
        for topic in MOTOR_COMMAND_TOPICS:
            self.subscribers.append(
                rospy.Subscriber(
                    topic,
                    rospy.AnyMsg,
                    lambda message, motor_topic=topic:
                    self.motor_command_callback(message, motor_topic),
                    queue_size=1,
                )
            )
        for index, topic in enumerate(FOOT_TOPICS):
            self.subscribers.append(
                rospy.Subscriber(
                    topic,
                    WrenchStamped,
                    lambda message, leg=index: self.foot_callback(
                        message, leg
                    ),
                    queue_size=20,
                )
            )

    def now_sim(self):
        return rospy.Time.now().to_sec()

    def map_callback(self, message):
        with self.lock:
            self.map_message = message

    def mapping_status_callback(self, message):
        with self.lock:
            self.mapping_status = message

    def controller_ready_callback(self, message):
        with self.lock:
            was_ready = self.controller_ready
            self.controller_ready = bool(message.data)
            self.controller_ready_stamp = self.now_sim()
            if message.data and not self.mode5_entered:
                self.mode5_entered = True
                self.mode5_entered_sim_time = self.controller_ready_stamp
            if was_ready and not message.data:
                self.safe_stand_gyro_filter.reset()
                self.filtered_gyro = None

    def safe_stand_callback(self, message):
        with self.lock:
            rising_edge = bool(message.data) and not self.safe_stand_ready
            self.safe_stand_ready = bool(message.data)
            self.safe_stand_seen = (
                self.safe_stand_seen or bool(message.data)
            )
            if rising_edge and self.safe_stand_edge is None:
                self.safe_stand_edge = safe_stand_edge_evidence(
                    self.now_sim(),
                    self.foot_forces,
                    self.foot_stamps,
                    self.imu,
                    self.filtered_gyro,
                )

    def safety_lock_callback(self, message):
        with self.lock:
            self.safety_locked = bool(message.data)

    def cmd_callback(self, message):
        now = self.now_sim()
        values = (
            message.linear.x,
            message.linear.y,
            message.angular.z,
        )
        with self.lock:
            self.final_cmd = values
            self.final_cmd_stamp = now
            if any(abs(value) > 0.01 for value in values):
                self.last_nonzero_cmd_stamp = now

    def imu_callback(self, message):
        now = self.now_sim()
        roll, pitch = quaternion_roll_pitch(message.orientation)
        gyro = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        with self.lock:
            self.imu = (roll, pitch, gyro)
            self.imu_stamp = now
            self.filtered_gyro = self.safe_stand_gyro_filter.update(
                now, gyro
            )
            self.max_roll = max(self.max_roll, abs(roll))
            self.max_pitch = max(self.max_pitch, abs(pitch))

    def odom_callback(self, message):
        now = self.now_sim()
        roll, pitch = quaternion_roll_pitch(
            message.pose.pose.orientation
        )
        with self.lock:
            self.odom = (
                message.pose.pose.position.z,
                roll,
                pitch,
            )
            self.odom_stamp = now

    def joint_state_callback(self, message):
        now = self.now_sim()
        velocity = tuple(message.velocity)
        with self.lock:
            self.joint_velocity = velocity
            self.joint_stamp = now

    def joy_callback(self, _message):
        with self.lock:
            self.joy_message_stamp = self.now_sim()

    def motor_command_callback(self, _message, topic):
        with self.lock:
            self.motor_command_stamps[topic] = self.now_sim()

    def trajectory_callback(self, message):
        with self.lock:
            self.trajectory = message

    def foot_callback(self, message, index):
        with self.lock:
            self.foot_forces[index] = abs(message.wrench.force.z)
            self.foot_stamps[index] = self.now_sim()

    def door_result_callback(self, message):
        result = message.result
        with self.lock:
            self.door_result = {
                "success": bool(result.success),
                "error_code": int(result.error_code),
                "message": result.message,
                "sim_time": self.now_sim(),
            }

    @staticmethod
    def mapping_usable(message):
        if message is None:
            return False
        values = {item.key: item.value for item in message.values}
        return (
            message.message == "MAPPING"
            and values.get("state", "MAPPING") == "MAPPING"
            and values.get("map_valid") == "true"
            and values.get("obstacle_cloud_valid") == "true"
            and values.get("marking_cloud_valid", "true") == "true"
        )

    def wait_for(self, predicate, description, sim_timeout=None,
                 wall_timeout=None):
        sim_limit = (
            self.prerequisite_sim_timeout
            if sim_timeout is None else float(sim_timeout)
        )
        wall_limit = (
            self.wall_timeout if wall_timeout is None
            else float(wall_timeout)
        )
        start_sim = self.now_sim()
        wall_deadline = time.monotonic() + wall_limit
        while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
            now = self.now_sim()
            if predicate():
                return
            if now < start_sim:
                raise AcceptanceFailure(
                    "simulation clock moved backwards while waiting for "
                    + description
                )
            if sim_limit > 0.0 and now - start_sim >= sim_limit:
                raise AcceptanceFailure(
                    "simulation timeout waiting for " + description
                )
            time.sleep(0.02)
        raise AcceptanceFailure("wall timeout waiting for " + description)

    def wait_for_wall(self, predicate, description, wall_timeout=None):
        wall_limit = (
            self.wall_timeout if wall_timeout is None
            else float(wall_timeout)
        )
        deadline = time.monotonic() + wall_limit
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        raise AcceptanceFailure("wall timeout waiting for " + description)

    def bag_recorder_probe(self):
        try:
            system_state = self.master.getSystemState()
            probe = bag_recorder_graph_diagnostic(
                system_state,
                self.bag_recorder_node,
                REQUIRED_BAG_TOPICS,
                rospy.resolve_name,
            )
        except Exception as error:
            probe = {
                "ready": False,
                "recorder_node": rospy.resolve_name(
                    self.bag_recorder_node
                ),
                "required_topics": sorted(REQUIRED_BAG_TOPICS),
                "subscribers_by_topic": {},
                "missing_topics": sorted(REQUIRED_BAG_TOPICS),
                "graph_error": repr(error),
            }
        self.bag_recorder_probe_diagnostic = probe
        return probe

    def start_acceptance_clock(self):
        if not self.manage_physics_unpause:
            self.wait_for(
                lambda: self.now_sim() > 0.0,
                "non-zero simulation clock",
                sim_timeout=10.0,
            )
            return

        self.wait_for_wall(
            lambda: self.bag_recorder_probe()["ready"],
            "bag recorder subscriptions before managed unpause",
        )
        before = self.now_sim()
        if before < 0.0:
            raise AcceptanceFailure(
                "simulation clock is invalid before managed unpause"
            )
        try:
            rospy.wait_for_service(
                "/gazebo/unpause_physics", timeout=self.wall_timeout
            )
            unpause = rospy.ServiceProxy(
                "/gazebo/unpause_physics", Empty
            )
            unpause()
        except Exception as error:
            raise AcceptanceFailure(
                "managed Gazebo unpause failed: %r" % (error,)
            )
        self.wait_for(
            lambda: self.now_sim() >= before + 0.02,
            "simulation clock advancement after managed unpause",
            sim_timeout=1.0,
        )
        self.physics_unpause_evidence = {
            "managed": True,
            "clock_before_sim_s": before,
            "clock_after_sim_s": self.now_sim(),
            "bag_recorder": copy.deepcopy(
                self.bag_recorder_probe_diagnostic
            ),
        }

    def sleep_sim(self, duration):
        start = self.now_sim()
        wall_deadline = time.monotonic() + self.wall_timeout
        while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
            now = self.now_sim()
            if now < start:
                raise AcceptanceFailure("simulation clock moved backwards")
            if now - start >= duration:
                return
            time.sleep(0.02)
        raise AcceptanceFailure("wall timeout advancing simulation")

    def controller_probe(self):
        try:
            system_state = self.master.getSystemState()
            graph = controller_graph_diagnostic(
                system_state, rospy.resolve_name
            )
        except Exception as error:
            graph = {
                "joy_subscribers": [],
                "motor_publishers": [],
                "intersection": [],
                "motor_topic_publishers": {},
                "motor_topics_by_publisher": {},
                "graph_error": repr(error),
            }
        with self.lock:
            joy_stamp = self.joy_message_stamp
            motor_stamps = dict(self.motor_command_stamps)
        probe = evaluate_controller_probe(
            graph,
            self.now_sim(),
            joy_stamp,
            motor_stamps,
        )
        with self.lock:
            self.controller_probe_diagnostic = probe
            self.controller_node = probe["selected_node"]
        return probe

    def publish_neutral_joy(self):
        message = Joy(axes=[0.0] * 6, buttons=[0] * 11)
        message.header.stamp = rospy.Time.now()
        self.joy_pub.publish(message)

    def wait_for_live_controller_path(self):
        start_sim = self.now_sim()
        wall_deadline = time.monotonic() + self.wall_timeout
        last_probe = None
        while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
            self.publish_neutral_joy()
            last_probe = self.controller_probe()
            if last_probe["ready"]:
                rospy.loginfo(
                    "unique live A1 controller path: %s",
                    json.dumps(last_probe, sort_keys=True),
                )
                return
            now = self.now_sim()
            if now < start_sim:
                raise AcceptanceFailure(
                    "simulation clock moved backwards while probing "
                    "the A1 controller: %s"
                    % json.dumps(last_probe, sort_keys=True)
                )
            if now - start_sim >= self.prerequisite_sim_timeout:
                raise AcceptanceFailure(
                    "simulation timeout waiting for unique live A1 "
                    "controller path: %s"
                    % json.dumps(last_probe, sort_keys=True)
                )
            time.sleep(0.02)
        raise AcceptanceFailure(
            "wall timeout waiting for unique live A1 controller path: %s"
            % json.dumps(last_probe, sort_keys=True)
        )

    def cmd_vel_publishers_are_guarded(self):
        try:
            publishers, _, _ = self.master.getSystemState()
        except Exception:
            return False
        for topic, nodes in publishers:
            if rospy.resolve_name(topic) == "/cmd_vel":
                return {
                    rospy.resolve_name(node) for node in nodes
                } == {"/cmd_vel_guard"}
        return False

    def publish_button(self, index, wall_seconds=0.35):
        pressed = Joy(axes=[0.0] * 6, buttons=[0] * 11)
        pressed.buttons[index] = 1
        deadline = time.monotonic() + wall_seconds
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            pressed.header.stamp = rospy.Time.now()
            self.joy_pub.publish(pressed)
            time.sleep(0.02)
        self.publish_neutral_joy()

    def prepare_controller_fixed_stand(self):
        self.wait_for_live_controller_path()
        self.publish_button(1)
        self.sleep_sim(4.0)
        with self.lock:
            self.fixed_stand_candidate_since = None
            self.fixed_stand_preflight = None
        self.wait_for(
            self.fixed_stand_physical_ready,
            "DEV-ONLY physical fixed-stand stability before mode 5",
            sim_timeout=30.0,
            wall_timeout=600.0,
        )

    def fixed_stand_physical_ready(self):
        now = self.now_sim()
        with self.lock:
            evidence = fixed_stand_evidence(
                now,
                self.odom,
                self.odom_stamp,
                self.joint_velocity,
                self.joint_stamp,
                self.foot_forces,
                self.foot_stamps,
                self.imu,
                self.imu_stamp,
            )
            sample_ready = fixed_stand_sample_is_ready(evidence)
            if not sample_ready:
                self.fixed_stand_candidate_since = None
            elif (
                    self.fixed_stand_candidate_since is None
                    or now < self.fixed_stand_candidate_since):
                self.fixed_stand_candidate_since = now
            stable_duration = (
                0.0
                if self.fixed_stand_candidate_since is None
                else now - self.fixed_stand_candidate_since
            )
            evidence["sample_ready"] = sample_ready
            evidence["stable_duration_sim_s"] = stable_duration
            evidence["ready"] = bool(
                sample_ready
                and stable_duration >= FIXED_STAND_STABLE_DURATION_S
            )
            self.fixed_stand_preflight = evidence
            return evidence["ready"]

    def enter_controller_mode5(self):
        self.publish_button(5)
        self.wait_for(
            lambda: self.controller_ready_fresh(),
            "fresh controller_ready=true",
            sim_timeout=8.0,
        )
        self.wait_for(
            self.cmd_vel_publishers_are_guarded,
            "cmd_vel_guard to be the sole /cmd_vel publisher",
            sim_timeout=5.0,
        )

    def dev_tf_probe(self):
        now = self.now_sim()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.dev_global_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.0),
            )
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            expected_parent = self.dev_global_frame.lstrip("/")
            expected_child = self.base_frame.lstrip("/")
            evidence = dev_tf_sample_evidence(
                now,
                transform.header.stamp.to_sec(),
                (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ),
                (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ),
            )
            evidence.update({
                "parent_frame": parent,
                "child_frame": child,
                "expected_parent_frame": expected_parent,
                "expected_child_frame": expected_child,
                "frame_match": (
                    parent == expected_parent and child == expected_child
                ),
            })
            evidence["ready"] = bool(
                evidence["ready"] and evidence["frame_match"]
            )
        except Exception as error:
            evidence = {
                "ready": False,
                "parent_frame": None,
                "child_frame": None,
                "expected_parent_frame": self.dev_global_frame.lstrip("/"),
                "expected_child_frame": self.base_frame.lstrip("/"),
                "frame_match": False,
                "error": str(error),
            }
        with self.lock:
            self.dev_tf_probe_evidence = evidence
        return evidence

    def controller_ready_fresh(self):
        with self.lock:
            stamp = self.controller_ready_stamp
            ready = self.controller_ready
        now = self.now_sim()
        return (
            ready
            and stamp is not None
            and 0.0 <= now - stamp <= 0.35
        )

    def pose_in_frame(self, frame):
        transform = self.tf_buffer.lookup_transform(
            frame,
            self.base_frame,
            rospy.Time(0),
            rospy.Duration(20.0),
        )
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def establish_geometry(self, map_message):
        self.start_pose = self.pose_in_frame(map_message.header.frame_id)
        yaw = quaternion_yaw(self.start_pose.pose.orientation)
        entry = PoseStamped()
        entry.header.frame_id = map_message.header.frame_id
        entry.header.stamp = rospy.Time.now()
        entry.pose.position.x = (
            self.start_pose.pose.position.x
            + math.cos(yaw) * self.entry_forward
        )
        entry.pose.position.y = (
            self.start_pose.pose.position.y
            + math.sin(yaw) * self.entry_forward
        )
        (
            entry.pose.orientation.x,
            entry.pose.orientation.y,
            entry.pose.orientation.z,
            entry.pose.orientation.w,
        ) = quaternion_from_yaw(yaw)
        self.entry_pose = entry
        forward_dot = (
            (entry.pose.position.x - self.start_pose.pose.position.x)
            * math.cos(yaw)
            + (entry.pose.position.y - self.start_pose.pose.position.y)
            * math.sin(yaw)
        )
        if forward_dot <= 0.0 or self.roi_depth <= 0.0:
            raise AcceptanceFailure(
                "entry-local ROI does not extend forward from the entrance"
            )

    def entry_probe_xy(self):
        yaw = quaternion_yaw(self.entry_pose.pose.orientation)
        return (
            self.entry_pose.pose.position.x
            + math.cos(yaw) * self.entry_probe,
            self.entry_pose.pose.position.y
            + math.sin(yaw) * self.entry_probe,
        )

    def start_xy(self):
        return (
            self.start_pose.pose.position.x,
            self.start_pose.pose.position.y,
        )

    def map_evidence(self, message):
        return corridor_evidence(
            message,
            self.start_xy(),
            self.entry_probe_xy(),
        )

    def roi_metrics(self, message):
        spec = grid_spec(message)
        anchor = (
            self.entry_pose.pose.position.x,
            self.entry_pose.pose.position.y,
        )
        polygon = transform_local_polygon(
            self.roi_local,
            anchor,
            quaternion_yaw(self.entry_pose.pose.orientation),
        )
        mask = polygon_mask(
            spec, polygon, self.roi_boundary_margin
        )
        return {
            "denominator_cells": int(mask.sum()),
            "known_cells": known_cell_count(message.data, mask),
            "coverage_ratio": coverage_ratio(message.data, mask),
        }

    def baseline_ready(self):
        with self.lock:
            message = copy.deepcopy(self.map_message)
            status = copy.deepcopy(self.mapping_status)
        if message is None or not self.mapping_usable(status):
            return False
        evidence = self.map_evidence(message)
        return (
            not evidence["known_free_path"]
            and evidence["occupied_cells"] > 0
        )

    def feedback_callback(self, message):
        capture_entry_pose = False
        row = {
            "sim_time": self.now_sim(),
            "state": int(message.state),
            "coverage_ratio": float(message.coverage_ratio),
            "message": message.message,
            "target": {
                "frame_id": message.current_target.header.frame_id,
                "x": message.current_target.pose.position.x,
                "y": message.current_target.pose.position.y,
            },
        }
        with self.lock:
            self.feedback.append(row)
            self.feedback_states.add(int(message.state))
            if message.state == ExploreFloorFeedback.TRANSIT_TO_ENTRY:
                if self.post_open_map is None:
                    self.post_open_map = copy.deepcopy(self.map_message)
            if message.state == ExploreFloorFeedback.ENTERED_FLOOR:
                self.entry_state_seen = True
            if (
                    message.state == ExploreFloorFeedback.ENTERED_FLOOR
                    and self.entered_map is None):
                self.entered_map = copy.deepcopy(self.map_message)
                capture_entry_pose = True
            if (
                message.state == ExploreFloorFeedback.NAVIGATING
                and message.current_target.header.frame_id
                and "frontier length=" in message.message
            ):
                candidate = row["target"]
                if not self.frontier_targets or math.hypot(
                    candidate["x"] - self.frontier_targets[-1]["x"],
                    candidate["y"] - self.frontier_targets[-1]["y"],
                ) > 0.05:
                    self.frontier_targets.append(candidate)
            if (
                message.state == ExploreFloorFeedback.UPDATE_COVERAGE
                and "frontier reached" in message.message
            ):
                self.frontier_successes += 1
            if (
                message.state == ExploreFloorFeedback.UPDATE_COVERAGE
                and "frontier failure" in message.message
            ):
                self.frontier_failures += 1
            if (
                message.state == ExploreFloorFeedback.UPDATE_COVERAGE
                and "cancelled/preempted" in message.message
            ):
                self.frontier_cancelled += 1
            if message.state == ExploreFloorFeedback.EXPLORATION_DONE:
                self.completion_reason = message.message
        if capture_entry_pose:
            try:
                pose = self.pose_in_frame(self.entry_pose.header.frame_id)
                observation = {
                    "position_error_m": math.hypot(
                        pose.pose.position.x
                        - self.entry_pose.pose.position.x,
                        pose.pose.position.y
                        - self.entry_pose.pose.position.y,
                    ),
                    "yaw_error_rad": angle_error(
                        quaternion_yaw(pose.pose.orientation),
                        quaternion_yaw(
                            self.entry_pose.pose.orientation
                        ),
                    ),
                    "sim_time": self.now_sim(),
                }
                with self.lock:
                    self.entry_observation = observation
            except Exception as error:
                rospy.logwarn(
                    "could not snapshot entry pose for acceptance: %s",
                    error,
                )

    def send_exploration(self, door_id):
        if not self.action_client.wait_for_server(rospy.Duration(60.0)):
            raise AcceptanceFailure("ExploreFloor action server unavailable")
        goal = ExploreFloorGoal()
        goal.floor_id = self.floor_id
        goal.target_coverage_ratio = 0.0
        goal.timeout_s = self.action_timeout_sim
        goal.floor_entry_pose = copy.deepcopy(self.entry_pose)
        goal.roi_local.points = [
            Point32(x=x, y=y) for x, y in self.roi_local
        ]
        goal.entry_door_id = door_id
        if not self.controller_ready_fresh():
            raise AcceptanceFailure(
                "controller_ready was not fresh immediately before action send"
            )
        self.action_start_sim = self.now_sim()
        self.action_client.send_goal(
            goal, feedback_cb=self.feedback_callback
        )
        terminal = {
            GoalStatus.PREEMPTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        }
        wall_deadline = time.monotonic() + self.action_wall_timeout
        while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
            state = self.action_client.get_state()
            if state in terminal:
                self.action_end_sim = self.now_sim()
                return state, self.action_client.get_result()
            if (
                    self.entry_motion_only
                    and self.entry_state_seen
                    and not self.entry_cancel_sent):
                self.action_client.cancel_goal()
                self.entry_cancel_sent = True
            with self.lock:
                tilt = max(self.max_roll, self.max_pitch)
                locked = self.safety_locked
            if locked:
                self.action_client.cancel_goal()
                raise AcceptanceFailure(
                    "safety lock asserted during exploration"
                )
            if tilt >= self.tilt_abort_rad:
                self.action_client.cancel_goal()
                raise AcceptanceFailure(
                    "tilt safety threshold exceeded: %.2f deg"
                    % math.degrees(tilt)
                )
            time.sleep(0.05)
        self.action_client.cancel_goal()
        raise AcceptanceFailure("ExploreFloor action wall timeout")

    def final_zero_evidence(self):
        with self.lock:
            values = self.final_cmd
            stamp = self.final_cmd_stamp
            last_nonzero = self.last_nonzero_cmd_stamp
        now = self.now_sim()
        fresh = stamp is not None and 0.0 <= now - stamp <= 0.25
        zero_age = (
            float("inf")
            if last_nonzero is None
            else now - last_nonzero
        )
        return {
            "values": list(values) if values is not None else None,
            "fresh": fresh,
            "message_age_sim_s": (
                None if stamp is None else now - stamp
            ),
            "last_nonzero_age_sim_s": (
                None if not math.isfinite(zero_age) else zero_age
            ),
            "settled_for_0_5s": zero_age >= 0.5,
            "zero": (
                values is not None
                and all(abs(value) <= 0.01 for value in values)
            ),
        }

    def safe_stop(self):
        for _ in range(10):
            self.zero_pub.publish(Twist())
            time.sleep(0.02)
        with self.lock:
            ready = self.controller_ready
            mode5_entered = self.mode5_entered
        policy = safe_stop_policy(mode5_entered)
        if not mode5_entered:
            self.safety_lock_pub.publish(Bool(data=True))
            try:
                self.wait_for(
                    lambda: (
                        self.safety_locked
                        and self.final_zero_evidence()["fresh"]
                        and self.final_zero_evidence()["zero"]
                        and self.final_zero_evidence()[
                            "settled_for_0_5s"
                        ]
                    ),
                    "pre-motion zero velocity and safety lock",
                    sim_timeout=3.0,
                    wall_timeout=60.0,
                )
            except Exception as error:
                return {
                    "success": False,
                    "classification": policy,
                    "mode5_entered": False,
                    "safe_stand_required": False,
                    "error": str(error),
                    "final_zero": self.final_zero_evidence(),
                    "safety_lock_latched": self.safety_locked,
                }
            return {
                "success": True,
                "classification": policy,
                "mode5_entered": False,
                "safe_stand_required": False,
                "safe_stand_ready_seen": self.safe_stand_seen,
                "fixed_stand_transition_inferred": False,
                "final_zero": self.final_zero_evidence(),
                "safety_lock_latched": True,
            }
        if ready:
            self.publish_button(1)
        try:
            self.wait_for(
                lambda: (
                    self.safe_stand_edge is not None
                    and not self.controller_ready
                ),
                "guarded all-foot stable stand transition",
                sim_timeout=12.0,
                wall_timeout=240.0,
            )
            self.sleep_sim(0.6)
        except Exception as error:
            self.safety_lock_pub.publish(Bool(data=True))
            return {
                "success": False,
                "classification": policy,
                "mode5_entered": True,
                "safe_stand_required": True,
                "error": str(error),
                "safety_lock_latched": True,
            }
        with self.lock:
            edge = copy.deepcopy(self.safe_stand_edge)
            ready = self.controller_ready
        zero = self.final_zero_evidence()
        success = (
            safe_stand_edge_is_valid(edge)
            and not ready
            and zero["fresh"]
            and zero["zero"]
            and zero["settled_for_0_5s"]
        )
        if not success:
            self.safety_lock_pub.publish(Bool(data=True))
        return {
            "success": bool(success),
            "classification": policy,
            "mode5_entered": True,
            "safe_stand_required": True,
            "safe_stand_ready_seen": self.safe_stand_seen,
            "controller_ready": ready,
            "fixed_stand_transition_inferred": (
                edge is not None and not ready
            ),
            "safe_stand_edge": edge,
            "safe_stand_edge_valid": safe_stand_edge_is_valid(edge),
            "final_zero": zero,
            "safety_lock_latched": not success,
        }

    def trajectory_metrics(self):
        with self.lock:
            trajectory = copy.deepcopy(self.trajectory)
        if trajectory is None:
            return {"poses": 0, "distance_m": 0.0}
        distance = sum(
            math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y,
            )
            for previous, current in zip(
                trajectory.poses, trajectory.poses[1:]
            )
        )
        if self.entry_pose is None:
            return {
                "poses": len(trajectory.poses),
                "distance_m": distance,
                "indoor_poses": None,
                "indoor_distance_m": None,
            }
        indoor_distance = 0.0
        indoor_poses = 0
        anchor = (
            self.entry_pose.pose.position.x,
            self.entry_pose.pose.position.y,
        )
        polygon = transform_local_polygon(
            self.roi_local,
            anchor,
            quaternion_yaw(self.entry_pose.pose.orientation),
        )
        inside = [
            point_in_polygon(
                pose.pose.position.x,
                pose.pose.position.y,
                polygon,
                self.roi_boundary_margin,
            )
            for pose in trajectory.poses
        ]
        indoor_poses = sum(bool(value) for value in inside)
        for index, (previous, current) in enumerate(zip(
            trajectory.poses, trajectory.poses[1:]
        )):
            segment = math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y,
            )
            if inside[index] and inside[index + 1]:
                indoor_distance += segment
        return {
            "poses": len(trajectory.poses),
            "distance_m": distance,
            "indoor_poses": indoor_poses,
            "indoor_distance_m": indoor_distance,
        }

    def return_errors(self):
        pose = self.pose_in_frame(self.start_pose.header.frame_id)
        return {
            "position_m": math.hypot(
                pose.pose.position.x - self.start_pose.pose.position.x,
                pose.pose.position.y - self.start_pose.pose.position.y,
            ),
            "yaw_rad": angle_error(
                quaternion_yaw(pose.pose.orientation),
                quaternion_yaw(self.start_pose.pose.orientation),
            ),
        }

    def validate_action(self, action_state, action_result, baseline):
        if action_result is None:
            raise AcceptanceFailure("ExploreFloor returned no result")
        if (
            action_state != GoalStatus.SUCCEEDED
            or not action_result.success
            or action_result.error_code != 0
        ):
            raise AcceptanceFailure(
                "ExploreFloor failed: state=%d code=%d message=%s"
                % (
                    action_state,
                    action_result.error_code,
                    action_result.message,
                )
            )
        required_states = {
            ExploreFloorFeedback.RECORD_START,
            ExploreFloorFeedback.REQUEST_ENTRY_DOOR_OPEN,
            ExploreFloorFeedback.TRANSIT_TO_ENTRY,
            ExploreFloorFeedback.ENTERED_FLOOR,
            ExploreFloorFeedback.NAVIGATING,
            ExploreFloorFeedback.EXPLORATION_DONE,
            ExploreFloorFeedback.RETURNING,
            ExploreFloorFeedback.RETURNED,
        }
        missing = required_states - self.feedback_states
        if missing:
            raise AcceptanceFailure(
                "missing ExploreFloor feedback states: %s"
                % sorted(missing)
            )
        if self.door_result is None or not self.door_result["success"]:
            raise AcceptanceFailure(
                "public entry-door behavior did not return success"
            )
        if self.post_open_map is None:
            raise AcceptanceFailure("post-open OccupancyGrid was not captured")
        post = self.map_evidence(self.post_open_map)
        if not post["known_free_path"]:
            raise AcceptanceFailure(
                "post-open OccupancyGrid has no known-free entry path"
            )
        if post["occupied_cells"] >= baseline["occupied_cells"]:
            raise AcceptanceFailure(
                "entry corridor occupied-cell count did not decrease"
            )
        if self.entered_map is None:
            raise AcceptanceFailure("post-entry OccupancyGrid was not captured")
        if self.entry_observation is None:
            raise AcceptanceFailure("entry pose observation was not captured")
        if (
            self.entry_observation["position_error_m"] > 0.40
            or self.entry_observation["yaw_error_rad"] > 0.65
        ):
            raise AcceptanceFailure(
                "entry pose outside tolerance: %.3f m / %.3f rad"
                % (
                    self.entry_observation["position_error_m"],
                    self.entry_observation["yaw_error_rad"],
                )
            )
        post_roi = self.roi_metrics(self.post_open_map)
        entered_roi = self.roi_metrics(self.entered_map)
        if entered_roi["known_cells"] - post_roi["known_cells"] < 20:
            raise AcceptanceFailure(
                "entering the floor did not reveal 20 new ROI cells"
            )
        if self.frontier_successes < self.minimum_frontier_successes:
            raise AcceptanceFailure(
                "only %d frontier goals succeeded; need at least %d"
                % (
                    self.frontier_successes,
                    self.minimum_frontier_successes,
                )
            )
        if not self.completion_reason:
            raise AcceptanceFailure("exploration completion reason is empty")
        final_zero = self.final_zero_evidence()
        if not (
            final_zero["fresh"]
            and final_zero["zero"]
            and final_zero["settled_for_0_5s"]
        ):
            raise AcceptanceFailure(
                "final /cmd_vel did not satisfy fresh continuous zero"
            )
        with self.lock:
            final_map = copy.deepcopy(self.map_message)
        final_roi = self.roi_metrics(final_map)
        return post, post_roi, entered_roi, final_roi, final_zero

    def validate_entry_microtest(
            self, action_state, action_result, baseline):
        if not entry_micro_action_is_expected(
                action_state, action_result):
            raise AcceptanceFailure(
                "entry microtest expected a deliberate cancellation "
                "after ENTERED_FLOOR: state=%d result=%r"
                % (action_state, action_result)
            )
        required_states = {
            ExploreFloorFeedback.RECORD_START,
            ExploreFloorFeedback.REQUEST_ENTRY_DOOR_OPEN,
            ExploreFloorFeedback.TRANSIT_TO_ENTRY,
            ExploreFloorFeedback.ENTERED_FLOOR,
        }
        missing = required_states - self.feedback_states
        if missing:
            raise AcceptanceFailure(
                "entry microtest missing feedback states: %s"
                % sorted(missing)
            )
        if self.door_result is None or not self.door_result["success"]:
            raise AcceptanceFailure(
                "public entry-door behavior did not return success"
            )
        if self.post_open_map is None:
            raise AcceptanceFailure(
                "post-open OccupancyGrid was not captured"
            )
        post = self.map_evidence(self.post_open_map)
        if (
                not post["known_free_path"]
                or post["occupied_cells"] >= baseline["occupied_cells"]):
            raise AcceptanceFailure(
                "entry microtest did not obtain a sensor-confirmed "
                "open passage"
            )
        if self.entered_map is None or self.entry_observation is None:
            raise AcceptanceFailure(
                "entry microtest did not capture ENTERED_FLOOR evidence"
            )
        if (
                self.entry_observation["position_error_m"] > 0.40
                or self.entry_observation["yaw_error_rad"] > 0.65):
            raise AcceptanceFailure(
                "entry microtest pose outside tolerance: %.3f m / %.3f rad"
                % (
                    self.entry_observation["position_error_m"],
                    self.entry_observation["yaw_error_rad"],
                )
            )
        post_roi = self.roi_metrics(self.post_open_map)
        entered_roi = self.roi_metrics(self.entered_map)
        if entered_roi["known_cells"] - post_roi["known_cells"] < 20:
            raise AcceptanceFailure(
                "entry microtest did not reveal 20 new ROI cells"
            )
        self.wait_for(
            lambda: (
                self.final_zero_evidence()["fresh"]
                and self.final_zero_evidence()["zero"]
                and self.final_zero_evidence()["settled_for_0_5s"]
            ),
            "entry microtest final zero before safe stop",
            sim_timeout=3.0,
            wall_timeout=60.0,
        )
        return (
            post,
            post_roi,
            entered_roi,
            self.final_zero_evidence(),
        )

    def run(self):
        document = {
            "run_id": self.run_id,
            "success": False,
            "failure_stage": "",
            "error": "",
        }
        safe_stop_result = None
        action_result = None
        try:
            document["failure_stage"] = "BAG_AND_PHYSICS_PREFLIGHT"
            self.start_acceptance_clock()
            self.test_start_sim = self.now_sim()
            public_scene = load_public_scene(self.team_scene_info)
            door_id = resolve_entry_door(
                public_scene, self.floor_id, ""
            )
            document["public_entry_door_id"] = door_id

            document["failure_stage"] = "DEV_TF_PREFLIGHT"
            self.prepare_controller_fixed_stand()
            self.wait_for(
                lambda: self.dev_tf_probe()["ready"],
                "fresh dev-only odom-to-base TF before mode 5",
                sim_timeout=5.0,
                wall_timeout=120.0,
            )
            document["dev_truth_tf_preflight"] = copy.deepcopy(
                self.dev_tf_probe_evidence
            )
            document["bag_recorder_preflight"] = copy.deepcopy(
                self.bag_recorder_probe_diagnostic
            )
            document["physics_unpause"] = copy.deepcopy(
                self.physics_unpause_evidence
            )
            document["fixed_stand_preflight"] = copy.deepcopy(
                self.fixed_stand_preflight
            )
            document["failure_stage"] = "SENSOR_MAP_PREFLIGHT"
            self.wait_for(
                lambda: (
                    self.map_message is not None
                    and self.mapping_usable(self.mapping_status)
                    and all(
                        force is not None for force in self.foot_forces
                    )
                    and self.imu is not None
                ),
                "Livox floor map, mapping health, IMU, and foot forces",
            )
            with self.lock:
                initial_map = copy.deepcopy(self.map_message)
            self.establish_geometry(initial_map)

            document["failure_stage"] = "ROI_AND_CLOSED_DOOR_PREFLIGHT"
            self.wait_for(
                self.baseline_ready,
                "closed-door OccupancyGrid evidence",
                sim_timeout=30.0,
                wall_timeout=600.0,
            )
            with self.lock:
                baseline_map = copy.deepcopy(self.map_message)
            baseline = self.map_evidence(baseline_map)
            document["door_map_before"] = baseline
            document["roi_local_m"] = [
                list(point) for point in self.roi_local
            ]
            document["roi_before"] = self.roi_metrics(baseline_map)
            document["start_pose"] = {
                "frame_id": self.start_pose.header.frame_id,
                "x": self.start_pose.pose.position.x,
                "y": self.start_pose.pose.position.y,
                "yaw": quaternion_yaw(
                    self.start_pose.pose.orientation
                ),
            }
            document["floor_entry_pose"] = {
                "frame_id": self.entry_pose.header.frame_id,
                "x": self.entry_pose.pose.position.x,
                "y": self.entry_pose.pose.position.y,
                "yaw": quaternion_yaw(
                    self.entry_pose.pose.orientation
                ),
            }

            document["failure_stage"] = "CONTROLLER_MODE5"
            self.enter_controller_mode5()
            document["failure_stage"] = "EXPLORE_FLOOR_ACTION"
            action_state, action_result = self.send_exploration(door_id)
            if self.entry_motion_only:
                (
                    post,
                    post_roi,
                    entered_roi,
                    final_zero,
                ) = self.validate_entry_microtest(
                    action_state, action_result, baseline
                )
                final_roi = self.roi_metrics(self.entered_map)
            else:
                (
                    post,
                    post_roi,
                    entered_roi,
                    final_roi,
                    final_zero,
                ) = self.validate_action(
                    action_state, action_result, baseline
                )
            document["door_map_after"] = post
            document["roi_after_door_open"] = post_roi
            document["roi_after_entering"] = entered_roi
            document["roi_final"] = final_roi
            document["entry_new_known_cells"] = (
                entered_roi["known_cells"] - post_roi["known_cells"]
            )
            document["entry_result"] = self.entry_observation
            document["final_zero_before_safe_stop"] = final_zero
            if not self.entry_motion_only:
                document["return_error"] = self.return_errors()

            document["failure_stage"] = "SAFE_STAND_FIXED_STAND"
            safe_stop_result = self.safe_stop()
            if not safe_stop_result["success"]:
                raise AcceptanceFailure(
                    "guarded safe-stop/fixed-stand acceptance failed"
                )

            document["success"] = True
            document["failure_stage"] = ""
        except Exception as error:
            document["error"] = str(error)
            try:
                self.action_client.cancel_goal()
            except Exception:
                pass
            if safe_stop_result is None:
                safe_stop_result = self.safe_stop()
        finally:
            now = self.now_sim()
            document["action"] = {
                "state": int(self.action_client.get_state()),
                "success": (
                    None if action_result is None
                    else bool(action_result.success)
                ),
                "error_code": (
                    None if action_result is None
                    else int(action_result.error_code)
                ),
                "message": (
                    "" if action_result is None else action_result.message
                ),
                "final_coverage_ratio": (
                    None if action_result is None
                    else float(action_result.final_coverage_ratio)
                ),
                "sim_duration_s": (
                    None
                    if (
                        self.action_start_sim is None
                        or self.action_end_sim is None
                    )
                    else self.action_end_sim - self.action_start_sim
                ),
            }
            document["test_total_sim_duration_s"] = (
                None if self.test_start_sim is None
                else now - self.test_start_sim
            )
            document["frontier"] = {
                "targets": self.frontier_targets,
                "selected_count": len(self.frontier_targets),
                "successes": self.frontier_successes,
                "failures": self.frontier_failures,
                "cancelled_or_preempted": self.frontier_cancelled,
            }
            document["completion_reason"] = self.completion_reason
            document["door_behavior_result"] = self.door_result
            document["trajectory"] = self.trajectory_metrics()
            document["max_abs_roll_rad"] = self.max_roll
            document["max_abs_pitch_rad"] = self.max_pitch
            document["controller_probe"] = (
                self.controller_probe_diagnostic
            )
            document["controller_node"] = self.controller_node
            document["mode5_entered"] = self.mode5_entered
            document["mode5_entered_sim_time"] = (
                self.mode5_entered_sim_time
            )
            document["dev_truth_tf_preflight"] = copy.deepcopy(
                self.dev_tf_probe_evidence
            )
            document["safe_stop"] = safe_stop_result
            document["feedback"] = self.feedback
            atomic_json(self.output, document)
            rospy.loginfo(
                "single-floor Gazebo acceptance result: %s",
                json.dumps(document, sort_keys=True),
            )
        return document["success"]


if __name__ == "__main__":
    rospy.init_node("single_floor_gazebo_acceptance")
    success = GazeboAcceptance().run()
    rospy.signal_shutdown("single-floor Gazebo acceptance complete")
    raise SystemExit(0 if success else 1)
