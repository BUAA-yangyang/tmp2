#!/usr/bin/env python3
import math
import threading
import time
import unittest

import actionlib
from a1_navigation_interfaces.msg import (
    ExploreFloorAction,
    ExploreFloorFeedback,
    ExploreFloorGoal,
    ExploreFloorResult,
)
from building_generator_interfaces.srv import (
    SetDoorState,
    SetDoorStateResponse,
)
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from dynamic_reconfigure.msg import DoubleParameter
from dynamic_reconfigure.srv import Reconfigure, ReconfigureResponse
from geometry_msgs.msg import Point32, PoseStamped, TransformStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanResponse
import rospy
import rostest
from std_msgs.msg import Bool
import tf2_ros
from visualization_msgs.msg import Marker


class RuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("single_floor_exploration_runtime_test")
        cls.lock = threading.Lock()
        cls.robot_pose = [-2.0, 0.0, 0.0]
        cls.map_stage = 0
        cls.door_open = False
        cls.door_calls = []
        cls.move_goals = []
        cls.feedback_states = []
        cls.navigation_feedback = threading.Event()
        cls.trajectories = []
        cls.plan_failures_remaining = 1
        cls.reject_entry_plan_with_detour = False
        cls.mapping_degraded_until = 0.0
        cls.mapping_degraded_published = False
        cls.final_cmd_nonzero_until = 0.0
        cls.final_cmd_delay_exercised = False
        cls.controller_ready_enabled = False
        cls.move_delay = 0.15
        cls.failed_markers = []
        cls.dwa_config = {
            "max_vel_x": 0.20,
            "max_vel_y": 0.0,
            "max_vel_trans": 0.20,
            "max_vel_theta": 0.30,
            "min_vel_trans": 0.08,
            "min_vel_theta": 0.25,
        }
        cls.dwa_requests = []

        cls.map_pub = rospy.Publisher(
            "/test/exploration/map", OccupancyGrid,
            queue_size=1, latch=True
        )
        cls.mapping_pub = rospy.Publisher(
            "/test/exploration/mapping_status", DiagnosticStatus,
            queue_size=1, latch=True
        )
        cls.cmd_pub = rospy.Publisher(
            "/test/exploration/cmd_vel", Twist, queue_size=1
        )
        cls.safety_pub = rospy.Publisher(
            "/test/exploration/safety_lock", Bool,
            queue_size=1, latch=True
        )
        cls.ready_pub = rospy.Publisher(
            "/test/exploration/controller_ready", Bool, queue_size=1
        )
        rospy.Subscriber(
            "/test/exploration/trajectory", Path,
            lambda message: cls.trajectories.append(message),
            queue_size=5,
        )
        rospy.Subscriber(
            "/test/exploration/failed", Marker,
            lambda message: cls.failed_markers.append(message),
            queue_size=5,
        )

        cls.tf_broadcaster = tf2_ros.TransformBroadcaster()
        cls.make_plan_service = rospy.Service(
            "/test/move_base/make_plan", GetPlan, cls.make_plan
        )
        cls.door_service = rospy.Service(
            "/test/set_door_state", SetDoorState, cls.set_door_state
        )
        cls.dwa_service = rospy.Service(
            "/test/move_base/DWAPlannerROS/set_parameters",
            Reconfigure,
            cls.reconfigure_dwa,
        )
        cls.move_server = actionlib.SimpleActionServer(
            "/test/move_base",
            MoveBaseAction,
            execute_cb=cls.execute_move,
            auto_start=False,
        )
        cls.move_server.start()
        cls.timer = rospy.Timer(rospy.Duration(0.05), cls.publish_world)
        cls.safety_pub.publish(Bool(data=False))
        time.sleep(1.0)

    @classmethod
    def build_map(cls):
        message = OccupancyGrid()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "odom"
        message.info.map_load_time = message.header.stamp
        message.info.resolution = 0.5
        message.info.width = 20
        message.info.height = 20
        message.info.origin.position.x = -5.0
        message.info.origin.position.y = -5.0
        message.info.origin.orientation.w = 1.0
        data = [-1] * 400
        # Outdoor approach, with a closed two-cell-wide barrier at x=-0.25 m.
        for row in range(8, 12):
            for col in range(4, 10):
                data[row * 20 + col] = 0
        for row in range(8, 12):
            data[row * 20 + 9] = 100

        if cls.door_open:
            # Public door service opens a known-free sensor corridor. No truth
            # topic is involved; this OccupancyGrid is the evidence consumed by
            # the exploration node.
            for row in range(9, 11):
                for col in range(9, 14):
                    data[row * 20 + col] = 0

        if cls.map_stage == 1:
            for row in range(6, 14):
                for col in range(10, 14):
                    data[row * 20 + col] = 0
        elif cls.map_stage == 2:
            for row in range(4, 16):
                for col in range(10, 17):
                    data[row * 20 + col] = 0
        elif cls.map_stage >= 3:
            data = [0] * 400
        message.data = data
        return message

    @classmethod
    def publish_world(cls, _event):
        with cls.lock:
            x, y, yaw = cls.robot_pose
            map_message = cls.build_map()
            mapping_degraded = (
                time.monotonic() < cls.mapping_degraded_until
            )
            if mapping_degraded:
                cls.mapping_degraded_published = True
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(0.5 * yaw)
        transform.transform.rotation.w = math.cos(0.5 * yaw)
        cls.tf_broadcaster.sendTransform(transform)
        cls.map_pub.publish(map_message)

        status = DiagnosticStatus()
        status.level = (
            DiagnosticStatus.WARN
            if mapping_degraded else DiagnosticStatus.OK
        )
        status.name = "test_floor_mapping"
        status.message = "DEGRADED" if mapping_degraded else "MAPPING"
        status.values = [
            KeyValue(
                "state", "DEGRADED" if mapping_degraded else "MAPPING"
            ),
            KeyValue("map_valid", "true"),
            KeyValue("obstacle_cloud_valid", "true"),
            KeyValue("marking_cloud_valid", "true"),
            KeyValue("localization_generation", "7"),
            KeyValue("floor_session_id", "2"),
            KeyValue("floor_id", "1"),
        ]
        cls.mapping_pub.publish(status)
        cls.ready_pub.publish(Bool(data=cls.controller_ready_enabled))
        command = Twist()
        if time.monotonic() < cls.final_cmd_nonzero_until:
            command.angular.z = 0.3
            cls.final_cmd_delay_exercised = True
        cls.cmd_pub.publish(command)

    @classmethod
    def make_plan(cls, request):
        with cls.lock:
            if cls.plan_failures_remaining > 0:
                cls.plan_failures_remaining -= 1
                raise rospy.ServiceException(
                    "intentional one-shot make_plan transport failure"
                )
        response = GetPlanResponse()
        # move_base's standard make_plan service may leave Path.header empty
        # while every returned PoseStamped has the global frame. The business
        # gate validates those explicit pose frames instead of guessing.
        response.plan.header.frame_id = ""
        response.plan.header.stamp = rospy.Time.now()
        if cls.reject_entry_plan_with_detour:
            detour = PoseStamped()
            detour.header.frame_id = "odom"
            detour.header.stamp = response.plan.header.stamp
            detour.pose.position.x = -1.0
            detour.pose.position.y = 2.0
            detour.pose.orientation.w = 1.0
            response.plan.poses = [request.start, detour, request.goal]
        else:
            response.plan.poses = [request.start, request.goal]
        return response

    @classmethod
    def set_door_state(cls, request):
        with cls.lock:
            cls.door_calls.append((request.door_id, bool(request.open)))
            if (
                request.door_id != "runtime-main-entrance"
                or not request.open
            ):
                return SetDoorStateResponse(
                    accepted=False,
                    state="closed",
                    message="unexpected runtime-test door request",
                )
            cls.door_open = True
        return SetDoorStateResponse(
            accepted=True,
            state="open",
            message="runtime-test public door opened",
        )

    @classmethod
    def reconfigure_dwa(cls, request):
        requested = {
            parameter.name: parameter.value
            for parameter in request.config.doubles
        }
        with cls.lock:
            cls.dwa_requests.append(dict(requested))
            cls.dwa_config.update(requested)
            response_values = dict(cls.dwa_config)
        response = ReconfigureResponse()
        response.config.doubles = [
            DoubleParameter(name=name, value=value)
            for name, value in response_values.items()
        ]
        return response

    @classmethod
    def execute_move(cls, goal):
        target = goal.target_pose
        yaw = math.atan2(
            2.0 * target.pose.orientation.w * target.pose.orientation.z,
            1.0 - 2.0 * target.pose.orientation.z ** 2,
        )
        with cls.lock:
            cls.move_goals.append(target)
            is_entry = (
                cls.map_stage == 0
                and math.hypot(
                    target.pose.position.x,
                    target.pose.position.y,
                ) < 0.35
            )
            is_return = (
                cls.map_stage >= 3
                and math.hypot(
                    target.pose.position.x + 2.0,
                    target.pose.position.y,
                ) < 0.45
            )
            cls.robot_pose = [
                target.pose.position.x,
                target.pose.position.y,
                yaw,
            ]
            if is_entry:
                cls.map_stage = 1
            elif not is_return:
                # Two interior frontier successes advance 1 -> 2 -> 3. Stage
                # 3 is completely known and therefore completion-eligible.
                cls.map_stage = min(3, cls.map_stage + 1)
            else:
                # A short scheduling/input-health blip during a long return
                # must stay inside the configured grace window.
                cls.mapping_degraded_until = time.monotonic() + 0.12
                # Model MoveBaseAction reaching success before cmd_mux has
                # propagated and settled the final command to zero.
                cls.final_cmd_nonzero_until = time.monotonic() + 0.70
        time.sleep(cls.move_delay)
        if cls.move_server.is_preempt_requested():
            cls.move_server.set_preempted(MoveBaseResult())
        else:
            cls.move_server.set_succeeded(MoveBaseResult())

    @classmethod
    def on_feedback(cls, message):
        cls.feedback_states.append(message.state)
        if message.state in (
            ExploreFloorFeedback.TRANSIT_TO_ENTRY,
            ExploreFloorFeedback.NAVIGATING,
        ):
            cls.navigation_feedback.set()

    @staticmethod
    def exploration_goal(timeout):
        goal = ExploreFloorGoal()
        goal.floor_id = 0
        goal.timeout_s = timeout
        goal.floor_entry_pose.header.frame_id = "odom"
        goal.floor_entry_pose.pose.orientation.w = 1.0
        goal.roi_local.points = [
            Point32(x=0.0, y=-3.0),
            Point32(x=4.5, y=-3.0),
            Point32(x=4.5, y=3.0),
            Point32(x=0.0, y=3.0),
        ]
        return goal

    def test_00_entry_detour_plan_fails_before_move_base_goal(self):
        client = actionlib.SimpleActionClient(
            "/test/explore_floor", ExploreFloorAction
        )
        self.assertTrue(client.wait_for_server(rospy.Duration(8.0)))
        with RuntimeTest.lock:
            RuntimeTest.robot_pose = [-2.0, 0.0, 0.0]
            RuntimeTest.map_stage = 0
            RuntimeTest.door_open = False
            RuntimeTest.reject_entry_plan_with_detour = True
            RuntimeTest.plan_failures_remaining = 0
        RuntimeTest.controller_ready_enabled = True
        baseline_goals = len(self.move_goals)
        try:
            client.send_goal(self.exploration_goal(5.0))
            self.assertTrue(client.wait_for_result(rospy.Duration(5.0)))
            result = client.get_result()
            self.assertIsNotNone(result)
            self.assertFalse(result.success)
            self.assertEqual(
                result.error_code,
                ExploreFloorResult.ERROR_ENTRY_TRANSIT,
            )
            self.assertIn("no safe local entry plan", result.message)
            self.assertEqual(
                len(self.move_goals),
                baseline_goals,
                "a detouring entry plan must be rejected before send_goal",
            )
        finally:
            RuntimeTest.controller_ready_enabled = False
            with RuntimeTest.lock:
                RuntimeTest.robot_pose = [-2.0, 0.0, 0.0]
                RuntimeTest.map_stage = 0
                RuntimeTest.door_open = False
                RuntimeTest.reject_entry_plan_with_detour = False
                RuntimeTest.plan_failures_remaining = 1
            time.sleep(0.35)

    def test_invalid_floor_entry_pose_fails_before_motion(self):
        client = actionlib.SimpleActionClient(
            "/test/explore_floor", ExploreFloorAction
        )
        self.assertTrue(client.wait_for_server(rospy.Duration(8.0)))
        cases = []

        tilted = self.exploration_goal(5.0)
        tilted.floor_entry_pose.pose.orientation.x = math.sin(0.05)
        tilted.floor_entry_pose.pose.orientation.w = math.cos(0.05)
        cases.append(("tilted", tilted))

        nonunit = self.exploration_goal(5.0)
        nonunit.floor_entry_pose.pose.orientation.w = 0.8
        cases.append(("nonunit", nonunit))

        nonfinite = self.exploration_goal(5.0)
        nonfinite.floor_entry_pose.pose.position.x = float("nan")
        cases.append(("nonfinite", nonfinite))

        nonfinite_orientation = self.exploration_goal(5.0)
        nonfinite_orientation.floor_entry_pose.pose.orientation.z = float(
            "nan"
        )
        cases.append(("nonfinite_orientation", nonfinite_orientation))

        baseline_goals = len(self.move_goals)
        baseline_door_calls = len(self.door_calls)
        for label, goal in cases:
            client.send_goal(goal)
            self.assertTrue(
                client.wait_for_result(rospy.Duration(3.0)), label
            )
            result = client.get_result()
            self.assertIsNotNone(result, label)
            self.assertFalse(result.success, label)
            self.assertEqual(
                result.error_code,
                ExploreFloorResult.ERROR_INVALID_ENTRY_POSE,
                label,
            )
            self.assertIn("invalid floor_entry_pose", result.message)

        wrong_frame = self.exploration_goal(5.0)
        wrong_frame.floor_entry_pose.header.frame_id = "wrong_map_frame"
        RuntimeTest.controller_ready_enabled = True
        try:
            client.send_goal(wrong_frame)
            self.assertTrue(
                client.wait_for_result(rospy.Duration(5.0)),
                "wrong_frame",
            )
            result = client.get_result()
            self.assertFalse(result.success)
            self.assertEqual(
                result.error_code,
                ExploreFloorResult.ERROR_INVALID_ENTRY_POSE,
            )
            self.assertIn("does not match current map frame", result.message)
        finally:
            RuntimeTest.controller_ready_enabled = False
        # Prevent this test's deliberately enabled heartbeat from being a
        # still-fresh prerequisite sample for the following lifecycle test.
        time.sleep(0.35)
        self.assertEqual(len(self.move_goals), baseline_goals)
        self.assertEqual(len(self.door_calls), baseline_door_calls)

    def test_multiple_frontiers_then_return(self):
        client = actionlib.SimpleActionClient(
            "/test/explore_floor", ExploreFloorAction
        )
        self.assertTrue(client.wait_for_server(rospy.Duration(8.0)))
        goal = self.exploration_goal(25.0)
        baseline_goal_count = len(self.move_goals)
        baseline_dwa_request_count = len(self.dwa_requests)
        client.send_goal(goal, feedback_cb=self.on_feedback)
        time.sleep(0.40)
        self.assertEqual(
            len(self.move_goals),
            baseline_goal_count,
            "frontier goal must not be sent before controller_ready",
        )
        RuntimeTest.controller_ready_enabled = True
        self.assertTrue(client.wait_for_result(rospy.Duration(35.0)))
        result = client.get_result()
        self.assertIsNotNone(result)
        self.assertTrue(result.success, result.message)
        self.assertEqual(
            self.plan_failures_remaining,
            0,
            "runtime test must exercise make_plan transient retry",
        )
        self.assertTrue(
            self.mapping_degraded_published,
            "runtime test must exercise a tolerated mapping-health blip",
        )
        self.assertTrue(
            self.final_cmd_delay_exercised,
            "runtime test must exercise delayed final command settling",
        )
        self.assertEqual(result.error_code, 0)
        entry_dwa_requests = self.dwa_requests[
            baseline_dwa_request_count:baseline_dwa_request_count + 3
        ]
        self.assertEqual(entry_dwa_requests[0], {})
        self.assertEqual(
            entry_dwa_requests[1],
            {
                "max_vel_x": 0.05,
                "max_vel_y": 0.0,
                "max_vel_trans": 0.05,
                "max_vel_theta": 0.15,
                "min_vel_trans": 0.02,
                "min_vel_theta": 0.05,
            },
        )
        self.assertEqual(
            entry_dwa_requests[2],
            {
                "max_vel_x": 0.20,
                "max_vel_y": 0.0,
                "max_vel_trans": 0.20,
                "max_vel_theta": 0.30,
                "min_vel_trans": 0.08,
                "min_vel_theta": 0.25,
            },
        )
        self.assertEqual(self.dwa_config["max_vel_x"], 0.20)
        self.assertEqual(self.dwa_config["max_vel_trans"], 0.20)
        self.assertEqual(self.dwa_config["min_vel_trans"], 0.08)
        self.assertEqual(self.dwa_config["min_vel_theta"], 0.25)
        self.assertGreaterEqual(len(self.move_goals), 4)
        self.assertGreaterEqual(self.map_stage, 3)
        self.assertTrue(self.door_calls)
        self.assertEqual(
            self.door_calls[0], ("runtime-main-entrance", True)
        )
        final = self.move_goals[-1]
        self.assertLess(
            math.hypot(
                final.pose.position.x + 2.0,
                final.pose.position.y,
            ),
            0.45,
        )
        for state in (
            ExploreFloorFeedback.RECORD_START,
            ExploreFloorFeedback.REQUEST_ENTRY_DOOR_OPEN,
            ExploreFloorFeedback.TRANSIT_TO_ENTRY,
            ExploreFloorFeedback.ENTERED_FLOOR,
            ExploreFloorFeedback.NAVIGATING,
            ExploreFloorFeedback.EXPLORATION_DONE,
            ExploreFloorFeedback.RETURNING,
            ExploreFloorFeedback.RETURNED,
        ):
            self.assertIn(state, self.feedback_states)
        self.assertTrue(self.trajectories)
        self.assertGreaterEqual(len(self.trajectories[-1].poses), 1)

        code, _message, state = rospy.get_master().getSystemState()
        self.assertEqual(code, 1)
        publishers = {topic: nodes for topic, nodes in state[0]}
        for topic in (
            "/cmd_vel",
            "/cmd_vel_nav",
            "/test/exploration/cmd_vel",
        ):
            self.assertNotIn(
                "/frontier_explorer_runtime_test",
                publishers.get(topic, []),
                "exploration must never publish velocity",
            )

        # Loss of controller-ready during a real navigation attempt aborts the
        # action, but it is an infrastructure failure rather than evidence that
        # the frontier is unreachable.
        with RuntimeTest.lock:
            RuntimeTest.robot_pose = [-2.0, 0.0, 0.0]
            RuntimeTest.map_stage = 0
            RuntimeTest.door_open = False
            RuntimeTest.move_delay = 1.0
            RuntimeTest.failed_markers = []
        self.navigation_feedback.clear()
        readiness_client = actionlib.SimpleActionClient(
            "/test/explore_floor", ExploreFloorAction
        )
        self.assertTrue(readiness_client.wait_for_server(rospy.Duration(3.0)))
        readiness_goal = self.exploration_goal(10.0)
        readiness_client.send_goal(
            readiness_goal, feedback_cb=self.on_feedback
        )
        self.assertTrue(self.navigation_feedback.wait(8.0))
        RuntimeTest.controller_ready_enabled = False
        self.assertTrue(
            readiness_client.wait_for_result(rospy.Duration(8.0))
        )
        readiness_result = readiness_client.get_result()
        self.assertFalse(readiness_result.success)
        self.assertEqual(
            readiness_result.error_code,
            ExploreFloorResult.ERROR_PRECONDITION,
        )
        self.assertFalse(
            any(marker.points for marker in RuntimeTest.failed_markers),
            "controller loss must not add a permanent unreachable target",
        )
        RuntimeTest.controller_ready_enabled = True
        RuntimeTest.move_delay = 0.15
        time.sleep(0.35)

        # Safety lock must cancel the active MoveBaseAction and terminate
        # without attempting an autonomous return while the lock is active.
        with RuntimeTest.lock:
            RuntimeTest.robot_pose = [-2.0, 0.0, 0.0]
            RuntimeTest.map_stage = 0
            RuntimeTest.door_open = False
        time.sleep(0.25)
        self.navigation_feedback.clear()
        safety_client = actionlib.SimpleActionClient(
            "/test/explore_floor", ExploreFloorAction
        )
        self.assertTrue(safety_client.wait_for_server(rospy.Duration(3.0)))
        safety_goal = self.exploration_goal(10.0)
        safety_client.send_goal(
            safety_goal, feedback_cb=self.on_feedback
        )
        self.assertTrue(self.navigation_feedback.wait(8.0))
        self.safety_pub.publish(Bool(data=True))
        self.assertTrue(
            safety_client.wait_for_result(rospy.Duration(8.0))
        )
        safety_result = safety_client.get_result()
        self.assertFalse(safety_result.success)
        self.assertEqual(
            safety_result.error_code,
            ExploreFloorResult.ERROR_SAFETY_STOP,
        )
        self.assertFalse(
            any(marker.points for marker in RuntimeTest.failed_markers),
            "safety lock must not add a permanent unreachable target",
        )
        self.safety_pub.publish(Bool(data=False))


if __name__ == "__main__":
    rostest.rosrun(
        "a1_navigation_tests",
        "single_floor_exploration_runtime",
        RuntimeTest,
    )
