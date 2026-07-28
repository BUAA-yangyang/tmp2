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
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanResponse
import rospy
import rostest
from std_msgs.msg import Bool
import tf2_ros


class RuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("single_floor_exploration_runtime_test")
        cls.lock = threading.Lock()
        cls.robot_pose = [0.0, 0.0, 0.0]
        cls.map_stage = 0
        cls.move_goals = []
        cls.feedback_states = []
        cls.navigation_feedback = threading.Event()
        cls.trajectories = []
        cls.plan_failures_remaining = 1
        cls.mapping_degraded_until = 0.0
        cls.mapping_degraded_published = False
        cls.final_cmd_nonzero_until = 0.0
        cls.final_cmd_delay_exercised = False

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
        rospy.Subscriber(
            "/test/exploration/trajectory", Path,
            lambda message: cls.trajectories.append(message),
            queue_size=5,
        )

        cls.tf_broadcaster = tf2_ros.TransformBroadcaster()
        cls.make_plan_service = rospy.Service(
            "/test/move_base/make_plan", GetPlan, cls.make_plan
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
        if cls.map_stage == 0:
            bounds = (7, 13)
        elif cls.map_stage == 1:
            bounds = (4, 16)
        else:
            bounds = (0, 20)
        for row in range(bounds[0], bounds[1]):
            for col in range(bounds[0], bounds[1]):
                data[row * 20 + col] = 0
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
            KeyValue("localization_generation", "7"),
            KeyValue("floor_session_id", "2"),
            KeyValue("floor_id", "1"),
        ]
        cls.mapping_pub.publish(status)
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
        response.plan.header.frame_id = "odom"
        response.plan.header.stamp = rospy.Time.now()
        response.plan.poses = [request.start, request.goal]
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
            is_return = (
                cls.map_stage >= 2
                and math.hypot(
                    target.pose.position.x, target.pose.position.y
                ) < 0.45
            )
            cls.robot_pose = [
                target.pose.position.x,
                target.pose.position.y,
                yaw,
            ]
            if not is_return:
                cls.map_stage = min(2, cls.map_stage + 1)
            else:
                # A short scheduling/input-health blip during a long return
                # must stay inside the configured grace window.
                cls.mapping_degraded_until = time.monotonic() + 0.12
                # Model MoveBaseAction reaching success before cmd_mux has
                # propagated and settled the final command to zero.
                cls.final_cmd_nonzero_until = time.monotonic() + 0.70
        time.sleep(0.15)
        if cls.move_server.is_preempt_requested():
            cls.move_server.set_preempted(MoveBaseResult())
        else:
            cls.move_server.set_succeeded(MoveBaseResult())

    @classmethod
    def on_feedback(cls, message):
        cls.feedback_states.append(message.state)
        if message.state == ExploreFloorFeedback.NAVIGATING:
            cls.navigation_feedback.set()

    def test_multiple_frontiers_then_return(self):
        client = actionlib.SimpleActionClient(
            "/test/explore_floor", ExploreFloorAction
        )
        self.assertTrue(client.wait_for_server(rospy.Duration(8.0)))
        goal = ExploreFloorGoal()
        goal.floor_id = 1
        goal.timeout_s = 25.0
        client.send_goal(goal, feedback_cb=self.on_feedback)
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
        self.assertGreaterEqual(len(self.move_goals), 3)
        self.assertGreaterEqual(self.map_stage, 2)
        final = self.move_goals[-1]
        self.assertLess(
            math.hypot(
                final.pose.position.x,
                final.pose.position.y,
            ),
            0.45,
        )
        for state in (
            ExploreFloorFeedback.RECORD_START,
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

        # Safety lock must cancel the active MoveBaseAction and terminate
        # without attempting an autonomous return while the lock is active.
        with RuntimeTest.lock:
            RuntimeTest.robot_pose = [0.0, 0.0, 0.0]
            RuntimeTest.map_stage = 0
        time.sleep(0.25)
        self.navigation_feedback.clear()
        safety_client = actionlib.SimpleActionClient(
            "/test/explore_floor", ExploreFloorAction
        )
        self.assertTrue(safety_client.wait_for_server(rospy.Duration(3.0)))
        safety_goal = ExploreFloorGoal()
        safety_goal.floor_id = 1
        safety_goal.timeout_s = 10.0
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
        self.safety_pub.publish(Bool(data=False))


if __name__ == "__main__":
    rostest.rosrun(
        "a1_navigation_tests",
        "single_floor_exploration_runtime",
        RuntimeTest,
    )
