#!/usr/bin/env python3
"""ROS-independent checks for the Gazebo acceptance evidence helpers."""

import importlib.util
import math
import os
import pathlib
import threading
import unittest

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import yaml


SCRIPT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "scripts",
        "single_floor_gazebo_acceptance.py",
    )
)
SPEC = importlib.util.spec_from_file_location(
    "single_floor_gazebo_acceptance", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class AcceptanceHelperTest(unittest.TestCase):
    @staticmethod
    def controller_state(*nodes):
        publishers = []
        for topic in MODULE.MOTOR_COMMAND_TOPICS:
            publishers.append((topic, list(nodes)))
        subscribers = [("/joy", list(nodes))]
        return publishers, subscribers, []

    @staticmethod
    def fresh_motor_stamps(now):
        return {
            topic: now for topic in MODULE.MOTOR_COMMAND_TOPICS
        }

    @staticmethod
    def grid():
        message = OccupancyGrid()
        message.info.width = 12
        message.info.height = 8
        message.info.resolution = 0.5
        message.info.origin.position.x = -3.0
        message.info.origin.position.y = -2.0
        data = [-1] * (12 * 8)
        for row in (3, 4):
            for column in range(1, 10):
                data[row * 12 + column] = 0
        message.data = data
        return message

    def test_corridor_reports_closed_then_known_free_entry(self):
        message = self.grid()
        data = list(message.data)
        data[3 * 12 + 6] = 100
        data[4 * 12 + 6] = 100
        message.data = data
        closed = MODULE.corridor_evidence(
            message, (-2.0, 0.0), (1.5, 0.0), half_width=0.35
        )
        self.assertFalse(closed["known_free_path"])
        self.assertGreater(closed["occupied_cells"], 0)

        data[3 * 12 + 6] = 0
        data[4 * 12 + 6] = 0
        message.data = data
        opened = MODULE.corridor_evidence(
            message, (-2.0, 0.0), (1.5, 0.0), half_width=0.35
        )
        self.assertTrue(opened["known_free_path"])
        self.assertLess(
            opened["occupied_cells"], closed["occupied_cells"]
        )

    def test_floor_mapping_costmap_separates_marking_and_clearing_clouds(self):
        a1_root = pathlib.Path(__file__).resolve().parents[2]
        floor_config = yaml.safe_load(
            (
                a1_root / "floor_mapping" / "config" / "floor_mapping.yaml"
            ).read_text()
        )
        obstacle_config = yaml.safe_load(
            (
                a1_root
                / "navigation"
                / "config"
                / "obstacle_layer_floor_mapping_params.yaml"
            ).read_text()
        )
        self.assertNotEqual(
            floor_config["topics"]["obstacle_cloud"],
            floor_config["topics"]["clearing_cloud"],
        )
        self.assertEqual(
            obstacle_config["mapping_marking"]["topic"],
            "/a1_nav/obstacle_cloud",
        )
        self.assertTrue(obstacle_config["mapping_marking"]["marking"])
        self.assertFalse(obstacle_config["mapping_marking"]["clearing"])
        self.assertEqual(
            obstacle_config["mapping_clearing"]["topic"],
            "/a1_nav/clearing_cloud",
        )
        self.assertFalse(obstacle_config["mapping_clearing"]["marking"])
        self.assertTrue(obstacle_config["mapping_clearing"]["clearing"])

    def test_corridor_uses_bounded_known_free_anchor_under_robot_occlusion(self):
        message = self.grid()
        spec = MODULE.grid_spec(message)
        start = (-2.0, 0.0)
        start_cell = spec.world_to_cell(*start)
        data = list(message.data)
        data[start_cell[0] * spec.width + start_cell[1]] = -1
        message.data = data
        evidence = MODULE.corridor_evidence(
            message, start, (1.5, 0.0), half_width=0.35
        )
        self.assertTrue(evidence["known_free_path"])
        self.assertIsNotNone(evidence["path_anchor"])
        self.assertLessEqual(evidence["path_anchor_offset_m"], 0.60)

    def test_angle_error_wraps_at_pi(self):
        self.assertAlmostEqual(
            MODULE.angle_error(math.pi - 0.1, -math.pi + 0.1),
            0.2,
        )

    def test_actual_controller_node_name_is_discovered_dynamically(self):
        state = self.controller_state("/unitree_gazebo_servo")
        diagnostic = MODULE.controller_graph_diagnostic(state)
        probe = MODULE.evaluate_controller_probe(
            diagnostic,
            10.0,
            10.0,
            self.fresh_motor_stamps(10.0),
        )
        self.assertTrue(probe["ready"])
        self.assertEqual(
            probe["selected_node"], "/unitree_gazebo_servo"
        )

    def test_remapped_controller_node_name_is_supported(self):
        state = self.controller_state("/test/remapped_a1_controller")
        diagnostic = MODULE.controller_graph_diagnostic(state)
        probe = MODULE.evaluate_controller_probe(
            diagnostic,
            12.0,
            12.0,
            self.fresh_motor_stamps(12.0),
        )
        self.assertTrue(probe["ready"])
        self.assertEqual(
            probe["selected_node"], "/test/remapped_a1_controller"
        )

    def test_zero_controller_candidates_fails_closed_with_sets(self):
        publishers = [
            (topic, ["/motor_only"])
            for topic in MODULE.MOTOR_COMMAND_TOPICS
        ]
        diagnostic = MODULE.controller_graph_diagnostic(
            (publishers, [("/joy", ["/joy_only"])], [])
        )
        probe = MODULE.evaluate_controller_probe(
            diagnostic,
            3.0,
            3.0,
            self.fresh_motor_stamps(3.0),
        )
        self.assertFalse(probe["ready"])
        self.assertEqual(probe["candidate_count"], 0)
        self.assertEqual(probe["intersection"], [])
        self.assertEqual(probe["joy_subscribers"], ["/joy_only"])
        self.assertEqual(probe["motor_publishers"], ["/motor_only"])

    def test_multiple_controller_candidates_fails_closed_with_sets(self):
        state = self.controller_state("/controller_a", "/controller_b")
        diagnostic = MODULE.controller_graph_diagnostic(state)
        probe = MODULE.evaluate_controller_probe(
            diagnostic,
            4.0,
            4.0,
            self.fresh_motor_stamps(4.0),
        )
        self.assertFalse(probe["ready"])
        self.assertEqual(probe["candidate_count"], 2)
        self.assertEqual(
            probe["intersection"],
            ["/controller_a", "/controller_b"],
        )
        self.assertIn("multiple", probe["reason"])

    def test_stale_joy_or_motor_message_fails_closed(self):
        state = self.controller_state("/controller")
        diagnostic = MODULE.controller_graph_diagnostic(state)
        stale_joy = MODULE.evaluate_controller_probe(
            diagnostic,
            5.0,
            4.0,
            self.fresh_motor_stamps(5.0),
        )
        self.assertFalse(stale_joy["ready"])
        self.assertIn("Joy", stale_joy["reason"])

        motor_stamps = self.fresh_motor_stamps(5.0)
        motor_stamps[MODULE.MOTOR_COMMAND_TOPICS[0]] = 4.0
        stale_motor = MODULE.evaluate_controller_probe(
            diagnostic, 5.0, 5.0, motor_stamps
        )
        self.assertFalse(stale_motor["ready"])
        self.assertEqual(
            stale_motor["stale_motor_topics"],
            [MODULE.MOTOR_COMMAND_TOPICS[0]],
        )

    def test_bag_recorder_requires_all_goal_topics(self):
        recorder = MODULE.DEFAULT_BAG_RECORDER_NODE
        subscribers = [
            (topic, [recorder]) for topic in MODULE.REQUIRED_BAG_TOPICS
        ]
        ready = MODULE.bag_recorder_graph_diagnostic(
            ([], subscribers, [])
        )
        self.assertTrue(ready["ready"])
        self.assertEqual(ready["missing_topics"], [])

        subscribers.pop()
        incomplete = MODULE.bag_recorder_graph_diagnostic(
            ([], subscribers, [])
        )
        self.assertFalse(incomplete["ready"])
        self.assertEqual(incomplete["missing_topics"], ["/move_base/goal"])

    def test_bag_recorder_name_is_resolved_without_hardcoding_namespace(self):
        def resolve(name):
            if name.startswith("/"):
                return name
            return "/test/" + name

        subscribers = [
            (topic, ["/test/recorder"])
            for topic in MODULE.REQUIRED_BAG_TOPICS
        ]
        diagnostic = MODULE.bag_recorder_graph_diagnostic(
            ([], subscribers, []),
            recorder_node="recorder",
            resolve_name=resolve,
        )
        self.assertTrue(diagnostic["ready"])
        self.assertEqual(diagnostic["recorder_node"], "/test/recorder")

    def test_entry_quaternion_is_planar_and_preserves_yaw(self):
        yaw = -1.73
        x, y, z, w = MODULE.quaternion_from_yaw(yaw)
        self.assertEqual(x, 0.0)
        self.assertEqual(y, 0.0)
        self.assertAlmostEqual(math.hypot(z, w), 1.0)
        quaternion = type(
            "Quaternion", (),
            {"x": x, "y": y, "z": z, "w": w},
        )()
        self.assertAlmostEqual(MODULE.quaternion_yaw(quaternion), yaw)
        with self.assertRaises(ValueError):
            MODULE.quaternion_from_yaw(float("nan"))

    def test_pre_motion_abort_does_not_require_safe_stand(self):
        self.assertEqual(
            MODULE.safe_stop_policy(False), "PRE_MOTION_ABORT"
        )
        self.assertEqual(
            MODULE.safe_stop_policy(True), "FULL_GAIT_SAFE_STOP"
        )

    def test_fixed_stand_requires_fresh_physical_support(self):
        evidence = MODULE.fixed_stand_evidence(
            10.0,
            (0.256, 0.01, -0.02),
            9.95,
            [0.10] * 12,
            9.95,
            [20.0, 20.0, 20.0, 20.0],
            [9.95] * 4,
            (0.01, -0.02, (0.01, 0.02, 0.03)),
            9.95,
        )
        self.assertTrue(MODULE.fixed_stand_sample_is_ready(evidence))
        self.assertAlmostEqual(
            evidence["dev_only_odom_height_m"], 0.256
        )
        self.assertGreater(
            MODULE.FIXED_STAND_MIN_BASE_HEIGHT_M, 0.12
        )

    def test_fixed_stand_rejects_folded_stale_or_moving_state(self):
        arguments = [
            10.0,
            (0.12, 0.01, -0.02),
            9.95,
            [0.10] * 12,
            9.95,
            [20.0, 20.0, 20.0, 20.0],
            [9.95] * 4,
            (0.01, -0.02, (0.01, 0.02, 0.03)),
            9.95,
        ]
        folded = MODULE.fixed_stand_evidence(*arguments)
        self.assertFalse(MODULE.fixed_stand_sample_is_ready(folded))

        arguments[1] = (0.35, 0.01, -0.02)
        arguments[6] = [9.95, 9.95, 9.95, 9.0]
        stale_force = MODULE.fixed_stand_evidence(*arguments)
        self.assertFalse(
            MODULE.fixed_stand_sample_is_ready(stale_force)
        )

        arguments[6] = [9.95] * 4
        arguments[3] = [0.10] * 11 + [0.60]
        moving = MODULE.fixed_stand_evidence(*arguments)
        self.assertFalse(MODULE.fixed_stand_sample_is_ready(moving))

    def test_fixed_stand_does_not_relax_tilt_gyro_or_force_limits(self):
        def evidence(imu, forces):
            return MODULE.fixed_stand_evidence(
                20.0,
                (0.35, 0.0, 0.0),
                19.95,
                [0.10] * 12,
                19.95,
                forces,
                [19.95] * 4,
                imu,
                19.95,
            )

        tilted = evidence(
            (MODULE.FIXED_STAND_MAX_TILT_RAD + 0.01, 0.0,
             (0.0, 0.0, 0.0)),
            [10.0] * 4,
        )
        rotating = evidence(
            (0.0, 0.0,
             (0.0, 0.0, MODULE.FIXED_STAND_MAX_GYRO_RAD_S + 0.01)),
            [10.0] * 4,
        )
        unsupported = evidence(
            (0.0, 0.0, (0.0, 0.0, 0.0)),
            [10.0, 10.0, 10.0, 4.99],
        )
        self.assertFalse(MODULE.fixed_stand_sample_is_ready(tilted))
        self.assertFalse(MODULE.fixed_stand_sample_is_ready(rotating))
        self.assertFalse(MODULE.fixed_stand_sample_is_ready(unsupported))

    def test_entry_microtest_only_accepts_deliberate_cancel(self):
        result = type(
            "Result",
            (),
            {
                "success": False,
                "error_code": MODULE.ExploreFloorResult.ERROR_CANCELLED,
            },
        )()
        self.assertTrue(
            MODULE.entry_micro_action_is_expected(
                MODULE.GoalStatus.PREEMPTED, result
            )
        )
        result.success = True
        self.assertFalse(
            MODULE.entry_micro_action_is_expected(
                MODULE.GoalStatus.PREEMPTED, result
            )
        )

    def test_dev_tf_preflight_requires_fresh_finite_transform(self):
        valid = MODULE.dev_tf_sample_evidence(
            10.0,
            9.9,
            (1.0, 2.0, 0.4),
            (0.0, 0.0, 0.0, 1.0),
        )
        self.assertTrue(valid["ready"])
        self.assertTrue(valid["fresh"])

        stale = MODULE.dev_tf_sample_evidence(
            10.0,
            9.0,
            (1.0, 2.0, 0.4),
            (0.0, 0.0, 0.0, 1.0),
        )
        future = MODULE.dev_tf_sample_evidence(
            10.0,
            10.1,
            (1.0, 2.0, 0.4),
            (0.0, 0.0, 0.0, 1.0),
        )
        nonfinite = MODULE.dev_tf_sample_evidence(
            10.0,
            10.0,
            (float("nan"), 2.0, 0.4),
            (0.0, 0.0, 0.0, 1.0),
        )
        zero_quaternion = MODULE.dev_tf_sample_evidence(
            10.0,
            10.0,
            (1.0, 2.0, 0.4),
            (0.0, 0.0, 0.0, 0.0),
        )
        self.assertFalse(stale["ready"])
        self.assertFalse(future["ready"])
        self.assertFalse(nonfinite["ready"])
        self.assertFalse(zero_quaternion["ready"])

    def test_safe_stand_filter_rejects_sustained_rotation_not_spike(self):
        filter_under_test = MODULE.SafeStandGyroFilter()
        first = filter_under_test.update(1.0, (0.0, 0.0, 0.0))
        spike = filter_under_test.update(1.01, (0.0, 0.0, 0.30))
        recovered = filter_under_test.update(1.02, (0.0, 0.0, 0.0))
        self.assertTrue(first["valid"])
        self.assertLess(spike["norm"], MODULE.SAFE_STAND_MAX_GYRO_RAD_S)
        self.assertLess(
            recovered["norm"], MODULE.SAFE_STAND_MAX_GYRO_RAD_S
        )

        result = None
        for index in range(1, 31):
            result = filter_under_test.update(
                1.02 + index * 0.01, (0.0, 0.0, 0.30)
            )
        self.assertGreater(
            result["norm"], MODULE.SAFE_STAND_MAX_GYRO_RAD_S
        )

    def test_safe_stand_filter_discontinuity_and_nonfinite_fail_closed(self):
        filter_under_test = MODULE.SafeStandGyroFilter()
        filter_under_test.update(2.0, (0.0, 0.0, 0.0))
        gap = filter_under_test.update(2.1, (0.0, 0.0, 0.0))
        self.assertTrue(gap["valid"])
        self.assertTrue(gap["discontinuity"])
        rollback = filter_under_test.update(
            1.9, (0.0, 0.0, 0.0)
        )
        self.assertTrue(rollback["discontinuity"])
        nonfinite = filter_under_test.update(
            1.91, (float("nan"), 0.0, 0.0)
        )
        self.assertFalse(nonfinite["valid"])
        self.assertTrue(nonfinite["discontinuity"])

    def test_safe_stand_edge_uses_latched_gate_time_evidence(self):
        filtered = {
            "valid": True,
            "discontinuity": False,
            "norm": 0.10,
        }
        evidence = MODULE.safe_stand_edge_evidence(
            20.0,
            [10.0, 11.0, 12.0, 13.0],
            [19.9, 19.9, 19.9, 19.9],
            (0.05, -0.04, (0.0, 0.0, 0.20)),
            filtered,
        )
        self.assertTrue(MODULE.safe_stand_edge_is_valid(evidence))
        self.assertGreater(
            evidence["raw_gyro_norm_rad_s"],
            MODULE.SAFE_STAND_MAX_GYRO_RAD_S,
        )

        post_transition = dict(evidence)
        post_transition["foot_forces_n"] = [0.0, 0.0, 30.0, 30.0]
        self.assertFalse(
            MODULE.safe_stand_edge_is_valid(post_transition)
        )

    def test_safe_stand_edge_fails_for_stale_force_tilt_or_filter(self):
        base = MODULE.safe_stand_edge_evidence(
            30.0,
            [8.0, 8.0, 8.0, 8.0],
            [29.9, 29.9, 29.9, 29.9],
            (0.0, 0.0, (0.0, 0.0, 0.0)),
            {
                "valid": True,
                "discontinuity": False,
                "norm": 0.0,
            },
        )
        self.assertTrue(MODULE.safe_stand_edge_is_valid(base))

        stale = dict(base)
        stale["foot_force_fresh"] = [True, True, True, False]
        tilted = dict(base)
        tilted["roll_rad"] = MODULE.SAFE_STAND_MAX_TILT_RAD + 0.01
        discontinuous = dict(base)
        discontinuous["filtered_gyro"] = {
            "valid": True,
            "discontinuity": True,
            "norm": 0.0,
        }
        rotating = dict(base)
        rotating["filtered_gyro"] = {
            "valid": True,
            "discontinuity": False,
            "norm": MODULE.SAFE_STAND_MAX_GYRO_RAD_S + 0.01,
        }
        self.assertFalse(MODULE.safe_stand_edge_is_valid(stale))
        self.assertFalse(MODULE.safe_stand_edge_is_valid(tilted))
        self.assertFalse(MODULE.safe_stand_edge_is_valid(discontinuous))
        self.assertFalse(MODULE.safe_stand_edge_is_valid(rotating))

    def test_safe_stand_rising_edge_is_latched_once(self):
        acceptance = MODULE.GazeboAcceptance.__new__(
            MODULE.GazeboAcceptance
        )
        acceptance.lock = threading.RLock()
        acceptance.safe_stand_ready = False
        acceptance.safe_stand_seen = False
        acceptance.safe_stand_edge = None
        acceptance.foot_forces = [9.0, 9.0, 9.0, 9.0]
        acceptance.foot_stamps = [39.9, 39.9, 39.9, 39.9]
        acceptance.imu = (0.0, 0.0, (0.0, 0.0, 0.0))
        acceptance.filtered_gyro = {
            "valid": True,
            "discontinuity": False,
            "norm": 0.0,
        }
        acceptance.now_sim = lambda: 40.0

        acceptance.safe_stand_callback(Bool(data=True))
        first = acceptance.safe_stand_edge
        acceptance.foot_forces = [0.0, 0.0, 30.0, 30.0]
        acceptance.safe_stand_callback(Bool(data=True))
        self.assertIs(acceptance.safe_stand_edge, first)
        self.assertEqual(
            acceptance.safe_stand_edge["foot_forces_n"],
            [9.0, 9.0, 9.0, 9.0],
        )


if __name__ == "__main__":
    unittest.main()
