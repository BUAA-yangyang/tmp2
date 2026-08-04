#!/usr/bin/env python3
"""Static contract checks for the dev-only indoor-start adapter."""

import ast
import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
ADAPTER = PACKAGE / "scripts" / "indoor_start_frontier_explorer.py"
INDOOR_LAUNCH = (
    PACKAGE / "launch" / "single_floor_indoor_start_acceptance.launch"
)
PRODUCTION_LAUNCH = PACKAGE / "launch" / "single_floor_exploration_dev.launch"
RUNNER = PACKAGE / "scripts" / "run_indoor_start_once.sh"


class IndoorStartContractTest(unittest.TestCase):
    def test_adapter_is_acknowledged_and_delegates_normal_navigation(self):
        source = ADAPTER.read_text()
        tree = ast.parse(source)
        class_node = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "IndoorStartFrontierExplorer"
        )
        methods = {
            node.name: node
            for node in class_node.body
            if isinstance(node, ast.FunctionDef)
        }
        self.assertIn("dev_indoor_start_ack", source)
        self.assertIn("_skip_entry_navigation", source)
        navigate = methods["navigate"]
        super_navigation_calls = [
            node
            for node in ast.walk(navigate)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "navigate"
            and isinstance(node.func.value, ast.Call)
            and isinstance(node.func.value.func, ast.Name)
            and node.func.value.func.id == "super"
        ]
        self.assertEqual(len(super_navigation_calls), 1)

    def test_only_indoor_launch_disables_production_entry_nodes(self):
        indoor = ET.parse(INDOOR_LAUNCH).getroot()
        includes = indoor.findall("include")
        self.assertEqual(len(includes), 1)
        include_args = {
            item.attrib["name"]: item.attrib["value"]
            for item in includes[0].findall("arg")
        }
        self.assertEqual(include_args["start_building_behavior"], "false")
        self.assertEqual(include_args["start_exploration"], "false")
        adapter = next(
            node
            for node in indoor.findall("node")
            if node.attrib.get("type") == "indoor_start_frontier_explorer.py"
        )
        params = {
            item.attrib["name"]: item.attrib["value"]
            for item in adapter.findall("param")
        }
        self.assertEqual(params["dev_indoor_start_ack"], "true")

        production = ET.parse(PRODUCTION_LAUNCH).getroot()
        defaults = {
            item.attrib["name"]: item.attrib.get("default")
            for item in production.findall("arg")
        }
        self.assertEqual(defaults["start_building_behavior"], "true")
        self.assertEqual(defaults["start_exploration"], "true")

    def test_test_coordinates_are_not_passed_to_production_launch(self):
        production_text = PRODUCTION_LAUNCH.read_text()
        for name in (
            "expected_start_x",
            "expected_start_y",
            "expected_start_yaw",
            "minimum_roi_global_y",
        ):
            self.assertNotIn(name, production_text)

    def test_runner_waits_for_stable_pause_and_parses_top_level_result(self):
        source = RUNNER.read_text()
        cleanup_marker = source.index(
            'grep -Fq "Sourcing ROS environment..." "$SIM_LOG"'
        )
        readiness = source.index('"$RUNNER_HELPER" wait-ready')
        controller_pid_argument = source.index(
            '--controller-pid-file "$WORKSPACE_DIR/logs/junior_ctrl.pid"'
        )
        self.assertLess(
            cleanup_marker,
            readiness,
        )
        self.assertLess(
            readiness,
            controller_pid_argument,
        )
        self.assertLess(
            readiness,
            source.index(
                "roslaunch a1_navigation_tests "
                "single_floor_indoor_start_acceptance.launch"
            ),
        )
        self.assertIn(
            "simulation process exited during auto.sh startup cleanup",
            source,
        )
        self.assertIn(
            "timed out waiting for auto.sh startup cleanup to complete",
            source,
        )
        self.assertIn("check-result", source)
        self.assertNotIn("grep -q '\"success\": true'", source)
        self.assertIn('[ ! -s "$BAG_PATH" ]', source)
        self.assertIn('rosbag info --yaml "$BAG_PATH"', source)
        self.assertIn('kill -TERM -- "-$SIM_PID"', source)
        self.assertIn('kill -KILL -- "-$SIM_PID"', source)


if __name__ == "__main__":
    unittest.main()
