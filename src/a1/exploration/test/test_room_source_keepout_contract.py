#!/usr/bin/env python3
"""Keep the official source-free zone out of mandatory traversal paths."""

import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


class RoomSourceKeepoutContractTest(unittest.TestCase):
    @staticmethod
    def load_explorer_module():
        script = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "frontier_explorer_node.py"
        )
        spec = importlib.util.spec_from_file_location(
            "frontier_explorer_node_keepout_contract", script
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_only_optional_room_targets_consume_the_keepout_mask(self):
        script = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "frontier_explorer_node.py"
        )
        tree = ast.parse(script.read_text(encoding="utf-8"))
        callers = set()
        for function in (
                node for node in ast.walk(tree)
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))):
            if any(
                    isinstance(call, ast.Call)
                    and isinstance(call.func, ast.Attribute)
                    and call.func.attr == "build_room_source_keepout_mask"
                    for call in ast.walk(function)):
                callers.add(function.name)
        self.assertEqual(
            callers,
            {"room_transaction_frontiers", "room_camera_gap"},
            "source-free placement knowledge may suppress optional room "
            "frontiers/coverage only, never mandatory doorway or corridor "
            "traversal",
        )

    def test_room_frontier_dilation_cannot_reintroduce_keepout_cells(self):
        module = self.load_explorer_module()
        explorer = object.__new__(module.FrontierExplorer)
        component = np.zeros((7, 7), dtype=bool)
        component[2:5, 2:5] = True
        keepout = np.zeros((7, 7), dtype=bool)
        keepout[1:6, 4:6] = True
        captured = {}

        explorer.roi_enabled = False
        explorer.room_free_component_mask = (
            lambda _map, _branch, _pose: component.copy()
        )
        explorer.build_room_source_keepout_mask = (
            lambda _map, _branch: keepout.copy()
        )
        explorer.grid_spec = lambda _map: SimpleNamespace()
        explorer.min_frontier_length = 0.2
        explorer.obstacle_clearance = 0.2
        explorer.goal_standoff = 0.2
        explorer.goal_search_radius = 0.2
        explorer.room_frontier_min_distance = 0.1
        explorer.max_goal_distance = 10.0
        explorer.free_threshold = 20
        explorer.occupied_threshold = 65
        explorer.information_gain_weight = 1.0
        explorer.distance_weight = 0.25

        original_extract = module.extract_frontiers
        try:
            def capture_extract(*_args, **kwargs):
                captured["allowed"] = kwargs["allowed_mask"].copy()
                return []

            module.extract_frontiers = capture_extract
            pose = SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0)
                )
            )
            result = explorer.room_transaction_frontiers(
                # room_transaction_frontiers reports every pipeline stage
                # against the identity of the map it ran on, so the fake grid
                # has to carry one.
                SimpleNamespace(
                    data=[],
                    header=SimpleNamespace(
                        seq=7,
                        stamp=SimpleNamespace(to_sec=lambda: 12.5),
                    ),
                ), pose, (1, 2)
            )
        finally:
            module.extract_frontiers = original_extract

        expected = module._dilate(component, 1) & ~keepout
        self.assertEqual(result, [])
        np.testing.assert_array_equal(captured["allowed"], expected)
        self.assertFalse(captured["allowed"][keepout].any())


if __name__ == "__main__":
    unittest.main()
