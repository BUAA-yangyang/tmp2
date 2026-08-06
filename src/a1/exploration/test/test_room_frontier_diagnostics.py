#!/usr/bin/env python3
"""An early room exit must name the stage that emptied its candidate list.

mf49's floor 1 ended two room transactions after a single goal each, with
sizable UNKNOWN grey still adjacent to known-free space, and the only thing
the log said was "no reachable frontier remains inside the room".  Two
separate blind spots produced that:

* the room-scoped candidates were recomputed every cycle but never published,
  so the green RViz markers were the pre-transaction snapshot -- the operator
  correctly saw them not move, and that told nobody anything about the
  candidates;
* every filter stage collapsed into one sentence.  Station 5 right generated
  four raw candidates and admitted three, then reported the same sentence a
  room with zero candidates reports.

These tests pin the observability, and the one behavioural defect the reading
turned up: a transiently unavailable planner used to close the room as PROVEN.
"""
import ast
from pathlib import Path
import unittest

from a1_exploration.frontier import FailedGoal, room_frontier_rejection


NODE = (Path(__file__).resolve().parents[1] / "scripts" /
        "frontier_explorer_node.py")


class RoomFrontierRejectionTest(unittest.TestCase):
    """The pure gate that says WHICH rule barred a room frontier."""

    def reject(self, score=5.0, goal=(0.0, 0.0), visited=(), failed=(),
               now=100.0, minimum_score=-0.5, maximum_failures=2):
        return room_frontier_rejection(
            score, goal[0], goal[1], minimum_score,
            list(visited), 0.7, list(failed), 0.75, now, maximum_failures,
        )

    def test_an_ordinary_candidate_is_not_rejected(self):
        self.assertIsNone(self.reject())

    def test_score_below_the_room_minimum(self):
        self.assertEqual(self.reject(score=-0.51), "score")
        # The threshold itself is admissible: the loop rejects strictly below.
        self.assertIsNone(self.reject(score=-0.5))

    def test_a_goal_inside_the_visited_radius(self):
        self.assertEqual(self.reject(visited=[(0.5, 0.0)]), "visited")
        self.assertIsNone(self.reject(visited=[(0.8, 0.0)]))

    def test_permanently_unreachable_and_cooling_history_both_report_history(self):
        permanent = FailedGoal(x=0.0, y=0.0, failures=2, retry_after=0.0,
                               unreachable_failures=2)
        self.assertEqual(self.reject(failed=[permanent]), "history")
        cooling = FailedGoal(x=0.0, y=0.0, failures=1, retry_after=150.0,
                             unreachable_failures=0)
        self.assertEqual(self.reject(failed=[cooling]), "history")
        expired = FailedGoal(x=0.0, y=0.0, failures=1, retry_after=50.0,
                             unreachable_failures=0)
        self.assertIsNone(self.reject(failed=[expired]))

    def test_precedence_matches_the_transaction_loop_order(self):
        # Score is checked before history, so a low-scoring blacklisted goal
        # must be attributed to score -- otherwise the counts would silently
        # move between columns when a threshold is retuned.
        permanent = FailedGoal(x=0.0, y=0.0, failures=2, retry_after=0.0,
                               unreachable_failures=2)
        self.assertEqual(
            self.reject(score=-9.0, visited=[(0.0, 0.0)], failed=[permanent]),
            "score",
        )
        self.assertEqual(
            self.reject(visited=[(0.0, 0.0)], failed=[permanent]), "visited")

    def test_reachability_is_deliberately_not_decided_here(self):
        # It needs the planner, and "the planner did not answer" is not the
        # same fact as "the goal is unreachable".
        source = Path(
            room_frontier_rejection.__globals__["__file__"]
        ).read_text(encoding="utf-8")
        body = source[source.index("def room_frontier_rejection("):]
        body = body[:body.index("\ndef ", 1)]
        self.assertNotIn("path_exists", body)
        self.assertNotIn("make_plan", body)


class RoomTransactionObservabilityContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = NODE.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)
        explorer = next(
            node for node in cls.tree.body
            if isinstance(node, ast.ClassDef) and node.name == "FrontierExplorer"
        )
        cls.methods = {
            node.name: node for node in explorer.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }

    def method_source(self, name):
        return ast.get_source_segment(self.source, self.methods[name])

    def test_the_transaction_publishes_the_markers_it_actually_scored(self):
        body = self.method_source("explore_room_transaction")
        compute = body.index("self.room_transaction_frontiers(")
        publish = body.index("self.publish_frontiers(map_message, frontiers or [])")
        self.assertLess(compute, publish)
        # It must publish before the None branch leaves, so an unbounded room
        # clears the stale markers instead of freezing them.
        self.assertLess(publish, body.index("could not bound the room"))

    def test_every_pipeline_stage_reports_against_a_map_identity(self):
        body = self.method_source("room_transaction_frontiers")
        self.assertIn("ROOM_FRONTIER_PIPELINE", body)
        self.assertIn("map_message.header.seq", body)
        self.assertIn("map_message.header.stamp.to_sec()", body)
        for field in ("component_cells", "roi_allowed_cells", "keepout_cells",
                      "searched_cells", "raw=", "shared_admissible",
                      "shared_rejected"):
            self.assertIn(field, body)
        # The unconditional log replaced one that only fired on rejection,
        # which made "nothing was rejected" indistinguishable from "nothing
        # was generated".
        self.assertNotIn("frontiers rejected by the shared admissibility",
                         body)

    def test_the_selection_loop_reports_which_gate_consumed_each_candidate(self):
        body = self.method_source("explore_room_transaction")
        self.assertIn("room_frontier_rejection(", body)
        self.assertIn("ROOM_FRONTIER_SELECTION", body)
        for field in ("rejected_score", "rejected_visited",
                      "rejected_failed_or_cooling", "unreachable=",
                      "planner_unavailable", "selected="):
            self.assertIn(field, body)

    def test_an_unavailable_planner_never_closes_the_room_as_proven(self):
        body = self.method_source("explore_room_transaction")
        self.assertIn("planner_unavailable = True", body)
        guard = body.index("if planner_unavailable:")
        proven = body.index("self.last_room_transaction_proven = True")
        self.assertLess(guard, proven)
        # The retry path must not fall through into the completion branch.
        between = body[guard:proven]
        self.assertIn("continue", between)
        self.assertNotIn("self.last_room_transaction_proven = True",
                         body[guard:body.index("continue", guard)])

    def test_completion_states_which_stage_emptied_the_list(self):
        body = self.method_source("explore_room_transaction")
        self.assertIn("no frontier candidate was generated inside", body)
        self.assertIn("admitted candidates were filtered", body)
        # The old catch-all sentence must not survive as a literal.
        self.assertNotIn(
            '"reachable frontier remains inside the room", goals,', body)


if __name__ == "__main__":
    unittest.main()
