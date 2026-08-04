#!/usr/bin/env python3
"""DEV-ONLY indoor-start ExploreFloor server.

This test adapter reuses the production frontier selection, MoveBaseAction,
completion, return, and final-zero implementation.  It suppresses only the
main-entrance sequence so an indoor spawn can validate post-entry capability.
It must never be used by competition bringup.
"""

import copy
import importlib.util
import os

from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool
import rospkg
import rospy


def load_production_node():
    package = rospkg.RosPack().get_path("a1_exploration")
    path = os.path.join(package, "scripts", "frontier_explorer_node.py")
    specification = importlib.util.spec_from_file_location(
        "a1_production_frontier_explorer_node", path
    )
    if specification is None or specification.loader is None:
        raise RuntimeError("cannot load production frontier explorer: " + path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


PRODUCTION = load_production_node()


class IndoorStartFrontierExplorer(PRODUCTION.FrontierExplorer):
    """Test-only adapter whose externally visible path is RECORD_START→SELECT."""

    ENTRY_STATES = {
        "REQUEST_ENTRY_DOOR_OPEN",
        "TRANSIT_TO_ENTRY",
        "ENTERED_FLOOR",
    }

    def __init__(self):
        if not rospy.get_param("~dev_indoor_start_ack", False):
            raise RuntimeError(
                "indoor-start adapter requires ~dev_indoor_start_ack=true"
            )
        self._skip_entry_navigation = False
        super().__init__()
        self._mode_publisher = rospy.Publisher(
            "/a1/navigation_tests/indoor_start_mode",
            Bool,
            queue_size=1,
            latch=True,
        )
        self._mode_publisher.publish(Bool(data=True))
        rospy.logwarn(
            "DEV-ONLY indoor-start explorer active: entrance door/transit "
            "states are suppressed; production frontier/navigation/return "
            "logic is unchanged"
        )

    def transition(self, state, message, target=None):
        if state in self.ENTRY_STATES:
            rospy.loginfo(
                "DEV-ONLY indoor-start skipped state %s: %s",
                state,
                message,
            )
            return
        return super().transition(state, message, target)

    def request_entry_door_open(self, goal):
        del goal
        rospy.loginfo("DEV-ONLY indoor-start skipped entry-door request")

    def wait_for_entry_passage(self, baseline_message, start_pose):
        del start_pose
        rospy.loginfo("DEV-ONLY indoor-start skipped entry-passage map gate")
        return copy.deepcopy(baseline_message)

    def publish_target(self, pose, namespace):
        if namespace == "floor_entry_target":
            rospy.loginfo(
                "DEV-ONLY indoor-start suppressed floor-entry target"
            )
            return
        return super().publish_target(pose, namespace)

    def wait_for_local_entry_plan(self):
        self._skip_entry_navigation = True
        rospy.loginfo("DEV-ONLY indoor-start skipped entry make_plan gate")

    def apply_entry_speed_limit(self):
        rospy.loginfo("DEV-ONLY indoor-start skipped entry speed override")

    def restore_entry_speed_limit(self):
        # No override was applied, so there is nothing to restore.
        return

    def navigate(self, target, timeout, returning=False):
        if self._skip_entry_navigation:
            self._skip_entry_navigation = False
            rospy.loginfo(
                "DEV-ONLY indoor-start suppressed entry MoveBaseAction goal"
            )
            return True, GoalStatus.SUCCEEDED, False
        return super().navigate(target, timeout, returning=returning)

    def wait_for_entered_floor(self, baseline_message):
        del baseline_message
        rospy.loginfo(
            "DEV-ONLY indoor-start skipped post-entry map-growth gate"
        )


if __name__ == "__main__":
    rospy.init_node("a1_frontier_explorer")
    try:
        IndoorStartFrontierExplorer()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("DEV indoor-start explorer failed: %s", error)
        raise
