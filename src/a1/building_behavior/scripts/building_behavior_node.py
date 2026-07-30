#!/usr/bin/env python3
"""SpecialBehavior action server using only competition-public interfaces."""

import time

import actionlib
from a1_navigation_interfaces.msg import (
    SpecialBehaviorAction,
    SpecialBehaviorFeedback,
    SpecialBehaviorGoal,
    SpecialBehaviorResult,
)
from building_generator_interfaces.srv import SetDoorState
import rospy

from a1_building_behavior.public_scene import (
    PublicSceneError,
    load_public_scene,
    resolve_entry_door,
)


class BuildingBehaviorServer:
    def __init__(self):
        self.action_name = rospy.get_param(
            "~action", "/a1/building_behavior/special"
        )
        self.scene_info_path = rospy.get_param(
            "~team_scene_info",
            "/workspace/SimEnv/generated_building/team_scene_info.json",
        )
        self.door_service_name = rospy.get_param(
            "~set_door_service", "/set_door_state"
        )
        self.default_timeout = float(
            rospy.get_param("~service_timeout_wall", 10.0)
        )
        if self.default_timeout <= 0.0:
            raise ValueError("service_timeout_wall must be positive")
        self.door_service = rospy.ServiceProxy(
            self.door_service_name, SetDoorState
        )
        self.server = actionlib.SimpleActionServer(
            self.action_name,
            SpecialBehaviorAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo(
            "a1_building_behavior ready: action=%s public_scene=%s service=%s",
            self.action_name,
            self.scene_info_path,
            self.door_service_name,
        )

    def feedback(self, state, progress, message):
        value = SpecialBehaviorFeedback()
        value.state = state
        value.progress = progress
        value.message = message
        self.server.publish_feedback(value)

    def abort(self, code, message):
        result = SpecialBehaviorResult()
        result.success = False
        result.error_code = code
        result.message = message
        self.server.set_aborted(result)

    def preempt_if_requested(self):
        if not self.server.is_preempt_requested():
            return False
        result = SpecialBehaviorResult()
        result.success = False
        result.error_code = SpecialBehaviorResult.ERROR_CANCELLED
        result.message = "building behavior cancelled"
        self.server.set_preempted(result)
        return True

    def wait_for_service(self, timeout):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.preempt_if_requested():
                return False
            try:
                rospy.wait_for_service(self.door_service_name, timeout=0.10)
                return True
            except rospy.ROSException:
                pass
        return False

    def execute(self, goal):
        if goal.behavior_type != SpecialBehaviorGoal.OPEN_DOOR:
            self.abort(
                SpecialBehaviorResult.ERROR_UNSUPPORTED,
                "only OPEN_DOOR is implemented for the single-floor milestone",
            )
            return
        if self.preempt_if_requested():
            return

        self.feedback(
            SpecialBehaviorFeedback.WAITING,
            0.05,
            "resolving main entrance from public team_scene_info",
        )
        try:
            document = load_public_scene(self.scene_info_path)
            door_id = resolve_entry_door(
                document, goal.target_floor_id, goal.target_id
            )
        except PublicSceneError as error:
            self.abort(
                SpecialBehaviorResult.ERROR_PUBLIC_SCENE,
                str(error),
            )
            return

        timeout = (
            float(goal.timeout_s)
            if goal.timeout_s > 0.0 else self.default_timeout
        )
        self.feedback(
            SpecialBehaviorFeedback.CALLING_SERVICE,
            0.35,
            "requesting public door %s open" % door_id,
        )
        if not self.wait_for_service(timeout):
            if not self.server.is_active():
                return
            self.abort(
                SpecialBehaviorResult.ERROR_SERVICE_UNAVAILABLE,
                "%s unavailable for %.1f wall seconds"
                % (self.door_service_name, timeout),
            )
            return
        if self.preempt_if_requested():
            return

        try:
            response = self.door_service(door_id=door_id, open=True)
        except rospy.ServiceException as error:
            self.abort(
                SpecialBehaviorResult.ERROR_SERVICE_UNAVAILABLE,
                "%s call failed: %s" % (self.door_service_name, error),
            )
            return
        if self.preempt_if_requested():
            return

        self.feedback(
            SpecialBehaviorFeedback.VERIFYING,
            0.85,
            "checking service response for %s" % door_id,
        )
        if not response.accepted or response.state.lower() != "open":
            self.abort(
                SpecialBehaviorResult.ERROR_SERVICE_REJECTED,
                "door %s rejected: accepted=%s state=%r message=%s"
                % (
                    door_id,
                    response.accepted,
                    response.state,
                    response.message,
                ),
            )
            return

        result = SpecialBehaviorResult()
        result.success = True
        result.error_code = SpecialBehaviorResult.ERROR_NONE
        result.message = "public door %s is open: %s" % (
            door_id, response.message
        )
        self.server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("a1_building_behavior")
    BuildingBehaviorServer()
    rospy.spin()
