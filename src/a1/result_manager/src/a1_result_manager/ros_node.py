from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple

import rospy
from std_msgs.msg import String
from a1_navigation_interfaces.msg import DangerDetection, DangerDetectionArray

from a1_result_manager.scoring import (
    MissionClock,
    WorldAnchor,
    distance,
    official_payload,
    return_verdict,
    world_anchor_from_start,
)

Position = Tuple[float, float, float]


@dataclass
class ResultSource:
    source_id: int
    position: Position          # map frame, as observed
    generation: int             # localization generation it was observed in
    confidence: float
    observations: int
    first_seen_s: float
    last_seen_s: float
    track_ids: Set[int] = field(default_factory=set)


class ResultManagerNode:
    """Aggregate confirmed detections and write the competition result JSON.

    Two things this node is responsible for that the official evaluator cannot
    check for us, and therefore has to be right here:

    * ``exploration_time``.  evaulate_danger.py has no ROS dependency at all --
      it reads our number and scores it.  The competition PDF defines that
      number as the time to finish full coverage *and return to the start*, so
      it is bounded by two explicit mission events on the sim clock, not by
      when this process happened to start and get killed on the wall clock.
    * the coordinate frame.  The result file is specified in the Gazebo
      ``world`` frame; our perception publishes in our own SLAM ``map`` frame,
      and no world frame exists in TF.  The only permitted bridge is
      team_scene_info.json's ``robot_start``, applied at the instant the robot
      is standing on it.  An unanchored detection is withheld rather than
      written in the wrong frame -- a wrong coordinate is not a near miss, it
      is a miss *and* a false alarm, so it costs twice.
    """

    def __init__(self) -> None:
        self.detections_topic = str(rospy.get_param("~detections_topic", "/danger_perception/detections"))
        self.mission_status_topic = str(rospy.get_param("~mission_status_topic", "/a1/mission_manager/status"))
        self.result_file = Path(str(rospy.get_param("~result_file", "/workspace/SimEnv/results/detected_danger.json")))
        self.audit_file = Path(str(rospy.get_param("~audit_file", "") or
                                   self.result_file.with_suffix(".audit.json")))
        self.team_scene_info_file = Path(str(rospy.get_param(
            "~team_scene_info_file",
            "/workspace/SimEnv/generated_building/team_scene_info.json")))

        self.merge_distance_m = float(rospy.get_param("~merge_distance_m", 1.0))
        self.track_id_match_distance_m = float(rospy.get_param("~track_id_match_distance_m", self.merge_distance_m))
        self.position_alpha = float(rospy.get_param("~position_alpha", 0.35))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.45))
        self.require_confirmed_status = bool(rospy.get_param("~require_confirmed_status", True))
        self.accepted_frames = _as_string_list(rospy.get_param("~accepted_frames", ["map"]))
        self.write_rate_hz = float(rospy.get_param("~write_rate_hz", 1.0))
        self.output_precision = int(rospy.get_param("~output_precision", 3))
        self.log_throttle_s = float(rospy.get_param("~log_throttle_s", 2.0))

        self.timing_start_states = _as_string_list(rospy.get_param("~timing_start_states", ["MISSION_TIMING_START"]))
        self.timing_stop_states = _as_string_list(rospy.get_param("~timing_stop_states", ["MISSION_COMPLETE"]))
        self.timing_abort_states = _as_string_list(rospy.get_param("~timing_abort_states", ["MISSION_FAILED"]))
        self.anchor_states = _as_string_list(rospy.get_param("~anchor_states", ["MISSION_TIMING_START", "WORLD_ANCHOR"]))
        self.require_world_anchor = bool(rospy.get_param("~require_world_anchor", True))
        self.world_z_mode = str(rospy.get_param("~world_z_mode", "from_robot_start"))

        self._sources: List[ResultSource] = []
        self._next_source_id = 1
        self._dirty = True
        self._clock = MissionClock()
        self._anchors: Dict[int, WorldAnchor] = {}
        self._generation = 0
        self._world_start = self._load_world_start()
        self._return: Dict[str, object] = {}

        self._clock.note_node_start(self._now_s())
        self._write_result_file()

        rospy.Subscriber(self.mission_status_topic, String,
                         self._mission_status_callback, queue_size=50)
        self._sub = rospy.Subscriber(
            self.detections_topic,
            DangerDetectionArray,
            self._detections_callback,
            queue_size=10,
        )
        period = 1.0 / max(0.1, self.write_rate_hz)
        self._timer = rospy.Timer(rospy.Duration(period), self._timer_callback)

        rospy.on_shutdown(self._write_result_file)
        rospy.loginfo(
            "a1_result_manager started: clock=sim detections=%s status=%s "
            "result=%s audit=%s accepted_frames=%s require_world_anchor=%s",
            self.detections_topic, self.mission_status_topic, self.result_file,
            self.audit_file, self.accepted_frames, self.require_world_anchor,
        )

    # -- clock -------------------------------------------------------------
    def _now_s(self) -> float:
        """Sim time.  ``/use_sim_time`` is true in this environment, so
        rospy.Time.now() follows /clock.  Wall time would make the score a
        function of the grader's machine load: mf41 spent 569 s of wall clock
        to advance the sim by 157 s (RTF 0.276)."""
        return float(rospy.Time.now().to_sec())

    # -- mission lifecycle -------------------------------------------------
    def _mission_status_callback(self, message: String) -> None:
        try:
            body = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(body, dict):
            return

        generation = body.get("mission_generation")
        if isinstance(generation, (int, float)) and int(generation) >= 0:
            self._generation = int(generation)

        state = str(body.get("state", ""))
        now_s = self._now_s()

        if state in self.anchor_states:
            self._register_anchor(body)
        if state in self.timing_start_states and self._clock.start(now_s):
            rospy.loginfo("exploration clock started at sim t=%.2f (%s)", now_s, state)
            self._dirty = True
        elif state in self.timing_stop_states:
            self._capture_return(body)
            if self._clock.stop(now_s):
                rospy.loginfo("exploration clock stopped at sim t=%.2f: %.2f s (%s)",
                              now_s, self._clock.elapsed(now_s), state)
                self._dirty = True
        elif state in self.timing_abort_states:
            self._capture_return(body)
            if self._clock.abort(now_s):
                rospy.logwarn("exploration clock aborted at sim t=%.2f after %.2f s (%s)",
                              now_s, self._clock.elapsed(now_s), state)
                self._dirty = True

    def _register_anchor(self, body: dict) -> None:
        """The mission tells us where it is in the map frame while it is
        standing on the published world start pose."""
        if self._world_start is None:
            rospy.logwarn_throttle(
                30.0, "no robot_start in %s; detections cannot be put in the "
                "world frame", self.team_scene_info_file)
            return
        try:
            map_pose = (float(body["anchor_x"]), float(body["anchor_y"]),
                        float(body["anchor_z"]), float(body["anchor_yaw"]))
        except (KeyError, TypeError, ValueError):
            return
        generation = int(body.get("mission_generation", self._generation))
        if generation in self._anchors:
            return
        anchor = world_anchor_from_start(generation, map_pose, self._world_start)
        self._anchors[generation] = anchor
        self._dirty = True
        rospy.loginfo(
            "world anchor for generation %d: map(%.3f, %.3f, %.3f, yaw %.3f) "
            "== world(%.3f, %.3f, %.3f), dyaw %.3f",
            generation, map_pose[0], map_pose[1], map_pose[2], map_pose[3],
            anchor.world_x, anchor.world_y, anchor.world_z, anchor.dyaw)

    def _capture_return(self, body: dict) -> None:
        residual = body.get("return_residual_m")
        tolerance = body.get("return_tolerance_m")
        if residual is None and tolerance is None:
            return
        self._return = {
            "residual_m": _as_optional_float(residual),
            "tolerance_m": _as_optional_float(tolerance),
            "verdict": return_verdict(_as_optional_float(residual),
                                      _as_optional_float(tolerance)),
            "achieved_x": _as_optional_float(body.get("final_x")),
            "achieved_y": _as_optional_float(body.get("final_y")),
            "target_x": _as_optional_float(body.get("target_x")),
            "target_y": _as_optional_float(body.get("target_y")),
            "detail": body.get("detail"),
        }

    # -- detections --------------------------------------------------------
    def _detections_callback(self, msg: DangerDetectionArray) -> None:
        accepted = 0
        for detection in msg.detections:
            if not self._is_usable_detection(detection):
                continue
            self._upsert_detection(detection)
            accepted += 1
        if accepted > 0:
            self._dirty = True
        rospy.loginfo_throttle(
            self.log_throttle_s,
            "result_manager received=%d accepted=%d final_sources=%d anchored=%d",
            len(msg.detections), accepted, len(self._sources),
            len(self._anchored_positions()),
        )

    def _is_usable_detection(self, detection: DangerDetection) -> bool:
        if not detection.is_valid:
            return False
        if detection.class_name != "danger_red_sphere":
            return False
        if detection.confidence < self.min_confidence:
            return False
        if self.require_confirmed_status and "confirmed" not in detection.status:
            return False
        if self.accepted_frames and detection.header.frame_id not in self.accepted_frames:
            rospy.logwarn_throttle(
                self.log_throttle_s,
                "Ignoring detection in frame %s; accepted frames are %s",
                detection.header.frame_id,
                self.accepted_frames,
            )
            return False
        position = _position_from_detection(detection)
        if not _is_finite_position(position):
            return False
        return True

    def _upsert_detection(self, detection: DangerDetection) -> None:
        position = _position_from_detection(detection)
        now_s = self._now_s()
        source = self._match_existing_source(detection.track_id, position)
        if source is None:
            source = ResultSource(
                source_id=self._next_source_id,
                position=position,
                generation=self._generation,
                confidence=float(detection.confidence),
                observations=1,
                first_seen_s=now_s,
                last_seen_s=now_s,
            )
            self._next_source_id += 1
            self._sources.append(source)
        else:
            alpha = _clamp(self.position_alpha)
            source.position = (
                (1.0 - alpha) * source.position[0] + alpha * position[0],
                (1.0 - alpha) * source.position[1] + alpha * position[1],
                (1.0 - alpha) * source.position[2] + alpha * position[2],
            )
            source.confidence = max(source.confidence, float(detection.confidence))
            source.observations += 1
            source.last_seen_s = now_s
        if detection.track_id:
            source.track_ids.add(int(detection.track_id))

    def _match_existing_source(self, track_id: int, position: Position) -> Optional[ResultSource]:
        if track_id:
            for source in self._sources:
                if (source.generation == self._generation
                        and int(track_id) in source.track_ids
                        and distance(source.position, position) <= self.track_id_match_distance_m):
                    return source

        best_source = None
        best_distance = self.merge_distance_m
        for source in self._sources:
            # Positions from different localization generations are not
            # comparable, so merging across them would fuse unrelated rooms.
            if source.generation != self._generation:
                continue
            gap = distance(source.position, position)
            if gap <= best_distance:
                best_distance = gap
                best_source = source
        return best_source

    # -- output ------------------------------------------------------------
    def _anchored_positions(self) -> List[Position]:
        positions: List[Position] = []
        for source in self._sources:
            anchor = self._anchors.get(source.generation)
            if anchor is None:
                if not self.require_world_anchor:
                    positions.append(source.position)
                continue
            positions.append(anchor.apply(source.position))
        return positions

    def _withheld_sources(self) -> List[ResultSource]:
        if not self.require_world_anchor:
            return []
        return [source for source in self._sources
                if source.generation not in self._anchors]

    def _timer_callback(self, _event) -> None:
        if self._dirty:
            self._write_result_file()
            self._dirty = False

    def _write_result_file(self) -> None:
        now_s = self._now_s()
        self._write_json(self.result_file,
                         official_payload(self._clock.elapsed(now_s),
                                          self._anchored_positions(),
                                          self.output_precision))
        self._write_json(self.audit_file, self._audit_json(now_s))

    def _write_json(self, path: Path, data: dict) -> None:
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp_path = path.with_name(path.name + ".tmp")
            tmp_path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n",
                                encoding="utf-8")
            os.replace(str(tmp_path), str(path))
        except Exception as exc:
            rospy.logerr("Failed to write %s: %s", path, exc)

    def _audit_json(self, now_s: float) -> dict:
        """Everything the submitted file must not carry, kept next to it so a
        number we hand in can always be traced back to what produced it."""
        withheld = self._withheld_sources()
        return {
            "clock": {
                "source": "sim_time",
                "use_sim_time": bool(rospy.get_param("/use_sim_time", False)),
                **self._clock.summary(now_s),
            },
            "definition": (
                "PDF: 从指定起点出发,自主探索全部可通行区域后返回起点所需的时间。"
                "起点事件=%s 终点事件=%s"
                % (self.timing_start_states, self.timing_stop_states)
            ),
            "return_to_start": self._return or None,
            "world_frame": {
                "robot_start": self._world_start,
                "world_z_mode": self.world_z_mode,
                "expected_sphere_z": [0.15, 2.75, 5.35],
                "team_scene_info_file": str(self.team_scene_info_file),
                "anchored_generations": sorted(self._anchors),
                "require_world_anchor": self.require_world_anchor,
                "withheld_source_count": len(withheld),
                "withheld_generations": sorted({s.generation for s in withheld}),
            },
            "sources": [
                {
                    "id": source.source_id,
                    "generation": source.generation,
                    "map_position": [round(v, self.output_precision) for v in source.position],
                    "world_position": (
                        [round(v, self.output_precision)
                         for v in self._anchors[source.generation].apply(source.position)]
                        if source.generation in self._anchors else None),
                    "confidence": round(source.confidence, 3),
                    "observations": source.observations,
                    "first_seen_s": round(source.first_seen_s, 2),
                    "last_seen_s": round(source.last_seen_s, 2),
                }
                for source in self._sources
            ],
        }

    def _load_world_start(self) -> Optional[dict]:
        """team_scene_info.json is the one scene file the rules let us read
        (docs/competition-rules.md: 允许读取).  danger_truth.json and the layout
        files are referee-only and are never touched here."""
        try:
            data = json.loads(self.team_scene_info_file.read_text(encoding="utf-8"))
        except Exception as exc:
            rospy.logwarn("Could not read %s: %s", self.team_scene_info_file, exc)
            return None
        start = data.get("robot_start")
        if not isinstance(start, dict) or "x" not in start or "y" not in start:
            rospy.logwarn("%s has no usable robot_start", self.team_scene_info_file)
            return None
        keys = ("x", "y", "z", "yaw")
        if self.world_z_mode == "identity":
            # robot_start.z is the spawn drop height (0.6), while our map frame
            # measures height from wherever localization initialised.  Which of
            # the two is right for the z term is not decidable from the docs,
            # so it is a parameter and the audit prints every world z next to
            # the sphere heights the truth file uses (0.15 / 2.75 / 5.35) --
            # one real run settles it instead of an assumption doing so.
            keys = ("x", "y", "yaw")
        return {key: float(start[key]) for key in keys if key in start}


def _position_from_detection(detection: DangerDetection) -> Position:
    return (
        float(detection.position.x),
        float(detection.position.y),
        float(detection.position.z),
    )


def _clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def _is_finite_position(position: Iterable[float]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in position)
    except (TypeError, ValueError):
        return False


def _as_optional_float(value) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_string_list(value) -> List[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item) for item in value]


def main() -> None:
    rospy.init_node("result_manager")
    # rospy.Time.now() is 0 until the first /clock message; starting the audit
    # trail at t=0 would misreport every timestamp in it.
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() <= 0.0:
        rospy.sleep(0.1)
    ResultManagerNode()
    rospy.spin()
