from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List, Optional, Set, Tuple

import rospy
from a1_navigation_interfaces.msg import DangerDetection, DangerDetectionArray


Position = Tuple[float, float, float]


@dataclass
class ResultSource:
    source_id: int
    position: Position
    confidence: float
    observations: int
    first_seen_s: float
    last_seen_s: float
    track_ids: Set[int] = field(default_factory=set)


class ResultManagerNode:
    """Aggregate confirmed detections and write the competition result JSON."""

    def __init__(self) -> None:
        self.detections_topic = str(rospy.get_param("~detections_topic", "/danger_perception/detections"))
        self.result_file = Path(str(rospy.get_param("~result_file", "/workspace/SimEnv/results/detected_danger.json")))
        self.merge_distance_m = float(rospy.get_param("~merge_distance_m", 1.0))
        self.track_id_match_distance_m = float(rospy.get_param("~track_id_match_distance_m", self.merge_distance_m))
        self.position_alpha = float(rospy.get_param("~position_alpha", 0.35))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.45))
        self.require_confirmed_status = bool(rospy.get_param("~require_confirmed_status", True))
        self.accepted_frames = _as_string_list(rospy.get_param("~accepted_frames", ["world", "map"]))
        self.load_existing_results = bool(rospy.get_param("~load_existing_results", True))
        self.write_empty_on_start = bool(rospy.get_param("~write_empty_on_start", True))
        self.write_rate_hz = float(rospy.get_param("~write_rate_hz", 1.0))
        self.output_precision = int(rospy.get_param("~output_precision", 3))
        self.log_throttle_s = float(rospy.get_param("~log_throttle_s", 2.0))

        self._sources: List[ResultSource] = []
        self._next_source_id = 1
        self._dirty = False
        self._time_offset_s = 0.0
        self._start_wall_s = time.time()

        if self.load_existing_results:
            self._load_existing_result_file()
        if self.write_empty_on_start or self._sources:
            self._write_result_file()

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
            "a1_result_manager started: detections=%s result_file=%s merge_distance=%.2f accepted_frames=%s",
            self.detections_topic,
            self.result_file,
            self.merge_distance_m,
            self.accepted_frames,
        )

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
            "result_manager received=%d accepted=%d final_sources=%d",
            len(msg.detections),
            accepted,
            len(self._sources),
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
        now_s = self._elapsed_s()
        source = self._match_existing_source(detection.track_id, position)
        if source is None:
            source = ResultSource(
                source_id=self._next_source_id,
                position=position,
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
                if int(track_id) in source.track_ids and _distance(source.position, position) <= self.track_id_match_distance_m:
                    return source

        best_source = None
        best_distance = self.merge_distance_m
        for source in self._sources:
            distance = _distance(source.position, position)
            if distance <= best_distance:
                best_distance = distance
                best_source = source
        return best_source

    def _timer_callback(self, _event) -> None:
        if self._dirty:
            self._write_result_file()
            self._dirty = False

    def _load_existing_result_file(self) -> None:
        if not self.result_file.exists():
            return
        try:
            data = json.loads(self.result_file.read_text(encoding="utf-8"))
        except Exception as exc:
            rospy.logwarn("Could not load existing result file %s: %s", self.result_file, exc)
            return

        self._time_offset_s = float(data.get("exploration_time", 0.0) or 0.0)
        for item in data.get("detected_danger_sources", []):
            position = item.get("position") if isinstance(item, dict) else item
            if not _is_position_like(position):
                continue
            source = ResultSource(
                source_id=self._next_source_id,
                position=(float(position[0]), float(position[1]), float(position[2])),
                confidence=float(item.get("confidence", 0.0)) if isinstance(item, dict) else 0.0,
                observations=1,
                first_seen_s=0.0,
                last_seen_s=self._time_offset_s,
            )
            self._next_source_id += 1
            self._sources.append(source)

    def _write_result_file(self) -> None:
        try:
            self.result_file.parent.mkdir(parents=True, exist_ok=True)
            data = self._result_json()
            tmp_path = self.result_file.with_name(self.result_file.name + ".tmp")
            tmp_path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            os.replace(str(tmp_path), str(self.result_file))
        except Exception as exc:
            rospy.logerr("Failed to write result file %s: %s", self.result_file, exc)

    def _result_json(self) -> dict:
        return {
            "exploration_time": round(self._elapsed_s(), 2),
            "detected_danger_sources": [
                {"position": _rounded_position(source.position, self.output_precision)}
                for source in self._sources
            ],
        }

    def _elapsed_s(self) -> float:
        return max(0.0, self._time_offset_s + time.time() - self._start_wall_s)


def _position_from_detection(detection: DangerDetection) -> Position:
    return (
        float(detection.position.x),
        float(detection.position.y),
        float(detection.position.z),
    )


def _rounded_position(position: Position, precision: int) -> List[float]:
    return [round(float(value), precision) for value in position]


def _distance(a: Position, b: Position) -> float:
    return math.sqrt(
        (a[0] - b[0]) * (a[0] - b[0])
        + (a[1] - b[1]) * (a[1] - b[1])
        + (a[2] - b[2]) * (a[2] - b[2])
    )


def _clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def _is_position_like(value) -> bool:
    return isinstance(value, (list, tuple)) and len(value) == 3 and _is_finite_position(tuple(value))


def _is_finite_position(position: Iterable[float]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in position)
    except (TypeError, ValueError):
        return False


def _as_string_list(value) -> List[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item) for item in value]


def main() -> None:
    rospy.init_node("result_manager")
    ResultManagerNode()
    rospy.spin()
