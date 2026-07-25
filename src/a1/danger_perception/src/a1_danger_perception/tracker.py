from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import List, Optional, Tuple


@dataclass
class Track:
    track_id: int
    class_name: str
    position: Tuple[float, float, float]
    confidence: float
    observations: int
    last_seen: float


@dataclass
class TrackerParams:
    merge_distance_m: float
    position_alpha: float
    stale_after_s: float
    min_observations: int
    publish_min_confidence: float


class DangerTracker:
    def __init__(self, params: TrackerParams) -> None:
        self.params = params
        self._tracks: List[Track] = []
        self._next_track_id = 1

    def update(
        self,
        class_name: str,
        position: Tuple[float, float, float],
        confidence: float,
        stamp_s: float,
    ) -> Track:
        self._drop_stale(stamp_s)
        nearest = self._nearest_track(class_name, position)
        if nearest is None:
            track = Track(
                track_id=self._next_track_id,
                class_name=class_name,
                position=position,
                confidence=confidence,
                observations=1,
                last_seen=stamp_s,
            )
            self._next_track_id += 1
            self._tracks.append(track)
            return track

        alpha = max(0.0, min(1.0, self.params.position_alpha))
        nearest.position = (
            (1.0 - alpha) * nearest.position[0] + alpha * position[0],
            (1.0 - alpha) * nearest.position[1] + alpha * position[1],
            (1.0 - alpha) * nearest.position[2] + alpha * position[2],
        )
        nearest.confidence = max(nearest.confidence, confidence)
        nearest.observations += 1
        nearest.last_seen = stamp_s
        return nearest

    def publishable_tracks(self, stamp_s: float) -> List[Track]:
        self._drop_stale(stamp_s)
        return [
            track
            for track in self._tracks
            if track.observations >= self.params.min_observations
            and track.confidence >= self.params.publish_min_confidence
        ]

    def _nearest_track(self, class_name: str, position: Tuple[float, float, float]) -> Optional[Track]:
        best_track = None
        best_distance = self.params.merge_distance_m
        for track in self._tracks:
            if track.class_name != class_name:
                continue
            distance = _distance(track.position, position)
            if distance <= best_distance:
                best_distance = distance
                best_track = track
        return best_track

    def _drop_stale(self, stamp_s: float) -> None:
        self._tracks = [
            track
            for track in self._tracks
            if stamp_s - track.last_seen <= self.params.stale_after_s
        ]


def _distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return sqrt(
        (a[0] - b[0]) * (a[0] - b[0])
        + (a[1] - b[1]) * (a[1] - b[1])
        + (a[2] - b[2]) * (a[2] - b[2])
    )
