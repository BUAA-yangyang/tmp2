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
    source_id: Optional[int] = None


@dataclass
class ConfirmedSource:
    source_id: int
    class_name: str
    position: Tuple[float, float, float]
    confidence: float
    observations: int
    last_seen: float


@dataclass
class TrackerParams:
    merge_distance_m: float
    confirmed_source_merge_distance_m: float
    position_alpha: float
    stale_after_s: float
    min_observations: int
    publish_min_confidence: float


class DangerTracker:
    def __init__(self, params: TrackerParams) -> None:
        self.params = params
        self._tracks: List[Track] = []
        self._next_track_id = 1
        self._confirmed_sources: List[ConfirmedSource] = []
        self._next_source_id = 1

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
            source = self._nearest_confirmed_source(class_name, position)
            track = Track(
                track_id=self._next_track_id,
                class_name=class_name,
                position=position,
                confidence=confidence,
                observations=1,
                last_seen=stamp_s,
                source_id=source.source_id if source is not None else None,
            )
            self._next_track_id += 1
            self._tracks.append(track)
            self._maybe_confirm(track)
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
        self._maybe_confirm(nearest)
        return nearest

    def publishable_tracks(self, stamp_s: float) -> List[Track]:
        self._drop_stale(stamp_s)
        return [
            track
            for track in self._tracks
            if track.source_id is not None
            and track.confidence >= self.params.publish_min_confidence
        ]

    def _maybe_confirm(self, track: Track) -> None:
        if track.source_id is not None:
            self._update_confirmed_source(track)
            return
        if track.observations < self.params.min_observations:
            return
        if track.confidence < self.params.publish_min_confidence:
            return

        source = self._nearest_confirmed_source(track.class_name, track.position)
        if source is None:
            source = ConfirmedSource(
                source_id=self._next_source_id,
                class_name=track.class_name,
                position=track.position,
                confidence=track.confidence,
                observations=track.observations,
                last_seen=track.last_seen,
            )
            self._next_source_id += 1
            self._confirmed_sources.append(source)
        track.source_id = source.source_id
        self._update_confirmed_source(track)

    def _update_confirmed_source(self, track: Track) -> None:
        source = self._confirmed_source_by_id(track.source_id)
        if source is None:
            return
        alpha = max(0.0, min(1.0, self.params.position_alpha))
        source.position = (
            (1.0 - alpha) * source.position[0] + alpha * track.position[0],
            (1.0 - alpha) * source.position[1] + alpha * track.position[1],
            (1.0 - alpha) * source.position[2] + alpha * track.position[2],
        )
        source.confidence = max(source.confidence, track.confidence)
        source.observations = max(source.observations, track.observations)
        source.last_seen = track.last_seen
        track.position = source.position
        track.confidence = source.confidence

    def _confirmed_source_by_id(self, source_id: Optional[int]) -> Optional[ConfirmedSource]:
        if source_id is None:
            return None
        for source in self._confirmed_sources:
            if source.source_id == source_id:
                return source
        return None

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

    def _nearest_confirmed_source(
        self,
        class_name: str,
        position: Tuple[float, float, float],
    ) -> Optional[ConfirmedSource]:
        best_source = None
        best_distance = self.params.confirmed_source_merge_distance_m
        for source in self._confirmed_sources:
            if source.class_name != class_name:
                continue
            distance = _distance(source.position, position)
            if distance <= best_distance:
                best_distance = distance
                best_source = source
        return best_source

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
