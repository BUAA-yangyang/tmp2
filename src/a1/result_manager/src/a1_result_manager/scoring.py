"""Pure scoring/timing helpers for the competition result file.

Nothing here imports rospy.  The官方 evaluator (evaulate_danger.py) never
observes the simulation -- it reads whatever number we put in
``exploration_time`` -- so the definition of that number lives here, in code
that can be unit tested without a running sim.

The definition follows the competition PDF rather than the shipped evaluator,
because the evaluator does not define it at all:

    "机器人在规定区域内完成全覆盖探索并返回出发点所需的平均时间"

so the clock runs from the moment the robot starts moving autonomously to the
moment it has finished covering the building *and* is back at the start.  Both
ends are explicit events, not side effects of a process starting or being
killed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

Position = Tuple[float, float, float]

# Timing lifecycle.  A run that never reached STOPPED has no valid exploration
# time under the PDF definition, and the audit file has to say so.
NOT_STARTED = "NOT_STARTED"
RUNNING = "RUNNING"
STOPPED = "STOPPED"
ABORTED = "ABORTED"


class MissionClock:
    """Sim-time stopwatch with explicit start, explicit stop, and latching.

    The old implementation started at node construction, used ``time.time()``
    (wall clock), and reported whatever the elapsed time happened to be when
    the process was killed.  Three separate ways to report a number that has
    nothing to do with what the robot did.
    """

    def __init__(self) -> None:
        self._state = NOT_STARTED
        self._node_start_s: Optional[float] = None
        self._start_s: Optional[float] = None
        self._stop_s: Optional[float] = None
        self._latched_s: Optional[float] = None

    # -- lifecycle ---------------------------------------------------------
    def note_node_start(self, now_s: float) -> None:
        if self._node_start_s is None:
            self._node_start_s = float(now_s)

    def start(self, now_s: float) -> bool:
        """Begin timing.  Ignored once running or finished: the first
        MISSION_TIMING_START wins, so a retransmitted latched status message
        cannot silently reset the clock to zero mid-run."""
        if self._state != NOT_STARTED:
            return False
        self._start_s = float(now_s)
        self._state = RUNNING
        return True

    def stop(self, now_s: float) -> bool:
        """Finish timing and latch the value.  Later writes reuse the latched
        number instead of continuing to count while the process lingers."""
        if self._state != RUNNING:
            return False
        self._stop_s = float(now_s)
        self._latched_s = max(0.0, self._stop_s - float(self._start_s))
        self._state = STOPPED
        return True

    def abort(self, now_s: float) -> bool:
        """The mission failed.  Freeze the elapsed time for the record but
        mark the run as having no valid exploration time."""
        if self._state not in (NOT_STARTED, RUNNING):
            return False
        if self._state == RUNNING:
            self._stop_s = float(now_s)
            self._latched_s = max(0.0, self._stop_s - float(self._start_s))
        else:
            self._latched_s = 0.0
        self._state = ABORTED
        return True

    # -- readout -----------------------------------------------------------
    @property
    def state(self) -> str:
        return self._state

    @property
    def valid(self) -> bool:
        """True only for a run that started and reached an explicit stop."""
        return self._state == STOPPED

    def elapsed(self, now_s: float) -> float:
        if self._latched_s is not None:
            return self._latched_s
        if self._state == RUNNING and self._start_s is not None:
            return max(0.0, float(now_s) - self._start_s)
        return 0.0

    def elapsed_since_node_start(self, now_s: float) -> float:
        if self._node_start_s is None:
            return 0.0
        end = self._stop_s if self._stop_s is not None else float(now_s)
        return max(0.0, end - self._node_start_s)

    def summary(self, now_s: float) -> Dict[str, object]:
        return {
            "state": self._state,
            "valid": self.valid,
            "node_start_sim_s": self._node_start_s,
            "start_sim_s": self._start_s,
            "stop_sim_s": self._stop_s,
            "elapsed_s": round(self.elapsed(now_s), 3),
            "elapsed_since_node_start_s": round(
                self.elapsed_since_node_start(now_s), 3),
        }


@dataclass(frozen=True)
class WorldAnchor:
    """Rigid map->world transform valid for exactly one localization generation.

    The competition wants danger-source positions in the Gazebo ``world``
    frame.  Our perception publishes them in our own SLAM ``map`` frame, and
    there is no world frame in TF -- the only legal way to relate the two is
    the one fact the rules let us read: team_scene_info.json says where the
    robot starts in world coordinates, and at that instant the robot is there.

    FAST-LIO re-anchors its frame at every localization reinitialization, so an
    anchor is only meaningful inside the generation it was taken in.  The
    generation is part of the anchor for that reason: a stale anchor must
    refuse to convert rather than quietly emit a wrong coordinate.
    """

    generation: int
    dyaw: float
    map_x: float
    map_y: float
    map_z: float
    world_x: float
    world_y: float
    world_z: float

    def apply(self, position: Position) -> Position:
        dx = float(position[0]) - self.map_x
        dy = float(position[1]) - self.map_y
        cos_a = math.cos(self.dyaw)
        sin_a = math.sin(self.dyaw)
        return (
            self.world_x + dx * cos_a - dy * sin_a,
            self.world_y + dx * sin_a + dy * cos_a,
            self.world_z + (float(position[2]) - self.map_z),
        )


def world_anchor_from_start(generation, map_pose, world_start) -> WorldAnchor:
    """Build the map->world transform from the robot's own start pose.

    ``map_pose`` is (x, y, z, yaw) measured in the map frame at the instant the
    mission starts moving; ``world_start`` is the ``robot_start`` block of
    team_scene_info.json.  Reading that file is explicitly permitted by
    docs/competition-rules.md; the truth and layout files are not.
    """
    mx, my, mz, myaw = (float(value) for value in map_pose)
    wx = float(world_start["x"])
    wy = float(world_start["y"])
    wz = float(world_start.get("z", mz))
    wyaw = float(world_start.get("yaw", myaw))
    return WorldAnchor(
        generation=int(generation),
        dyaw=normalize_angle(wyaw - myaw),
        map_x=mx, map_y=my, map_z=mz,
        world_x=wx, world_y=wy, world_z=wz,
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def planar_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def distance(a: Position, b: Position) -> float:
    return math.sqrt(
        (a[0] - b[0]) * (a[0] - b[0])
        + (a[1] - b[1]) * (a[1] - b[1])
        + (a[2] - b[2]) * (a[2] - b[2])
    )


def return_verdict(residual_m, tolerance_m) -> Optional[bool]:
    """None means "not measured" -- which is not the same as "failed", and the
    audit file must be able to tell those apart."""
    if residual_m is None or tolerance_m is None:
        return None
    return float(residual_m) <= float(tolerance_m)


def official_payload(exploration_time_s: float,
                     positions: Iterable[Position],
                     precision: int = 3) -> Dict[str, object]:
    """Exactly the two fields docs/evaluation.md specifies, nothing else.

    Extra top-level keys would in fact be ignored by the evaluator, but the
    submitted file is the one artefact that should carry no local invention;
    everything diagnostic goes to the audit sidecar instead.
    """
    return {
        "exploration_time": round(float(exploration_time_s), 2),
        "detected_danger_sources": [
            {"position": [round(float(value), precision) for value in position]}
            for position in positions
        ],
    }
