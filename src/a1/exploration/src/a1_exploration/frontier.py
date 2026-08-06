"""ROS-independent frontier extraction and scoring.

The module deliberately consumes only an occupancy grid.  It does not know
about Gazebo, building metadata, or referee truth, so the same selector can be
reused when floor_mapping grows from one floor session to a multi-floor map
cache.
"""

from collections import deque
from dataclasses import dataclass
import math
import zlib

import numpy as np


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def cell_to_world(self, cell):
        row, col = cell
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def world_to_cell(self, x, y):
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if row < 0 or col < 0 or row >= self.height or col >= self.width:
            return None
        return row, col


@dataclass(frozen=True)
class NearFieldBlocker:
    """First explicit obstacle found in a body-aligned short corridor."""

    x: float
    y: float
    value: int
    longitudinal: float
    lateral: float


def _closed_interval_samples(start, end, maximum_step):
    """Return endpoint-inclusive samples whose spacing never exceeds a bound."""
    if end < start:
        raise ValueError("sample interval end precedes start")
    intervals = max(1, int(math.ceil((end - start) / maximum_step)))
    span = end - start
    return tuple(start + span * index / intervals for index in range(intervals + 1))


def first_near_field_blocker(
        data, spec, body_x, body_y, yaw, direction, distance,
        half_width, occupied_threshold, start_distance=0.18,
        minimum_step=0.04):
    """Return the first explicit occupied cell in a short body corridor.

    Unknown and out-of-grid samples remain non-blocking, matching the runtime
    entry gate.  The caller chooses the lateral band explicitly: forward entry
    can stay conservative while reverse return uses the robot-footprint band
    proven by its own geometry.
    """
    scalars = (
        body_x, body_y, yaw, direction, distance, half_width,
        occupied_threshold, start_distance, minimum_step,
        spec.resolution, spec.origin_x, spec.origin_y,
    )
    if not all(math.isfinite(float(value)) for value in scalars):
        raise ValueError("near-field geometry must be finite")
    if (
            spec.width <= 0 or spec.height <= 0 or spec.resolution <= 0.0
            or direction == 0.0 or distance < start_distance
            or start_distance < 0.0 or half_width < 0.0
            or minimum_step <= 0.0):
        raise ValueError("near-field geometry is invalid")
    if len(data) != spec.width * spec.height:
        raise ValueError("occupancy data size does not match grid geometry")

    direction = 1.0 if direction > 0.0 else -1.0
    cos_yaw = math.cos(yaw) * direction
    sin_yaw = math.sin(yaw) * direction
    step = max(minimum_step, spec.resolution)
    longitudinal_samples = _closed_interval_samples(
        start_distance, distance, step
    )
    lateral_samples = _closed_interval_samples(-half_width, half_width, step)
    for longitudinal in longitudinal_samples:
        for lateral in lateral_samples:
            x = body_x + longitudinal * cos_yaw - lateral * sin_yaw
            y = body_y + longitudinal * sin_yaw + lateral * cos_yaw
            cell = spec.world_to_cell(x, y)
            if cell is None:
                continue
            row, col = cell
            value = int(data[row * spec.width + col])
            if value >= occupied_threshold:
                return NearFieldBlocker(
                    x=x,
                    y=y,
                    value=value,
                    longitudinal=longitudinal,
                    lateral=lateral,
                )
    return None


@dataclass
class FrontierCluster:
    cells: tuple
    goal_cell: tuple
    goal_x: float
    goal_y: float
    yaw: float
    length_m: float
    distance_m: float
    score: float


@dataclass
class FailedGoal:
    x: float
    y: float
    failures: int
    retry_after: float
    # Only failures proven to be genuinely unreachable may ever escalate a
    # target to "permanent".  Transient outcomes (timeout, cancel, preempt,
    # controller-not-ready, safety lock) increment `failures` for cooldown
    # purposes but must never exhaust the permanent budget.
    unreachable_failures: int = 0


class NoFrontierEvidence:
    """Accumulate completion evidence without counting repeated map headers.

    Completion is permitted either after the configured number of distinct map
    *contents* or after the same eligible, frontier-free content remains stable
    for a ROS-time dwell.  Callers must not call ``observe`` while a retry is in
    cooldown or the planner is degraded.
    """

    def __init__(self, distinct_required, stable_duration):
        if int(distinct_required) < 1:
            raise ValueError("distinct_required must be positive")
        if not math.isfinite(stable_duration) or stable_duration <= 0.0:
            raise ValueError("stable_duration must be positive")
        self.distinct_required = int(distinct_required)
        self.stable_duration = float(stable_duration)
        self.reset()

    def reset(self):
        self.versions = []
        self.stable_since = None
        self.last_time = None

    def observe(self, version, now):
        if not math.isfinite(now) or now < 0.0:
            self.reset()
            return {
                "complete": False,
                "count": 0,
                "reason": "ROS time is invalid",
            }
        if self.last_time is not None and now < self.last_time:
            self.reset()
            self.last_time = now
            return {
                "complete": False,
                "count": 0,
                "reason": "ROS time moved backwards",
            }
        self.last_time = now

        if version not in self.versions:
            self.versions.append(version)
            self.stable_since = now
        elif self.stable_since is None:
            self.stable_since = now

        distinct_complete = len(self.versions) >= self.distinct_required
        stable_for = (
            0.0 if self.stable_since is None else now - self.stable_since
        )
        stable_complete = bool(self.versions) and (
            stable_for >= self.stable_duration
        )
        if distinct_complete:
            reason = "no eligible frontier on %d distinct map contents" % (
                len(self.versions)
            )
        elif stable_complete:
            reason = (
                "no eligible frontier on unchanged map content for %.2f ROS s"
                % stable_for
            )
        else:
            reason = (
                "%d/%d distinct frontier-free map contents; stable %.2f/%.2f s"
                % (
                    len(self.versions),
                    self.distinct_required,
                    stable_for,
                    self.stable_duration,
                )
            )
        return {
            "complete": distinct_complete or stable_complete,
            "count": len(self.versions),
            "stable_for": stable_for,
            "reason": reason,
        }


@dataclass(frozen=True)
class ProgressObservation:
    progressed: bool
    stalled: bool
    moved_m: float
    turned_rad: float
    elapsed_s: float
    reason: str


@dataclass(frozen=True)
class CorridorGateDecision:
    stalls: int
    hold_frontiers: bool
    released: bool
    reason: str


def corridor_gate_decision(
        enabled, exhausted, has_entry, target_available,
        current_stalls, maximum_stalls):
    """Advance the bounded corridor-probe gate without implying completion.

    A successful probe resets the consecutive-stall count.  A missing probe
    target holds unrestricted frontier selection only for a bounded number of
    fresh selection cycles, then releases it as a deadlock escape.  Neither a
    hold nor a release is evidence that exploration is complete.
    """
    current_stalls = int(current_stalls)
    maximum_stalls = int(maximum_stalls)
    if current_stalls < 0 or maximum_stalls < 0:
        raise ValueError("corridor gate stall counts must be non-negative")
    if target_available:
        return CorridorGateDecision(
            0, False, False, "probe target available; stall count reset"
        )
    if not enabled or exhausted or not has_entry:
        return CorridorGateDecision(
            current_stalls,
            False,
            False,
            "corridor gate inactive",
        )
    next_stalls = current_stalls + 1
    if next_stalls <= maximum_stalls:
        return CorridorGateDecision(
            next_stalls,
            True,
            False,
            "waiting for a fresher map before releasing frontier selection",
        )
    return CorridorGateDecision(
        next_stalls,
        False,
        True,
        "probe unavailable past bounded stall budget; frontier gate released",
    )


class NoProgressWatchdog:
    """ROS-independent motion/yaw progress evidence for one navigation goal."""

    def __init__(self, timeout_s, distance_m, yaw_rad):
        values = (float(timeout_s), float(distance_m), float(yaw_rad))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("watchdog thresholds must be finite")
        self.timeout_s, self.distance_m, self.yaw_rad = values
        if self.timeout_s <= 0.0:
            raise ValueError("watchdog timeout must be positive")
        if self.distance_m < 0.0 or self.yaw_rad < 0.0:
            raise ValueError("watchdog progress thresholds must be non-negative")
        if self.distance_m == 0.0 and self.yaw_rad == 0.0:
            raise ValueError("at least one watchdog progress threshold is required")
        self.reset()

    def reset(self):
        self._anchor = None
        self._anchor_time = None
        self._last_time = None

    def observe(self, now_s, x, y, yaw, yaw_counts_as_progress=True):
        """``yaw_counts_as_progress`` False 时只有平移算作进展。

        原判据把原地旋转也算进展,理由是「转向目标位姿是合法进展」——那对最后
        的对准阶段成立,但 DWA 的振荡恢复同样是原地转圈,于是 anchor 被无限
        重置,看门狗永远不会超时。mf44 因此在 world (6.70, 18.59) 冻结 53 s,
        期间 bounded_backout 一次都没被调用。默认值保持旧行为。
        """
        values = (float(now_s), float(x), float(y), float(yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("watchdog observation must be finite")
        now_s, x, y, yaw = values
        if now_s < 0.0:
            raise ValueError("watchdog time must be non-negative")
        if self._last_time is not None and now_s < self._last_time:
            self.reset()
            reason = "time moved backwards; watchdog reset"
        else:
            reason = "initial observation"
        self._last_time = now_s
        if self._anchor is None:
            self._anchor = (x, y, yaw)
            self._anchor_time = now_s
            return ProgressObservation(False, False, 0.0, 0.0, 0.0, reason)

        anchor_x, anchor_y, anchor_yaw = self._anchor
        moved = math.hypot(x - anchor_x, y - anchor_y)
        turned = abs(math.atan2(
            math.sin(yaw - anchor_yaw), math.cos(yaw - anchor_yaw)
        ))
        elapsed = now_s - self._anchor_time
        progressed = (
            (self.distance_m > 0.0 and moved >= self.distance_m)
            or (yaw_counts_as_progress
                and self.yaw_rad > 0.0 and turned >= self.yaw_rad)
        )
        if progressed:
            self._anchor = (x, y, yaw)
            self._anchor_time = now_s
            return ProgressObservation(
                True, False, moved, turned, elapsed,
                "translation or yaw progress threshold reached",
            )
        return ProgressObservation(
            False,
            elapsed >= self.timeout_s,
            moved,
            turned,
            elapsed,
            "no progress timeout" if elapsed >= self.timeout_s else "waiting",
        )


def occupancy_content_fingerprint(flat_data, spec):
    """Return a deterministic fingerprint of OccupancyGrid geometry/content."""
    if spec.width <= 0 or spec.height <= 0 or spec.resolution <= 0.0:
        raise ValueError("invalid grid geometry")
    data = np.asarray(flat_data, dtype=np.int16)
    if data.size != spec.width * spec.height:
        raise ValueError("occupancy data size does not match grid geometry")
    if np.any(data < -1) or np.any(data > 100):
        raise ValueError("occupancy values must be in [-1, 100]")
    encoded = (data + 1).astype(np.uint8, copy=False).tobytes()
    return (
        spec.width,
        spec.height,
        float(spec.resolution),
        float(spec.origin_x),
        float(spec.origin_y),
        zlib.crc32(encoded) & 0xFFFFFFFF,
    )


def known_cell_count(flat_data, allowed_mask=None):
    data = np.asarray(flat_data, dtype=np.int16)
    if allowed_mask is None:
        return int(np.count_nonzero(data >= 0))
    allowed = np.asarray(allowed_mask, dtype=bool)
    if allowed.size != data.size:
        raise ValueError("allowed mask size does not match occupancy data")
    return int(np.count_nonzero((data >= 0) & allowed.reshape(data.shape)))


def segment_corridor_mask(spec, start_xy, goal_xy, half_width):
    """Return cells whose centres lie in a finite-width entry corridor."""
    values = (
        start_xy[0],
        start_xy[1],
        goal_xy[0],
        goal_xy[1],
        half_width,
        spec.resolution,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("entry corridor geometry must be finite")
    if (
            spec.width <= 0
            or spec.height <= 0
            or spec.resolution <= 0.0
            or half_width <= 0.0):
        raise ValueError("entry corridor geometry must be positive")

    sx, sy = start_xy
    gx, gy = goal_xy
    dx = gx - sx
    dy = gy - sy
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        raise ValueError("entry corridor endpoints must be distinct")

    rows, columns = np.indices((spec.height, spec.width), dtype=float)
    world_x = spec.origin_x + (columns + 0.5) * spec.resolution
    world_y = spec.origin_y + (rows + 0.5) * spec.resolution
    projection = ((world_x - sx) * dx + (world_y - sy) * dy) / length_squared
    projection = np.clip(projection, 0.0, 1.0)
    closest_x = sx + projection * dx
    closest_y = sy + projection * dy
    return (
        (world_x - closest_x) ** 2 + (world_y - closest_y) ** 2
        <= half_width * half_width
    )


def local_plan_is_acceptable(
        path_xy,
        start_xy,
        goal_xy,
        corridor_half_width,
        endpoint_tolerance,
        maximum_length_ratio,
        maximum_length_slack,
):
    """Fail-closed validation for an entry plan before MoveBaseAction starts."""
    limits = (
        corridor_half_width,
        endpoint_tolerance,
        maximum_length_ratio,
        maximum_length_slack,
    )
    if not all(math.isfinite(value) for value in limits):
        return False
    if (
            corridor_half_width <= 0.0
            or endpoint_tolerance <= 0.0
            or maximum_length_ratio < 1.0
            or maximum_length_slack < 0.0):
        return False
    if len(path_xy) < 2:
        return False
    points = []
    for point in path_xy:
        if len(point) != 2 or not all(math.isfinite(value) for value in point):
            return False
        points.append((float(point[0]), float(point[1])))
    endpoints = tuple(start_xy) + tuple(goal_xy)
    if len(endpoints) != 4 or not all(math.isfinite(value) for value in endpoints):
        return False

    sx, sy = start_xy
    gx, gy = goal_xy
    direct = math.hypot(gx - sx, gy - sy)
    if direct <= 1e-9:
        return False
    if math.hypot(points[0][0] - sx, points[0][1] - sy) > endpoint_tolerance:
        return False
    if math.hypot(points[-1][0] - gx, points[-1][1] - gy) > endpoint_tolerance:
        return False

    dx = gx - sx
    dy = gy - sy
    length_squared = direct * direct
    path_length = 0.0
    previous = points[0]
    for point in points:
        projection = (
            (point[0] - sx) * dx + (point[1] - sy) * dy
        ) / length_squared
        projection = min(1.0, max(0.0, projection))
        closest = (sx + projection * dx, sy + projection * dy)
        if math.hypot(point[0] - closest[0], point[1] - closest[1]) > (
                corridor_half_width):
            return False
        path_length += math.hypot(
            point[0] - previous[0], point[1] - previous[1]
        )
        previous = point
    return path_length <= (
        direct * maximum_length_ratio + maximum_length_slack
    )


def known_free_path_exists(
    flat_data,
    spec,
    start_xy,
    goal_xy,
    free_threshold=20,
    allowed_mask=None,
):
    """Check 4-connected known-free passage using OccupancyGrid evidence only."""
    data = np.asarray(flat_data, dtype=np.int16)
    if data.size != spec.width * spec.height:
        raise ValueError("occupancy data size does not match grid geometry")
    grid = data.reshape((spec.height, spec.width))
    start = spec.world_to_cell(*start_xy)
    goal = spec.world_to_cell(*goal_xy)
    if start is None or goal is None:
        return False
    free = (grid >= 0) & (grid <= free_threshold)
    if allowed_mask is not None:
        allowed = np.asarray(allowed_mask, dtype=bool)
        if allowed.shape != grid.shape:
            raise ValueError("allowed mask shape does not match grid geometry")
        free &= allowed
    if not free[start] or not free[goal]:
        return False

    remaining = free.copy()
    queue = deque([start])
    remaining[start] = False
    while queue:
        row, col = queue.popleft()
        if (row, col) == goal:
            return True
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            next_row = row + dy
            next_col = col + dx
            if (
                0 <= next_row < spec.height
                and 0 <= next_col < spec.width
                and remaining[next_row, next_col]
            ):
                remaining[next_row, next_col] = False
                queue.append((next_row, next_col))
    return False


def nearest_known_free_anchor(
    flat_data,
    spec,
    anchor_xy,
    search_radius_m,
    free_threshold=20,
    allowed_mask=None,
):
    """Find the nearest known-free cell center in a bounded self-clear region.

    A range sensor cannot observe the exact cell occupied by the robot body, so
    an otherwise valid passage check must not depend on that one cell becoming
    known.  The search stays strictly local to the supplied anchor and never
    treats unknown space as traversable.
    """
    values = (
        anchor_xy[0],
        anchor_xy[1],
        search_radius_m,
        spec.resolution,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("anchor search geometry must be finite")
    if (
        spec.width <= 0
        or spec.height <= 0
        or spec.resolution <= 0.0
        or search_radius_m <= 0.0
    ):
        raise ValueError("anchor search geometry must be positive")

    data = np.asarray(flat_data, dtype=np.int16)
    if data.size != spec.width * spec.height:
        raise ValueError("occupancy data size does not match grid geometry")
    grid = data.reshape((spec.height, spec.width))
    center = spec.world_to_cell(*anchor_xy)
    if center is None:
        return None

    allowed = np.ones(grid.shape, dtype=bool)
    if allowed_mask is not None:
        allowed = np.asarray(allowed_mask, dtype=bool)
        if allowed.shape != grid.shape:
            raise ValueError("allowed mask shape does not match grid geometry")

    radius_cells = int(math.ceil(search_radius_m / spec.resolution))
    radius_squared = search_radius_m * search_radius_m
    best = None
    for row in range(
        max(0, center[0] - radius_cells),
        min(spec.height - 1, center[0] + radius_cells) + 1,
    ):
        for col in range(
            max(0, center[1] - radius_cells),
            min(spec.width - 1, center[1] + radius_cells) + 1,
        ):
            if not allowed[row, col] or not 0 <= grid[row, col] <= free_threshold:
                continue
            world_x, world_y = spec.cell_to_world((row, col))
            distance_squared = (
                (world_x - anchor_xy[0]) ** 2
                + (world_y - anchor_xy[1]) ** 2
            )
            if distance_squared > radius_squared + 1e-12:
                continue
            candidate = (distance_squared, row, col, world_x, world_y)
            if best is None or candidate[:3] < best[:3]:
                best = candidate
    if best is None:
        return None
    return best[3], best[4]


def return_anchor_selection(
    anchor_xy,
    start_xy,
    position_tolerance_m,
    margin_m,
):
    """Decide whether a known-free cell may stand in for RECORD_START.

    RECORD_START is the pose the robot occupied before it ever moved, and the
    mapper is reset the moment the public door opens.  A range sensor cannot
    observe the ground it is standing on, and the mission only ever moves
    forward from there, so that cell stays unknown for the whole run.  A
    global planner that refuses unknown space can therefore never produce a
    plan to it.  Substituting the nearest known-free cell is only admissible
    while the substitution stays strictly inside the return position
    tolerance, so the verified return error is still measured against the real
    RECORD_START and cannot be relaxed by this choice.
    """
    values = (position_tolerance_m, margin_m, start_xy[0], start_xy[1])
    if not all(math.isfinite(value) for value in values):
        raise ValueError("return anchor limits must be finite")
    if position_tolerance_m <= 0.0 or margin_m < 0.0:
        raise ValueError(
            "return position tolerance must be positive and its margin "
            "non-negative"
        )
    limit = position_tolerance_m - margin_m
    if limit <= 0.0:
        return {
            "accepted": False,
            "offset_m": None,
            "limit_m": limit,
            "reason": (
                "return anchor margin %.2f m consumes the %.2f m position "
                "tolerance" % (margin_m, position_tolerance_m)
            ),
        }
    if anchor_xy is None:
        return {
            "accepted": False,
            "offset_m": None,
            "limit_m": limit,
            "reason": "no known-free cell near RECORD_START",
        }
    if not all(math.isfinite(value) for value in anchor_xy):
        raise ValueError("return anchor must be finite")
    offset = math.hypot(anchor_xy[0] - start_xy[0], anchor_xy[1] - start_xy[1])
    if offset > limit:
        return {
            "accepted": False,
            "offset_m": offset,
            "limit_m": limit,
            "reason": (
                "nearest known-free cell is %.3f m from RECORD_START, beyond "
                "the %.3f m admissible offset" % (offset, limit)
            ),
        }
    return {
        "accepted": True,
        "offset_m": offset,
        "limit_m": limit,
        "reason": (
            "nearest known-free cell is %.3f m from RECORD_START, inside the "
            "%.3f m admissible offset" % (offset, limit)
        ),
    }


def has_pending_retry(entries, now, maximum_failures):
    """Return whether a valid failed attempt is still inside retry cooldown."""
    return any(
        item.failures < maximum_failures and now < item.retry_after
        for item in entries
    )


def _validate_polygon(polygon_xy):
    points = tuple((float(point[0]), float(point[1])) for point in polygon_xy)
    if len(points) < 3:
        raise ValueError("ROI polygon requires at least three vertices")
    if not all(math.isfinite(value) for point in points for value in point):
        raise ValueError("ROI polygon vertices must be finite")
    twice_area = sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(points, points[1:] + points[:1])
    )
    if abs(twice_area) < 1e-9:
        raise ValueError("ROI polygon must have non-zero area")
    return points


def transform_local_polygon(local_polygon_xy, anchor_xy, anchor_yaw):
    """Transform a floor-entry-relative polygon into its map frame."""
    points = _validate_polygon(local_polygon_xy)
    values = (anchor_xy[0], anchor_xy[1], anchor_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("ROI anchor must be finite")
    cosine = math.cos(anchor_yaw)
    sine = math.sin(anchor_yaw)
    return tuple(
        (
            anchor_xy[0] + cosine * local_x - sine * local_y,
            anchor_xy[1] + sine * local_x + cosine * local_y,
        )
        for local_x, local_y in points
    )


def point_in_polygon(x, y, polygon_xy, boundary_margin_m=0.0):
    """Return whether a point is inside a polygon and its optional inset."""
    points = _validate_polygon(polygon_xy)
    if not all(math.isfinite(value) for value in (x, y, boundary_margin_m)):
        raise ValueError("ROI point and boundary margin must be finite")
    if boundary_margin_m < 0.0:
        raise ValueError("ROI boundary margin cannot be negative")

    inside = False
    minimum_distance_squared = float("inf")
    for first, second in zip(points, points[1:] + points[:1]):
        x1, y1 = first
        x2, y2 = second
        dx = x2 - x1
        dy = y2 - y1
        length_squared = dx * dx + dy * dy
        if length_squared > 0.0:
            fraction = max(
                0.0,
                min(1.0, ((x - x1) * dx + (y - y1) * dy) / length_squared),
            )
            closest_x = x1 + fraction * dx
            closest_y = y1 + fraction * dy
            minimum_distance_squared = min(
                minimum_distance_squared,
                (x - closest_x) ** 2 + (y - closest_y) ** 2,
            )
        intersects = (
            (y1 > y) != (y2 > y)
            and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1
        )
        if intersects:
            inside = not inside
    return (
        inside
        and minimum_distance_squared + 1e-12
        >= boundary_margin_m * boundary_margin_m
    )


def polygon_mask(spec, polygon_xy, boundary_margin_m=0.0):
    """Build a cell-center mask for a map-frame floor ROI polygon."""
    if spec.width <= 0 or spec.height <= 0 or spec.resolution <= 0.0:
        raise ValueError("invalid grid geometry")
    points = _validate_polygon(polygon_xy)
    if not math.isfinite(boundary_margin_m) or boundary_margin_m < 0.0:
        raise ValueError("ROI boundary margin must be finite and non-negative")

    rows, cols = np.indices((spec.height, spec.width), dtype=np.float64)
    world_x = spec.origin_x + (cols + 0.5) * spec.resolution
    world_y = spec.origin_y + (rows + 0.5) * spec.resolution
    inside = np.zeros((spec.height, spec.width), dtype=bool)
    minimum_distance_squared = np.full(
        (spec.height, spec.width), np.inf, dtype=np.float64
    )
    for first, second in zip(points, points[1:] + points[:1]):
        x1, y1 = first
        x2, y2 = second
        dx = x2 - x1
        dy = y2 - y1
        length_squared = dx * dx + dy * dy
        if length_squared > 0.0:
            fraction = np.clip(
                ((world_x - x1) * dx + (world_y - y1) * dy) / length_squared,
                0.0,
                1.0,
            )
            closest_x = x1 + fraction * dx
            closest_y = y1 + fraction * dy
            minimum_distance_squared = np.minimum(
                minimum_distance_squared,
                (world_x - closest_x) ** 2 + (world_y - closest_y) ** 2,
            )
        if abs(y2 - y1) > 0.0:
            intersects = (
                ((y1 > world_y) != (y2 > world_y))
                & (
                    world_x
                    < (x2 - x1) * (world_y - y1) / (y2 - y1) + x1
                )
            )
            inside ^= intersects
    return (
        inside
        & (
            minimum_distance_squared + 1e-12
            >= boundary_margin_m * boundary_margin_m
        )
    )


def map_margin_mask(spec, margin_m):
    """Cell centers at least margin_m inside every OccupancyGrid edge.

    Cells nearer than one sensor range to the grid border can never be
    observed properly: the border is not a wall, so unknown cells behind it
    are an artifact of the grid, not of the building. Intersecting an ROI with
    this mask keeps the "everything in scope is observable" contract without
    having to reject an ROI whose far corner merely overhangs the grid.
    """
    if spec.width <= 0 or spec.height <= 0 or spec.resolution <= 0.0:
        raise ValueError("invalid grid geometry")
    if not math.isfinite(margin_m) or margin_m < 0.0:
        raise ValueError("map boundary margin must be finite and non-negative")
    rows, cols = np.indices((spec.height, spec.width), dtype=np.float64)
    world_x = spec.origin_x + (cols + 0.5) * spec.resolution
    world_y = spec.origin_y + (rows + 0.5) * spec.resolution
    maximum_x = spec.origin_x + spec.width * spec.resolution
    maximum_y = spec.origin_y + spec.height * spec.resolution
    return (
        (world_x >= spec.origin_x + margin_m)
        & (world_x <= maximum_x - margin_m)
        & (world_y >= spec.origin_y + margin_m)
        & (world_y <= maximum_y - margin_m)
    )


def dominant_axis_correction(
    reference_yaw,
    segments,
    maximum_correction_rad,
):
    """Signed yaw correction from reference_yaw onto the dominant wall axis.

    ``segments`` are ``(yaw, weight)`` pairs. Walls carry direction only, so
    each is folded into the +/- 90 degree band around the reference and the
    mean is taken on doubled angles, which is the correct circular statistic
    for an axis. Anything needing more than ``maximum_correction_rad`` is a
    different wall family (a cross wall, a room's far side) and is discarded
    rather than allowed to rotate the reference onto it.

    Returns ``(correction_rad, used_weight, used_count)``, or ``None`` when no
    segment survives -- the caller is expected to keep its declared axis.
    """
    if not math.isfinite(maximum_correction_rad) or maximum_correction_rad <= 0.0:
        raise ValueError("maximum axis correction must be finite and positive")
    if maximum_correction_rad > 0.5 * math.pi:
        raise ValueError("axis correction beyond 90 deg is not identifiable")
    total_weight = 0.0
    sine = 0.0
    cosine = 0.0
    used = 0
    for yaw, weight in segments:
        if not math.isfinite(yaw) or not math.isfinite(weight) or weight <= 0.0:
            continue
        delta = math.atan2(
            math.sin(yaw - reference_yaw), math.cos(yaw - reference_yaw)
        )
        # Fold the antiparallel wall onto the same axis.
        if delta > 0.5 * math.pi:
            delta -= math.pi
        elif delta < -0.5 * math.pi:
            delta += math.pi
        if abs(delta) > maximum_correction_rad:
            continue
        sine += weight * math.sin(2.0 * delta)
        cosine += weight * math.cos(2.0 * delta)
        total_weight += weight
        used += 1
    if used == 0 or total_weight <= 0.0:
        return None
    if abs(sine) < 1e-12 and abs(cosine) < 1e-12:
        return None
    correction = 0.5 * math.atan2(sine, cosine)
    if abs(correction) > maximum_correction_rad:
        return None
    return correction, total_weight, used


def _validate_scope_geometry(
    forward_distance_m,
    rear_distance_m,
    lateral_half_width_m,
    boundary_margin_m,
):
    values = (
        forward_distance_m,
        rear_distance_m,
        lateral_half_width_m,
        boundary_margin_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("scope geometry must be finite")
    if forward_distance_m <= 0.0:
        raise ValueError("scope forward distance must be positive")
    if rear_distance_m < 0.0 or boundary_margin_m < 0.0:
        raise ValueError("scope rear distance and boundary margin cannot be negative")
    if lateral_half_width_m <= 0.0:
        raise ValueError("scope lateral half-width must be positive")
    if forward_distance_m <= boundary_margin_m:
        raise ValueError("scope boundary margin consumes the forward extent")
    if rear_distance_m < boundary_margin_m:
        raise ValueError("scope rear distance must include the boundary margin")
    if lateral_half_width_m <= boundary_margin_m:
        raise ValueError("scope boundary margin consumes the lateral extent")


def point_in_start_aligned_scope(
    x,
    y,
    start_xy,
    start_yaw,
    forward_distance_m,
    rear_distance_m,
    lateral_half_width_m,
    boundary_margin_m=0.0,
):
    """Return whether a map-frame point lies in the start-aligned floor scope."""
    _validate_scope_geometry(
        forward_distance_m,
        rear_distance_m,
        lateral_half_width_m,
        boundary_margin_m,
    )
    values = (x, y, start_xy[0], start_xy[1], start_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("scope point and anchor must be finite")
    delta_x = x - start_xy[0]
    delta_y = y - start_xy[1]
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    longitudinal = cosine * delta_x + sine * delta_y
    lateral = -sine * delta_x + cosine * delta_y
    return (
        -rear_distance_m + boundary_margin_m
        <= longitudinal
        <= forward_distance_m - boundary_margin_m
        and abs(lateral)
        <= lateral_half_width_m - boundary_margin_m
    )


def start_aligned_scope_mask(
    spec,
    start_xy,
    start_yaw,
    forward_distance_m,
    rear_distance_m,
    lateral_half_width_m,
    boundary_margin_m=0.0,
):
    """Build a cell-center mask for one floor's start-aligned work region."""
    if spec.width <= 0 or spec.height <= 0 or spec.resolution <= 0.0:
        raise ValueError("invalid grid geometry")
    _validate_scope_geometry(
        forward_distance_m,
        rear_distance_m,
        lateral_half_width_m,
        boundary_margin_m,
    )
    if not all(
        math.isfinite(value)
        for value in (start_xy[0], start_xy[1], start_yaw)
    ):
        raise ValueError("scope anchor must be finite")

    rows, cols = np.indices((spec.height, spec.width), dtype=np.float64)
    world_x = spec.origin_x + (cols + 0.5) * spec.resolution
    world_y = spec.origin_y + (rows + 0.5) * spec.resolution
    delta_x = world_x - start_xy[0]
    delta_y = world_y - start_xy[1]
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    longitudinal = cosine * delta_x + sine * delta_y
    lateral = -sine * delta_x + cosine * delta_y
    return (
        (longitudinal >= -rear_distance_m + boundary_margin_m)
        & (longitudinal <= forward_distance_m - boundary_margin_m)
        & (np.abs(lateral) <= lateral_half_width_m - boundary_margin_m)
    )


def _dilate(mask, iterations):
    """Chebyshev dilation, treating cells outside the fixed map as unsafe."""
    result = mask.copy()
    for _ in range(max(0, iterations)):
        padded = np.pad(result, 1, mode="constant", constant_values=True)
        expanded = result.copy()
        height, width = result.shape
        for dy in range(3):
            for dx in range(3):
                expanded |= padded[dy:dy + height, dx:dx + width]
        result = expanded
    return result


def frontier_mask(grid, free_threshold=20):
    """Return known-free cells with at least one 4-neighbor unknown cell."""
    free = (grid >= 0) & (grid <= free_threshold)
    unknown = grid < 0
    padded = np.pad(unknown, 1, mode="constant", constant_values=False)
    adjacent_unknown = (
        padded[:-2, 1:-1]
        | padded[2:, 1:-1]
        | padded[1:-1, :-2]
        | padded[1:-1, 2:]
    )
    return free & adjacent_unknown


def _clusters(mask):
    remaining = mask.copy()
    height, width = mask.shape
    result = []
    for row, col in np.argwhere(mask):
        if not remaining[row, col]:
            continue
        remaining[row, col] = False
        queue = deque([(int(row), int(col))])
        cells = []
        while queue:
            current_row, current_col = queue.popleft()
            cells.append((current_row, current_col))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    next_row = current_row + dy
                    next_col = current_col + dx
                    if (
                        0 <= next_row < height
                        and 0 <= next_col < width
                        and remaining[next_row, next_col]
                    ):
                        remaining[next_row, next_col] = False
                        queue.append((next_row, next_col))
        result.append(cells)
    return result


def _nearest_safe_cell(grid, unsafe, allowed, target_row, target_col,
                       radius_cells, free_threshold):
    height, width = grid.shape
    best = None
    best_distance = float("inf")
    row_min = max(0, int(math.floor(target_row - radius_cells)))
    row_max = min(height - 1, int(math.ceil(target_row + radius_cells)))
    col_min = max(0, int(math.floor(target_col - radius_cells)))
    col_max = min(width - 1, int(math.ceil(target_col + radius_cells)))
    for row in range(row_min, row_max + 1):
        for col in range(col_min, col_max + 1):
            if (
                unsafe[row, col]
                or not allowed[row, col]
                or not 0 <= grid[row, col] <= free_threshold
            ):
                continue
            distance = (row - target_row) ** 2 + (col - target_col) ** 2
            if distance < best_distance:
                best_distance = distance
                best = (row, col)
    return best


def extract_frontiers(
    flat_data,
    spec,
    robot_xy,
    min_frontier_length_m=0.35,
    obstacle_clearance_m=0.30,
    goal_standoff_m=0.35,
    goal_search_radius_m=0.60,
    minimum_goal_distance_m=0.45,
    maximum_goal_distance_m=9.0,
    free_threshold=20,
    occupied_threshold=65,
    information_gain_weight=1.0,
    distance_weight=0.25,
    allowed_mask=None,
):
    """Extract, cluster, and score frontiers from an OccupancyGrid snapshot."""
    if spec.width <= 0 or spec.height <= 0 or spec.resolution <= 0.0:
        raise ValueError("invalid grid geometry")
    grid = np.asarray(flat_data, dtype=np.int16)
    if grid.size != spec.width * spec.height:
        raise ValueError("occupancy data size does not match grid geometry")
    grid = grid.reshape((spec.height, spec.width))

    if allowed_mask is None:
        allowed = np.ones(grid.shape, dtype=bool)
    else:
        allowed = np.asarray(allowed_mask, dtype=bool)
        if allowed.shape != grid.shape:
            raise ValueError("allowed mask shape does not match grid geometry")

    mask = frontier_mask(grid, free_threshold=free_threshold)
    # Clip before clustering. Otherwise an exterior part of an 8-connected
    # frontier can dominate the centroid, length, score, and chosen goal even
    # when only a small part of that cluster lies in the floor work region.
    mask &= allowed
    occupied = grid >= occupied_threshold
    clearance_cells = int(math.ceil(obstacle_clearance_m / spec.resolution))
    unsafe = _dilate(occupied, clearance_cells)
    mask &= ~unsafe

    minimum_cells = max(
        1, int(math.ceil(min_frontier_length_m / spec.resolution))
    )
    standoff_cells = goal_standoff_m / spec.resolution
    search_cells = max(1, int(math.ceil(goal_search_radius_m / spec.resolution)))
    robot_x, robot_y = robot_xy
    frontiers = []

    for cells in _clusters(mask):
        if len(cells) < minimum_cells:
            continue
        rows = np.asarray([cell[0] for cell in cells], dtype=np.float64)
        cols = np.asarray([cell[1] for cell in cells], dtype=np.float64)
        centroid_row = float(np.mean(rows))
        centroid_col = float(np.mean(cols))
        # A frontier can be a closed ring around the initially observed island.
        # Its geometric centroid is then inside known space rather than on the
        # frontier, and averaging all unknown neighbors cancels the outward
        # normal. Anchor the goal on a real frontier cell nearest the centroid
        # and derive the normal from that cell's local unknown neighbors.
        representative = min(
            cells,
            key=lambda cell: (
                (cell[0] - centroid_row) ** 2
                + (cell[1] - centroid_col) ** 2
            ),
        )
        frontier_row = float(representative[0])
        frontier_col = float(representative[1])

        unknown_neighbors = []
        row, col = representative
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            next_row, next_col = row + dy, col + dx
            if (
                0 <= next_row < spec.height
                and 0 <= next_col < spec.width
                and grid[next_row, next_col] < 0
            ):
                unknown_neighbors.append((next_row, next_col))
        if not unknown_neighbors:
            continue
        unknown_row = float(np.mean([cell[0] for cell in unknown_neighbors]))
        unknown_col = float(np.mean([cell[1] for cell in unknown_neighbors]))
        normal_row = unknown_row - frontier_row
        normal_col = unknown_col - frontier_col
        normal_length = math.hypot(normal_row, normal_col)
        if normal_length < 1e-6:
            normal_row, normal_col, normal_length = 0.0, 1.0, 1.0

        target_row = frontier_row - standoff_cells * normal_row / normal_length
        target_col = frontier_col - standoff_cells * normal_col / normal_length
        goal_cell = _nearest_safe_cell(
            grid,
            unsafe,
            allowed,
            target_row,
            target_col,
            search_cells,
            free_threshold,
        )
        if goal_cell is None:
            continue
        goal_x, goal_y = spec.cell_to_world(goal_cell)
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if distance < minimum_goal_distance_m or distance > maximum_goal_distance_m:
            continue

        frontier_x, frontier_y = spec.cell_to_world(
            (frontier_row, frontier_col)
        )
        yaw = math.atan2(frontier_y - goal_y, frontier_x - goal_x)
        length_m = len(cells) * spec.resolution
        score = (
            information_gain_weight * length_m
            - distance_weight * distance
        )
        frontiers.append(
            FrontierCluster(
                cells=tuple(cells),
                goal_cell=goal_cell,
                goal_x=goal_x,
                goal_y=goal_y,
                yaw=yaw,
                length_m=length_m,
                distance_m=distance,
                score=score,
            )
        )

    frontiers.sort(key=lambda item: item.score, reverse=True)
    return frontiers


def coverage_ratio(flat_data, allowed_mask=None):
    """Known-cell fraction of the whole grid or an explicit floor scope."""
    data = np.asarray(flat_data, dtype=np.int16)
    if data.size == 0:
        return 0.0
    if allowed_mask is None:
        return float(np.count_nonzero(data >= 0)) / float(data.size)
    allowed = np.asarray(allowed_mask, dtype=bool)
    if allowed.size != data.size:
        raise ValueError("allowed mask size does not match occupancy data")
    allowed = allowed.reshape(data.shape)
    allowed_cells = int(np.count_nonzero(allowed))
    if allowed_cells == 0:
        raise ValueError("allowed mask contains no cells")
    return float(np.count_nonzero((data >= 0) & allowed)) / float(allowed_cells)


def point_near(points, x, y, radius):
    radius_squared = radius * radius
    return any(
        (point[0] - x) ** 2 + (point[1] - y) ** 2 <= radius_squared
        for point in points
    )


def failed_goal_state(entries, x, y, radius, now, maximum_failures):
    """Return ``permanent``, ``cooldown``, or ``available`` for a target."""
    radius_squared = radius * radius
    nearby = [
        item for item in entries
        if (item.x - x) ** 2 + (item.y - y) ** 2 <= radius_squared
    ]
    if not nearby:
        return "available"
    item = max(nearby, key=lambda entry: entry.unreachable_failures)
    if item.unreachable_failures >= maximum_failures:
        return "permanent"
    item = max(nearby, key=lambda entry: entry.failures)
    if now < item.retry_after:
        return "cooldown"
    return "available"


def room_frontier_rejection(
        score, goal_x, goal_y, minimum_score, visited_goals, visited_radius,
        failed_goals, failed_radius, now, maximum_failures):
    """Which gate bars a room-transaction frontier, or ``None`` if none does.

    Pulled out of the transaction loop so that a room which ends early can say
    WHICH stage emptied its candidate list.  mf49's floor-1 station 5 right
    room reported "no reachable frontier remains" after generating four raw
    candidates and admitting three of them, and nothing in the log could
    distinguish "scored too low" from "already visited" from "blacklisted"
    from "the planner said no".

    Reachability is deliberately NOT decided here: it needs the planner, and
    an unavailable planner is not the same answer as an unreachable goal.
    Returns ``"score"``, ``"visited"``, ``"history"``, or ``None``.
    """
    if score < minimum_score:
        return "score"
    if point_near(visited_goals, goal_x, goal_y, visited_radius):
        return "visited"
    if failed_goal_state(
            failed_goals, goal_x, goal_y, failed_radius, now,
            maximum_failures) != "available":
        return "history"
    return None


def corridor_probe_goal_state(
        entries, x, y, radius, now, maximum_attempts):
    """Return corridor-probe history state without inventing unreachability.

    Synthetic corridor probes need a finite per-action attempt budget: a
    no-progress cancellation is transient and must not become globally
    ``permanent``, but retrying the identical synthetic target forever is a
    livelock.  ``attempt_budget`` retires only this probe coordinate for the
    current floor action.  The shared frontier selector still uses
    :func:`failed_goal_state`, so a real frontier at the same location remains
    available after its transient cooldown.
    """
    maximum_attempts = int(maximum_attempts)
    if maximum_attempts < 1:
        raise ValueError("corridor probe maximum attempts must be positive")
    state = failed_goal_state(
        entries, x, y, radius, now, maximum_attempts
    )
    if state != "available":
        return state
    radius_squared = radius * radius
    nearby = [
        item for item in entries
        if (item.x - x) ** 2 + (item.y - y) ** 2 <= radius_squared
    ]
    if nearby and max(item.failures for item in nearby) >= maximum_attempts:
        return "attempt_budget"
    return "available"


def record_failure(entries, x, y, radius, now, cooldown, kind="unreachable"):
    """Record a failed attempt.

    ``kind`` is ``"unreachable"`` for outcomes that prove the target cannot be
    reached, or ``"transient"`` for timeouts, cancellations, preemptions and
    safety interlocks.  Only ``"unreachable"`` counts toward the permanent
    exclusion budget; transient failures merely trigger a retry cooldown.
    """
    unreachable = 1 if kind == "unreachable" else 0
    radius_squared = radius * radius
    for item in entries:
        if (item.x - x) ** 2 + (item.y - y) ** 2 <= radius_squared:
            item.failures += 1
            item.unreachable_failures += unreachable
            item.retry_after = now + cooldown
            item.x = x
            item.y = y
            return item
    item = FailedGoal(x=x, y=y, failures=1, retry_after=now + cooldown,
                      unreachable_failures=unreachable)
    entries.append(item)
    return item


def room_axis_bounds(
        own_longitudinal,
        own_lateral,
        doorway_coordinates,
        minimum_station_separation):
    """Return midpoint bounds to the nearest perceived same-side door stations.

    The room door plane constrains motion across the corridor but not along it.
    When a transient map gap joins adjacent rooms, perceived door stations are
    the only live delimiter available.  A neighbour contributes the midpoint
    between its longitudinal coordinate and this room's coordinate.  Duplicate
    observations inside ``minimum_station_separation`` and opposite-side or
    centreline detections do not delimit this room.

    The result is ``(lower, upper)``; either value is ``None`` when no delimiter
    has yet been perceived on that side.  Open bounds are deliberate: callers
    must not invent a building dimension or hidden scene boundary.
    """
    values = (
        float(own_longitudinal),
        float(own_lateral),
        float(minimum_station_separation),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("room station geometry must be finite")
    own_longitudinal, own_lateral, minimum_station_separation = values
    if minimum_station_separation < 0.0:
        raise ValueError("minimum_station_separation must be non-negative")
    if own_lateral == 0.0:
        raise ValueError("room station must lie on one side of the corridor")

    own_side = 1 if own_lateral > 0.0 else -1
    lower = None
    upper = None
    for other_longitudinal, other_lateral in doorway_coordinates:
        other_longitudinal = float(other_longitudinal)
        other_lateral = float(other_lateral)
        if not (
                math.isfinite(other_longitudinal)
                and math.isfinite(other_lateral)):
            raise ValueError("neighbour room station geometry must be finite")
        if other_lateral == 0.0:
            continue
        other_side = 1 if other_lateral > 0.0 else -1
        if other_side != own_side:
            continue
        delta = other_longitudinal - own_longitudinal
        if abs(delta) < minimum_station_separation:
            continue
        midpoint = own_longitudinal + 0.5 * delta
        if delta > 0.0:
            upper = midpoint if upper is None else min(upper, midpoint)
        elif delta < 0.0:
            lower = midpoint if lower is None else max(lower, midpoint)
    return lower, upper


def _validated_room_source_keepout_geometry(
        door_xy, inward_xy, depth_m, half_width_m):
    values = (
        float(door_xy[0]),
        float(door_xy[1]),
        float(inward_xy[0]),
        float(inward_xy[1]),
        float(depth_m),
        float(half_width_m),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("room source keep-out geometry must be finite")
    door_x, door_y, inward_x, inward_y, depth_m, half_width_m = values
    inward_norm = math.hypot(inward_x, inward_y)
    if inward_norm <= 1e-9 or depth_m <= 0.0 or half_width_m <= 0.0:
        raise ValueError(
            "room source keep-out direction and dimensions must be positive"
        )
    return (
        door_x,
        door_y,
        inward_x / inward_norm,
        inward_y / inward_norm,
        depth_m,
        half_width_m,
    )


def point_in_room_source_keepout(
        x, y, door_xy, inward_xy, depth_m, half_width_m):
    """Whether a point lies in the source-free channel behind a room door.

    This is a source-placement rule, not a collision or traversal rule.  The
    rectangle starts at the perceived door centre and extends only into the
    room.  Mandatory doorway transit therefore remains the caller's separate
    responsibility.
    """
    if not all(math.isfinite(float(value)) for value in (x, y)):
        raise ValueError("room source keep-out point must be finite")
    door_x, door_y, inward_x, inward_y, depth_m, half_width_m = \
        _validated_room_source_keepout_geometry(
            door_xy, inward_xy, depth_m, half_width_m
        )
    delta_x = float(x) - door_x
    delta_y = float(y) - door_y
    longitudinal = delta_x * inward_x + delta_y * inward_y
    lateral = -delta_x * inward_y + delta_y * inward_x
    return (
        -1e-12 <= longitudinal <= depth_m + 1e-12
        and abs(lateral) <= half_width_m + 1e-12
    )


def room_source_keepout_mask(
        spec, door_xy, inward_xy, depth_m, half_width_m):
    """Cell-centre mask for a room door's source-free channel."""
    if spec.width <= 0 or spec.height <= 0 or spec.resolution <= 0.0:
        raise ValueError("invalid grid geometry")
    door_x, door_y, inward_x, inward_y, depth_m, half_width_m = \
        _validated_room_source_keepout_geometry(
            door_xy, inward_xy, depth_m, half_width_m
        )
    rows, cols = np.indices((spec.height, spec.width), dtype=np.float64)
    world_x = spec.origin_x + (cols + 0.5) * spec.resolution
    world_y = spec.origin_y + (rows + 0.5) * spec.resolution
    delta_x = world_x - door_x
    delta_y = world_y - door_y
    longitudinal = delta_x * inward_x + delta_y * inward_y
    lateral = -delta_x * inward_y + delta_y * inward_x
    return (
        (longitudinal >= -1e-12)
        & (longitudinal <= depth_m + 1e-12)
        & (np.abs(lateral) <= half_width_m + 1e-12)
    )


def room_queue_order(candidates, completed_branches):
    """Order outstanding room branches as a persistent work queue.

    ``candidates`` is an iterable of ``(branch, longitudinal, lateral)``.  A
    branch stays queued until its transaction proves coverage, so the only
    exclusion here is membership of ``completed_branches``; travel does not
    evict anything.

    The order is outward from the entrance -- ascending longitudinal, and at one
    door station robot-left before robot-right -- so the corridor is walked once
    instead of being re-traversed to collect rooms that a long frontier goal
    left behind.
    """
    outstanding = [
        (longitudinal, 0 if lateral > 0.0 else 1, branch)
        for branch, longitudinal, lateral in candidates
        if branch not in completed_branches
    ]
    outstanding.sort(key=lambda item: (item[0], item[1]))
    return [branch for _longitudinal, _side, branch in outstanding]


def room_completion_state(completed, unproven, target, revisit_attempts):
    """Decide whether a floor's room quota is genuinely satisfied.

    ``completed`` is the set of branches that have been entered, covered and
    exited. ``unproven`` maps a branch to how many times it has been left on
    budget rather than on proof -- the transaction exited cleanly but could not
    show that no reachable frontier remained inside.

    Counting an unproven room toward the quota is how a floor gets declared
    finished with a room that still had unexplored space in it, which is
    exactly where an unseen danger source would be. Such a branch stays
    revivable until its attempts are spent; only then does it count.

    Returns ``(floor_complete, revivable)``.
    """
    if target is None or target <= 0:
        return False, []
    revivable = sorted(
        branch for branch, spent in unproven.items()
        if branch in completed and spent < revisit_attempts
    )
    return (len(completed) >= target and not revivable), revivable
