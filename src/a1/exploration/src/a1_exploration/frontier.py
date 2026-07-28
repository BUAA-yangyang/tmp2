"""ROS-independent frontier extraction and scoring.

The module deliberately consumes only an occupancy grid.  It does not know
about Gazebo, building metadata, or referee truth, so the same selector can be
reused when floor_mapping grows from one floor session to a multi-floor map
cache.
"""

from collections import deque
from dataclasses import dataclass
import math

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
    item = max(nearby, key=lambda entry: entry.failures)
    if item.failures >= maximum_failures:
        return "permanent"
    if now < item.retry_after:
        return "cooldown"
    return "available"


def record_failure(entries, x, y, radius, now, cooldown):
    radius_squared = radius * radius
    for item in entries:
        if (item.x - x) ** 2 + (item.y - y) ** 2 <= radius_squared:
            item.failures += 1
            item.retry_after = now + cooldown
            item.x = x
            item.y = y
            return item
    item = FailedGoal(x=x, y=y, failures=1, retry_after=now + cooldown)
    entries.append(item)
    return item
