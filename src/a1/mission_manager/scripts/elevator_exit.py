#!/usr/bin/env python3
"""Pure geometry helpers for a fail-closed, map-checked elevator exit."""

import math


def _grid_value(grid, x, y):
    """Return a grid value at ``(x, y)``, or ``None`` outside/invalid grids."""
    info = grid.info
    resolution = float(info.resolution)
    if resolution <= 0.0 or info.width <= 0 or info.height <= 0:
        return None
    column = int(math.floor((x - info.origin.position.x) / resolution))
    row = int(math.floor((y - info.origin.position.y) / resolution))
    if not (0 <= column < info.width and 0 <= row < info.height):
        return None
    index = row * info.width + column
    if index < 0 or index >= len(grid.data):
        return None
    return int(grid.data[index])


def known_free_run_in_grid(grid, x, y, bearing, max_range,
                           occupied_threshold, half_width=0.0):
    """Measure a footprint-width run of known-free occupancy cells.

    Unknown, occupied, out-of-grid and malformed cells all stop the run.  A
    non-zero ``half_width`` checks a swept strip rather than only a centre ray,
    so a goal is not accepted merely because one line fits between obstacles.
    """
    if grid is None or max_range <= 0.0 or half_width < 0.0:
        return 0.0
    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        return 0.0
    step = 0.5 * resolution
    forward_steps = max(1, int(math.ceil(max_range / step)))
    lateral_steps = max(1, int(math.ceil(half_width / step)))
    if half_width == 0.0:
        offsets = (0.0,)
    else:
        offsets = tuple(
            -half_width + 2.0 * half_width * index / float(2 * lateral_steps)
            for index in range(2 * lateral_steps + 1)
        )
    cos_b = math.cos(bearing)
    sin_b = math.sin(bearing)
    for index in range(1, forward_steps + 1):
        distance = min(max_range, index * step)
        for lateral in offsets:
            sample_x = x + distance * cos_b - lateral * sin_b
            sample_y = y + distance * sin_b + lateral * cos_b
            value = _grid_value(grid, sample_x, sample_y)
            if value is None or value < 0 or value >= occupied_threshold:
                return max(0.0, distance - step)
        if distance >= max_range:
            break
    return float(max_range)


def bounded_exit_step(remaining, free_run, maximum_step, minimum_goal,
                      goal_margin):
    """Choose one map-proven step, or return zero until more map is known."""
    if remaining <= 0.0 or maximum_step <= 0.0 or minimum_goal <= 0.0:
        return 0.0
    desired = min(maximum_step, max(remaining, minimum_goal))
    if free_run - max(0.0, goal_margin) + 1.0e-9 < desired:
        return 0.0
    return desired


def choose_corridor_side(left_run, right_run, minimum_run,
                         minimum_advantage):
    """Select a measured corridor side, failing closed on weak/ambiguous data."""
    left_ok = left_run >= minimum_run
    right_ok = right_run >= minimum_run
    if not left_ok and not right_ok:
        return None
    if left_ok and not right_ok:
        return "left"
    if right_ok and not left_ok:
        return "right"
    if abs(left_run - right_run) < minimum_advantage:
        return None
    return "left" if left_run > right_run else "right"
