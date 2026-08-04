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
    margin = max(0.0, goal_margin)
    # Walk as far as the map has proved, not all-or-nothing.
    #
    # This used to make "desired" the whole remaining leg and refuse to move
    # unless the grid proved that entire distance at once. mf28 stood in the
    # car for 45 wall seconds and failed with 1.05 m of proven strip ahead,
    # because the leg was 2.30 m: room for a 0.70 m step existed the whole time
    # and was never taken. Raising maximum_step to 2.40 made it worse, since
    # desired then became the full leg; the old 0.85 cap had been accidentally
    # forcing the small steps that made progress.
    #
    # A goal shorter than minimum_goal is still refused: move_base accepts
    # anything inside xy_goal_tolerance, so such a goal commands no motion.
    # That is why a remaining distance below minimum_goal still asks for
    # minimum_goal rather than the remainder.
    want = max(remaining, minimum_goal)
    provable = min(maximum_step, max(0.0, free_run - margin))
    step = min(want, provable)
    # minimum_goal is not only a move_base tolerance floor, it is a momentum
    # floor. Measured peak |vx| against outcome at the 6 cm elevator sill:
    #   0.416 (mf20 exit step 3)   stuck
    #   0.459 (mf29 car entry)     stuck
    #   0.690 (mf25 car entry)     crossed
    #   0.767 (mf22 2.30 m step)   crossed
    # DWA decelerates into its goal, so a short goal caps the peak speed and
    # the rear legs never follow the front ones onto the sill -- the mechanism
    # exploration.yaml already records for the entrance apron. Waiting for the
    # map to prove a step long enough to build speed is therefore better than
    # taking a short one that cannot cross; a short step is not a small amount
    # of progress, it is none.
    if step + 1.0e-9 < minimum_goal:
        return 0.0
    return step


def apply_forward_speed_floor(vx, wz, minimum_speed, maximum_abs_yaw_rate):
    """Raise a low straight-ahead command while leaving all other motion alone.

    The caller owns the spatial and temporal gate.  This helper only decides
    whether an already-forward DWA command is straight enough for a bounded
    elevator-sill speed floor.
    """
    values = (vx, wz, minimum_speed, maximum_abs_yaw_rate)
    if not all(math.isfinite(value) for value in values):
        return vx, False
    if minimum_speed <= 0.0 or maximum_abs_yaw_rate < 0.0:
        return vx, False
    if vx <= 0.0 or vx >= minimum_speed:
        return vx, False
    if abs(wz) > maximum_abs_yaw_rate:
        return vx, False
    return minimum_speed, True


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
