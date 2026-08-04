#!/usr/bin/env python3
"""Find one map-observed elevator opening without building coordinates.

The target-floor localization frame is restarted after every elevator transfer,
so the body's arrival yaw is not evidence that it faces the door.  This helper
looks for the one footprint-wide, sufficiently long free-space lobe around the
robot.  Unknown cells, occupied cells and cells outside the current grid all
stop a probe through :func:`known_free_run_in_grid`.

The result is deliberately fail-closed.  A needle-like gap, an over-wide open
area, or two similarly deep openings returns ``None`` instead of inventing a
direction.  Mapping-generation freshness and agreement across distinct map
timestamps remain the caller's responsibility.
"""

import math

from elevator_exit import known_free_run_in_grid


def _circular_components(mask):
    """Return contiguous True-index components, joining the circular seam."""
    count = len(mask)
    if count == 0 or not any(mask):
        return []
    if all(mask):
        return [list(range(count))]

    first_false = next(index for index, value in enumerate(mask) if not value)
    components = []
    current = []
    for offset in range(1, count + 1):
        index = (first_false + offset) % count
        if mask[index]:
            current.append(index)
        elif current:
            components.append(current)
            current = []
    if current:
        components.append(current)
    return components


def _lobe(indices, bearings, runs, angular_step, minimum_open_run,
          resolution):
    weights = [max(resolution, runs[index] - minimum_open_run)
               for index in indices]
    sin_sum = sum(weight * math.sin(bearings[index])
                  for index, weight in zip(indices, weights))
    cos_sum = sum(weight * math.cos(bearings[index])
                  for index, weight in zip(indices, weights))
    return {
        "bearing": math.atan2(sin_sum, cos_sum),
        "width": len(indices) * angular_step,
        "peak_run": max(runs[index] for index in indices),
        "mean_run": (sum(runs[index] for index in indices) /
                     float(len(indices))),
        "ray_count": len(indices),
    }


def opening_bearing(grid, robot_x, robot_y, occupied_threshold=65,
                    max_range=2.75, ray_count=180, min_contrast=0.8,
                    half_width=0.22, minimum_open_run=1.40,
                    minimum_lobe_width=math.radians(8.0),
                    maximum_lobe_width=math.radians(80.0),
                    minimum_peak_advantage=0.40, diagnostics=None):
    """Return the grid-frame bearing of one uniquely supported opening.

    Every angular probe checks a swept strip of ``2 * half_width`` rather than
    a centre ray.  Qualifying adjacent probes form circular lobes.  Exactly one
    plausible lobe must be deep, wide enough to be real, narrow enough still to
    describe a doorway, and clearly better than a runner-up.
    """
    if diagnostics is not None:
        diagnostics.clear()
        diagnostics["accepted"] = False

    if (grid is None or ray_count < 8 or max_range <= 0.0 or
            minimum_open_run <= 0.0 or minimum_open_run > max_range or
            half_width < 0.0 or minimum_lobe_width <= 0.0 or
            maximum_lobe_width < minimum_lobe_width or
            minimum_peak_advantage < 0.0):
        if diagnostics is not None:
            diagnostics["reason"] = "invalid_arguments"
        return None

    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        if diagnostics is not None:
            diagnostics["reason"] = "invalid_grid"
        return None

    angular_step = 2.0 * math.pi / float(ray_count)
    bearings = [-math.pi + angular_step * index
                for index in range(ray_count)]
    runs = [
        known_free_run_in_grid(
            grid, robot_x, robot_y, bearing, max_range,
            occupied_threshold, half_width)
        for bearing in bearings
    ]
    sorted_runs = sorted(runs)
    median = sorted_runs[len(sorted_runs) // 2]
    components = _circular_components(
        [run >= minimum_open_run for run in runs])
    raw_lobes = [
        _lobe(component, bearings, runs, angular_step,
              minimum_open_run, resolution)
        for component in components
    ]
    lobes = [
        lobe for lobe in raw_lobes
        if minimum_lobe_width <= lobe["width"] <= maximum_lobe_width
    ]
    lobes.sort(key=lambda item: (item["peak_run"], item["mean_run"]),
               reverse=True)

    if diagnostics is not None:
        diagnostics.update({
            "median_run": median,
            "max_range": max_range,
            "minimum_open_run": minimum_open_run,
            "minimum_lobe_width": minimum_lobe_width,
            "maximum_lobe_width": maximum_lobe_width,
            "minimum_peak_advantage": minimum_peak_advantage,
            "raw_lobes": raw_lobes,
            "lobes": lobes,
            "runs": [(bearings[index], runs[index])
                     for index in range(ray_count)],
        })

    if not lobes:
        if diagnostics is not None:
            diagnostics["reason"] = "no_plausible_lobe"
        return None

    best = lobes[0]
    contrast = best["peak_run"] - median
    if diagnostics is not None:
        diagnostics["contrast"] = contrast
    if contrast < min_contrast:
        if diagnostics is not None:
            diagnostics["reason"] = "insufficient_contrast"
        return None

    if (len(lobes) > 1 and
            best["peak_run"] - lobes[1]["peak_run"] <
            minimum_peak_advantage):
        if diagnostics is not None:
            diagnostics["reason"] = "ambiguous_runner_up"
        return None

    if diagnostics is not None:
        diagnostics["accepted"] = True
        diagnostics["reason"] = "unique_opening"
        diagnostics["best_bearing"] = best["bearing"]
        diagnostics["best_run"] = best["peak_run"]
        diagnostics["best_lobe_width"] = best["width"]
    return best["bearing"]
