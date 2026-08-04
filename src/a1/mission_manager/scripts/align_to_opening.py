#!/usr/bin/env python3
"""Find the elevator car's open side from the occupancy grid.

Why this is needed
------------------
turn_inside_elevator_before_transfer() aligns to the door by pure geometry:
``target_yaw = start_yaw + pi``. That is sound BEFORE the transfer, because the
robot walked into the car facing inwards, so 180 degrees necessarily faces the
door. After the transfer that reference is gone -- the localization generation
has been reset and there is no "direction I walked in from" any more. The fixed
route then leaves along whatever yaw the robot happens to hold.

Measured on seed 382835531, mf05: the robot exited the car but drifted from
86 deg to 112 deg over the 1.1 m it travelled, i.e. 26 degrees off, and the
subsequent fixed 95 deg turn aborted with "Rotation cmd in collision". mf04 on
the same seed and code failed one step earlier, at the straight 2 m exit, with
"Failed to get a plan". Same seed, same code, different failure step: the
variable is the post-transfer heading, exactly as the multi-floor author
suspected ("有时候机器人来到二楼后不是直接面对着电梯门出口而是斜对着的").

The approach
------------
A car is walls on three sides and an opening on the fourth. Cast rays from the
robot over a full circle in the published grid and measure how far each ray
travels before hitting an occupied cell. The opening is the direction with the
largest free run. This uses only the map the formal chain already builds; it
needs no elevator template, no floor-specific constant, and works the same on
every floor -- which is the point, since the author's own notes say upper
floors deliberately do not detect the elevator doorway.

Returns the opening bearing in the grid frame, or None when the evidence is too
weak to act on (no clear maximum, or too few valid rays).

⚠️ The caller no longer falls back on None. mf13 measured what falling back
costs: floor 2 skipped the alignment, the fixed route departed on an
uncorrected heading, and the robot walked off the floor (oracle z 5.52 -> 3.49
-> 0.06 m). align_to_car_opening() now raises instead.

⚠️ And the acceptance test below is weaker than it looks. It is contrast only
-- best free run minus median >= min_contrast. Because rays stop at UNKNOWN
cells, "longest free run" is really "the direction the map has been observed
furthest", which straight after a floor reset is just wherever the robot has
been looking. Dumped on mf15 floor 1: 0.24% of the grid known and the bearing
accepted on the first attempt. lobe_concentration is recorded in diagnostics
but does NOT participate in the decision. Do not read a returned bearing as
evidence that the car opening was found.
"""

import math


def opening_bearing(grid, robot_x, robot_y, occupied_threshold=65,
                    max_range=3.0, ray_count=180, min_contrast=0.8,
                    diagnostics=None):
    """Bearing (rad, grid frame) of the widest free run around the robot.

    ``min_contrast`` is how much further the best ray must reach than the
    median ray before the result is trusted, in metres. A car with one open
    side gives a large contrast; a robot already in open space gives almost
    none, and this returns None rather than inventing a direction.
    """
    info = grid.info
    step = info.resolution * 0.5
    steps = max(1, int(max_range / step))

    def free_run(bearing):
        cos_b, sin_b = math.cos(bearing), math.sin(bearing)
        for index in range(1, steps + 1):
            distance = index * step
            x = robot_x + distance * cos_b
            y = robot_y + distance * sin_b
            column = int((x - info.origin.position.x) / info.resolution)
            row = int((y - info.origin.position.y) / info.resolution)
            if not (0 <= column < info.width and 0 <= row < info.height):
                return distance
            value = grid.data[row * info.width + column]
            if value >= occupied_threshold:
                return distance
            # Unknown cells stop the ray too: an opening has to be observed,
            # not assumed. Treating unknown as free is what lets a robot aim
            # confidently at a wall it has never seen.
            if value < 0:
                return distance
        return max_range

    runs = []
    for index in range(ray_count):
        bearing = -math.pi + (2.0 * math.pi) * index / ray_count
        runs.append((free_run(bearing), bearing))
    if not runs:
        return None

    runs.sort()
    median = runs[len(runs) // 2][0]
    best_run, best_bearing = runs[-1]
    if diagnostics is not None:
        # Pure observation: never changes the decision below.
        diagnostics["rays"] = [(round(b, 4), round(r, 4)) for r, b in runs]
        diagnostics["median_run"] = median
        diagnostics["best_run"] = best_run
        diagnostics["best_bearing"] = best_bearing
        diagnostics["contrast"] = best_run - median
        diagnostics["min_contrast"] = min_contrast
        diagnostics["max_range"] = max_range
        diagnostics["at_max_range"] = sum(
            1 for r, _ in runs if r >= max_range - 1e-9)
        diagnostics["capped_fraction"] = (
            diagnostics["at_max_range"] / float(len(runs)))
    if best_run - median < min_contrast:
        if diagnostics is not None:
            diagnostics["accepted"] = False
        return None

    # Average the bearings within 90% of the best run so the result is the
    # centre of the opening rather than one lucky ray at its edge.
    good = [b for run, b in runs if run >= 0.9 * best_run]
    sin_sum = sum(math.sin(b) for b in good)
    cos_sum = sum(math.cos(b) for b in good)
    if diagnostics is not None:
        diagnostics["accepted"] = True
        diagnostics["good_count"] = len(good)
        # Angular spread of the averaged lobe. A car opening is one narrow
        # lobe; a wide or split spread means the circular mean lands between
        # lobes -- i.e. possibly at a wall.
        diagnostics["good_bearings"] = [round(b, 4) for b in sorted(good)]
        resultant = math.hypot(sin_sum, cos_sum) / max(1, len(good))
        diagnostics["lobe_concentration"] = resultant
    if abs(sin_sum) < 1e-9 and abs(cos_sum) < 1e-9:
        return best_bearing
    return math.atan2(sin_sum, cos_sum)
