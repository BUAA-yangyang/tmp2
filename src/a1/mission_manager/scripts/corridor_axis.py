#!/usr/bin/env python3
"""Pure geometry for a generation-local main-corridor axis estimate."""

from dataclasses import dataclass
import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _axis_near_reference(axis, reference):
    first = normalize_angle(axis)
    second = normalize_angle(axis + math.pi)
    if abs(normalize_angle(first - reference)) <= abs(
            normalize_angle(second - reference)):
        return first
    return second


def _undirected_error(first, second):
    error = abs(normalize_angle(first - second))
    return min(error, abs(math.pi - error))


def generation_is_new(current, previous):
    """A restart is proven only by a present generation that changed."""
    if (current is None or previous is None or
            str(current).strip() == "" or str(previous).strip() == ""):
        return False
    return str(current) != str(previous)


def measurement_matches_identity(measurement, generation, session):
    """Reject measurements with absent, malformed, or stale ownership."""
    try:
        measured_generation = int(measurement.localization_generation)
        measured_session = int(measurement.floor_session_id)
        expected_generation = int(generation)
        expected_session = int(session)
    except (AttributeError, TypeError, ValueError):
        return False
    return (
        expected_generation >= 0
        and expected_session >= 0
        and measured_generation == expected_generation
        and measured_session == expected_session
    )


def stamped_snapshot_can_bind(message_stamp, epoch_not_before,
                              generation, session):
    """Prove an unlabelled stamped snapshot belongs after an epoch cutover.

    OccupancyGrid carries a ROS timestamp but no floor-session fields.  It may
    be associated with the current mapping identity only when that identity is
    complete and the sample was produced no earlier than the explicit reset
    barrier.  Zero, malformed and non-finite stamps fail closed.
    """
    try:
        message_stamp = float(message_stamp)
        epoch_not_before = float(epoch_not_before)
        generation = int(generation)
        session = int(session)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(message_stamp)
        and math.isfinite(epoch_not_before)
        and message_stamp > 0.0
        and epoch_not_before >= 0.0
        and message_stamp >= epoch_not_before
        and generation >= 0
        and session >= 0
    )


@dataclass(frozen=True)
class CorridorAxisEstimate:
    yaw: float
    width: float
    correction: float
    left_id: int
    right_id: int
    score: float


def estimate_corridor_axis(
        walls,
        robot_xy,
        reference_yaw,
        maximum_correction,
        parallel_tolerance,
        minimum_wall_length,
        minimum_width,
        maximum_width,
        maximum_midpoint_distance):
    """Return the best opposite-wall corridor pair, or ``None``.

    Wall direction is axial (yaw and yaw+pi are equivalent).  A valid pair must
    be fresh/session-filtered by the caller, stable, observed, approximately
    parallel to the declared entry axis, bracket the robot, and have a plausible
    corridor width.  No simulator or layout coordinates enter this estimate.
    """
    values = (
        robot_xy[0], robot_xy[1], reference_yaw, maximum_correction,
        parallel_tolerance, minimum_wall_length, minimum_width,
        maximum_width, maximum_midpoint_distance,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("corridor-axis inputs must be finite")
    if (
            maximum_correction <= 0.0
            or parallel_tolerance <= 0.0
            or minimum_wall_length <= 0.0
            or minimum_width <= 0.0
            or maximum_width <= minimum_width
            or maximum_midpoint_distance <= 0.0):
        raise ValueError("corridor-axis limits are invalid")

    candidates = []
    for wall in walls:
        if (
                not bool(getattr(wall, "stable", False))
                or str(getattr(wall, "status", "")) != "observed"
                or float(getattr(wall, "length", 0.0))
                < minimum_wall_length):
            continue
        start = wall.start
        end = wall.end
        dx = float(end.x) - float(start.x)
        dy = float(end.y) - float(start.y)
        length = math.hypot(dx, dy)
        if length < minimum_wall_length:
            continue
        midpoint = (
            0.5 * (float(start.x) + float(end.x)),
            0.5 * (float(start.y) + float(end.y)),
        )
        if math.hypot(
                midpoint[0] - robot_xy[0],
                midpoint[1] - robot_xy[1]) > maximum_midpoint_distance:
            continue
        yaw = _axis_near_reference(math.atan2(dy, dx), reference_yaw)
        correction = normalize_angle(yaw - reference_yaw)
        if abs(correction) > maximum_correction:
            continue
        confidence = max(0.05, float(getattr(wall, "confidence", 0.0)))
        weight = min(length, 8.0) * confidence
        candidates.append((yaw, midpoint, weight, wall))

    pairs = []
    for first_index in range(len(candidates)):
        first_yaw, first_midpoint, first_weight, first_wall = \
            candidates[first_index]
        for second_index in range(first_index + 1, len(candidates)):
            second_yaw, second_midpoint, second_weight, second_wall = \
                candidates[second_index]
            if _undirected_error(first_yaw, second_yaw) > parallel_tolerance:
                continue
            cosine = (
                first_weight * math.cos(2.0 * first_yaw)
                + second_weight * math.cos(2.0 * second_yaw)
            )
            sine = (
                first_weight * math.sin(2.0 * first_yaw)
                + second_weight * math.sin(2.0 * second_yaw)
            )
            if math.hypot(cosine, sine) < 1.0e-6:
                continue
            yaw = _axis_near_reference(0.5 * math.atan2(sine, cosine),
                                       reference_yaw)
            correction = normalize_angle(yaw - reference_yaw)
            if abs(correction) > maximum_correction:
                continue
            normal_x = -math.sin(yaw)
            normal_y = math.cos(yaw)
            first_lateral = (
                (first_midpoint[0] - robot_xy[0]) * normal_x
                + (first_midpoint[1] - robot_xy[1]) * normal_y
            )
            second_lateral = (
                (second_midpoint[0] - robot_xy[0]) * normal_x
                + (second_midpoint[1] - robot_xy[1]) * normal_y
            )
            if first_lateral * second_lateral >= -0.01:
                continue
            width = abs(first_lateral - second_lateral)
            if not minimum_width <= width <= maximum_width:
                continue
            score = (
                first_weight + second_weight
                - 0.50 * abs(width - 2.2)
                - 0.25 * abs(correction)
            )
            if first_lateral > second_lateral:
                left_wall, right_wall = first_wall, second_wall
            else:
                left_wall, right_wall = second_wall, first_wall
            pairs.append(CorridorAxisEstimate(
                yaw=yaw,
                width=width,
                correction=correction,
                left_id=int(getattr(left_wall, "detection_id", -1)),
                right_id=int(getattr(right_wall, "detection_id", -1)),
                score=score,
            ))
    if not pairs:
        return None
    return max(pairs, key=lambda item: item.score)
