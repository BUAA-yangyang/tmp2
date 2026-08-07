#!/usr/bin/env python3
"""Planar SE(2) bookkeeping that carries one frame across a floor change.

The five functions below are transcribed from yh's return-development tree
(simenv_elevator_pawn, src/a1/mission_manager/scripts/home_return.py). That
tree is read-only for us, so they are copied rather than imported; only the
SE(2) accumulation is taken, none of the occupancy-grid or active_map parts.

Naming: ``first_from_second`` maps coordinates expressed in ``second`` into
``first``. A pose is ``(x, y, yaw)``.

Why this exists. The competition result file wants Gazebo ``world``
coordinates, and the only legal bridge to that frame is team_scene_info.json's
robot_start paired with the map pose the robot had while standing on it. That
pairing is valid for exactly one localization generation, because FAST-LIO
re-anchors at every floor change. Carrying it forward needs one physical pose
expressed on both sides of the restart -- which is what the elevator transfer
provides, and what settle_body_before_elevator_call() makes trustworthy.
"""

import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(first_from_middle, middle_from_second):
    ax, ay, ayaw = first_from_middle
    bx, by, byaw = middle_from_second
    cosine = math.cos(ayaw)
    sine = math.sin(ayaw)
    return (
        ax + cosine * bx - sine * by,
        ay + sine * bx + cosine * by,
        normalize_angle(ayaw + byaw),
    )


def inverse(first_from_second):
    x, y, yaw = first_from_second
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        -cosine * x - sine * y,
        sine * x - cosine * y,
        normalize_angle(-yaw),
    )


def transform_pose(first_from_second, pose_in_second):
    tx, ty, yaw = first_from_second
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    x, y = pose_in_second[0], pose_in_second[1]
    return (
        tx + cosine * x - sine * y,
        ty + sine * x + cosine * y,
        normalize_angle(yaw + pose_in_second[2]),
    )


def propagate_home_transform(home_from_source, source_base, target_base):
    """Carry the home frame over a pose-preserving elevator transfer.

    ``source_base`` and ``target_base`` are the SAME physical base pose,
    expressed in the outgoing and incoming localization generations. mf61's
    referee truth put the position part of that assumption at 0.003 / 0.001 m
    across two transfers, so it holds to millimetres. The heading part only
    holds once the body has actually stopped -- see A13 in the issue ledger.
    """
    source_from_target = compose(source_base, inverse(target_base))
    return compose(home_from_source, source_from_target)
