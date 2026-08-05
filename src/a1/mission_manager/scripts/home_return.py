#!/usr/bin/env python3
"""Planar frame and occupancy-grid helpers for the floor-zero return."""

import math

import numpy as np


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(first_from_middle, middle_from_second):
    """Compose two planar transforms represented as ``(x, y, yaw)``."""
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


def transform_xy(first_from_second, x, y):
    tx, ty, yaw = first_from_second
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        tx + cosine * x - sine * y,
        ty + sine * x + cosine * y,
    )


def transform_pose(first_from_second, pose_in_second):
    x, y = transform_xy(first_from_second, pose_in_second[0], pose_in_second[1])
    return x, y, normalize_angle(first_from_second[2] + pose_in_second[2])


def propagate_home_transform(home_from_source, source_base, target_base):
    """Carry the home frame over a pose-preserving elevator transfer.

    The passenger keeps its horizontal world pose and yaw while the estimator
    restarts. Therefore the same physical base pose, expressed immediately
    before and after the restart, relates the two generation-local odom frames.
    """
    source_from_target = compose(source_base, inverse(target_base))
    return compose(home_from_source, source_from_target)


def warp_occupancy_grid(
        source_data, width, height, resolution, origin_x, origin_y,
        source_from_target, unknown_value=-1, chunk_rows=64):
    """Resample an axis-aligned source grid into an axis-aligned target grid.

    Source and target use identical grid geometry. ``source_from_target`` maps
    target-frame coordinates into the remembered source frame. Nearest-cell
    sampling preserves the occupancy values and leaves out-of-bounds cells
    unknown.
    """
    if width <= 0 or height <= 0 or resolution <= 0.0 or chunk_rows <= 0:
        raise ValueError("grid geometry and chunk size must be positive")
    if not all(math.isfinite(value) for value in (
            resolution, origin_x, origin_y, *source_from_target)):
        raise ValueError("grid transform must be finite")

    source = np.asarray(source_data, dtype=np.int16)
    if source.size != width * height:
        raise ValueError("occupancy data size does not match grid geometry")
    source = source.reshape((height, width))
    target = np.full((height, width), int(unknown_value), dtype=np.int16)

    tx, ty, yaw = source_from_target
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    target_x = origin_x + (np.arange(width, dtype=np.float64) + 0.5) * resolution

    for first_row in range(0, height, chunk_rows):
        last_row = min(height, first_row + chunk_rows)
        target_y = origin_y + (
            np.arange(first_row, last_row, dtype=np.float64) + 0.5
        ) * resolution
        grid_x, grid_y = np.meshgrid(target_x, target_y)
        source_x = tx + cosine * grid_x - sine * grid_y
        source_y = ty + sine * grid_x + cosine * grid_y
        source_columns = np.floor((source_x - origin_x) / resolution).astype(np.int64)
        source_rows = np.floor((source_y - origin_y) / resolution).astype(np.int64)
        valid = (
            (source_columns >= 0) & (source_columns < width) &
            (source_rows >= 0) & (source_rows < height)
        )
        block = target[first_row:last_row]
        block[valid] = source[source_rows[valid], source_columns[valid]]

    return target.reshape(-1).astype(np.int8).tolist()


def nearest_known_free_anchor(
        flat_data, width, height, resolution, origin_x, origin_y,
        anchor_x, anchor_y, search_radius, free_threshold=20):
    """Return the nearest known-free cell centre around a remembered pose."""
    values = (resolution, origin_x, origin_y, anchor_x, anchor_y, search_radius)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("anchor search geometry must be finite")
    if width <= 0 or height <= 0 or resolution <= 0.0 or search_radius <= 0.0:
        raise ValueError("anchor search geometry must be positive")
    data = np.asarray(flat_data, dtype=np.int16)
    if data.size != width * height:
        raise ValueError("occupancy data size does not match grid geometry")
    grid = data.reshape((height, width))
    center_column = int(math.floor((anchor_x - origin_x) / resolution))
    center_row = int(math.floor((anchor_y - origin_y) / resolution))
    if not (0 <= center_column < width and 0 <= center_row < height):
        return None

    radius_cells = int(math.ceil(search_radius / resolution))
    radius_squared = search_radius * search_radius
    best = None
    for row in range(max(0, center_row - radius_cells),
                     min(height - 1, center_row + radius_cells) + 1):
        for column in range(max(0, center_column - radius_cells),
                            min(width - 1, center_column + radius_cells) + 1):
            if not 0 <= grid[row, column] <= free_threshold:
                continue
            x = origin_x + (column + 0.5) * resolution
            y = origin_y + (row + 0.5) * resolution
            distance_squared = (x - anchor_x) ** 2 + (y - anchor_y) ** 2
            if distance_squared > radius_squared + 1.0e-12:
                continue
            candidate = (distance_squared, row, column, x, y)
            if best is None or candidate[:3] < best[:3]:
                best = candidate
    return None if best is None else (best[3], best[4])
