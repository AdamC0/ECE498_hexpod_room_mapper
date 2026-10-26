#!/usr/bin/env python3
"""Project live 2-D LiDAR rays into occupied and observed-free map cells."""

import math

import numpy as np

from depth_mapping import _clustered_cells, quaternion_matrix


def project_lidar_cells(ranges, angle_min, angle_increment, transform,
                        cell=0.10, range_min=0.15, range_max=2.0,
                        decimation=2, min_rays=2, free_margin=0.10,
                        yaw_offset=0.0):
    """Return ``(occupied_cells, observed_free_cells)`` in the world grid.

    Finite returns inside ``range_max`` provide occupied endpoints. Longer or
    positive-infinite returns still provide free evidence up to ``range_max``;
    NaN and negative-infinite samples provide no evidence. Free cells require
    multiple distinct rays so one grazing beam cannot erase map occupancy.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    values = (angle_min, angle_increment, cell, range_min, range_max,
              free_margin, yaw_offset)
    if (ranges.ndim != 1 or ranges.size == 0 or
            not all(math.isfinite(float(value)) for value in values) or
            angle_increment == 0 or cell <= 0 or
            not 0 < range_min < range_max or
            not 0 < free_margin < range_max or
            not 1 <= int(decimation) <= 32 or int(min_rays) < 1):
        raise ValueError("Invalid LiDAR projection parameters")

    translation, quaternion = transform
    translation = np.asarray(translation, dtype=np.float64)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("Invalid transform translation")
    rotation = quaternion_matrix(quaternion)

    indices = np.arange(0, ranges.size, int(decimation), dtype=np.int64)
    sampled = ranges[indices]
    usable = ((np.isfinite(sampled) & (sampled >= range_min)) |
              np.isposinf(sampled))
    if not usable.any():
        return [], []
    indices = indices[usable]
    sampled = sampled[usable]
    angles = angle_min + indices.astype(np.float64) * angle_increment + yaw_offset
    finite = np.isfinite(sampled)
    lengths = np.where(finite, np.minimum(sampled, range_max), range_max)

    # Only real returns inside the fusion range are occupied endpoints. A hit
    # exactly at the configured limit remains occupied rather than becoming a
    # synthetic clear-at-limit ray.
    hit = finite & (sampled <= range_max)
    hit_points = np.vstack((sampled[hit] * np.cos(angles[hit]),
                            sampled[hit] * np.sin(angles[hit]),
                            np.zeros(np.count_nonzero(hit), np.float64)))
    if hit_points.shape[1]:
        hit_world = rotation.dot(hit_points) + translation[:, None]
        hit_cols = np.floor(hit_world[0] / cell).astype(np.int64)
        hit_rows = np.floor(hit_world[1] / cell).astype(np.int64)
        occupied = _clustered_cells(hit_rows, hit_cols, 1)
    else:
        occupied = set()

    sample_step = cell * 0.5
    ray_distance = np.arange(sample_step, range_max + sample_step * .5,
                             sample_step, dtype=np.float64)
    clear_limit = np.where(hit, np.maximum(0.0, lengths - free_margin), lengths)
    before_surface = ray_distance[None, :] <= clear_limit[:, None]
    if not before_surface.any():
        return sorted(occupied), []
    ray_index, distance_index = np.nonzero(before_surface)
    distances = ray_distance[distance_index]
    ray_angles = angles[ray_index]
    points = np.vstack((distances * np.cos(ray_angles),
                        distances * np.sin(ray_angles),
                        np.zeros(distances.size, np.float64)))
    world = rotation.dot(points) + translation[:, None]
    cols = np.floor(world[0] / cell).astype(np.int64)
    rows = np.floor(world[1] / cell).astype(np.int64)
    observed_free = _clustered_cells(rows, cols, int(min_rays),
                                     ray_ids=ray_index)
    observed_free.difference_update(occupied)
    return sorted(occupied), sorted(observed_free)
