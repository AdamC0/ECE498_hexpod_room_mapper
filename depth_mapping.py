#!/usr/bin/env python3
"""Project registered depth into a small-obstacle layer for the 2-D planner."""

import math

import numpy as np


def quaternion_matrix(quaternion):
    """Return a 3x3 rotation matrix for an xyzw quaternion."""
    q = np.asarray(quaternion, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("Invalid quaternion")
    norm = float(np.dot(q, q))
    if norm < 1e-12:
        raise ValueError("Zero-length quaternion")
    q *= math.sqrt(2.0 / norm)
    outer = np.outer(q, q)
    return np.array([
        [1.0 - outer[1, 1] - outer[2, 2],
         outer[0, 1] - outer[2, 3],
         outer[0, 2] + outer[1, 3]],
        [outer[0, 1] + outer[2, 3],
         1.0 - outer[0, 0] - outer[2, 2],
         outer[1, 2] - outer[0, 3]],
        [outer[0, 2] - outer[1, 3],
         outer[1, 2] + outer[0, 3],
         1.0 - outer[0, 0] - outer[1, 1]],
    ], dtype=np.float64)


def project_obstacle_cells(depth, k, transform, cell=0.10, decimation=4,
                           range_min=0.15, range_max=2.0,
                           height_min=0.025, height_max=0.50,
                           min_points=3):
    """Return world-grid cells containing clustered above-floor depth points.

    ``transform`` is ``map_T_depth_optical`` as ``(translation, quaternion)``.
    The persistent room map remains LiDAR-only; these cells form a short-lived
    planner overlay so low geometry outside the laser plane can block routes.
    """
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim != 2 or depth.size == 0:
        raise ValueError("Depth must be a non-empty 2-D image")
    if len(k) < 9:
        raise ValueError("Camera intrinsics must contain nine values")
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    values = (fx, fy, cx, cy, cell, range_min, range_max,
              height_min, height_max)
    if (not all(math.isfinite(value) for value in values) or fx <= 0 or fy <= 0 or
            cell <= 0 or not 1 <= int(decimation) <= 32 or
            not 0 < range_min < range_max or not 0 <= height_min < height_max or
            int(min_points) < 1):
        raise ValueError("Invalid depth obstacle projection parameters")

    step = int(decimation)
    sampled = depth[::step, ::step]
    vv, uu = np.indices(sampled.shape, dtype=np.float32)
    uu *= step
    vv *= step
    valid = (np.isfinite(sampled) & (sampled >= range_min) &
             (sampled <= range_max))
    if not valid.any():
        return []
    z = sampled[valid].astype(np.float64)
    points = np.vstack(((uu[valid] - cx) * z / fx,
                        (vv[valid] - cy) * z / fy,
                        z))
    translation, quaternion = transform
    translation = np.asarray(translation, dtype=np.float64)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("Invalid transform translation")
    world = quaternion_matrix(quaternion).dot(points) + translation[:, None]
    obstacle = ((world[2] >= height_min) & (world[2] <= height_max))
    if not obstacle.any():
        return []

    cols = np.floor(world[0, obstacle] / cell).astype(np.int64)
    rows = np.floor(world[1, obstacle] / cell).astype(np.int64)
    bins = np.column_stack((rows, cols))
    unique, counts = np.unique(bins, axis=0, return_counts=True)
    return [tuple(int(value) for value in item)
            for item, count in zip(unique, counts) if count >= int(min_points)]
