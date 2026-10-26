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


def _clustered_cells(rows, cols, min_points, ray_ids=None):
    """Return grid cells supported by at least ``min_points`` distinct rays."""
    if len(rows) == 0:
        return set()
    rows = np.asarray(rows, dtype=np.int64)
    cols = np.asarray(cols, dtype=np.int64)
    min_row, min_col = int(rows.min()), int(cols.min())
    width = int(cols.max()) - min_col + 1
    # Packing the two grid indices into one integer is substantially faster on
    # PT's older NumPy than np.unique(..., axis=0), while decoding to the exact
    # same signed row/column pairs.  The projected map spans only a few dozen
    # cells, so the int64 key has ample headroom.
    cell_keys = (rows - min_row) * width + (cols - min_col)
    if ray_ids is not None:
        # Ray marching can place several samples from one ray in the same cell.
        # Count that as one observation so a single pixel cannot clear a cell.
        ray_ids = np.asarray(ray_ids, dtype=np.int64)
        ray_stride = int(ray_ids.max()) + 1
        ray_cell_keys = cell_keys * ray_stride + ray_ids
        cell_keys = np.unique(ray_cell_keys) // ray_stride
    unique, counts = np.unique(cell_keys, return_counts=True)
    accepted = unique[counts >= int(min_points)]
    return {(int(key // width) + min_row, int(key % width) + min_col)
            for key in accepted}


def project_depth_cells(depth, k, transform, cell=0.10, decimation=4,
                        range_min=0.15, range_max=2.0,
                        height_min=0.025, height_max=0.50,
                        min_points=3, protected_range=0.60,
                        free_margin=0.05):
    """Return ``(occupied_cells, observed_free_cells)`` in the world grid.

    ``transform`` is ``map_T_depth_optical`` as ``(translation, quaternion)``.
    The persistent room map remains LiDAR-only; these cells form a short-lived
    planner overlay so low geometry outside the laser plane can block routes.

    Free evidence comes only from valid rays before their measured surface.
    Cells close to the transformed camera origin are excluded because geometry
    below or beside the registered-depth view can remain hidden there.
    """
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim != 2 or depth.size == 0:
        raise ValueError("Depth must be a non-empty 2-D image")
    if len(k) < 9:
        raise ValueError("Camera intrinsics must contain nine values")
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    values = (fx, fy, cx, cy, cell, range_min, range_max,
              height_min, height_max, protected_range, free_margin)
    if (not all(math.isfinite(value) for value in values) or fx <= 0 or fy <= 0 or
            cell <= 0 or not 1 <= int(decimation) <= 32 or
            not 0 < range_min < range_max or not 0 <= height_min < height_max or
            not 0 <= protected_range <= range_max or
            not 0 < free_margin < range_max or
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
        return [], []
    valid_u = uu[valid].astype(np.float64)
    valid_v = vv[valid].astype(np.float64)
    z = sampled[valid].astype(np.float64)
    points = np.vstack(((valid_u - cx) * z / fx,
                        (valid_v - cy) * z / fy,
                        z))
    translation, quaternion = transform
    translation = np.asarray(translation, dtype=np.float64)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("Invalid transform translation")
    rotation = quaternion_matrix(quaternion)
    world = rotation.dot(points) + translation[:, None]
    obstacle = ((world[2] >= height_min) & (world[2] <= height_max))
    obstacle_cols = np.floor(world[0, obstacle] / cell).astype(np.int64)
    obstacle_rows = np.floor(world[1, obstacle] / cell).astype(np.int64)
    occupied_evidence = _clustered_cells(obstacle_rows, obstacle_cols, 1)
    occupied = _clustered_cells(obstacle_rows, obstacle_cols, min_points)

    # March every valid optical ray only as far as the measured surface minus
    # a noise margin. Sampling at half a planner cell avoids holes while the
    # distinct-ray count prevents oversampling one pixel into false confidence.
    # Occupied projection keeps the configured decimation. Free-space rays use
    # every second sample in each image dimension (4x fewer rays) because they
    # require repeated frames and clustered support before deleting anything.
    # A 10 cm planner cell still spans several such rays at the 2 m limit.
    sample_rows = np.rint(valid_v / step).astype(np.int64)
    sample_cols = np.rint(valid_u / step).astype(np.int64)
    free_ray_mask = ((sample_rows % 2) == 0) & ((sample_cols % 2) == 0)
    free_u = valid_u[free_ray_mask]
    free_v = valid_v[free_ray_mask]
    free_z = z[free_ray_mask]
    sample_step = cell * 0.5
    ray_z = np.arange(sample_step, range_max, sample_step, dtype=np.float64)
    before_surface = ray_z[None, :] <= (free_z[:, None] - free_margin)
    if not before_surface.any():
        return sorted(occupied), []
    ray_index, distance_index = np.nonzero(before_surface)
    distances = ray_z[distance_index]
    ray_points = np.vstack((
        (free_u[ray_index] - cx) * distances / fx,
        (free_v[ray_index] - cy) * distances / fy,
        distances))
    free_world = rotation.dot(ray_points) + translation[:, None]
    in_height_band = ((free_world[2] >= height_min) &
                      (free_world[2] <= height_max))
    free_cols = np.floor(free_world[0] / cell).astype(np.int64)
    free_rows = np.floor(free_world[1] / cell).astype(np.int64)
    center_x = (free_cols.astype(np.float64) + 0.5) * cell
    center_y = (free_rows.astype(np.float64) + 0.5) * cell
    outside_protected = (np.hypot(center_x - translation[0],
                                  center_y - translation[1]) >= protected_range)
    admissible = in_height_band & outside_protected
    observed_free = _clustered_cells(
        free_rows[admissible], free_cols[admissible], min_points,
        ray_ids=ray_index[admissible])
    # Any current low-surface evidence vetoes clearing, even if it does not
    # have enough neighboring points to refresh/add an occupied cell.
    observed_free.difference_update(occupied_evidence)
    return sorted(occupied), sorted(observed_free)


def project_obstacle_cells(depth, k, transform, cell=0.10, decimation=4,
                           range_min=0.15, range_max=2.0,
                           height_min=0.025, height_max=0.50,
                           min_points=3):
    """Compatibility wrapper returning only clustered occupied cells."""
    occupied, _ = project_depth_cells(
        depth, k, transform, cell=cell, decimation=decimation,
        range_min=range_min, range_max=range_max,
        height_min=height_min, height_max=height_max,
        min_points=min_points)
    return occupied
