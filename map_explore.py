#!/usr/bin/env python3
"""Small, ROS-free frontier planner used by room_mapper.py.

The SLAM node owns the occupancy map.  This module only converts that map to a
coarser planning grid, inflates obstacles for the hexapod's leg radius, and
selects the nearest reachable free cell beside unknown space (a frontier).
"""

import math
import time
from collections import deque

import cv2
import numpy as np


UNKNOWN, FREE, OCCUPIED = 0, 1, 2


class ExploreMap:
    def __init__(self, size_m=25.6, cell=0.10, robot_radius=0.30, margin=0.13,
                 occupied_threshold=75, frontier_goal_distance=1.50,
                 frontier_max_distance=3.0):
        self.size_m = float(size_m)
        self.cell = float(cell)
        self.n = int(round(self.size_m / self.cell))
        self.half = self.size_m / 2.0
        self.clearance = float(robot_radius) + float(margin)
        self.occupied_threshold = int(occupied_threshold)
        self.frontier_goal_distance = float(frontier_goal_distance)
        self.frontier_max_distance = float(frontier_max_distance)
        if not 51 <= self.occupied_threshold <= 100:
            raise ValueError('occupied_threshold must be between 51 and 100')
        if not 0.5 <= self.frontier_goal_distance <= 4.0:
            raise ValueError('frontier_goal_distance must be between 0.5 and 4.0 metres')
        if not self.frontier_goal_distance <= self.frontier_max_distance <= 8.0:
            raise ValueError('frontier_max_distance must be at least the goal distance')
        self.state = np.full((self.n, self.n), UNKNOWN, np.uint8)
        self.slam_state = self.state.copy()
        self.depth_obstacles = {}
        self.depth_obstacle_clear_counts = {}
        self.depth_obstacle_hit_counts = {}
        self.persistent_depth_obstacles = set()
        self.suppressed_slam_obstacles = set()
        self.joint_clear_counts = {}
        self.depth_obstacle_ttl = 30.0
        self._depth_overlay_active = False
        self.have_map = False
        self.last_plan_kind = None

    def reset(self):
        """Forget the planner's copy while Hector starts a fresh map."""
        self.state.fill(UNKNOWN)
        self.slam_state.fill(UNKNOWN)
        self.depth_obstacles.clear()
        self.depth_obstacle_clear_counts.clear()
        self.depth_obstacle_hit_counts.clear()
        self.persistent_depth_obstacles.clear()
        self.suppressed_slam_obstacles.clear()
        self.joint_clear_counts.clear()
        self._depth_overlay_active = False
        self.have_map = False
        self.last_plan_kind = None

    def to_cell(self, x, y):
        return int((y + self.half) / self.cell), int((x + self.half) / self.cell)

    def to_world(self, r, c):
        return (c + 0.5) * self.cell - self.half, (r + 0.5) * self.cell - self.half

    def inside(self, r, c):
        return 0 <= r < self.n and 0 <= c < self.n

    def update_from_slam(self, data, width, height, resolution, origin_x, origin_y):
        m = np.asarray(data, dtype=np.int16).reshape(int(height), int(width))
        k = max(1, int(round(self.cell / float(resolution))))
        need = self.n * k
        off_c = int(round((-self.half - origin_x) / resolution))
        off_r = int(round((-self.half - origin_y) / resolution))
        big = np.full((need, need), -1, np.int16)
        r0, c0 = max(0, off_r), max(0, off_c)
        r1, c1 = min(m.shape[0], off_r + need), min(m.shape[1], off_c + need)
        if r1 > r0 and c1 > c0:
            big[r0 - off_r:r1 - off_r, c0 - off_c:c1 - off_c] = m[r0:r1, c0:c1]
        blocks = big.reshape(self.n, k, self.n, k).swapaxes(1, 2).reshape(self.n, self.n, k * k)
        # Hector now starts a hit at 65% confidence. Requiring 75 means a
        # consistent wall must be observed again before the planner blocks it.
        occ = (blocks >= self.occupied_threshold).any(axis=2)
        free = ((blocks >= 0) & (blocks < 25)).sum(axis=2) >= (k * k + 1) // 2
        state = np.full((self.n, self.n), UNKNOWN, np.uint8)
        state[free] = FREE
        state[occ] = OCCUPIED
        self.slam_state = state
        self.state = state.copy()
        self._apply_depth_obstacles(rebase=False)
        self.have_map = True

    def update_depth_obstacles(self, world_cells, now=None, ttl=30.0,
                               observed_free_cells=None, clear_confirmations=2):
        """Overlay temporary depth obstacles without changing the LiDAR map.

        Cells are expressed in the same world-aligned grid as ``self.cell``.
        Expiration prevents moved objects and stale camera data from becoming
        permanent walls. ``observed_free_cells`` contains only cells positively
        traversed by valid depth rays. A live overlay cell is removed after the
        configured number of consecutive clear observations.
        """
        now = time.time() if now is None else float(now)
        ttl = float(ttl)
        clear_confirmations = int(clear_confirmations)
        if not math.isfinite(now) or not math.isfinite(ttl) or ttl <= 0:
            raise ValueError('Depth obstacle time and TTL must be valid')
        if not 1 <= clear_confirmations <= 10:
            raise ValueError('Depth clear confirmations must be between 1 and 10')
        self.depth_obstacle_ttl = ttl
        occupied = set()
        for world_r, world_c in world_cells:
            r = int(world_r + self.half / self.cell)
            c = int(world_c + self.half / self.cell)
            if self.inside(r, c):
                occupied.add((r, c))
                self.depth_obstacles[(r, c)] = now
                self.depth_obstacle_clear_counts.pop((r, c), None)

        observed_free = set()
        for world_r, world_c in observed_free_cells or ():
            r = int(world_r + self.half / self.cell)
            c = int(world_c + self.half / self.cell)
            if self.inside(r, c) and (r, c) not in occupied:
                observed_free.add((r, c))

        # A missing, invalid, occluded, or out-of-view update cannot extend a
        # clear streak. Only consecutive positive observations may delete.
        for cell in list(self.depth_obstacle_clear_counts):
            if cell not in observed_free or cell not in self.depth_obstacles:
                del self.depth_obstacle_clear_counts[cell]
        cleared = 0
        for cell in observed_free.intersection(self.depth_obstacles):
            count = self.depth_obstacle_clear_counts.get(cell, 0) + 1
            if count >= clear_confirmations:
                del self.depth_obstacles[cell]
                self.depth_obstacle_clear_counts.pop(cell, None)
                cleared += 1
            else:
                self.depth_obstacle_clear_counts[cell] = count
        self._apply_depth_obstacles(now)
        return cleared

    def update_fused_observations(self, depth_occupied_cells,
                                  depth_free_cells, lidar_occupied_cells,
                                  lidar_free_cells, now=None, ttl=30.0,
                                  temporary_clear_confirmations=2,
                                  promotion_confirmations=6,
                                  joint_clear_confirmations=4):
        """Fuse depth occupancy with live LiDAR without altering ``slam_state``.

        Repeated depth hits promote a temporary obstacle to session-persistent
        occupancy. A persistent depth cell or raw Hector-occupied cell becomes
        free in the fused planner only after both sensors positively ray-trace
        the cell as free for several consecutive observations. Any current hit
        from either sensor vetoes clearing and restores suppressed occupancy.
        """
        now = time.time() if now is None else float(now)
        promotion_confirmations = int(promotion_confirmations)
        joint_clear_confirmations = int(joint_clear_confirmations)
        if not 2 <= promotion_confirmations <= 100:
            raise ValueError('Depth promotion confirmations must be between 2 and 100')
        if not 2 <= joint_clear_confirmations <= 100:
            raise ValueError('Joint clear confirmations must be between 2 and 100')

        def planner_cells(world_cells):
            result = set()
            for world_r, world_c in world_cells or ():
                r = int(world_r + self.half / self.cell)
                c = int(world_c + self.half / self.cell)
                if self.inside(r, c):
                    result.add((r, c))
            return result

        depth_occupied = planner_cells(depth_occupied_cells)
        depth_free = planner_cells(depth_free_cells) - depth_occupied
        lidar_occupied = planner_cells(lidar_occupied_cells)
        lidar_free = planner_cells(lidar_free_cells) - lidar_occupied
        occupied = depth_occupied | lidar_occupied

        # Retain the established short-lived overlay and its depth-only clear
        # policy until a cell earns promotion.
        cleared_temporary = self.update_depth_obstacles(
            depth_occupied_cells, now, ttl,
            observed_free_cells=depth_free_cells,
            clear_confirmations=temporary_clear_confirmations)

        for cell in list(self.depth_obstacle_hit_counts):
            if cell not in depth_occupied:
                del self.depth_obstacle_hit_counts[cell]
        promoted = 0
        for cell in depth_occupied:
            count = self.depth_obstacle_hit_counts.get(cell, 0) + 1
            self.depth_obstacle_hit_counts[cell] = min(count, promotion_confirmations)
            if count >= promotion_confirmations and cell not in self.persistent_depth_obstacles:
                self.persistent_depth_obstacles.add(cell)
                promoted += 1

        # update_depth_obstacles() sees every current depth hit before the
        # promotion tier does. Do not let later frames re-add an already
        # persistent cell to the temporary mask, which would paint magenta
        # over its black persistent occupancy in the web map.
        for cell in self.persistent_depth_obstacles:
            self.depth_obstacles.pop(cell, None)
            self.depth_obstacle_clear_counts.pop(cell, None)

        # A current occupied endpoint protects its own cell and immediate
        # neighbors from grid quantization disagreement between the sensors.
        occupied_guard = set()
        for r, c in occupied:
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if self.inside(r + dr, c + dc):
                        occupied_guard.add((r + dr, c + dc))
        joint_free = (depth_free & lidar_free) - occupied_guard

        # Reappearance in either sensor immediately cancels a prior downstream
        # suppression; raw Hector occupancy itself remains untouched.
        self.suppressed_slam_obstacles.difference_update(occupied)
        for cell in occupied:
            self.joint_clear_counts.pop(cell, None)

        candidates = (self.persistent_depth_obstacles |
                      {cell for cell in joint_free
                       if self.slam_state[cell] == OCCUPIED})
        for cell in list(self.joint_clear_counts):
            if cell not in joint_free or cell not in candidates:
                del self.joint_clear_counts[cell]

        cleared_persistent = 0
        suppressed_slam = 0
        for cell in joint_free & candidates:
            count = self.joint_clear_counts.get(cell, 0) + 1
            if count < joint_clear_confirmations:
                self.joint_clear_counts[cell] = count
                continue
            self.joint_clear_counts.pop(cell, None)
            if cell in self.persistent_depth_obstacles:
                self.persistent_depth_obstacles.remove(cell)
                self.depth_obstacle_hit_counts.pop(cell, None)
                cleared_persistent += 1
            if (self.slam_state[cell] == OCCUPIED and
                    cell not in self.suppressed_slam_obstacles):
                self.suppressed_slam_obstacles.add(cell)
                suppressed_slam += 1

        # Suppression is needed only while Hector still reports occupancy.
        self.suppressed_slam_obstacles = {
            cell for cell in self.suppressed_slam_obstacles
            if self.slam_state[cell] == OCCUPIED}
        self._apply_depth_obstacles(now)
        return {
            'promoted': promoted,
            'cleared_temporary': cleared_temporary,
            'cleared_persistent': cleared_persistent,
            'suppressed_slam': suppressed_slam,
        }

    def _apply_depth_obstacles(self, now=None, rebase=True):
        if (not self._depth_overlay_active and not self.depth_obstacles and
                not self.persistent_depth_obstacles and
                not self.suppressed_slam_obstacles):
            return
        now = time.time() if now is None else float(now)
        cutoff = now - self.depth_obstacle_ttl
        self.depth_obstacles = {cell: stamp for cell, stamp in self.depth_obstacles.items()
                                if stamp >= cutoff}
        self.depth_obstacle_clear_counts = {
            cell: count for cell, count in self.depth_obstacle_clear_counts.items()
            if cell in self.depth_obstacles}
        if rebase:
            self.state = self.slam_state.copy()
        for r, c in self.suppressed_slam_obstacles:
            self.state[r, c] = FREE
        for r, c in self.persistent_depth_obstacles:
            self.state[r, c] = OCCUPIED
        for r, c in self.depth_obstacles:
            self.state[r, c] = OCCUPIED
        self._depth_overlay_active = bool(
            self.depth_obstacles or self.persistent_depth_obstacles or
            self.suppressed_slam_obstacles)

    def depth_obstacle_count(self, now=None):
        self._apply_depth_obstacles(now)
        return len(self.depth_obstacles)

    def depth_obstacle_mask(self, now=None):
        """Return current depth-sourced cells separately from LiDAR state."""
        self._apply_depth_obstacles(now)
        mask = np.zeros_like(self.state, dtype=bool)
        if self.depth_obstacles:
            rows, cols = zip(*self.depth_obstacles)
            mask[rows, cols] = True
        return mask

    def persistent_obstacle_count(self):
        self._apply_depth_obstacles()
        return len(self.persistent_depth_obstacles)

    def suppressed_slam_count(self):
        self._apply_depth_obstacles()
        return len(self.suppressed_slam_obstacles)

    def obstacle_distance(self):
        self._apply_depth_obstacles()
        return cv2.distanceTransform((self.state != OCCUPIED).astype(np.uint8),
                                     cv2.DIST_L2, 5) * self.cell

    def walkable(self, distance=None):
        distance = self.obstacle_distance() if distance is None else distance
        ok = (self.state == FREE) & (distance > self.clearance)
        return ok

    def path_prefix_clear(self, pose, path, horizon=0.80):
        """Return whether the immediate planned path remains safe in this map.

        Paths are planned from a snapshot, but registered depth can add a low
        obstacle while continuous translation is already active. Sample the
        next portion at half-cell spacing and reject occupancy, unknown space,
        or decreasing clearance below what is available at the robot now.
        """
        if not path:
            return True
        horizon = float(horizon)
        if not math.isfinite(horizon) or horizon <= 0:
            raise ValueError('Path validation horizon must be positive')
        start = (float(pose[0]), float(pose[1]))
        sr, sc = self.to_cell(*start)
        if not self.inside(sr, sc):
            return False
        distance = self.obstacle_distance()
        baseline = min(float(distance[sr, sc]), self.clearance)
        previous = start
        remaining = horizon
        step = self.cell * 0.5
        for target in path:
            tx, ty = float(target[0]), float(target[1])
            length = math.hypot(tx - previous[0], ty - previous[1])
            if length <= 1e-9:
                previous = (tx, ty)
                continue
            checked = min(length, remaining)
            samples = max(1, int(math.ceil(checked / step)))
            for index in range(1, samples + 1):
                travel = min(checked, index * step)
                ratio = travel / length
                x = previous[0] + (tx - previous[0]) * ratio
                y = previous[1] + (ty - previous[1]) * ratio
                r, c = self.to_cell(x, y)
                if (not self.inside(r, c) or self.state[r, c] != FREE or
                        float(distance[r, c]) + 1e-6 < baseline):
                    return False
            remaining -= checked
            if remaining <= 1e-9:
                break
            previous = (tx, ty)
        return True

    def plan_to_point(self, pose, target, standoff=0.0, max_cells=60000):
        """Plan through connected known-free space toward an arbitrary point.

        The requested point does not itself need to be free.  This is
        intentional for map clicks and detected objects: the selected goal is
        the reachable cell nearest the point, or nearest a safe stand-off ring
        around it.  Unknown cells and inflated obstacles are never traversed.
        Returns ``(path, actual_goal)`` or ``(None, None)``.
        """
        self.last_plan_kind = None
        if not self.have_map or len(target) != 2:
            return None, None
        tx, ty = float(target[0]), float(target[1])
        if not all(math.isfinite(v) for v in (tx, ty, standoff)) or standoff < 0:
            return None, None
        sr, sc = self.to_cell(pose[0], pose[1])
        if not self.inside(sr, sc):
            return None, None
        dist = self.obstacle_distance()
        ok = self.walkable(dist)
        # The robot can start close to a mapped wall, but each subsequent cell
        # must maintain at least the clearance available at the start.
        k = int(math.ceil(self.clearance / self.cell))
        win = (slice(max(0, sr-k), min(self.n, sr+k+1)),
               slice(max(0, sc-k), min(self.n, sc+k+1)))
        ok[win] |= ((dist[win] >= min(dist[sr, sc], self.clearance)) &
                    (self.state[win] == FREE))

        parent = {(sr, sc): None}
        steps = {(sr, sc): 0}
        q = deque([(sr, sc)])
        while q and len(parent) < max_cells:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nr, nc = r + dr, c + dc
                if dr and dc and (not self.inside(r + dr, c) or
                                  not self.inside(r, c + dc) or
                                  not ok[r + dr, c] or not ok[r, c + dc]):
                    continue
                if self.inside(nr, nc) and ok[nr, nc] and (nr, nc) not in parent:
                    parent[(nr, nc)] = (r, c)
                    steps[(nr, nc)] = steps[(r, c)] + 1
                    q.append((nr, nc))
        if not parent:
            return None, None

        def score(cell):
            x, y = self.to_world(*cell)
            radial_error = abs(math.hypot(x - tx, y - ty) - standoff)
            # Break same-distance ties with the shorter route.
            return radial_error, steps[cell]

        found = min(parent, key=score)
        cells, cur = [], found
        while cur is not None:
            cells.append(cur)
            cur = parent[cur]
        cells.reverse()
        # Keep waypoints at most 20 cm apart. Wider thinning combined with a
        # generous controller tolerance can cut an inflated obstacle corner.
        path = [self.to_world(r, c) for r, c in cells[::2]]
        goal = self.to_world(*found)
        if not path or path[-1] != goal:
            path.append(goal)
        self.last_plan_kind = "object approach" if standoff else "selected goal"
        return path, goal

    def plan_frontier(self, pose, max_cells=60000):
        """Return a route to a useful reachable frontier, or ``(None, None)``.

        Selecting the first frontier only half a metre away made the robot
        alternate between a few seconds of walking and a full stationary
        survey. Search the connected free region once, then prefer a frontier
        at least ``frontier_goal_distance`` away with high nearby unknown-space
        gain. A closer frontier remains eligible when the known region is
        still too small for a longer route.
        """
        self.last_plan_kind = None
        if not self.have_map:
            return None, None
        sr, sc = self.to_cell(pose[0], pose[1])
        if not self.inside(sr, sc):
            return None, None
        dist = self.obstacle_distance()
        ok = self.walkable(dist)
        # Allow the current cell as a start even if the robot is temporarily
        # close to an obstacle; paths still move away from it.
        k = int(math.ceil(self.clearance / self.cell))
        win = (slice(max(0, sr-k), min(self.n, sr+k+1)), slice(max(0, sc-k), min(self.n, sc+k+1)))
        ok[win] |= (dist[win] >= min(dist[sr, sc], self.clearance)) & (self.state[win] == FREE)
        unknown = self.state == UNKNOWN
        frontier = ok & (cv2.dilate(unknown.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0)
        # Ignore isolated map specks and prefer a useful patch of frontier.
        frontier &= cv2.boxFilter(frontier.astype(np.uint8), -1, (5, 5), normalize=False) >= 3

        parent = {(sr, sc): None}
        steps = {(sr, sc): 0}
        q = deque([(sr, sc)])
        while q and len(parent) < max_cells:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nr, nc = r + dr, c + dc
                if dr and dc and (not self.inside(r + dr, c) or
                                  not self.inside(r, c + dc) or
                                  not ok[r + dr, c] or not ok[r, c + dc]):
                    continue
                if self.inside(nr, nc) and ok[nr, nc] and (nr, nc) not in parent:
                    parent[(nr, nc)] = (r, c)
                    steps[(nr, nc)] = steps[(r, c)] + 1
                    q.append((nr, nc))
        candidates = [cell for cell in parent
                      if frontier[cell] and steps[cell] * self.cell >= 0.50]
        found = None
        if candidates:
            preferred = [cell for cell in candidates
                         if self.frontier_goal_distance <= steps[cell] * self.cell <=
                         self.frontier_max_distance]
            nearer = [cell for cell in candidates
                      if steps[cell] * self.cell < self.frontier_goal_distance]
            farther = [cell for cell in candidates
                       if steps[cell] * self.cell > self.frontier_max_distance]
            pool = preferred or nearer
            if not pool and farther:
                # When mapping has pushed every frontier farther away, take
                # the closest one rather than choosing an impractically long
                # information-rich route that will hit the route timeout.
                pool = [min(farther, key=lambda cell: steps[cell])]
            radius = max(3, int(round(1.0 / self.cell)))
            unknown_near = cv2.boxFilter(
                unknown.astype(np.float32), -1, (2*radius+1, 2*radius+1),
                normalize=False)

            def frontier_score(cell):
                r, c = cell
                gx, gy = self.to_world(r, c)
                bearing = math.atan2(gy - pose[1], gx - pose[0])
                heading = abs(math.atan2(math.sin(bearing - pose[2]),
                                         math.cos(bearing - pose[2])))
                # Information gain dominates; route length and initial turn
                # break ties so decisions remain useful and inexpensive.
                return (float(unknown_near[r, c]) - 0.20 * steps[cell],
                        -heading, -steps[cell])

            found = max(pool, key=frontier_score)
        if found is None:
            # A mapped wall can separate every reachable free cell from the
            # unknown boundary. Move to a different, confirmed-free viewpoint
            # instead of stopping forever; a scan there can refine geometry
            # and reveal openings that the first viewpoint could not resolve.
            radius = max(3, int(round(1.5 / self.cell)))
            unknown_near = cv2.boxFilter(unknown.astype(np.float32), -1,
                                         (2*radius+1, 2*radius+1), normalize=False)
            choices = []
            for r, c in parent:
                x, y = self.to_world(r, c)
                travel = math.hypot(x-pose[0], y-pose[1])
                if travel >= 0.50 and unknown_near[r, c] > 0:
                    preferred = travel >= self.frontier_goal_distance
                    choices.append((preferred, float(unknown_near[r, c]), -travel, r, c))
            if not choices:
                return None, None
            _, _, _, r, c = max(choices)
            found = (r, c)
            self.last_plan_kind = "survey viewpoint"
        else:
            self.last_plan_kind = "frontier"
        cells = []
        cur = found
        while cur is not None:
            cells.append(cur)
            cur = parent[cur]
        cells.reverse()
        path = [self.to_world(r, c) for r, c in cells[::2]]
        path.append(self.to_world(*found))
        return path, self.to_world(*found)
