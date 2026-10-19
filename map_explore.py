#!/usr/bin/env python3
"""Small, ROS-free frontier planner used by room_mapper.py.

The SLAM node owns the occupancy map.  This module only converts that map to a
coarser planning grid, inflates obstacles for the hexapod's leg radius, and
selects the nearest reachable free cell beside unknown space (a frontier).
"""

import math
from collections import deque

import cv2
import numpy as np


UNKNOWN, FREE, OCCUPIED = 0, 1, 2


class ExploreMap:
    def __init__(self, size_m=25.6, cell=0.10, robot_radius=0.30, margin=0.03,
                 occupied_threshold=75):
        self.size_m = float(size_m)
        self.cell = float(cell)
        self.n = int(round(self.size_m / self.cell))
        self.half = self.size_m / 2.0
        self.clearance = float(robot_radius) + float(margin)
        self.occupied_threshold = int(occupied_threshold)
        if not 51 <= self.occupied_threshold <= 100:
            raise ValueError('occupied_threshold must be between 51 and 100')
        self.state = np.full((self.n, self.n), UNKNOWN, np.uint8)
        self.have_map = False
        self.last_plan_kind = None

    def reset(self):
        """Forget the planner's copy while Hector starts a fresh map."""
        self.state.fill(UNKNOWN)
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
        self.state = state
        self.have_map = True

    def obstacle_distance(self):
        return cv2.distanceTransform((self.state != OCCUPIED).astype(np.uint8),
                                     cv2.DIST_L2, 5) * self.cell

    def walkable(self, distance=None):
        distance = self.obstacle_distance() if distance is None else distance
        ok = (self.state == FREE) & (distance > self.clearance)
        return ok

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
        """Return waypoints to the nearest reachable frontier, or (None, None)."""
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
        q = deque([(sr, sc)])
        found = None
        while q and len(parent) < max_cells:
            r, c = q.popleft()
            # Goals inside the controller's 0.30 m arrival tolerance cause an
            # endless plan/arrive loop without collecting a new viewpoint.
            gx, gy = self.to_world(r, c)
            if math.hypot(gx - pose[0], gy - pose[1]) >= 0.50 and frontier[r, c]:
                found = (r, c)
                break
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nr, nc = r + dr, c + dc
                if dr and dc and (not self.inside(r + dr, c) or
                                  not self.inside(r, c + dc) or
                                  not ok[r + dr, c] or not ok[r, c + dc]):
                    continue
                if self.inside(nr, nc) and ok[nr, nc] and (nr, nc) not in parent:
                    parent[(nr, nc)] = (r, c)
                    q.append((nr, nc))
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
                    choices.append((float(unknown_near[r, c]), -travel, r, c))
            if not choices:
                return None, None
            _, _, r, c = max(choices)
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

    def coverage(self):
        known = self.state != UNKNOWN
        return float(known.sum()) / max(1, known.size)
