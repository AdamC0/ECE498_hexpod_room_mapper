"""Print why the live map does or does not have a reachable survey goal."""
import json
import math
from collections import deque

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

from map_explore import ExploreMap, FREE, OCCUPIED, UNKNOWN


def main():
    rospy.init_node('diagnose_frontier', anonymous=True)
    grid_msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=5)
    pose_msg = rospy.wait_for_message('/slam_out_pose', PoseStamped, timeout=5)
    q = pose_msg.pose.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    pose = (pose_msg.pose.position.x, pose_msg.pose.position.y, yaw)
    grid = ExploreMap()
    info = grid_msg.info
    grid.update_from_slam(grid_msg.data, info.width, info.height, info.resolution,
                          info.origin.position.x, info.origin.position.y)
    sr, sc = grid.to_cell(*pose[:2])
    distance = grid.obstacle_distance()
    walkable = grid.walkable(distance)
    unknown = grid.state == UNKNOWN
    frontier = walkable & (cv2.dilate(unknown.astype(np.uint8),
                                      np.ones((3, 3), np.uint8)) > 0)
    frontier &= cv2.boxFilter(frontier.astype(np.uint8), -1, (5, 5),
                              normalize=False) >= 3
    reachable = np.zeros_like(walkable)
    if grid.inside(sr, sc):
        reachable[sr, sc] = True
        queue = deque([(sr, sc)])
        while queue:
            r, c = queue.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if grid.inside(nr, nc) and walkable[nr, nc] and not reachable[nr, nc]:
                    reachable[nr, nc] = True
                    queue.append((nr, nc))
    radius = max(3, int(round(1.5 / grid.cell)))
    unknown_near = cv2.boxFilter(unknown.astype(np.float32), -1,
                                 (2*radius+1, 2*radius+1), normalize=False)
    candidates = reachable.copy()
    yy, xx = np.indices(candidates.shape)
    candidates &= np.hypot((xx-sc)*grid.cell, (yy-sr)*grid.cell) >= .5
    best = None
    if candidates.any():
        score = np.where(candidates, unknown_near, -1)
        br, bc = np.unravel_index(np.argmax(score), score.shape)
        best = {'world': grid.to_world(br, bc), 'unknown_near': int(score[br, bc]),
                'distance_m': math.hypot((bc-sc)*grid.cell, (br-sr)*grid.cell)}
    print(json.dumps({
        'pose': pose, 'start_cell': [sr, sc],
        'start_state': int(grid.state[sr, sc]) if grid.inside(sr, sc) else None,
        'start_clearance_m': float(distance[sr, sc]) if grid.inside(sr, sc) else None,
        'free_cells': int(np.count_nonzero(grid.state == FREE)),
        'occupied_cells': int(np.count_nonzero(grid.state == OCCUPIED)),
        'walkable_cells': int(np.count_nonzero(walkable)),
        'reachable_cells': int(np.count_nonzero(reachable)),
        'frontier_cells': int(np.count_nonzero(frontier)),
        'reachable_frontiers': int(np.count_nonzero(frontier & reachable)),
        'fallback_viewpoint': best,
    }, indent=2))


if __name__ == '__main__':
    main()
