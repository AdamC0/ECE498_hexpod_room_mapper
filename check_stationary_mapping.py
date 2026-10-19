"""Read-only 15-second robot-side check of scan/pose/map stability."""
import json
import math
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('check_stationary_mapping', anonymous=True)
    lock = threading.Lock()
    poses, maps, stamps = [], [], []

    def pose(msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with lock:
            poses.append((p.x, p.y, yaw, msg.header.frame_id))

    def grid(msg):
        with lock:
            maps.append((np.asarray(msg.data, dtype=np.int16), msg.header.frame_id,
                         msg.info.width, msg.info.height, msg.info.resolution))

    def scan(msg):
        with lock:
            stamps.append(msg.header.stamp.to_sec())

    subscriptions = [rospy.Subscriber('/slam_out_pose', PoseStamped, pose),
                     rospy.Subscriber('/map', OccupancyGrid, grid),
                     rospy.Subscriber('/scan_stable', LaserScan, scan)]
    time.sleep(15)
    for sub in subscriptions:
        sub.unregister()
    with lock:
        result = dict(poses=len(poses), maps=len(maps), scans=len(stamps))
        if poses:
            x, y, yaw, _ = poses[0]
            result['max_translation_from_first_m'] = max(math.hypot(p[0]-x, p[1]-y) for p in poses)
            result['max_heading_from_first_deg'] = math.degrees(max(
                abs(math.atan2(math.sin(p[2]-yaw), math.cos(p[2]-yaw))) for p in poses))
            result['pose_frames'] = sorted(set(p[3] for p in poses))
        if len(stamps) > 1:
            result['scan_hz'] = (len(stamps)-1)/(stamps[-1]-stamps[0])
        if len(maps) > 1:
            result['fixed_map_metadata'] = all(m[1:] == maps[0][1:] for m in maps)
            if result['fixed_map_metadata']:
                result['changed_cells_first_to_last'] = int(np.count_nonzero(maps[0][0] != maps[-1][0]))
        print(json.dumps(result, indent=2))
        if not poses or len(maps) < 2 or len(stamps) < 5:
            raise SystemExit('Insufficient live scan/pose/map data')


if __name__ == '__main__':
    main()
