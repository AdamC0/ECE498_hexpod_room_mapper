#!/usr/bin/env python3
"""Bounded live navigation monitor with conservative SLAM consistency checks.

This script can assess internal localization consistency, but it cannot see the
robot.  A run never becomes a physical-navigation pass without an observer
confirming actual translation and rotation.
"""
import argparse
import json
import math
import threading
import time


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def analyze(poses, transitions, coverage=0.0, settle_seconds=1.0,
            max_step_m=0.25, max_still_m=0.10, max_still_deg=5.0):
    """Analyze ``(time, x, y, yaw, moving)`` samples without importing ROS."""
    result = {
        'pose_samples': len(poses),
        'motion_bursts': sum(1 for _, moving in transitions if moving),
        'map_known_percent': round(100.0 * coverage, 2),
        'physical_confirmation_required': True,
    }
    failures = []
    if len(poses) < 10:
        failures.append('insufficient pose samples')
    if result['motion_bursts'] < 1:
        failures.append('no commanded motion burst observed')
    if len(poses) > 1:
        steps = [math.hypot(b[1] - a[1], b[2] - a[2])
                 for a, b in zip(poses, poses[1:])]
        yaws = [abs(angle_delta(b[3], a[3])) for a, b in zip(poses, poses[1:])]
        result.update(
            start_pose=[round(poses[0][1], 3), round(poses[0][2], 3),
                        round(math.degrees(poses[0][3]), 1)],
            end_pose=[round(poses[-1][1], 3), round(poses[-1][2], 3),
                      round(math.degrees(poses[-1][3]), 1)],
            net_translation_m=round(math.hypot(poses[-1][1] - poses[0][1],
                                                poses[-1][2] - poses[0][2]), 3),
            net_heading_deg=round(math.degrees(angle_delta(poses[-1][3],
                                                            poses[0][3])), 1),
            max_pose_step_m=round(max(steps), 4),
            accumulated_abs_heading_deg=round(math.degrees(sum(yaws)), 1),
        )
        if max(steps) > max_step_m:
            failures.append('pose step %.3f m exceeds %.3f m' %
                            (max(steps), max_step_m))

        # Ignore scan-matcher recovery immediately after a burst.  Within each
        # genuinely settled interval, compare every sample with its first pose.
        stop_times = [t for t, moving in transitions if not moving]
        max_still_translation = max_still_heading = 0.0
        anchor = None
        anchor_stop = None
        for sample in poses:
            t, x, y, yaw, moving = sample
            last_stop = max((s for s in stop_times if s <= t), default=None)
            settled = not moving and last_stop is not None and t >= last_stop + settle_seconds
            if not settled:
                anchor = anchor_stop = None
                continue
            if anchor is None or anchor_stop != last_stop:
                anchor, anchor_stop = sample, last_stop
                continue
            max_still_translation = max(max_still_translation,
                                        math.hypot(x - anchor[1], y - anchor[2]))
            max_still_heading = max(max_still_heading,
                                    abs(angle_delta(yaw, anchor[3])))
        result['max_settled_translation_m'] = round(max_still_translation, 4)
        result['max_settled_heading_deg'] = round(math.degrees(max_still_heading), 2)
        if max_still_translation > max_still_m:
            failures.append('settled drift %.3f m exceeds %.3f m' %
                            (max_still_translation, max_still_m))
        if math.degrees(max_still_heading) > max_still_deg:
            failures.append('settled heading drift %.1f deg exceeds %.1f deg' %
                            (math.degrees(max_still_heading), max_still_deg))

    result['slam_consistency_pass'] = not failures
    result['failures'] = failures
    result['verdict'] = ('SLAM internally consistent; awaiting physical confirmation'
                         if not failures else 'SLAM consistency failed')
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=45.0)
    args = parser.parse_args()
    if not 5 <= args.duration <= 300:
        parser.error('duration must be between 5 and 300 seconds')

    try:
        from urllib.request import Request, urlopen
    except ImportError:  # pragma: no cover - robot uses Python 3
        from urllib2 import Request, urlopen
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import Bool

    rospy.init_node('bounded_navigation_validation', anonymous=True)
    lock = threading.Lock()
    poses, transitions = [], []
    state = {'moving': False, 'coverage': 0.0}

    def pose_callback(msg):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                         1 - 2 * (q.y*q.y + q.z*q.z))
        with lock:
            poses.append((time.time(), msg.pose.position.x, msg.pose.position.y,
                          yaw, state['moving']))

    def motion_callback(msg):
        with lock:
            state['moving'] = bool(msg.data)
            transitions.append((time.time(), state['moving']))

    def map_callback(msg):
        with lock:
            state['coverage'] = (float(sum(v >= 0 for v in msg.data)) / len(msg.data)
                                 if msg.data else 0.0)

    subscribers = [
        rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback, queue_size=100),
        rospy.Subscriber('/room_mapper/moving', Bool, motion_callback, queue_size=20),
        rospy.Subscriber('/map', OccupancyGrid, map_callback, queue_size=2),
    ]
    time.sleep(2.0)
    resume = Request('http://127.0.0.1:8000/resume', data=b'', method='POST')
    pause = Request('http://127.0.0.1:8000/pause', data=b'', method='POST')
    urlopen(resume, timeout=3).read()
    try:
        time.sleep(args.duration)
    finally:
        urlopen(pause, timeout=3).read()
        time.sleep(2.0)
        for subscriber in subscribers:
            subscriber.unregister()
    with lock:
        print(json.dumps(analyze(list(poses), list(transitions), state['coverage']),
                         indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
