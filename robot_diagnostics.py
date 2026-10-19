#!/usr/bin/env python3
"""Read-only, bounded ROS sensor/TF inventory. Never starts drivers or sends motion.

Run on the stationary robot: python3 robot_diagnostics.py --output diagnostics.json
Only metadata and final pose/IMU samples are saved, not camera images.
"""
import argparse
import json
import math
import threading
import time


EXPECTED_TOPICS = {
    '/camera/rgb/image_raw': 'sensor_msgs/Image',
    '/camera/depth/image_raw': 'sensor_msgs/Image',
    '/camera/depth_registered/image_raw': 'sensor_msgs/Image',
    '/camera/rgb/camera_info': 'sensor_msgs/CameraInfo',
    '/scan': 'sensor_msgs/LaserScan',
    '/scan_stable': 'sensor_msgs/LaserScan',
    '/slam_out_pose': 'geometry_msgs/PoseStamped',
    '/map': 'nav_msgs/OccupancyGrid',
    '/odom/filtered': 'nav_msgs/Odometry',
    '/tf': 'tf2_msgs/TFMessage',
    '/tf_static': 'tf2_msgs/TFMessage',
}


def stamp_seconds(msg):
    try:
        return float(msg.header.stamp.to_sec())
    except AttributeError:
        return None


def quaternion_summary(q):
    values = [q.x, q.y, q.z, q.w]
    if not all(math.isfinite(v) for v in values):
        return {'valid': False}
    norm = math.sqrt(sum(v*v for v in values))
    valid = abs(norm - 1.) < .01
    return {'xyzw': values, 'norm': norm, 'valid': valid,
            'yaw_deg': math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y),
                                               1 - 2*(q.y*q.y + q.z*q.z))) if valid else None}


class Inventory:
    def __init__(self, topics):
        self.lock = threading.Lock()
        self.topics = {name: {'type': kind, 'count': 0} for name, kind in topics.items()}
        self.transforms = {}

    def observe(self, topic, msg, receipt):
        with self.lock:
            item = self.topics[topic]
            item['count'] += 1
            item.setdefault('first_receipt', receipt)
            item['last_receipt'] = receipt
            stamp = stamp_seconds(msg)
            if stamp is not None:
                previous = item.get('stamp')
                item['repeated_stamps'] = item.get('repeated_stamps', 0) + int(stamp == previous)
                item['backward_stamps'] = item.get('backward_stamps', 0) + int(
                    previous is not None and stamp < previous)
                item['stamp'] = stamp
                item['frame'] = msg.header.frame_id
            publisher = getattr(msg, '_connection_header', {}).get('callerid', 'unknown')
            item['publishers'] = sorted(set(item.get('publishers', []) + [publisher]))
            for field in ('width', 'height', 'encoding', 'step', 'child_frame_id'):
                if hasattr(msg, field):
                    item[field] = getattr(msg, field)
            if hasattr(msg, 'K'):
                item['K'] = list(msg.K)
            if hasattr(msg, 'ranges'):
                item['scan_samples'] = len(msg.ranges)
                item['scan_time'] = msg.scan_time
                item['time_increment'] = msg.time_increment
            if hasattr(msg, 'pose'):
                pose = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
                item['position'] = [pose.position.x, pose.position.y, pose.position.z]
                item['orientation'] = quaternion_summary(pose.orientation)
                if hasattr(msg.pose, 'covariance'):
                    item['pose_covariance'] = list(msg.pose.covariance)
            if hasattr(msg, 'orientation_covariance'):
                item['orientation'] = quaternion_summary(msg.orientation)
                item['orientation_covariance'] = list(msg.orientation_covariance)
                item['angular_velocity'] = [msg.angular_velocity.x, msg.angular_velocity.y,
                                            msg.angular_velocity.z]
            for tf in getattr(msg, 'transforms', []):
                key = (tf.header.frame_id, tf.child_frame_id, publisher, topic)
                self.transforms[key] = {
                    'parent': tf.header.frame_id, 'child': tf.child_frame_id,
                    'publisher': publisher, 'topic': topic, 'stamp': stamp_seconds(tf),
                    'rotation': quaternion_summary(tf.transform.rotation),
                    'translation': [tf.transform.translation.x, tf.transform.translation.y,
                                    tf.transform.translation.z]}

    def report(self, receipt_now, ros_now):
        with self.lock:
            topics = {}
            for name, original in self.topics.items():
                item = dict(original)
                if item['count']:
                    elapsed = item['last_receipt'] - item['first_receipt']
                    item['rate_hz'] = (item['count'] - 1) / elapsed if elapsed > 0 else None
                    item['receipt_age_s'] = receipt_now - item['last_receipt']
                    stamp = item.get('stamp')
                    item['stamp_age_s'] = ros_now - stamp if stamp and stamp > 0 else None
                topics[name] = item
            return {'topics': topics, 'transforms': list(self.transforms.values()),
                    'note': 'Presence is not health. Inspect rates, ages, repeated stamps, '
                            'TF publishers, covariance, and physical heading before movement.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20.)
    parser.add_argument('--output', default='diagnostics.json')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 120:
        parser.error('duration must be between 1 and 120 seconds')
    import rospy
    import roslib.message
    rospy.init_node('room_mapper_diagnostics', anonymous=True, disable_signals=True)
    published = dict(rospy.get_published_topics())
    topics = dict(EXPECTED_TOPICS)
    topics.update({name: kind for name, kind in published.items()
                   if kind in ('sensor_msgs/Imu', 'nav_msgs/Odometry')})
    inventory = Inventory(topics)
    subscribers = []
    for name, kind in topics.items():
        cls = roslib.message.get_message_class(kind)
        if cls is not None:
            subscribers.append(rospy.Subscriber(name, cls,
                lambda msg, topic=name: inventory.observe(topic, msg, time.monotonic()),
                queue_size=1, buff_size=2**24))
    started = time.monotonic()
    try:
        while not rospy.is_shutdown() and time.monotonic() - started < args.duration:
            time.sleep(.1)
    except KeyboardInterrupt:
        pass
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
    report = inventory.report(time.monotonic(), rospy.Time.now().to_sec())
    report['observed_duration_s'] = time.monotonic() - started
    report['advertised_topics'] = published
    with open(args.output, 'w') as output:
        json.dump(report, output, indent=2)
    print('Saved read-only inventory to ' + args.output)


if __name__ == '__main__':
    main()
