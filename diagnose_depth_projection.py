#!/usr/bin/env python3
"""Compare registered-depth free-space projection policies without commanding motion."""

import json
import time

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from depth_mapping import project_depth_cells, quaternion_matrix
from object_mapping import TransformCache, decode_depth_image


def main():
    rospy.init_node('diagnose_depth_projection', anonymous=True)
    transforms = TransformCache()

    def update(message, is_static):
        for item in message.transforms:
            t = item.transform.translation
            q = item.transform.rotation
            transforms.update(
                item.header.frame_id, item.child_frame_id,
                (t.x, t.y, t.z), (q.x, q.y, q.z, q.w),
                item.header.stamp.to_sec(), is_static)

    subscribers = [
        rospy.Subscriber('/tf', TFMessage, update, callback_args=False,
                         queue_size=20),
        rospy.Subscriber('/tf_static', TFMessage, update, callback_args=True,
                         queue_size=20),
    ]
    time.sleep(1.)
    info = rospy.wait_for_message(
        '/camera/depth_registered/camera_info', CameraInfo, timeout=8.)
    image = rospy.wait_for_message(
        '/camera/depth_registered/image_raw', Image, timeout=8.)
    depth = decode_depth_image(image)
    if (info.height, info.width) != depth.shape:
        raise SystemExit('Registered-depth calibration size does not match image')
    frame = image.header.frame_id.lstrip('/')
    translation, quaternion = transforms.lookup(
        'map', frame, image.header.stamp.to_sec(), max_age=1.)
    common = dict(cell=.10, decimation=4, range_min=.15, range_max=2.,
                  height_min=.025, height_max=.50, min_points=3,
                  protected_range=.60)
    policies = {
        'deployed': {'free_surface_max': 2.},
        'far_returns': {'free_surface_max': 8.},
        'protected_045': {
            'free_surface_max': 2., 'protected_range': .45},
        'protected_030': {
            'free_surface_max': 2., 'protected_range': .30},
    }
    results = {}
    for name, policy in policies.items():
        arguments = dict(common)
        arguments.update(policy)
        occupied, observed_free = project_depth_cells(
            depth, info.K, (translation, quaternion), **arguments)
        results[name] = {
            'occupied_cells': len(occupied),
            'free_cells': len(observed_free),
        }
    valid = np.isfinite(depth) & (depth >= .15) & (depth <= 2.)
    clear_valid = np.isfinite(depth) & (depth >= .15) & (depth <= 8.)
    vv, uu = np.indices(depth.shape, dtype=np.float64)
    z = depth[valid].astype(np.float64)
    points = np.vstack(((uu[valid] - info.K[2]) * z / info.K[0],
                        (vv[valid] - info.K[5]) * z / info.K[4], z))
    world_z = (quaternion_matrix(quaternion).dot(points) +
               np.asarray(translation, dtype=np.float64)[:, None])[2]
    edges = np.arange(-.10, .525, .025)
    counts, _ = np.histogram(world_z, bins=edges)
    height_histogram = {
        '%.3f..%.3f' % (edges[index], edges[index + 1]): int(count)
        for index, count in enumerate(counts) if count
    }
    print(json.dumps({
        'frame': frame,
        'stamp': image.header.stamp.to_sec(),
        'valid_fraction': float(np.count_nonzero(valid)) / depth.size,
        'clear_valid_fraction': float(np.count_nonzero(clear_valid)) / depth.size,
        'world_height_histogram_m': height_histogram,
        'policies': results,
    }, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
