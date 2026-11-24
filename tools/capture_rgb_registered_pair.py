#!/usr/bin/env python3
"""Capture one timestamp-matched raw RGB/CUDA-registered-depth pair."""

import os
import sys
import threading

import cv2
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import Image


def main():
    output_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/room_mapper_alignment"
    os.makedirs(output_dir, exist_ok=True)
    ready = threading.Event()
    result = {}

    def capture(rgb_msg, depth_msg):
        if ready.is_set():
            return
        rgb_raw = np.frombuffer(rgb_msg.data, np.uint8).reshape(
            rgb_msg.height, rgb_msg.step)
        rgb = rgb_raw[:, :rgb_msg.width * 3].reshape(
            rgb_msg.height, rgb_msg.width, 3).copy()
        if rgb_msg.encoding != "bgr8":
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        depth_raw = np.frombuffer(depth_msg.data, np.uint16).reshape(
            depth_msg.height, depth_msg.step // 2)
        depth = depth_raw[:, :depth_msg.width].copy()
        result.update(rgb=rgb, depth=depth,
                      rgb_stamp=rgb_msg.header.stamp.to_sec(),
                      depth_stamp=depth_msg.header.stamp.to_sec())
        ready.set()

    rospy.init_node("capture_rgb_registered_pair", anonymous=True)
    rgb_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
    depth_sub = message_filters.Subscriber(
        "/camera/depth_registered_cuda/image_raw", Image)
    sync = message_filters.ApproximateTimeSynchronizer(
        [rgb_sub, depth_sub], queue_size=20, slop=0.05)
    sync.registerCallback(capture)
    if not ready.wait(12.0):
        raise RuntimeError("No RGB/registered-depth pair arrived within 12 seconds")
    rgb_path = os.path.join(output_dir, "rgb.png")
    depth_path = os.path.join(output_dir, "registered_depth.png")
    if not cv2.imwrite(rgb_path, result["rgb"]):
        raise RuntimeError("Could not write %s" % rgb_path)
    if not cv2.imwrite(depth_path, result["depth"]):
        raise RuntimeError("Could not write %s" % depth_path)
    print("rgb=%s depth=%s delta_ms=%.3f" % (
        rgb_path, depth_path,
        1000.0 * (result["rgb_stamp"] - result["depth_stamp"])))


if __name__ == "__main__":
    main()
