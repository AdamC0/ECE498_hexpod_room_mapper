#!/usr/bin/env python3
"""Compare same-stamp CUDA and reference registered-depth images."""

import json
import threading

import numpy as np
import rospy
from sensor_msgs.msg import Image


def decode(message):
    endian = ">" if message.is_bigendian else "<"
    if message.encoding in ("16UC1", "mono16"):
        if message.step < message.width * 2:
            raise ValueError("invalid 16-bit depth stride")
        return np.ndarray(
            (message.height, message.width), dtype=endian + "u2",
            buffer=message.data, strides=(message.step, 2)).copy()
    if message.encoding == "32FC1":
        if message.step < message.width * 4:
            raise ValueError("invalid float depth stride")
        metres = np.ndarray(
            (message.height, message.width), dtype=endian + "f4",
            buffer=message.data, strides=(message.step, 4))
        valid = np.isfinite(metres) & (metres > 0.0) & (metres < 65.535)
        millimetres = np.zeros(metres.shape, dtype=np.uint16)
        millimetres[valid] = np.rint(metres[valid] * 1000.0).astype(np.uint16)
        return millimetres
    raise ValueError("unsupported reference depth encoding: " + message.encoding)


class Comparison:
    def __init__(self):
        self.reference_topic = rospy.get_param(
            "~reference", "/camera/depth_registered/image_raw")
        self.candidate_topic = rospy.get_param(
            "~candidate", "/camera/depth_registered_cuda/image_raw")
        self.frames_needed = int(rospy.get_param("~frames", 30))
        self.lock = threading.Lock()
        self.result_lock = threading.Lock()
        self.reference = {}
        self.candidate = {}
        self.results = []
        rospy.Subscriber(self.reference_topic, Image, self.on_reference,
                         queue_size=3, buff_size=2 ** 22)
        rospy.Subscriber(self.candidate_topic, Image, self.on_candidate,
                         queue_size=3, buff_size=2 ** 22)

    @staticmethod
    def key(message):
        return message.header.stamp.secs, message.header.stamp.nsecs

    def on_reference(self, message):
        with self.lock:
            self.reference[self.key(message)] = message
            if len(self.reference) > 10:
                del self.reference[sorted(self.reference)[0]]
        self.compare_stamp(self.key(message))

    def on_candidate(self, message):
        with self.lock:
            self.candidate[self.key(message)] = message
            if len(self.candidate) > 10:
                del self.candidate[sorted(self.candidate)[0]]
        self.compare_stamp(self.key(message))

    def compare_stamp(self, stamp):
        with self.lock:
            reference = self.reference.pop(stamp, None)
            message = self.candidate.pop(stamp, None)
            if reference is None or message is None:
                if reference is not None:
                    self.reference[stamp] = reference
                if message is not None:
                    self.candidate[stamp] = message
                return
        ref = decode(reference)
        candidate = decode(message)
        if ref.shape != candidate.shape:
            rospy.logerr("Registration shape mismatch: %s vs %s", ref.shape,
                         candidate.shape)
            return
        joint = (ref > 0) & (candidate > 0)
        union = (ref > 0) | (candidate > 0)
        errors = np.abs(ref[joint].astype(np.int32) -
                        candidate[joint].astype(np.int32))
        result = {
            "reference_valid_fraction":
                float(np.count_nonzero(ref)) / max(1, ref.size),
            "candidate_valid_fraction":
                float(np.count_nonzero(candidate)) / max(1, candidate.size),
            "joint_fraction": float(np.count_nonzero(joint)) / max(1, ref.size),
            "valid_iou": float(np.count_nonzero(joint)) / max(1, np.count_nonzero(union)),
            "median_error_mm": float(np.median(errors)) if errors.size else None,
            "p95_error_mm": float(np.percentile(errors, 95)) if errors.size else None,
        }
        with self.result_lock:
            self.results.append(result)
            if len(self.results) >= self.frames_needed:
                keys = self.results[0].keys()
                summary = {"frames": len(self.results)}
                for key in keys:
                    values = [item[key] for item in self.results
                              if item[key] is not None]
                    summary[key] = float(np.median(values)) if values else None
                print(json.dumps(summary, sort_keys=True))
                rospy.signal_shutdown("comparison complete")


if __name__ == "__main__":
    rospy.init_node("compare_cuda_depth_registration")
    Comparison()
    rospy.spin()
