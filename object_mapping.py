#!/usr/bin/env python3
"""YOLO inference, RGB-depth geometry, and persistent map object tracks.

This module has no ROS dependency so its geometry and tracking can be tested on
the development computer.  ROS/TF synchronization is handled by room_mapper.
"""

import math
import time

import cv2
import numpy as np


COCO_CLASSES = (
    "person|bicycle|car|motorcycle|airplane|bus|train|truck|boat|traffic light|fire hydrant|"
    "stop sign|parking meter|bench|bird|cat|dog|horse|sheep|cow|elephant|bear|zebra|giraffe|"
    "backpack|umbrella|handbag|tie|suitcase|frisbee|skis|snowboard|sports ball|kite|baseball bat|"
    "baseball glove|skateboard|surfboard|tennis racket|bottle|wine glass|cup|fork|knife|spoon|bowl|"
    "banana|apple|sandwich|orange|broccoli|carrot|hot dog|pizza|donut|cake|chair|couch|potted plant|"
    "bed|dining table|toilet|tv|laptop|mouse|remote|keyboard|cell phone|microwave|oven|toaster|sink|"
    "refrigerator|book|clock|vase|scissors|teddy bear|hair drier|toothbrush"
).split("|")


def decode_depth_image(msg):
    """Decode a ROS Image-like depth message to metres without resizing it."""
    if msg.encoding in ("16UC1", "mono16"):
        kind, scale = "u2", 0.001
    elif msg.encoding == "32FC1":
        kind, scale = "f4", 1.0
    else:
        raise ValueError("Unsupported depth encoding: " + msg.encoding)
    dtype = np.dtype((">" if msg.is_bigendian else "<") + kind)
    if (msg.height < 2 or msg.width < 2 or msg.step < msg.width * dtype.itemsize or
            len(msg.data) < msg.height * msg.step):
        raise ValueError("Invalid depth dimensions, stride, or buffer length")
    depth = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.step, dtype.itemsize)).astype(np.float32) * scale
    depth[~np.isfinite(depth)] = 0.0
    return depth


def sample_box_depth(depth, box, min_pixels=12, max_depth=8.0):
    """Return robust depth and sampling pixel from the central part of a box."""
    if depth is None:
        return None
    x1, y1, x2, y2 = [float(v) for v in box]
    # Avoid background near the edges of an imperfect detector box.
    x1, x2 = x1 + .30 * (x2 - x1), x2 - .30 * (x2 - x1)
    y1, y2 = y1 + .25 * (y2 - y1), y2 - .25 * (y2 - y1)
    h, w = depth.shape[:2]
    ix1, ix2 = max(0, int(x1)), min(w, int(math.ceil(x2)))
    iy1, iy2 = max(0, int(y1)), min(h, int(math.ceil(y2)))
    if ix2 <= ix1 or iy2 <= iy1:
        return None
    roi = depth[iy1:iy2, ix1:ix2]
    valid = roi[(roi > 0.10) & (roi < max_depth) & np.isfinite(roi)]
    if valid.size >= min_pixels:
        return float(np.median(valid)), .5 * (x1 + x2), .5 * (y1 + y2)

    # Dark/reflective objects can have a depth hole at the exact box center.
    # Fall back to the valid pixels nearest the center within an 80%-size box;
    # this is less likely to select background than a median of the full box.
    bx1, bx2 = max(0, int(box[0] + .10*(box[2]-box[0]))), \
               min(w, int(math.ceil(box[2] - .10*(box[2]-box[0]))))
    by1, by2 = max(0, int(box[1] + .10*(box[3]-box[1]))), \
               min(h, int(math.ceil(box[3] - .10*(box[3]-box[1]))))
    broad = depth[by1:by2, bx1:bx2]
    mask = (broad > .10) & (broad < max_depth) & np.isfinite(broad)
    yy, xx = np.nonzero(mask)
    if yy.size < min_pixels:
        return None
    center_x, center_y = .5*(box[0]+box[2]) - bx1, .5*(box[1]+box[3]) - by1
    order = np.argsort((xx-center_x)**2 + (yy-center_y)**2)
    take = order[:max(min_pixels, int(math.ceil(.20*order.size)))]
    return float(np.median(broad[yy[take], xx[take]])), \
        .5 * (box[0] + box[2]), .5 * (box[1] + box[3])


def backproject(u, v, depth, k):
    """Back-project an RGB pixel to its optical-frame XYZ point."""
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if (not all(math.isfinite(v) for v in (fx, fy, cx, cy, u, v, depth)) or
            fx <= 0 or fy <= 0 or depth <= 0):
        raise ValueError("Invalid camera intrinsics or depth")
    return ((float(u) - cx) * depth / fx,
            (float(v) - cy) * depth / fy,
            float(depth))


def transform_point(point, translation, quaternion):
    """Apply a geometry_msgs Transform-like rigid transform to XYZ."""
    x, y, z = point
    qx, qy, qz, qw = quaternion
    # Quaternion-vector rotation without constructing a matrix.
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)
    return (rx + translation[0], ry + translation[1], rz + translation[2])


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def compose_transform(a, b):
    """Compose parent_T_mid `a` with mid_T_child `b`."""
    at, aq = a
    bt, bq = b
    rotated = transform_point(bt, (0.0, 0.0, 0.0), aq)
    return ((at[0] + rotated[0], at[1] + rotated[1], at[2] + rotated[2]),
            quaternion_multiply(aq, bq))


def inverse_transform(transform):
    translation, quaternion = transform
    qx, qy, qz, qw = quaternion
    inverse_q = (-qx, -qy, -qz, qw)
    inverse_t = transform_point((-translation[0], -translation[1], -translation[2]),
                                (0.0, 0.0, 0.0), inverse_q)
    return inverse_t, inverse_q


class TransformCache:
    """Small Python-3-compatible cache for ROS /tf and /tf_static messages."""

    def __init__(self, history=100):
        self.history = int(history)
        self.dynamic = {}
        self.static = {}

    def update(self, parent, child, translation, quaternion, stamp, is_static=False):
        parent, child = parent.lstrip('/'), child.lstrip('/')
        record = (float(stamp), parent, tuple(translation), tuple(quaternion))
        if is_static:
            self.static[child] = record
            return
        records = self.dynamic.setdefault(child, [])
        records.append(record)
        if len(records) > self.history:
            del records[:-self.history]

    def _edge(self, child, stamp, max_age):
        child = child.lstrip('/')
        if child in self.static:
            return self.static[child]
        records = self.dynamic.get(child, [])
        if not records:
            raise KeyError("no TF parent for " + child)
        record = min(records, key=lambda item: abs(item[0] - stamp))
        if stamp > 0 and abs(record[0] - stamp) > max_age:
            raise KeyError("stale TF for " + child)
        return record

    def _root_transform(self, frame, stamp, max_age):
        current = frame.lstrip('/')
        root_t_frame = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        visited = set()
        while current not in visited:
            visited.add(current)
            try:
                _, parent, translation, quaternion = self._edge(current, stamp, max_age)
            except KeyError:
                return current, root_t_frame
            root_t_frame = compose_transform((translation, quaternion), root_t_frame)
            current = parent
        raise ValueError("TF cycle involving " + current)

    def lookup(self, target, source, stamp, max_age=.5):
        source_root, root_t_source = self._root_transform(source, stamp, max_age)
        target_root, root_t_target = self._root_transform(target, stamp, max_age)
        if source_root != target_root:
            raise KeyError("TF trees are disconnected: %s vs %s" %
                           (source_root, target_root))
        return compose_transform(inverse_transform(root_t_target), root_t_source)


class YoloOnnxDetector:
    """Small OpenCV-DNN runner for raw YOLOv5/v8-style ONNX exports."""

    def __init__(self, model_path, labels=None, input_size=640, confidence=.45, nms=.45):
        self.labels = list(labels or COCO_CLASSES)
        self.input_size = int(input_size)
        self.confidence = float(confidence)
        self.nms = float(nms)
        self.net = cv2.dnn.readNetFromONNX(model_path)

    def detect(self, image):
        h, w = image.shape[:2]
        scale = min(self.input_size / float(w), self.input_size / float(h))
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(image, (nw, nh))
        left, top = (self.input_size - nw) // 2, (self.input_size - nh) // 2
        canvas = np.full((self.input_size, self.input_size, 3), 114, np.uint8)
        canvas[top:top + nh, left:left + nw] = resized
        blob = cv2.dnn.blobFromImage(canvas, 1.0 / 255.0, (self.input_size, self.input_size),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        output = self.net.forward()
        return self._decode(output, scale, left, top, w, h)

    def _decode(self, output, scale, left, top, image_w, image_h):
        rows = np.asarray(output)
        rows = np.squeeze(rows)
        if rows.ndim == 1:
            rows = rows[None, :]
        if rows.ndim != 2:
            raise ValueError("Unsupported YOLO output shape: %r" % (np.asarray(output).shape,))
        expected4, expected5 = len(self.labels) + 4, len(self.labels) + 5
        if rows.shape[0] in (expected4, expected5) and rows.shape[1] not in (expected4, expected5):
            rows = rows.T
        boxes, scores, class_ids = [], [], []
        for row in rows:
            if row.size == expected5:  # YOLOv5: cx cy w h objectness classes...
                class_scores = row[5:]
                class_id = int(np.argmax(class_scores))
                score = float(row[4] * class_scores[class_id])
            elif row.size == expected4:  # YOLOv8: cx cy w h classes...
                class_scores = row[4:]
                class_id = int(np.argmax(class_scores))
                score = float(class_scores[class_id])
            else:
                continue
            if score < self.confidence:
                continue
            cx, cy, bw, bh = [float(v) for v in row[:4]]
            x = (cx - .5 * bw - left) / scale
            y = (cy - .5 * bh - top) / scale
            bw, bh = bw / scale, bh / scale
            boxes.append([int(round(x)), int(round(y)), int(round(bw)), int(round(bh))])
            scores.append(score)
            class_ids.append(class_id)
        if not boxes:
            return []
        kept = cv2.dnn.NMSBoxes(boxes, scores, self.confidence, self.nms)
        detections = []
        for i in np.asarray(kept).reshape(-1):
            x, y, bw, bh = boxes[int(i)]
            x1, y1 = max(0, x), max(0, y)
            x2, y2 = min(image_w - 1, x + bw), min(image_h - 1, y + bh)
            if x2 > x1 and y2 > y1:
                detections.append({"label": self.labels[class_ids[int(i)]],
                                   "confidence": scores[int(i)],
                                   "box": (x1, y1, x2, y2)})
        return detections


class ObjectTracker:
    """Merge repeated class observations into confirmed map landmarks."""

    def __init__(self, merge_distance=.65, min_observations=3, max_age=120.0):
        self.merge_distance = float(merge_distance)
        self.min_observations = int(min_observations)
        self.max_age = float(max_age)
        self.tracks = []
        self.suppressed = []
        self.next_id = 1

    def clear(self):
        self.tracks = []
        self.suppressed = []
        self.next_id = 1

    def update(self, label, x, y, confidence, now=None, observation_id=None):
        now = time.time() if now is None else float(now)
        self.prune(now)
        if any(item[0] == label and math.hypot(item[1] - x, item[2] - y) <=
               self.merge_distance for item in self.suppressed):
            return None
        candidates = [t for t in self.tracks if t["label"] == label]
        track = min(candidates, key=lambda t: math.hypot(t["x"] - x, t["y"] - y)) \
            if candidates else None
        if track is None or math.hypot(track["x"] - x, track["y"] - y) > self.merge_distance:
            track = {"id": self.next_id, "label": label, "x": float(x), "y": float(y),
                     "confidence": float(confidence), "count": 1, "m2": 0.0,
                     "last_seen": now, "last_observation": observation_id}
            self.next_id += 1
            self.tracks.append(track)
            return track
        if observation_id is not None and track["last_observation"] == observation_id:
            return track
        old_x, old_y = track["x"], track["y"]
        n = track["count"] + 1
        track["x"] += (float(x) - track["x"]) / n
        track["y"] += (float(y) - track["y"]) / n
        track["m2"] += (float(x) - old_x) * (float(x) - track["x"]) + \
                       (float(y) - old_y) * (float(y) - track["y"])
        track["count"], track["last_seen"] = n, now
        track["last_observation"] = observation_id
        track["confidence"] = max(float(confidence), track["confidence"] * .98)
        return track

    def rename(self, track_id, name):
        """Set a display name without changing the detector class used to merge."""
        name = str(name).strip()
        if not name or len(name) > 40 or any(ord(ch) < 32 for ch in name):
            return False
        for track in self.tracks:
            if track["id"] == int(track_id):
                track["name"] = name
                return True
        return False

    def remove(self, track_id):
        """Remove a landmark and suppress re-detection at the same location."""
        for index, track in enumerate(self.tracks):
            if track["id"] == int(track_id):
                self.suppressed.append((track["label"], track["x"], track["y"]))
                del self.tracks[index]
                return True
        return False

    def discard(self, track_id):
        """Drop an unconfirmed candidate without suppressing future detections."""
        for index, track in enumerate(self.tracks):
            if track["id"] == int(track_id):
                del self.tracks[index]
                return True
        return False

    def prune(self, now=None):
        now = time.time() if now is None else float(now)
        self.tracks = [t for t in self.tracks if now - t["last_seen"] <= self.max_age]

    def snapshot(self, confirmed_only=True, now=None):
        self.prune(now)
        result = []
        for track in self.tracks:
            if confirmed_only and track["count"] < self.min_observations:
                continue
            item = dict(track)
            item["name"] = track.get("name", track["label"])
            item["position_std"] = math.sqrt(track["m2"] / max(1, track["count"] - 1))
            result.append(item)
        return result
