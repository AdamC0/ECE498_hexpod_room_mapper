#!/usr/bin/env python3
"""YOLO inference, RGB-depth geometry, and persistent map object tracks.

This module has no ROS dependency so its geometry and tracking can be tested on
the development computer.  ROS/TF synchronization is handled by room_mapper.
"""

import hashlib
import json
import math
import os
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

OBJECT_LABEL_ALIASES = {
    "backpack": "luggage",
    "luggage": "luggage",
    "suitcase": "luggage",
}


def canonical_object_label(label):
    """Return a stable map label for detector synonyms of one object type."""
    normalized = str(label).strip().lower()
    return OBJECT_LABEL_ALIASES.get(normalized, str(label).strip())


def deduplicate_mapped_detections(detections, distance=.10, minimum_iou=.50,
                                  spatial_labels=None):
    """Return one map observation for overlapping same-object detections."""
    spatial_labels = {str(label).strip().lower() for label in
                      (spatial_labels or ("power outlet",))}
    accepted = []
    ordered = sorted(detections, key=lambda item: float(item.get("confidence", 0.0)),
                     reverse=True)
    for detection in ordered:
        point = detection.get("map_point")
        if point is None:
            continue
        label = canonical_object_label(detection.get("label", ""))
        duplicate = False
        for previous in accepted:
            if previous["map_label"] != label:
                continue
            previous_point = previous["detection"]["map_point"]
            if math.hypot(point[0] - previous_point[0],
                          point[1] - previous_point[1]) > float(distance):
                continue
            overlap = box_iou(detection.get("box"),
                              previous["detection"].get("box"))
            if label in spatial_labels or overlap >= float(minimum_iou):
                duplicate = True
                break
        if not duplicate:
            accepted.append({"detection": detection, "map_label": label})
    return accepted


def load_labels(path):
    """Load a one-label-per-line model vocabulary."""
    with open(path, "r", encoding="utf-8") as stream:
        labels = [line.strip() for line in stream if line.strip()]
    if not labels:
        raise ValueError("YOLO label file is empty: " + path)
    return labels


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


def sample_box_depth(depth, box, min_pixels=12, max_depth=8.0, return_quality=False):
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
        median = float(np.median(valid))
        quality = {
            "valid_pixels": int(valid.size),
            "coverage": float(valid.size) / max(1, roi.size),
            "mad": float(np.median(np.abs(valid - median))),
            "inlier_ratio": float(np.mean(
                np.abs(valid - median) <= max(.08, .08 * median))),
            "fallback": False,
        }
        result = (median, .5 * (x1 + x2), .5 * (y1 + y2))
        return result + (quality,) if return_quality else result

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
    values = broad[yy[take], xx[take]]
    median = float(np.median(values))
    quality = {
        "valid_pixels": int(values.size),
        "coverage": float(yy.size) / max(1, broad.size),
        "mad": float(np.median(np.abs(values - median))),
        "inlier_ratio": float(np.mean(
            np.abs(values - median) <= max(.08, .08 * median))),
        "fallback": True,
    }
    result = (median, .5 * (box[0] + box[2]), .5 * (box[1] + box[3]))
    return result + (quality,) if return_quality else result


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


def landmark_visibility(track, detections, depth, k, map_from_camera,
                        image_margin=.10, occlusion_margin=.15,
                        depth_window=9, minimum_depth_pixels=9):
    """Return ``(detected, clear_view)`` for one confirmed map landmark.

    A negative YOLO result is useful when the saved 3-D point projects safely
    inside the RGB image. A nearer valid depth return means the landmark may be
    occluded; missing/invalid local depth still supplies semantic absence
    evidence because registered depth has repeatable holes in the live scene.
    """
    if (depth is None or map_from_camera is None or track.get("z") is None or
            depth_window < 1 or depth_window % 2 == 0):
        return False, False
    try:
        camera_from_map = inverse_transform(map_from_camera)
        camera_point = transform_point(
            (float(track["x"]), float(track["y"]), float(track["z"])),
            *camera_from_map)
        fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    except (KeyError, TypeError, ValueError, OverflowError):
        return False, False
    x, y, z = camera_point
    if (not all(math.isfinite(value) for value in (x, y, z, fx, fy, cx, cy)) or
            z <= .10 or fx <= 0 or fy <= 0):
        return False, False
    u, v = fx * x / z + cx, fy * y / z + cy
    height, width = depth.shape[:2]
    margin_x, margin_y = float(image_margin) * width, float(image_margin) * height
    if not (margin_x <= u < width - margin_x and margin_y <= v < height - margin_y):
        return False, False

    label = canonical_object_label(track.get("label", ""))
    for detection in detections:
        if canonical_object_label(detection.get("label", "")) != label:
            continue
        box = detection.get("box")
        if box is None or len(box) != 4:
            continue
        x1, y1, x2, y2 = [float(value) for value in box]
        # Allow modest box jitter around the saved projected center.
        pad_x, pad_y = .10 * (x2 - x1), .10 * (y2 - y1)
        if x1 - pad_x <= u <= x2 + pad_x and y1 - pad_y <= v <= y2 + pad_y:
            detection_depth = detection.get("depth")
            if (detection_depth is not None and
                    math.isfinite(float(detection_depth)) and
                    abs(float(detection_depth) - z) > max(.25, .25 * z)):
                # A different same-class object can overlap the old projected
                # center in the image. Do not let a materially different depth
                # refresh the stale landmark's absence or age counters.
                continue
            return True, False

    radius = depth_window // 2
    center_x, center_y = int(round(u)), int(round(v))
    x1, x2 = max(0, center_x - radius), min(width, center_x + radius + 1)
    y1, y2 = max(0, center_y - radius), min(height, center_y + radius + 1)
    patch = np.asarray(depth[y1:y2, x1:x2], dtype=np.float32)
    valid = patch[(patch > .10) & (patch < 8.0) & np.isfinite(patch)]
    if valid.size < int(minimum_depth_pixels):
        # The registered-depth camera has large repeatable holes on dark, low,
        # and distant surfaces. The landmark is still centrally in view and
        # YOLO supplied no matching box, so count semantic absence. This does
        # not clear planner occupancy and automatic removal permits rediscovery.
        return False, True
    observed_depth = float(np.median(valid))
    if observed_depth + float(occlusion_margin) < z:
        return False, False
    return False, True


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

    def __init__(self, model_path, labels=None, input_size=640, confidence=.45, nms=.45,
                 class_thresholds=None, allowed_labels=None, backend="cpu"):
        self._configure(labels, input_size, confidence, nms,
                        class_thresholds, allowed_labels)
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.backend = str(backend).strip().lower()
        if self.backend == "cuda_fp16":
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        elif self.backend != "cpu":
            raise ValueError("OpenCV YOLO backend must be cpu or cuda_fp16")

    def _configure(self, labels, input_size, confidence, nms,
                   class_thresholds, allowed_labels):
        self.labels = list(labels or COCO_CLASSES)
        self.input_size = int(input_size)
        self.confidence = float(confidence)
        self.nms = float(nms)
        if self.input_size < 256 or self.input_size % 32:
            raise ValueError("YOLO input_size must be at least 256 and divisible by 32")
        self.class_thresholds = {
            str(label).strip().lower(): float(value)
            for label, value in (class_thresholds or {}).items()
        }
        if any(not 0.01 <= value <= 0.99 for value in self.class_thresholds.values()):
            raise ValueError("YOLO class thresholds must be between 0.01 and 0.99")
        self.allowed_labels = ({str(label).strip().lower() for label in allowed_labels}
                               if allowed_labels else None)

    def threshold_for(self, label):
        return getattr(self, "class_thresholds", {}).get(
            str(label).lower(), self.confidence)

    def warmup(self):
        """Run one blank frame so unsupported models/backends fail at startup."""
        self.detect(np.zeros((self.input_size, self.input_size, 3), np.uint8))

    def detect(self, image):
        blob, geometry = self._preprocess(image)
        self.net.setInput(blob)
        output = self.net.forward()
        return self._decode(output, *geometry)

    def close(self):
        """Release device allocations while their CUDA context is current."""
        context = getattr(self, "cuda_context", None)
        if context is None:
            return
        try:
            context.push()
            try:
                stream = getattr(self, "stream", None)
                if stream is not None:
                    stream.synchronize()
                # TensorRT owns CUDA allocations too; destroy its objects while
                # this detector's context is current, before detaching it.
                for name in ("context", "engine", "runtime", "stream"):
                    if hasattr(self, name):
                        setattr(self, name, None)
                for name in ("input_device", "output_device"):
                    allocation = getattr(self, name, None)
                    if allocation is not None:
                        allocation.free()
                        setattr(self, name, None)
            finally:
                context.pop()
            context.detach()
        finally:
            self.cuda_context = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def _preprocess(self, image):
        h, w = image.shape[:2]
        scale = min(self.input_size / float(w), self.input_size / float(h))
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(image, (nw, nh))
        left, top = (self.input_size - nw) // 2, (self.input_size - nh) // 2
        canvas = np.full((self.input_size, self.input_size, 3), 114, np.uint8)
        canvas[top:top + nh, left:left + nw] = resized
        blob = cv2.dnn.blobFromImage(canvas, 1.0 / 255.0, (self.input_size, self.input_size),
                                     swapRB=True, crop=False)
        return blob, (scale, left, top, w, h)

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
        if rows.shape[1] not in (6, expected4, expected5):
            raise ValueError("YOLO output has %d values per prediction, but %d labels were "
                             "configured" % (rows.shape[1], len(self.labels)))
        # Select the strongest class and reject low-confidence predictions in
        # NumPy.  The Objects365 output has 8,400 rows and 150 classes; doing
        # one argmax and several Python operations per row materially delays
        # the otherwise fast TensorRT path when the mapping stack loads PT's
        # four CPU cores.  Box conversion remains ordered below so observable
        # decode and class-aware NMS behavior is unchanged.
        end_to_end = rows.shape[1] == 6 and len(self.labels) != 2
        if end_to_end:  # Exported output: x1 y1 x2 y2 score class.
            finite_classes = np.isfinite(rows[:, 5])
            class_ids_all = np.zeros(rows.shape[0], dtype=np.int64)
            class_ids_all[finite_classes] = np.rint(
                rows[finite_classes, 5]).astype(np.int64)
            scores_all = rows[:, 4]
            valid = finite_classes
        elif rows.shape[1] == expected5:  # YOLOv5: cx cy w h objectness classes...
            class_scores = rows[:, 5:]
            class_ids_all = np.argmax(class_scores, axis=1)
            scores_all = rows[:, 4] * class_scores[
                np.arange(rows.shape[0]), class_ids_all]
            valid = np.ones(rows.shape[0], dtype=bool)
        else:  # YOLOv8: cx cy w h classes...
            class_scores = rows[:, 4:]
            class_ids_all = np.argmax(class_scores, axis=1)
            scores_all = class_scores[np.arange(rows.shape[0]), class_ids_all]
            valid = np.ones(rows.shape[0], dtype=bool)

        valid &= (class_ids_all >= 0) & (class_ids_all < len(self.labels))
        candidate_indices = np.flatnonzero(valid)
        if candidate_indices.size:
            candidate_classes = class_ids_all[candidate_indices]
            thresholds = np.asarray([
                self.threshold_for(label) for label in self.labels
            ], dtype=np.float64)
            keep = scores_all[candidate_indices] >= thresholds[candidate_classes]
            allowed_labels = getattr(self, "allowed_labels", None)
            if allowed_labels is not None:
                allowed_classes = np.asarray([
                    label.lower() in allowed_labels for label in self.labels
                ], dtype=bool)
                keep &= allowed_classes[candidate_classes]
            candidate_indices = candidate_indices[keep]

        boxes, scores, class_ids = [], [], []
        for row_index in candidate_indices:
            row = rows[row_index]
            class_id = int(class_ids_all[row_index])
            score = float(scores_all[row_index])
            label = self.labels[class_id]
            if end_to_end:
                x1, y1, x2, y2 = [float(v) for v in row[:4]]
                x = (x1 - left) / scale
                y = (y1 - top) / scale
                bw, bh = (x2 - x1) / scale, (y2 - y1) / scale
            if not end_to_end:
                cx, cy, bw, bh = [float(v) for v in row[:4]]
                x = (cx - .5 * bw - left) / scale
                y = (cy - .5 * bh - top) / scale
                bw, bh = bw / scale, bh / scale
            boxes.append([int(round(x)), int(round(y)), int(round(bw)), int(round(bh))])
            scores.append(score)
            class_ids.append(class_id)
        if not boxes:
            return []
        # NMS must be class-aware. Class-agnostic suppression can make a desk
        # erase an overlapping chair (or vice versa) in a crowded classroom.
        kept = []
        for class_id in sorted(set(class_ids)):
            indices = [i for i, value in enumerate(class_ids) if value == class_id]
            class_boxes = [boxes[i] for i in indices]
            class_scores = [scores[i] for i in indices]
            threshold = self.threshold_for(self.labels[class_id])
            selected = cv2.dnn.NMSBoxes(class_boxes, class_scores, threshold, self.nms)
            kept.extend(indices[int(i)] for i in np.asarray(selected).reshape(-1))
        detections = []
        for i in kept:
            x, y, bw, bh = boxes[int(i)]
            x1, y1 = max(0, x), max(0, y)
            x2, y2 = min(image_w - 1, x + bw), min(image_h - 1, y + bh)
            if x2 > x1 and y2 > y1:
                detections.append({"label": self.labels[class_ids[int(i)]],
                                   "confidence": scores[int(i)],
                                   "box": (x1, y1, x2, y2)})
        return detections


class TensorRtYoloDetector(YoloOnnxDetector):
    """TensorRT engine runner with no import or CUDA context cost on CPU startup.

    Engines are device-specific generated artifacts. A sidecar written by
    prepare_tensorrt_engine.py binds the engine to the exact source ONNX file,
    preventing a stale engine from silently being paired with a newer model.
    """

    def __init__(self, model_path, engine_path, labels=None, input_size=640,
                 confidence=.45, nms=.45, class_thresholds=None,
                 allowed_labels=None):
        self._configure(labels, input_size, confidence, nms,
                        class_thresholds, allowed_labels)
        self.backend = "tensorrt_fp16"
        self.engine_path = os.path.abspath(engine_path)
        self._validate_sidecar(model_path)
        # These dependencies deliberately remain outside the standard CPU path.
        import tensorrt as trt
        import pycuda.driver as cuda
        self.trt, self.cuda = trt, cuda
        cuda.init()
        # PT's PyCUDA 2019.1 does not expose retain_primary_context correctly.
        # make_context() is supported there and initially pushes the context;
        # pop it after allocation so the object worker can push it explicitly.
        self.cuda_context = cuda.Device(0).make_context()
        try:
            self.logger = trt.Logger(trt.Logger.WARNING)
            with open(self.engine_path, "rb") as stream:
                serialized = stream.read()
            self.runtime = trt.Runtime(self.logger)
            self.engine = self.runtime.deserialize_cuda_engine(serialized)
            if self.engine is None:
                raise RuntimeError("TensorRT could not deserialize engine")
            self.context = self.engine.create_execution_context()
            if self.context is None:
                raise RuntimeError("TensorRT could not create execution context")
            self._allocate_bindings()
        finally:
            self.cuda_context.pop()

    @staticmethod
    def _sha256(path):
        digest = hashlib.sha256()
        with open(path, "rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        return digest.hexdigest()

    def _validate_sidecar(self, model_path):
        if not os.path.isfile(self.engine_path):
            raise IOError("TensorRT engine is missing: " + self.engine_path)
        metadata_path = self.engine_path + ".json"
        if not os.path.isfile(metadata_path):
            raise IOError("TensorRT engine metadata is missing: " + metadata_path)
        with open(metadata_path, "r", encoding="utf-8") as stream:
            metadata = json.load(stream)
        if metadata.get("precision") != "fp16":
            raise ValueError("TensorRT engine metadata does not declare fp16 precision")
        if metadata.get("onnx_sha256") != self._sha256(model_path):
            raise ValueError("TensorRT engine was built from a different ONNX model")
        if metadata.get("engine_sha256") != self._sha256(self.engine_path):
            raise ValueError("TensorRT engine checksum does not match its metadata")

    def _allocate_bindings(self):
        trt, cuda = self.trt, self.cuda
        inputs, outputs = [], []
        self.bindings = [None] * self.engine.num_bindings
        for index in range(self.engine.num_bindings):
            shape = tuple(self.engine.get_binding_shape(index))
            if any(int(value) <= 0 for value in shape):
                raise ValueError("TensorRT engine must use static binding shapes: %r" %
                                 (shape,))
            dtype = np.dtype(trt.nptype(self.engine.get_binding_dtype(index)))
            host = cuda.pagelocked_empty(int(np.prod(shape)), dtype)
            device = cuda.mem_alloc(host.nbytes)
            self.bindings[index] = int(device)
            record = (index, shape, host, device)
            (inputs if self.engine.binding_is_input(index) else outputs).append(record)
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("TensorRT YOLO engine must have one input and one output")
        self.input_index, input_shape, self.input_host, self.input_device = inputs[0]
        self.output_index, self.output_shape, self.output_host, self.output_device = outputs[0]
        if input_shape != (1, 3, self.input_size, self.input_size):
            raise ValueError("TensorRT input shape %r does not match configured size %d" %
                             (input_shape, self.input_size))
        self.stream = cuda.Stream()

    def detect(self, image):
        blob, geometry = self._preprocess(image)
        self.cuda_context.push()
        try:
            np.copyto(self.input_host, blob.reshape(-1), casting="unsafe")
            self.cuda.memcpy_htod_async(self.input_device, self.input_host, self.stream)
            if not self.context.execute_async_v2(self.bindings, self.stream.handle):
                raise RuntimeError("TensorRT execution failed")
            self.cuda.memcpy_dtoh_async(self.output_host, self.output_device, self.stream)
            self.stream.synchronize()
            output = np.array(self.output_host, copy=True).reshape(self.output_shape)
        finally:
            self.cuda_context.pop()
        return self._decode(output, *geometry)


def box_iou(a, b):
    """Return intersection-over-union for two x1/y1/x2/y2 boxes."""
    ax1, ay1, ax2, ay2 = [float(value) for value in a]
    bx1, by1, bx2, by2 = [float(value) for value in b]
    intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * \
        max(0.0, min(ay2, by2) - max(ay1, by1))
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - intersection
    return intersection / union if union > 0.0 else 0.0


class ObjectTracker:
    """Merge repeated class observations into confirmed map landmarks."""

    def __init__(self, merge_distance=.65, min_observations=3, max_age=120.0,
                 priority_labels=None, expected_policy=None, other_policy=None,
                 large_labels=None, class_policies=None, static_labels=None):
        self.merge_distance = float(merge_distance)
        self.min_observations = int(min_observations)
        self.max_age = float(max_age)
        self.validation_enabled = priority_labels is not None
        self.priority_labels = {str(label).strip().lower()
                                for label in (priority_labels or [])}
        self.large_labels = {str(label).strip().lower()
                             for label in (large_labels or [])}
        self.static_labels = {str(label).strip().lower()
                              for label in (static_labels or [])}
        self.expected_policy = dict({
            "hits": 3, "window": 8, "minimum_confidence": .50,
            "median_confidence": .55, "position_tolerance": .25,
            "large_position_tolerance": .50, "box_iou": .40,
        }, **(expected_policy or {}))
        self.other_policy = dict({
            "hits": 3, "window": 8, "minimum_confidence": .60,
            "median_confidence": .75, "position_tolerance": .20,
            "large_position_tolerance": .35, "box_iou": .50,
        }, **(other_policy or {}))
        self.class_policies = {
            str(label).strip().lower(): dict(policy)
            for label, policy in (class_policies or {}).items()
        }
        self.tracks = []
        self.suppressed = []
        self.next_id = 1

    def clear(self):
        self.tracks = []
        self.suppressed = []
        self.next_id = 1

    def _policy(self, label):
        normalized = str(label).lower()
        policy = dict(self.expected_policy if normalized in self.priority_labels
                      else self.other_policy)
        policy.update(self.class_policies.get(normalized, {}))
        return policy

    def _confirmation_evaluation(self, track):
        policy = self._policy(track["label"])
        evidence = track.get("_evidence", [])[-int(policy["window"]):]
        required = int(policy["hits"])
        minimum_confidence = float(policy["minimum_confidence"])
        required_median = float(policy["median_confidence"])
        hits = [item for item in evidence if item.get("hit") and
                item["confidence"] >= minimum_confidence]
        status = {
            "required_hits": required,
            "window": int(policy["window"]),
            "observed_frames": len(evidence),
            "qualifying_hits": len(hits),
            "minimum_confidence": minimum_confidence,
            "required_median_confidence": required_median,
            "misses": sum(1 for item in evidence if not item.get("hit")),
        }
        if len(hits) < required:
            status["reason"] = ("%d/%d detections meet %.2f minimum confidence" %
                                (len(hits), required, minimum_confidence))
            return None, status
        hits = hits[-required:]
        median_confidence = float(np.median(
            [item["confidence"] for item in hits]))
        status["median_confidence"] = median_confidence
        if median_confidence < required_median:
            status["reason"] = ("median confidence %.2f is below %.2f" %
                                (median_confidence, required_median))
            return None, status
        tolerance_key = ("large_position_tolerance"
                         if track["label"].lower() in self.large_labels
                         else "position_tolerance")
        tolerance = float(policy[tolerance_key])
        maximum_distance = 0.0
        minimum_iou = 1.0
        for index, first in enumerate(hits):
            for second in hits[index + 1:]:
                distance = math.hypot(first["x"] - second["x"],
                                      first["y"] - second["y"])
                maximum_distance = max(maximum_distance, distance)
                if distance > tolerance:
                    status["maximum_position_delta"] = maximum_distance
                    status["position_tolerance"] = tolerance
                    status["reason"] = ("map positions differ by %.2f m (limit %.2f m)" %
                                        (distance, tolerance))
                    return None, status
                if (first.get("group") == second.get("group") and
                        first.get("box") is not None and second.get("box") is not None):
                    overlap = box_iou(first["box"], second["box"])
                    minimum_iou = min(minimum_iou, overlap)
                    if overlap < float(policy["box_iou"]):
                        status["minimum_box_iou"] = minimum_iou
                        status["required_box_iou"] = float(policy["box_iou"])
                        status["reason"] = ("same-view box overlap %.2f is below %.2f" %
                                            (overlap, float(policy["box_iou"])))
                        return None, status
        status["maximum_position_delta"] = maximum_distance
        status["position_tolerance"] = tolerance
        status["minimum_box_iou"] = minimum_iou
        status["required_box_iou"] = float(policy["box_iou"])
        status["reason"] = "confirmation requirements met"
        return hits, status

    def _confirmation_hits(self, track):
        return self._confirmation_evaluation(track)[0]

    def _confirmation_ready(self, track):
        return self._confirmation_hits(track) is not None

    def update(self, label, x, y, confidence, now=None, observation_id=None,
               validation_group=None, box=None, z=None):
        now = time.time() if now is None else float(now)
        self.prune(now)
        if any(item[0] == label and math.hypot(item[1] - x, item[2] - y) <=
               self.merge_distance for item in self.suppressed):
            return None
        # A single frame may contain several instances of the same class. Do
        # not let a track already updated by this observation absorb another
        # detection and make that second object disappear.
        candidates = [t for t in self.tracks if t["label"] == label and
                      (observation_id is None or
                       t.get("last_observation") != observation_id)]
        track = min(candidates, key=lambda t: math.hypot(t["x"] - x, t["y"] - y)) \
            if candidates else None
        if track is None or math.hypot(track["x"] - x, track["y"] - y) > self.merge_distance:
            track = {"id": self.next_id, "label": label, "x": float(x), "y": float(y),
                     "z": (float(z) if z is not None else None),
                     "confidence": float(confidence), "count": 0, "m2": 0.0,
                     "last_seen": now, "last_observation": None,
                     "confirmed": False, "tier": ("expected" if label.lower() in
                                                    self.priority_labels else "other"),
                     "static_landmark": label.lower() in self.static_labels,
                     "_evidence": [], "_validation_group": validation_group,
                     "_last_validation_observation": None, "_consecutive_misses": 0,
                     "_absence_misses": 0, "_last_absence_observation": None,
                     "_z_count": 0}
            self.next_id += 1
            self.tracks.append(track)
        if observation_id is not None and track["last_observation"] == observation_id:
            return track
        if track.get("confirmed"):
            # A confirmed landmark represents the validated session location.
            # Later weak/nearby detections may refresh its visibility but must
            # not drift its stored map coordinate.
            track["last_seen"] = now
            track["last_observation"] = observation_id
            track["confidence"] = max(float(confidence), track["confidence"] * .98)
            track["_absence_misses"] = 0
            return track
        old_x, old_y = track["x"], track["y"]
        n = track["count"] + 1
        track["x"] += (float(x) - track["x"]) / n
        track["y"] += (float(y) - track["y"]) / n
        track["m2"] += (float(x) - old_x) * (float(x) - track["x"]) + \
                       (float(y) - old_y) * (float(y) - track["y"])
        track["count"], track["last_seen"] = n, now
        if z is not None:
            z_count = track.get("_z_count", 0) + 1
            if track.get("z") is None:
                track["z"] = float(z)
            else:
                track["z"] += (float(z) - track["z"]) / z_count
            track["_z_count"] = z_count
        track["last_observation"] = observation_id
        track["confidence"] = max(float(confidence), track["confidence"] * .98)
        track["_absence_misses"] = 0
        if self.validation_enabled:
            track["_validation_group"] = validation_group
            track["_last_validation_observation"] = observation_id
            track["_consecutive_misses"] = 0
            track["_evidence"].append({
                "hit": True, "observation_id": observation_id,
                "confidence": float(confidence), "x": float(x), "y": float(y),
                "z": (float(z) if z is not None else None),
                "group": validation_group, "box": tuple(box) if box is not None else None,
            })
            window = int(self._policy(label)["window"])
            track["_evidence"] = track["_evidence"][-window:]
            confirmation_hits = self._confirmation_hits(track)
            if confirmation_hits is not None:
                xs = np.asarray([item["x"] for item in confirmation_hits], dtype=float)
                ys = np.asarray([item["y"] for item in confirmation_hits], dtype=float)
                track["x"], track["y"] = float(np.mean(xs)), float(np.mean(ys))
                track["count"] = len(confirmation_hits)
                track["m2"] = float(np.sum((xs - track["x"]) ** 2 +
                                           (ys - track["y"]) ** 2))
                track["confidence"] = max(item["confidence"]
                                          for item in confirmation_hits)
                zs = [item["z"] for item in confirmation_hits
                      if item.get("z") is not None]
                if zs:
                    track["z"] = float(np.mean(zs))
                track["confirmed"] = True
        else:
            track["confirmed"] = track["count"] >= self.min_observations
        return track

    def finish_observation(self, validation_group, observation_id, seen_track_ids,
                           eligible_track_ids=None, miss_limit=None):
        """Record pending evidence and remove repeatedly absent map tracks.

        Intermittent detectors commonly miss two adjacent frames even while an
        object remains visible. The rolling evidence window prevents stale hits
        from confirming a pending track, and normal max-age pruning removes
        abandoned candidates. Removal misses for pending and confirmed tracks
        must be prequalified by camera projection and depth visibility before
        they are supplied here.
        """
        if not self.validation_enabled:
            return []
        seen_track_ids = set(seen_track_ids)
        eligible_track_ids = set(eligible_track_ids or ())
        removed = []
        for track in self.tracks:
            static_confirmed = (track.get("confirmed", False) and
                                track.get("static_landmark", False))
            if track["id"] in seen_track_ids:
                track["_absence_misses"] = 0
                track["_last_absence_observation"] = observation_id
            elif (not static_confirmed and miss_limit is not None and
                  track["id"] in eligible_track_ids and
                  track.get("_last_absence_observation") != observation_id):
                track["_last_absence_observation"] = observation_id
                track["_absence_misses"] = track.get("_absence_misses", 0) + 1
                if track["_absence_misses"] >= int(miss_limit):
                    removed.append(track)
            if track.get("confirmed"):
                continue
            if (track["id"] in seen_track_ids or
                    track.get("_validation_group") != validation_group or
                    track.get("_last_validation_observation") == observation_id):
                continue
            track["_last_validation_observation"] = observation_id
            track["_consecutive_misses"] = track.get("_consecutive_misses", 0) + 1
            track["_evidence"].append({
                "hit": False, "observation_id": observation_id,
                "group": validation_group,
            })
            window = int(self._policy(track["label"])["window"])
            track["_evidence"] = track["_evidence"][-window:]
        if removed:
            removed_ids = {track["id"] for track in removed}
            # Automatic disappearance is not manual suppression: a returned
            # or moved object may form a fresh track later in the session.
            self.tracks = [track for track in self.tracks
                           if track["id"] not in removed_ids]
        return removed

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
        self.tracks = [t for t in self.tracks
                       if t.get("confirmed") or now - t["last_seen"] <= self.max_age]

    def snapshot(self, confirmed_only=True, now=None):
        self.prune(now)
        result = []
        for track in self.tracks:
            confirmed = track.get("confirmed", track["count"] >= self.min_observations)
            if confirmed_only and not confirmed:
                continue
            item = {key: value for key, value in track.items()
                    if not key.startswith("_")}
            item["confirmed"] = confirmed
            item["name"] = track.get("name", track["label"])
            item["position_std"] = math.sqrt(track["m2"] / max(1, track["count"] - 1))
            item["absence_misses"] = track.get("_absence_misses", 0)
            if self.validation_enabled:
                if confirmed:
                    validation = {"reason": "confirmed"}
                else:
                    _, validation = self._confirmation_evaluation(track)
                item["validation"] = validation
                item["validation_reason"] = validation["reason"]
            result.append(item)
        return result
