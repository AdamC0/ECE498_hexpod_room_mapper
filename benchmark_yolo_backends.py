#!/usr/bin/env python3
"""Compare CPU and TensorRT YOLO on the exact same saved RGB frames."""

import argparse
import json
import os
import statistics
import time

import cv2

from object_mapping import TensorRtYoloDetector, YoloOnnxDetector, load_labels


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[int(round((len(ordered) - 1) * fraction))]


def normalized(detections):
    return sorted([{"label": item["label"],
                    "confidence": round(float(item["confidence"]), 6),
                    "box": [int(value) for value in item["box"]]}
                   for item in detections],
                  key=lambda item: (item["label"], item["box"], -item["confidence"]))


def box_iou(a, b):
    left, top = max(a[0], b[0]), max(a[1], b[1])
    right, bottom = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, right - left) * max(0, bottom - top)
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    union = area_a + area_b - intersection
    return float(intersection) / union if union else 0.0


def compare_results(cpu_results, gpu_results, min_iou=.95, max_confidence_delta=.03):
    """Match same-label detections and summarize expected FP16 numeric drift."""
    if len(cpu_results) != len(gpu_results):
        return {"passed": False, "reason": "different image counts"}
    worst_iou, worst_confidence = 1.0, 0.0
    for cpu_frame, gpu_frame in zip(cpu_results, gpu_results):
        if cpu_frame["image"] != gpu_frame["image"]:
            return {"passed": False, "reason": "image order differs"}
        remaining = list(gpu_frame["detections"])
        for expected in cpu_frame["detections"]:
            candidates = [(box_iou(expected["box"], item["box"]), index, item)
                          for index, item in enumerate(remaining)
                          if item["label"] == expected["label"]]
            if not candidates:
                return {"passed": False, "reason": "missing %s in %s" %
                        (expected["label"], cpu_frame["image"])}
            iou, index, actual = max(candidates)
            confidence_delta = abs(expected["confidence"] - actual["confidence"])
            worst_iou = min(worst_iou, iou)
            worst_confidence = max(worst_confidence, confidence_delta)
            if iou < min_iou or confidence_delta > max_confidence_delta:
                return {"passed": False, "reason": "numeric difference exceeds tolerance",
                        "minimum_iou": worst_iou,
                        "maximum_confidence_delta": worst_confidence}
            del remaining[index]
        if remaining:
            return {"passed": False, "reason": "GPU added detections in %s" %
                    cpu_frame["image"]}
    return {"passed": True, "minimum_iou": worst_iou,
            "maximum_confidence_delta": worst_confidence,
            "iou_threshold": min_iou,
            "confidence_delta_threshold": max_confidence_delta}


def run(detector, frames, repeats):
    started = time.time()
    detector.warmup()
    warmup = time.time() - started
    timings, results = [], []
    for _ in range(repeats):
        iteration = []
        for path, image in frames:
            started = time.time()
            detections = detector.detect(image)
            timings.append(time.time() - started)
            iteration.append({"image": path, "detections": normalized(detections)})
        results = iteration
    return {"warmup_seconds": warmup,
            "median_seconds": statistics.median(timings),
            "p90_seconds": percentile(timings, .90),
            "worst_seconds": max(timings),
            "samples": len(timings),
            "results": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model")
    parser.add_argument("images", nargs="+")
    parser.add_argument("--labels", default="")
    parser.add_argument("--engine", required=True)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    frames = []
    for path in args.images:
        image = cv2.imread(path)
        if image is None:
            parser.error("could not read image: " + path)
        frames.append((path, image))
    labels_path = args.labels or os.path.splitext(args.model)[0] + ".names"
    labels = load_labels(labels_path) if os.path.isfile(labels_path) else None

    cpu = YoloOnnxDetector(args.model, labels=labels, input_size=args.imgsz)
    report = {"model": os.path.abspath(args.model),
              "engine": os.path.abspath(args.engine),
              "images": [path for path, _ in frames],
              "cpu": run(cpu, frames, args.repeats)}
    del cpu
    gpu = TensorRtYoloDetector(args.model, args.engine, labels=labels,
                               input_size=args.imgsz)
    report["tensorrt_fp16"] = run(gpu, frames, args.repeats)
    report["detection_parity"] = compare_results(
        report["cpu"]["results"], report["tensorrt_fp16"]["results"])
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    print(rendered)


if __name__ == "__main__":
    main()
