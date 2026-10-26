#!/usr/bin/env python3
"""Validate a detector artifact with the robot's lightweight OpenCV runner."""

import argparse
import os
import time

import cv2
import numpy as np

from object_mapping import TensorRtYoloDetector, YoloOnnxDetector, load_labels


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model")
    parser.add_argument("--labels", default="")
    parser.add_argument("--backend", choices=("cpu", "cuda_fp16", "tensorrt_fp16"),
                        default="cpu")
    parser.add_argument("--engine", default="",
                        help="required checksum-bound engine for tensorrt_fp16")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--list-layers", action="store_true")
    parser.add_argument("--probe-layer", type=int, default=-1,
                        help="forward only through this zero-based OpenCV layer index")
    args = parser.parse_args()

    labels_path = args.labels or os.path.splitext(args.model)[0] + ".names"
    labels = load_labels(labels_path) if os.path.isfile(labels_path) else None
    if args.backend == "tensorrt_fp16":
        if not args.engine:
            parser.error("--engine is required for tensorrt_fp16")
        detector = TensorRtYoloDetector(
            args.model, args.engine, labels=labels, input_size=args.imgsz)
    else:
        detector = YoloOnnxDetector(
            args.model, labels=labels, input_size=args.imgsz, backend=args.backend)
    image = np.zeros((args.imgsz, args.imgsz, 3), np.uint8)
    if (args.list_layers or args.probe_layer >= 0) and args.backend == "tensorrt_fp16":
        parser.error("layer listing/probing is available only for OpenCV backends")
    layer_names = detector.net.getLayerNames() if hasattr(detector, "net") else []
    if args.list_layers:
        for index, name in enumerate(layer_names):
            print("%d\t%s" % (index, name))
        return
    if args.probe_layer >= 0:
        if args.probe_layer >= len(layer_names):
            raise SystemExit("layer index %d is outside 0..%d" %
                             (args.probe_layer, len(layer_names) - 1))
        blob = cv2.dnn.blobFromImage(image, 1.0 / 255.0,
                                     (args.imgsz, args.imgsz), swapRB=True)
        detector.net.setInput(blob)
        started = time.time()
        output = detector.net.forward(layer_names[args.probe_layer])
        print("Layer %d/%d %s: %r in %.3f s" %
              (args.probe_layer, len(layer_names) - 1,
               layer_names[args.probe_layer], output.shape, time.time() - started))
        return
    started = time.time()
    detections = detector.detect(image)
    elapsed = time.time() - started
    print("Validated %s with %s labels on %s in %.3f s (%d blank detections)" %
          (args.model, len(detector.labels), args.backend, elapsed, len(detections)))


if __name__ == "__main__":
    main()
