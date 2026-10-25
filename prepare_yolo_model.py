#!/usr/bin/env python3
"""Export a classroom-oriented Ultralytics checkpoint for robot deployment.

Run this on a workstation, not on the JetHexa. The robot only needs the ONNX
file and its generated ``.names`` sidecar; it does not need Ultralytics or
PyTorch. Existing models are never overwritten unless ``--force`` is passed.
"""

import argparse
import shutil
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="yolo26n-objv1-150.pt",
                        help="Ultralytics checkpoint name or local .pt path")
    parser.add_argument("--output", default="models/yolo26n-objv1-150.onnx",
                        help="destination ONNX path")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--opset", type=int, default=12,
                        help="conservative ONNX opset for older robot OpenCV builds")
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def ordered_names(names):
    if isinstance(names, dict):
        return [str(names[index]) for index in sorted(names)]
    return [str(name) for name in names]


def main():
    args = parse_args()
    output = Path(args.output).expanduser().resolve()
    labels = output.with_suffix(".names")
    for path in (output, labels):
        if path.exists() and not args.force:
            raise SystemExit("Refusing to overwrite %s (use --force)" % path)
    if args.imgsz < 256 or args.imgsz % 32:
        raise SystemExit("--imgsz must be at least 256 and divisible by 32")

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("Install the workstation exporter with: pip install ultralytics")

    model = YOLO(args.model)
    exported = Path(model.export(format="onnx", imgsz=args.imgsz,
                                 opset=args.opset, simplify=True,
                                 dynamic=False, nms=None)).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if exported != output:
        shutil.copy2(str(exported), str(output))
    with labels.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("\n".join(ordered_names(model.names)) + "\n")
    print("Prepared %s" % output)
    print("Labels: %s" % labels)
    print("Copy both files to the same paths on the robot; keep yolov5n.onnx for rollback.")


if __name__ == "__main__":
    main()
