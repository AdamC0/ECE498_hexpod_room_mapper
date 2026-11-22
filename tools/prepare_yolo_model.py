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


def legacy_opencv_broadcast_shape(value_shape, constant_shape):
    """Return a fully materialized constant shape for OpenCV 4.5.5 DNN."""
    value_shape = list(value_shape)
    constant_shape = list(constant_shape)
    if (len(value_shape) == 3 and len(constant_shape) == 2 and
            value_shape[0] == constant_shape[0] == 1 and
            value_shape[-1] == constant_shape[-1] and
            value_shape[1] > 1):
        return value_shape
    return None


def patch_legacy_opencv_broadcasts(path):
    """Make rank-mismatched Mul constants explicit for the robot's OpenCV."""
    import onnx
    import numpy as np
    from onnx import numpy_helper

    model = onnx.load(str(path))
    inferred = onnx.shape_inference.infer_shapes(model)

    def dimensions(value):
        tensor_shape = value.type.tensor_type.shape
        return [int(item.dim_value) for item in tensor_shape.dim]

    shapes = {}
    for value in list(inferred.graph.input) + list(inferred.graph.value_info):
        shapes[value.name] = dimensions(value)
    initializers = {item.name: item for item in model.graph.initializer}
    rewrites = []
    for node in model.graph.node:
        if node.op_type != "Mul" or len(node.input) != 2:
            continue
        for value_name, constant_name in (node.input, reversed(node.input)):
            constant = initializers.get(constant_name)
            if constant is None or value_name not in shapes:
                continue
            replacement = legacy_opencv_broadcast_shape(
                shapes[value_name], list(constant.dims))
            if replacement is None:
                continue
            previous = list(constant.dims)
            values = numpy_helper.to_array(constant).reshape(
                [1, 1, previous[-1]])
            expanded = np.broadcast_to(values, replacement).copy()
            constant.CopyFrom(numpy_helper.from_array(expanded, constant.name))
            rewrites.append((node.name, constant_name, previous, replacement))
            break
    onnx.checker.check_model(model)
    onnx.save(model, str(path))
    return rewrites


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
    rewrites = patch_legacy_opencv_broadcasts(output)
    if rewrites:
        print("Applied %d legacy OpenCV broadcast rewrite(s)" % len(rewrites))
    with labels.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("\n".join(ordered_names(model.names)) + "\n")
    print("Prepared %s" % output)
    print("Labels: %s" % labels)
    print("Copy both files to the same paths on the robot; keep yolov5n.onnx for rollback.")


if __name__ == "__main__":
    main()
