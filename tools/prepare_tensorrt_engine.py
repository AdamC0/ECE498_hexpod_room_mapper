#!/usr/bin/env python3
"""Build an isolated, checksum-bound TensorRT FP16 YOLO engine on the Jetson."""

import argparse
import hashlib
import json
import os
import platform
import tempfile
import time


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_engine(model_path, output_path, workspace_mb=256, force=False):
    import tensorrt as trt

    model_path = os.path.abspath(os.path.expanduser(model_path))
    output_path = os.path.abspath(os.path.expanduser(output_path))
    metadata_path = output_path + ".json"
    if not os.path.isfile(model_path):
        raise ValueError("ONNX model does not exist: " + model_path)
    if (os.path.exists(output_path) or os.path.exists(metadata_path)) and not force:
        raise ValueError("output exists; pass --force to replace the isolated trial engine")
    output_dir = os.path.dirname(output_path)
    if not output_dir:
        raise ValueError("engine output must have a parent directory")
    os.makedirs(output_dir, exist_ok=True)

    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    if not builder.platform_has_fast_fp16:
        raise RuntimeError("this TensorRT platform does not report fast FP16 support")
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    with open(model_path, "rb") as stream:
        parsed = parser.parse(stream.read())
    if not parsed:
        errors = [str(parser.get_error(index)) for index in range(parser.num_errors)]
        raise RuntimeError("TensorRT ONNX parse failed: " + " | ".join(errors))
    if network.num_inputs != 1 or network.num_outputs != 1:
        raise RuntimeError("YOLO graph must have exactly one input and one output")
    input_shape = tuple(network.get_input(0).shape)
    if any(int(value) <= 0 for value in input_shape):
        raise RuntimeError("YOLO graph must have a static input shape: %r" % (input_shape,))

    config = builder.create_builder_config()
    config.max_workspace_size = int(workspace_mb) * 1024 * 1024
    config.set_flag(trt.BuilderFlag.FP16)
    started = time.time()
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("TensorRT failed to build the engine")

    fd, temporary = tempfile.mkstemp(prefix=".engine-", dir=output_dir)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(bytes(serialized))
        metadata = {
            "format": 1,
            "precision": "fp16",
            "onnx_sha256": sha256(model_path),
            "engine_sha256": sha256(temporary),
            "input_shape": list(input_shape),
            "tensorrt_version": trt.__version__,
            "platform": platform.platform(),
            "workspace_mb": int(workspace_mb),
            "build_seconds": round(time.time() - started, 3),
        }
        metadata_temporary = temporary + ".json"
        with open(metadata_temporary, "w", encoding="utf-8") as stream:
            json.dump(metadata, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, output_path)
        os.replace(metadata_temporary, metadata_path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
        if os.path.exists(temporary + ".json"):
            os.unlink(temporary + ".json")
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model")
    parser.add_argument("--output", required=True)
    parser.add_argument("--workspace-mb", type=int, default=256)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if not 64 <= args.workspace_mb <= 1024:
        parser.error("--workspace-mb must be between 64 and 1024")
    metadata = build_engine(args.model, args.output, args.workspace_mb, args.force)
    print("Built %s in %.3f s (TensorRT %s, FP16, source %s)" %
          (os.path.abspath(os.path.expanduser(args.output)), metadata["build_seconds"],
           metadata["tensorrt_version"], metadata["onnx_sha256"]))


if __name__ == "__main__":
    main()
