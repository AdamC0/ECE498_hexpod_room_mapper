# Jetson Nano GPU YOLO Evaluation Plan

Date: 2026-10-25

Status: planning only. No robot files, models, settings, or processes were
changed for this plan.

## Goal

Determine whether the Jetson Nano GPU can run the deployed Objects365 YOLO
model faster than the current CPU path while preserving detection results and
leaving the mapper's sensor and controller performance healthy. Any test
integration must be optional and reversible. If the GPU path is slower than
the CPU baseline, less reliable, or harms sensor rates, stop the trial and
restore the CPU configuration and artifacts.

## Baseline and target findings

- The deployed `yolo26n-objv1-150.onnx` model runs through OpenCV DNN on CPU.
- The measured CPU inference time on PT was approximately 5.8-6.9 seconds per
  stationary frame. Use the same model, input size, frames, and output decoder
  for comparisons.
- PT has JetPack CUDA 10.2, cuDNN 8.2, TensorRT 8.2.1, TensorRT Python
  bindings, and PyCUDA installed.
- Its OpenCV 4.5.5 build reports zero CUDA devices and was built without the
  CUDA modules. OpenCV's CUDA DNN constants being present does not mean that
  CUDA execution is available; requesting the existing `cuda_fp16` backend
  falls back to CPU after warm-up failure.
- A TensorRT route is the primary experiment. Do not assume the current ONNX
  graph builds or that FP16 is faster on Nano's Maxwell GPU. First verify the
  installed TensorRT parser/tooling and supported operators. Record the exact
  JetPack, TensorRT, CUDA, cuDNN, engine precision, and model checksums.
- During the inspection, PT reported about 1 GB available memory. The Nano
  shares system memory with the GPU, so record memory pressure and temperature
  along with inference latency.

## Phase 1: Freeze a CPU reference

1. Keep the deployed ONNX model and `.names` sidecar unchanged and record their
   checksums. Preserve the existing CPU-capable model as the production
   fallback.
2. Collect a small, fixed set of representative RGB frames from the robot,
   including empty scenes and labeled classroom objects. Do not use a changing
   camera stream to compare model output.
3. Run the current CPU backend on every frame. Record warm-up time, per-frame
   latency, median and 90th-percentile latency, detections, labels, confidence,
   boxes, memory use, and CPU load. Repeat enough times to separate first-run
   setup from steady-state inference.
4. Record the baseline ROS rates for `/scan`, `/map`, `/slam_out_pose`, RGB,
   native depth, and registered depth with the normal paused mapper active.

## Phase 2: Build an isolated GPU candidate

1. Keep the robot stationary and autonomous motion paused. Use a standalone
   benchmark process with `_drive:=false` or no mapper connection; never use a
   drive-enabled trial to benchmark inference.
2. Build a TensorRT engine from the unchanged ONNX model in a separate trial
   directory such as `models/gpu_trial/`. Never overwrite the production ONNX,
   labels, CPU model, or current engine/artifacts.
3. Start with FP16 only if TensorRT supports the model and the Nano reports
   successful engine execution. Keep a separate FP32 option if needed to
   compare compatibility and output fidelity.
4. If engine generation fails, isolate the unsupported ONNX operation and
   document it. Do not simplify, quantize, or rewrite the production model
   without a separate comparison against the CPU reference.
5. Add any runtime selector as an explicit opt-in backend (for example,
   `tensorrt_fp16`) with CPU as the default and automatic fallback if engine
   load, warm-up, or inference fails. Keep TensorRT-specific dependencies and
   engine paths out of the standard CPU startup path.

## Phase 3: Compare output and performance

Run CPU and GPU on the exact same saved frames and compare:

- Warm-up and steady-state inference time: median, 90th percentile, and worst
  observed time.
- Detection parity: matched labels, box overlap, confidence differences,
  missed detections, and additional detections. Report empty-scene false
  positives separately from labeled-object recall.
- GPU/CPU utilization, memory use, temperature, throttling, and failures using
  available Jetson telemetry such as `tegrastats`.
- With the mapper paused and drive disabled, sensor rates and endpoint capture
  behavior while GPU inference runs. The CPU baseline rates must not regress
  materially, and ROS callbacks must remain responsive.

Do not claim success from GPU utilization alone. The GPU path passes only if it
is reliably faster than the 5.8-6.9 s CPU baseline on the same workload,
preserves acceptable detection parity, and does not cause memory pressure,
thermal throttling, inference failures, or unhealthy ROS sensor rates. If it is
slower than the non-GPU run or degrades any of those behaviors, reject it and
restore CPU operation.

## Reversibility and rollback

- Keep all TensorRT engines, calibration data, and trial configuration in a
  separate, removable GPU-trial directory. Never replace the CPU ONNX model or
  its labels.
- Keep the default backend set to CPU. Enable the GPU candidate only with an
  explicit runtime parameter during the paused trial.
- If code changes are needed, commit them separately from model artifacts and
  keep the CPU route covered by startup tests. Roll back code by reverting the
  GPU-backend commit; select CPU explicitly when restarting the mapper.
- If performance is worse than the non-GPU test, stop the benchmark, terminate
  the GPU process, restart the standard CPU mapper in `AUTO PAUSED` mode, and
  confirm the status reports `(cpu)`. Remove only the isolated trial engine
  after confirming its exact path; keep the known-good ONNX and label files.
- Recheck CPU inference, ROS sensor rates, and mapper status after rollback.
  Do not resume autonomy as part of benchmarking or rollback.

## Gate before physical use

Require a successful isolated benchmark, output comparison, failure-to-CPU
fallback check, and paused/no-drive live soak first. If these pass, retain GPU
execution as opt-in until its operation has also been observed during normal
stationary surveys. Any physical movement remains a separate directly
supervised test; GPU benchmarking itself must not command movement.

## Decision record

The robot has a usable Jetson GPU software stack, but the deployed OpenCV
runtime cannot access it. TensorRT is the candidate because its runtime is
installed. Whether this model builds, whether FP16 helps on Nano, and whether
the resulting detector preserves CPU output remain untested. The current CPU
path remains the baseline and rollback target until measured results justify
otherwise.
