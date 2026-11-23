# CUDA Depth Registration

The CUDA depth pipeline was added to align the Astra camera's native depth image
with the RGB image without giving up the independent native-depth stream used by
the robot's immediate collision check. The original driver configuration did
not provide true RGB-aligned depth, and enabling the camera's hardware
registration would have removed that independent safety stream. Instead, a
standalone ROS/CUDA node was implemented for the Jetson Nano's `sm_53` GPU and
CUDA 10.2.

For each native `16UC1` depth pixel, the node reconstructs its 3-D position from
the depth-camera intrinsics, transforms the point into the RGB camera frame
using the Astra factory calibration, applies the RGB lens distortion model, and
projects the result onto the RGB pixel grid. A CUDA `atomicMin` z-buffer keeps
the nearest surface when multiple points land on the same pixel. A conservative
hole-fill pass only fills a missing pixel when at least two neighboring depths
agree within 50 mm. The result is published in millimetres under
`/camera/depth_registered_cuda`, with the original timestamp and matching RGB
camera information.

The CUDA output is now the mapper's default source for geometric obstacle
insertion, while native depth remains separate for the 0.60 m collision stop.
The safe launcher verifies the CUDA image and camera-information streams before
starting the mapper and fails closed if they are unavailable. The older driver
path remains available as an explicit rollback.

## Measured speedup

The benchmark compared identical CPU and CUDA implementations on the same
captured 640 x 480 frame. Each of three runs used 10 warm-up iterations and 100
measured iterations.

| Implementation | Median time |
| --- | ---: |
| Single-core CPU (`-O3`) | 29.09-29.61 ms |
| CUDA, end to end including memory copies | 1.65-1.68 ms |
| CUDA kernels only | 0.70-0.71 ms |

This corresponds to a **17.36-17.84x end-to-end speedup**, or about **41x** when
comparing the CPU time with kernel execution alone. The CPU and CUDA results
matched exactly in these tests: valid-mask IoU was 1.000, and median and 95th
percentile depth error were both 0 mm. In the recorded stationary trial, the
CUDA node sustained approximately 15.6-15.9 Hz with about 36 MB RSS and 5.4%
CPU usage.

The faster registration also allowed planner projection to increase from 2 Hz
to 5 Hz, reducing the scheduling interval for inserting a new temporary
blocking cell from roughly 0.5 s to 0.2 s. This is an algorithmic and stationary
pipeline result, not a claim of completed motion or safety validation. No new
robot test was performed for this writeup because the robot was unavailable;
supervised obstacle-motion, extended load/thermal, and failure-mode validation
remain pending.
