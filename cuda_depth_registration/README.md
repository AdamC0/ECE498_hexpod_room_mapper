# CUDA depth registration

This catkin package projects the Astra native `16UC1` depth image into the raw
RGB pixel grid on the Jetson Nano GPU. The safe room-mapper launcher uses
`/camera/depth_registered_cuda/image_raw` and its matching camera info by
default while preserving native depth for the independent collision check.
The legacy driver stream remains available through the launcher's explicit
`--registered-depth-backend driver` rollback option.

The node obtains the camera's factory right-to-left rotation and translation
from `/camera/get_camera_params`, combines those extrinsics with the live depth
and RGB `CameraInfo`, and uses a CUDA `atomicMin` z-buffer so the nearest surface
wins when multiple depth pixels project to one RGB pixel. Output keeps the
native depth timestamp, uses the RGB optical frame, and is published as
`16UC1` millimetres with matching RGB camera information.
One-pixel projection holes are filled only when at least two four-neighbours
agree within 50 mm; the nearer value wins to avoid weakening foreground depth.

## Target build

Copy or link this directory into the JetHexa catkin workspace, then build only
the package and its dependencies:

```bash
cd ~/jethexa
source /opt/ros/melodic/setup.bash
catkin build cuda_depth_registration
source devel/setup.bash
```

The checked-in CUDA target is `sm_53`, matching the Jetson Nano's Maxwell GPU
and its installed CUDA 10.2 toolchain.

## Standalone launch

Keep autonomy paused or leave the mapper stopped. Start the isolated output:

```bash
source ~/jethexa/devel/setup.bash
roslaunch cuda_depth_registration cuda_depth_registration.launch
```

Verify the candidate without changing the mapper:

```bash
rostopic hz /camera/depth_registered_cuda/image_raw
rostopic echo -n 1 /camera/depth_registered_cuda/image_raw/header
```

`compare_registration.py` accepts either `16UC1` or `32FC1` reference depth and
normalizes both to millimetres. Use it only when the reference is known to be
registered and acquired from an equivalent stationary scene:

```bash
rosrun cuda_depth_registration compare_registration.py _frames:=30
```

For an equivalent-algorithm speed comparison on one captured live frame, run:

```bash
rosrun cuda_depth_registration cuda_depth_registration_benchmark \
  _iterations:=100 _warmup_iterations:=10
```

The benchmark runs the same transform, distortion, nearest-depth z-buffer, and
hole-fill policy on one CPU core and CUDA. It reports CPU, CUDA end-to-end
(including host/device copies), CUDA kernel-only timing, speedups, and output
parity. It publishes no depth topic and never commands the robot.

On PT, three 640x480 runs with 10 warmups and 100 measured iterations produced
29.09-29.61 ms CPU medians, 1.65-1.68 ms CUDA end-to-end medians, and
0.70-0.71 ms CUDA kernel medians. The end-to-end speedup was 17.36-17.84x.
CPU and CUDA outputs matched exactly: valid-mask IoU 1.000 and median/p95 depth
error 0 mm in every run.

The production mapper now defaults to this CUDA topic and projects geometric
planner obstacles at 5 Hz. Native depth must remain available for collision
sensing. GPU registration does not improve the camera's reflective,
transparent, out-of-view, or invalid-depth limitations; supervised motion,
labeled alignment, load/thermal, and failure-mode validation remain required.
