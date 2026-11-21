# CUDA depth registration trial

This catkin package projects the Astra native `16UC1` depth image into the raw
RGB pixel grid on the Jetson Nano GPU. It is deliberately a side-by-side trial:
the default output is `/camera/depth_registered_cuda/image_raw`, and it never
remaps the mapper or replaces `/camera/depth_registered/image_raw`.

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

## Stationary trial

Keep autonomy paused or leave the mapper stopped. Start the isolated output:

```bash
source ~/jethexa/devel/setup.bash
roslaunch cuda_depth_registration cuda_depth_trial.launch
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

Do not point `room_mapper.py` at the CUDA topic until timestamp, calibration,
valid-mask, depth-error, labeled RGB alignment, load/thermal, and failure-mode
gates pass. Native depth must remain available for collision sensing. GPU
registration does not improve the camera's reflective, transparent,
out-of-view, or invalid-depth limitations.
