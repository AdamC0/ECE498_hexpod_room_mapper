# JetHexa Room Mapper Repository Instructions

Use `README.md` for project setup and usage. Read `HANDOFF.md` before robot or
deployment work, `RUN_SHEET.txt` before live validation, and the newest relevant
`DECISIONS.md` entries before changing architecture, calibration, or safety
behavior. Keep documentation synchronized with deployed behavior.

## Version control

At the start of every agent session, before inspecting or changing project
files, run `git status --short --branch`, then update from GitHub with
`git pull --ff-only origin main`. If uncommitted work or branch divergence
prevents the fast-forward pull, do not stash, reset, overwrite, or discard the
local work. Report the conflict, fetch `origin/main`, and reconcile it safely
before continuing.

All completed project changes must be committed and pushed to the GitHub
repository at `https://github.com/AdamC0/ECE498_hexpod_room_mapper`. Before
committing, review the diff and repository status. Use a clear, accurate commit
message that describes the changes, and do not leave completed work only in the
local working tree.

Robot: `hiwonder@192.168.137.223`

Robot project: `/home/hiwonder/room_mapper`

Web UI: `http://192.168.137.223:8000`

Do not store or repeat the SSH password in project files. Real motion requires
direct supervision. Prefer `_drive:=true _start_paused:=true` and confirm only
one mapper process exists.

## Current architecture

- YDLIDAR G4 raw `/scan` feeds Hector, which owns pose and the persistent map.
- Registered depth adds short-lived low-obstacle cells to the planner copy; it
  never feeds Hector. `visual_slam.launch` is non-production and must remain
  drive-disabled if used for research comparison. Never run both SLAM stacks.
- `map_explore.py` plans through known free space using a 0.30 m radius and
  0.03 m margin.
- `room_mapper.py` handles bounded gait bursts, recovery, sensor safety,
  manual control, clicked routes, and autonomous object focus.
- `map_web.py` provides RGB, a robot-following zoomable map, click navigation,
  Cancel Route, manual W/A/S/D, independent 50-200% translation/rotation
  sliders, object management, and a battery badge.
- Object mapping uses YOLO, synchronized registered depth, camera intrinsics,
  and timestamped TF. Two observations confirm a landmark.

## Current safety/calibration facts

- LiDAR emergency stop: 0.45 m.
- Depth stop: 0.60 m.
- Close-depth exclusion: disabled (0.00 m); close/dropout returns remain stops.
- Depth validity: 200 pixels and 2% upper-band coverage.
- Depth planner layer: 0.15-2.0 m range, 0.025-0.50 m height, 30 s TTL.
- Brief sparsity: reuse last valid result for at most 1.5 s; sustained sparsity
  or no previous valid result still faults.
- Map pose jump: 0.25 m / 20 degrees.
- Turn progress: three consecutive bursts below 3 degrees invalidate SLAM.
- Object focus: 15 s maximum, two misses, then replan.

Depth still cannot guarantee detection of reflective, transparent, out-of-view,
or invalid geometry. Do not describe the system as fully safe.

## Hector/depth hybrid deployed (2026-10-19)

The first RTAB-Map motion pulse moved forward physically, but the operator
reported broken map/robot-position behavior relative to the original LiDAR
mapper. Hector is restored as the production localization and mapping path.
Registered depth is projected through timestamped TF into temporary planner
cells for low geometry, without modifying Hector's map or pose.

All 138 robot-native tests pass. A supervised 50% speed, 0.55 s forward pulse
on the hybrid stack dead-man stopped normally; Hector reported about 0.027 m
translation and 0.9 degrees rotation while `/map` remained live. The operator
confirmed correct motion and a substantially improved map.

## Historical visual/depth experiment

PT is online and the visual/depth candidate is deployed. All 131 tests pass on
the robot. RTAB-Map must consume `/visual_slam/odom_rgbd_image`; direct
RGB/depth/odom/scan synchronization did not advance the graph. Both RTAB nodes
clear private parameters on launch to prevent stale ROS settings. The installed
reset service is `/visual_slam/reset`. The mapper uses `_pose_type:=tf` with
`map` and `jethexa/base_link`, because `localization_pose` is not emitted in
mapping mode. Never run Hector and RTAB-Map together.

RTAB-Map publishes `/visual_map` only when the grid changes. For the visual
path, the mapper uses `/visual_slam/info` as its five-second mapping-process
heartbeat; Hector continues to use `/map` update freshness.

Before the motion regression was found, the stationary stack produced
visual odometry near 3.1 Hz, graph updates near 0.9 Hz, and a depth-aware
occupancy grid. A seven-minute load check completed; reported pose moved about
0.02 m and 0.1 degrees during the mapper's three-minute sample. A later tall-box
check aligned depth and mapped face cells at reported 1.00 m and 0.75 m
placements. Treat low-obstacle alignment, clicked-route behavior, and all
physical motion as pending supervised tests. The battery was subsequently
recharged; the latest value is recorded below.

## Latest validation

PT was reachable on the latest 2026-10-19 check. RTAB-Map is stopped; Hector,
RGB/depth/LiDAR, TF, `/slam_out_pose`, `/map`, the temporary depth planner
layer, and the web UI are live. The full native suite passes 138 tests. Exactly
one drive-enabled mapper is `AUTO PAUSED`; the operator confirmed that two
observed resumptions were caused by pressing `RESUME AUTO`. Native depth is
upper-only, while the independent registered-depth planner overlay is active.

`/voltage` is `std_msgs/Float32`. The dedicated vendor `battery_low_alarm` node
is persistently excluded from both controller launch variants; launch backups
are stored beside them. Do not disable voltage telemetry or the website's
visual low warning.

After a reboot, RGB/depth may recover but LiDAR may require:

```bash
rosservice call /start_scan '{}'
```

If RGB has a publisher but no frames, run `recover_rgb_camera.sh` while autonomy
is paused, then verify actual RGB messages. This recovery has restored RGB
without interrupting depth, Hector, or the mapper. If it fails, reboot the
robot and restore LiDAR, Hector, and the mapper. If SLAM jumps or drifts while
stationary, discard the map and start a fresh Hector frame.

Always separate unit-test success, no-drive live evidence, SLAM-reported motion,
and physically observed robot motion in reports.
