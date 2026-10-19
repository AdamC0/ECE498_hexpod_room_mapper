# JetHexa Room Mapper - Agent Handoff

Updated: 2026-10-12

## Live checkpoint

Robot: `hiwonder@192.168.137.223`

Robot project: `/home/hiwonder/room_mapper`

Web UI: `http://192.168.137.223:8000`

The SSH password was supplied interactively and must not be written into the
repository or documentation.

The robot is online and `AUTO PAUSED`. RGB, depth, LiDAR, Hector, mapping,
`/voltage`, and the web UI are live. The battery endpoint initially reported
10.328 V / 18.9%. Keep the robot paused until supervised operation resumes.

The battery dashboard is deployed. `/voltage` is `std_msgs/Float32`; percentage
is estimated between the vendor's 9.8 V alarm cutoff and 12.6 V full. The
dedicated audible `battery_low_alarm` include is disabled in both controller
launch variants while voltage telemetry remains active. Backup launch files use
the suffix `.pre_room_mapper_alarm_disable_20261012`.

## Deployed architecture

- Raw `/scan` feeds Hector continuously; `/scan_stable` is diagnostic.
- Frontier and clicked routes use a 0.30 m circular footprint plus 0.03 m
  planning margin.
- Motion uses bounded bursts, a 0.65 s settle, and stationary survey checks.
- Depth is the forward primary signal; LiDAR is an independent 0.45 m
  emergency fallback and supplies rear/rotation clearance.
- Web UI provides robot-following zoomable/pannable map, click navigation,
  route cancellation, manual driving, RGB, telemetry, and object management.
- Independent 50-150% translation and rotation sliders scale manual and
  autonomous gait output without changing any sensor or planning threshold.
- The web UI includes a header battery percentage badge with voltage and
  charging details plus clear stale/unavailable states.
- YOLO landmarks require two distinct observations. Autonomous exploration
  holds in `object_focus` until confirmation, two misses, or a 15 s timeout.

## Important current calibration

- `body_radius = 0.30 m`
- planner margin `0.03 m`; total mapped clearance `0.33 m`
- `waypoint_tolerance = 0.12 m`; thinned waypoint spacing about `0.20 m`
- `lidar_stop = body_radius + 0.15 = 0.45 m`
- `depth_stop = 0.60 m`
- depth values `<= 0.50 m` are deliberately ignored
- upper-band depth validity: at least 200 pixels and 2% of the band
- transient sparse-depth grace: recent result for at most 1.5 s
- object confirmation: 2 observations
- object focus: 15 s or 2 distinct misses
- weak turn tolerance: fault after 3 consecutive bursts below 3 degrees
- pose jump guard: 0.25 m / 20 degrees

Ignoring depth at or below 0.50 m is an operator-requested tradeoff. It also
ignores floor/dropout alarms at those distances. LiDAR still stops at 0.45 m,
but close elevated objects outside the LiDAR plane can be missed.

## Session work completed

- Restored mapping, RGB, depth, LiDAR, and web services after robot reboots.
- Added telemetry reason text so SLAM and sensor faults are visible.
- Added blocked-route rotation and rearward recovery for mapping and routes.
- Added click-to-navigate, route cancellation, and an Objects tab with
  navigate/rename/remove actions.
- Added responsive manual controls usable while autonomy is paused.
- Added robot-centered moving map viewport, zoom, pan, Center reset, and
  transform-correct map clicking.
- Increased footprint from 20 cm to 30 cm radius and tightened path waypoint
  following to avoid leg corner collisions.
- Changed turn-progress validation from one failed burst to three consecutive
  failures to avoid false SLAM latches.
- Reduced object confirmation from three observations to two.
- Added autonomous object focus/confirm/exclude/replan behavior.
- Reduced sparse-depth coverage gate from 10% to 2% with a 200-pixel minimum,
  then added a 1.5 s last-valid-result grace for brief post-motion sparsity.
- Applied the requested `<= 0.50 m` depth-result exclusion.

The deployed suite passed all 122 tests on the robot after the speed-slider update.

## Restart procedure

After a full robot reboot, camera/depth normally return automatically. LiDAR
may advertise `/scan` without emitting messages; call:

```bash
source /opt/ros/melodic/setup.bash
rosservice call /start_scan '{}'
```

Then start Hector and the mapper:

```bash
roslaunch ~/room_mapper/hex_map.launch

cd ~/room_mapper
python3 room_mapper.py \
  _drive:=true _start_paused:=true _reset_map:=false \
  _detect_objects:=true _use_depth:=true
```

Use `_reset_map:=false` only when Hector's current frame is still trustworthy.
Reset the map after a Hector restart, pose jump, or sustained stationary drift.

## Verification commands

```bash
curl -fsS http://127.0.0.1:8000/status
curl -fsS http://127.0.0.1:8000/battery.json
pgrep -af '^/usr/bin/python3 room_mapper.py'
rostopic hz /scan /slam_out_pose /camera/rgb/image_raw /camera/depth/image_raw
python3 -m unittest discover -v
```

Only one mapper process should exist. Tests do not command the physical robot.

## Known limitations

- Hector has no useful odometry/loop closure in this installation and can
  drift or fold the map. Never treat SLAM coordinates alone as proof of motion.
- RGB/UVC has stalled while its ROS publisher remained registered. A full
  robot reboot restored it when in-process restarts did not.
- Floor calibration often falls back to upper-only mode, so floor bumps may
  be unavailable.
- A 30 Hz depth topic can still contain unusable pixels. Brief sparsity uses a
  1.5 s cached decision; sustained sparsity still faults. Inspect valid pixel
  coverage, not only topic rate.
- Rebooting or resetting Hector loses the in-memory map. Object landmarks are
  cleared when map trust is lost or the map is reset.
