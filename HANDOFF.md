# JetHexa Room Mapper - Agent Handoff

Updated: 2026-10-19

## Live checkpoint

The robot was reachable again on the latest continuation attempt (2026-10-19).
LiDAR was restarted with `/start_scan`, and RGB, depth, LiDAR, RTAB-Map, TF,
the occupancy grid, and the web UI were verified live. One mapper was restored
with `_drive:=false _start_paused:=true`; no motion was commanded. Its status
was localized and AUTO PAUSED with depth ready in upper-only mode, 0.97 m LiDAR
clearance, and 12.307 V battery telemetry (about 89.5%). RTAB-Map graph updates
were about 1 Hz. No Hector process was running.

The subsequent `map inactive` fault was traced to `/visual_map` being
event-driven: its valid 39 x 45 grid stopped republishing while the robot was
stationary even though `/visual_slam/info` remained near 0.84 Hz and visual
odometry near 4 Hz. The deployed mapper now treats `/visual_slam/info` as the
visual path's five-second liveness heartbeat. After waiting beyond the former
timeout, the mapper remained localized and AUTO PAUSED. All 131 tests pass on
the robot. The mapper remains drive-disabled; no motion was commanded.

Last-known box-validation checkpoint (2026-10-19): RTAB-Map and one mapper were
left running with `_drive:=false _start_paused:=true`, using the separate
`/tmp/room_mapper_box_validation.db` database. Latest sampled voltage: 10.069 V.
The operator's 0.3683 m wide, 0.5588 m tall box was checked at 1.00 m and
0.75 m from body center. At 0.75 m, depth plus the configured camera offset
gave 0.757 m; mapped face cells were around 0.79-0.80 m. Old occupied cells
remained at the previous position. See the newest DECISIONS.md entry for
measurements and limitations. Low-obstacle and motion validation remain pending.
The current dry-run stack uses `/tmp/room_mapper_continue.db`, not the earlier
box-validation database.

Robot: `hiwonder@192.168.137.223`

Robot project: `/home/hiwonder/room_mapper`

Web UI: `http://192.168.137.223:8000`

The SSH password was supplied interactively and must not be written into the
repository or documentation.

PT returned during the 2026-10-19 session. The visual/depth SLAM, navigation,
manual-control, and speed changes are deployed. The native suite passes all
131 tests, but no physical walking validation has occurred. After recharge and
a full reboot, the visual stack and mapper completed more than five stationary
minutes in DRY RUN / AUTO PAUSED without a controller fault. A later measured
box-alignment check left one drive-disabled mapper and RTAB-Map running. The
latest observed battery was 10.069 V; recharge before motion testing.

The battery dashboard is deployed. `/voltage` is `std_msgs/Float32`; percentage
is estimated between the vendor's 9.8 V alarm cutoff and 12.6 V full. The
dedicated audible `battery_low_alarm` include is disabled in both controller
launch variants while voltage telemetry remains active. Backup launch files use
the suffix `.pre_room_mapper_alarm_disable_20261012`.

## Deployed architecture

- Raw `/scan` feeds either Hector or RTAB-Map; never run both SLAM stacks.
- The visual path bundles registered RGB-D with visual odometry, fuses LiDAR
  in RTAB-Map, and publishes the depth-aware `/visual_map` grid.
- Visual mapping pose comes from TF (`map -> visual_odom -> jethexa/base_link`).
- Frontier and clicked routes use a 0.30 m circular footprint plus 0.03 m
  planning margin.
- Motion uses bounded bursts, a 0.65 s settle, and stationary survey checks.
- Depth is the forward primary signal; LiDAR is an independent 0.45 m
  emergency fallback and supplies rear/rotation clearance.
- Web UI provides robot-following zoomable/pannable map, click navigation,
  route cancellation, manual driving, RGB, telemetry, and object management.
- Independent 50-200% translation and rotation sliders scale manual and
  autonomous gait output without changing any sensor or planning threshold.
- The web UI includes a header battery percentage badge with voltage and
  charging details plus clear stale/unavailable states.
- YOLO landmarks require two distinct observations. Autonomous exploration
  holds in `object_focus` until confirmation, two misses, or a 15 s timeout.

## Visual/depth update (deployed; motion validation pending)

- Removed the 0.50 m close-depth exclusion and added directional
  left/center/right depth clearance.
- Recovery follows the selected route's turn direction when that side remains
  clear, then falls back to combined depth/LiDAR side clearance.
- Map JPEG responses carry their rendered center/scale; browser clicks return
  that snapshot, preventing pose drift between render and click conversion.
- Removed the click-to-navigate confirmation dialog.
- Manual commands directly reach the gait client while held and bypass all
  autonomous sensing/planning/burst restrictions. Release/blur and the 0.55 s
  server dead-man remain active.
- Slider range is 50-200% locally.
- Added `visual_slam.launch`, an RTAB-Map replacement for Hector that uses RGB-D
  visual odometry and loop closure, LiDAR registration, and depth projection
  for the planner occupancy grid. Do not run both SLAM stacks together.
- Added TF pose/map frame/reset service support so the controller consumes
  RTAB-Map's `map -> base_link` transform and `/visual_map`.

All 131 tests pass on the robot. Visual odometry ran near 3.1 Hz, RTAB graph
updates near 0.9 Hz, and the grid service returned an 80 x 86 map at 0.05 m.
Low-obstacle alignment, clicked routing, and supervised physical motion remain
pending. The measured tall-box result is recorded in the live checkpoint above.

After the UVC stall and low-battery stop, a full reboot and recharge restored
RGB. A stationary retry kept RGB near 29.8 Hz, visual odometry near 2.4 Hz, and
graph updates near 0.65 Hz under the full mapper load. Approximate CPU use was
94% for visual odometry, 17% for RTAB-Map, and 66% for the mapper. The grid
service contained 71 free and 76 occupied cells, TF remained localized, and
the web server rendered a 9.6 KB map JPEG. No motion command occurred.

The stack ran stationary for more than seven minutes. `rgbd_odometry` used
about 100% of one CPU core and 189 MB RSS; RTAB-Map used about 16% and 221 MB;
the dry-run mapper used about 63% and 193 MB. Over the mapper's three-minute
sample, reported pose changed about 0.02 m and 0.1 degrees. The map contained
67 free and 79 occupied cells, the web map returned HTTP 200, and the speed
endpoint reported a 50-200% range. Exactly one mapper process was present.

## Important current calibration

- `body_radius = 0.30 m`
- planner margin `0.03 m`; total mapped clearance `0.33 m`
- `waypoint_tolerance = 0.12 m`; thinned waypoint spacing about `0.20 m`
- `lidar_stop = body_radius + 0.15 = 0.45 m`
- `depth_stop = 0.60 m`
- close-depth exclusion is disabled (`0.00 m`)
- upper-band depth validity: at least 200 pixels and 2% of the band
- transient sparse-depth grace: recent result for at most 1.5 s
- object confirmation: 2 observations
- object focus: 15 s or 2 distinct misses
- weak turn tolerance: fault after 3 consecutive bursts below 3 degrees
- pose jump guard: 0.25 m / 20 degrees

Depth cannot guarantee detection of reflective, transparent, out-of-view, or
invalid geometry. LiDAR independently protects its scan plane at 0.45 m.

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
- Applied, then removed after diagnosis, the `<= 0.50 m` depth exclusion.
- Replaced visual-grid publication freshness with RTAB graph-heartbeat
  freshness so an unchanged stationary map remains active.

The deployed suite passes all 131 tests on the robot.

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

For the RTAB-Map path, follow README.md and RUN_SHEET.txt. Start it instead of
`hex_map.launch`; use `_map_topic:=/visual_map _pose_type:=tf _map_frame:=map
_pose_frame:=jethexa/base_link`. Keep `_drive:=false` until the pending physical
checks are complete.

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
