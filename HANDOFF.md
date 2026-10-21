# JetHexa Room Mapper - Agent Handoff

Updated: 2026-10-20

## Live checkpoint

Repository-only update on 2026-10-20: active paths now use the vendor's
body-relative direction command for continuous crab steering in a conservative
±45 degree forward envelope. Manual control adds true lateral strafe on A/D
and keeps rotation on Q/E or the arrow keys. Directional swept-body LiDAR is
checked for crab commands. Five regression cases were added, bringing the
suite to 150. This workstation lacks ROS and noninteractive SSH credentials,
so only ROS-free tests and syntax checks can be run here; PT deployment,
robot-native tests, and physical motion validation remain pending. The live
checkpoint below is the last verified robot state, not evidence that this new
revision is deployed.

The production stack is now Hector plus a registered-depth planner overlay.
The first RTAB-Map motion pulse physically moved forward correctly, but the
operator reported that its map/robot-position behavior was broken relative to
the original LiDAR mapper. RTAB-Map was stopped and Hector was restored as the
sole authority for `/slam_out_pose`, `/map`, and the robot marker.

Registered depth is now projected through timestamped TF into short-lived
planner cells for clustered geometry 0.025-0.50 m above the floor and 0.15-2 m
from the camera. These cells expire after 30 s and never feed Hector. The
persistent room map and localization therefore remain LiDAR-based while low
objects can block planned routes.

All 145 tests pass natively on the robot. One supervised 50% speed, 0.55 s
forward pulse on the hybrid stack completed and dead-man stopped. Hector pose
changed coherently from about `(0.000, 0.006)` to `(0.027, 0.004)` m with about
0.9 degrees heading change; `/map` stayed active at roughly 4% known. The
operator confirmed that it physically moved correctly and that the map looked
much better. One drive-enabled mapper is left `AUTO PAUSED`; no autonomy-resume
or additional movement command was sent.

During the latest deployment check, the mapper was twice found outside the
paused state and was paused immediately. The operator later confirmed pressing
`RESUME AUTO` both times. It is now restarted drive-enabled with
`_start_paused:=true`, `AUTO PAUSED`, and command `stop`. SLAM logged
translation, turn scans, and changing headings, but no physical observer
separately confirmed motion during those checks. Inspect the map frame before
the next autonomous run.

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

The earlier RTAB-Map stationary and tall-box measurements remain useful
research evidence but are superseded as the deployed navigation architecture.
They are retained in `DECISIONS.md`; they do not establish physical low-object
safety or justify using RTAB-Map for motion.

The battery dashboard is deployed. `/voltage` is `std_msgs/Float32`; percentage
is estimated between the vendor's 9.8 V alarm cutoff and 12.6 V full. The
dedicated audible `battery_low_alarm` include is disabled in both controller
launch variants while voltage telemetry remains active. Backup launch files use
the suffix `.pre_room_mapper_alarm_disable_20261012`.

## Deployed architecture

- Raw `/scan` feeds Hector, which owns localization and the persistent map.
- Registered depth adds temporary low-obstacle cells only to the planner copy;
  it does not alter Hector pose or `/map`.
- Frontier and clicked routes use a 0.30 m circular footprint plus 0.03 m
  planning margin.
- Active paths use continuous translation and turning, with heading progress
  checked during turns. Recovery retains bounded motion and settling.
- The repository can crab-steer active paths up to 45 degrees and provides
  supervised manual left/right strafe; this item is pending deployment.
- Depth is the forward primary signal; LiDAR is an independent 0.45 m
  emergency fallback and supplies rear/rotation clearance.
- Web UI provides robot-following zoomable/pannable map, click navigation,
  route cancellation, manual driving, RGB, telemetry, and object management.
- Independent 50-200% translation and rotation sliders scale manual and
  autonomous gait output without changing any sensor or planning threshold.
- The web UI includes a header battery percentage badge with voltage and
  charging details plus clear stale/unavailable states.
- YOLO landmarks require two distinct observations. Stationary exploration surveys
  holds in `object_focus` until confirmation, two misses, or a 15 s timeout.
- Active paths do not wait for YOLO or enter object focus. Object mapping still
  requires stationary observations; obstacles are handled by depth and LiDAR.

## Depth/navigation update

- Removed the 0.50 m close-depth exclusion and added directional
  left/center/right depth clearance.
- Recovery follows the selected route's turn direction when that side remains
  clear, then falls back to combined depth/LiDAR side clearance.
- A front obstacle that persists through 2.5 seconds of turning now escalates
  to a one-second reverse escape when rear LiDAR clearance is at least 0.65 m.
  Floor-bump and depth-dropout alarms still prohibit autonomous reversing.
- Map JPEG responses carry their rendered center/scale; browser clicks return
  that snapshot, preventing pose drift between render and click conversion.
- Removed the click-to-navigate confirmation dialog.
- Manual commands directly reach the gait client while held and bypass all
  autonomous sensing/planning/burst restrictions. Release/blur and the 0.55 s
  server dead-man remain active.
- Slider range is 50-200% locally.
- Added a Hector-frame registered-depth obstacle layer with 30 s expiry. This
  retains the original LiDAR mapping behavior while making clustered low
  geometry available to route inflation. The web map renders depth-only cells
  in magenta and LiDAR occupancy in black.
- `visual_slam.launch` remains for non-production experiments only. Never run
  it together with Hector.

Historical RTAB measurements: visual odometry ran near 3.1 Hz, RTAB graph
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
- depth planner layer: 0.15-2.0 m range, 0.025-0.50 m height, 30 s TTL
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
- Restored Hector as the production mapping/localization authority and added a
  separate, temporary registered-depth planner layer.

The deployed suite passes all 145 tests on the robot.

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
  _detect_objects:=true _use_depth:=true _use_depth_map:=true
```

Use `_reset_map:=false` only when Hector's current frame is still trustworthy.
Reset the map after a Hector restart, pose jump, or sustained stationary drift.

The RTAB-Map path is non-production and must remain drive-disabled if used for
research comparison. Start it instead of, never alongside, `hex_map.launch`.

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
- RGB/UVC has stalled while its ROS publisher remained registered.
  The web UI's `RECOVER CAMERA` button runs `recover_rgb_camera.sh` only while
  autonomy is paused. The script provides an in-place recovery: it restarts only
  `/camera/uvc_color` and re-authorizes the Orbbec RGB USB function
  (`2bc5:0559`, requires sudo) while depth, Hector, and the mapper stay up. A
  live stalled-camera check restored RGB to about 29-31 Hz while depth stayed
  near 30 Hz, Hector stayed live, and one drive-enabled mapper remained
  `AUTO PAUSED`. No movement was commanded. The script now verifies recovery
  by waiting up to 30 seconds for the relaunched node and then holding one
  subscription open for up to 90 seconds for an actual RGB message. This
  tolerates delayed USB recovery without repeatedly adding and removing camera
  subscribers. It also initializes the JetHexa ROS workspace itself so it
  works from a non-login SSH command. Its former `rostopic hz | grep -q`
  pipeline could report failure after finding a rate because `pipefail`
  propagated the producer's SIGPIPE status. A full reboot remains the fallback
  if in-place recovery produces no frame.
  `patches/astra_camera_subscription_restart.patch` changes the vendor
  `libuvc_camera` driver so a duplicate subscriber callback no longer restarts
  a running stream. That driver patch is not recorded as deployed or validated.
- Native-depth floor calibration often falls back to upper-only mode, so its
  floor-bump alarm may be unavailable. The registered-depth planner overlay is
  independent, but has not yet been validated with a measured low object.
- A 30 Hz depth topic can still contain unusable pixels. Brief sparsity uses a
  1.5 s cached decision; sustained sparsity still faults. Inspect valid pixel
  coverage, not only topic rate.
- Rebooting or resetting Hector loses the in-memory map. Object landmarks are
  cleared when map trust is lost or the map is reset.
