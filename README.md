# JetHexa Room Mapper

Autonomous room mapping for the Hiwonder JetHexa:

`YDLIDAR G4 -> Hector pose/map + RGB-D obstacle layer -> frontier planner -> JetHexa controller`

The deployed system also provides native-depth collision sensing, RGB/YOLO
object mapping, click-to-navigate, manual driving, recovery behaviors, and an
offline web dashboard at `http://192.168.137.223:8000`.

## Hector/depth hybrid deployed and motion-checked (2026-10-19)

Hector is again authoritative for robot pose and the persistent LiDAR room map.
Registered depth is projected through timestamped TF into a temporary planner
layer for geometry 0.025-0.50 m above the floor and 0.15-2.0 m from the camera.
Depth cells expire after 30 s, retaining low obstacles through the camera's
close-range blind area without making moved objects permanent walls.
The depth layer does not feed Hector and therefore cannot move the robot marker
or distort the LiDAR map.

The hybrid passed all 138 robot-native tests. During a supervised 50% speed,
0.55 s forward pulse, Hector reported a coherent change from approximately
`(0.000, 0.006)` to `(0.027, 0.004)` m and about 0.9 degrees. The dead-man stop,
LiDAR map, and depth planner layer remained active. Physical confirmation of
this latest pulse is still required from the supervising operator.

- Close depth results are no longer discarded at 0.50 m. Native depth retains
  left/center/right clearance and reacts to a new close return immediately.
- Obstacle recovery prefers the next route waypoint when that side is clear;
  otherwise it uses combined directional depth and LiDAR clearance.
- Click navigation is converted using the exact center/scale of the rendered
  map image, eliminating the race caused by a moving robot-centered viewport.
  The navigation confirmation dialog is removed.
- Manual keyboard/touch motion bypasses autonomous sensor, map, collision,
  burst, and settle gates. Key release, browser blur, and the 0.55 s dead-man
  still stop motion. Use manual mode only with direct sight of the robot.
- Translation and rotation slider maxima are raised from 150% to 200%.
- `visual_slam.launch` remains available only for experiments. Its first motion
  run did not preserve the expected LiDAR mapping behavior, so it is not the
  production localization or mapping path.

These are not claims of physically safe behavior. Keep the first motion run
paused until camera alignment, ground segmentation, CPU load, and measured
low-obstacle placement have been checked under direct supervision.

The dashboard includes a battery badge in the page header. It accepts
`sensor_msgs/BatteryState` and common numeric voltage topics, estimates charge
from verified 9.8 V alarm cutoff / 12.6 V full defaults, and visibly marks low,
medium, charging, stale, and unavailable telemetry.

## Current deployed behavior (2026-10-19)

- Hector consumes raw `/scan` continuously. `/scan_stable` is diagnostic only.
- Translation and rotation use bounded bursts separated by a 0.65 s settling
  interval. Stationary surveys require five fresh scans over six seconds.
- Routing models the robot as a 0.30 m radius circle. The planner adds a 0.03 m
  margin, so mapped obstacles require 0.33 m clearance.
- Paths use approximately 0.20 m waypoint spacing and a 0.12 m arrival
  tolerance to reduce corner cutting.
- A blocked autonomous route may rotate toward an open side or reverse when
  rotation is constrained and rear LiDAR clearance is available. This applies
  to mapping and clicked/object routes. Cancel Route stops and pauses a route.
- If the front remains blocked after 2.5 seconds of turning, corner recovery
  escalates to the same guarded reverse escape when rear LiDAR is clear.
- One weak turn no longer invalidates SLAM. Three consecutive turn bursts with
  less than 3 degrees of measured heading progress latch `SLAM LOST`.
- Pose updates larger than 0.25 m or 20 degrees still invalidate the map.
- The web map follows the robot by default. It supports wheel/button zoom,
  drag-to-pan, and a Center button that restores 100% robot-centered view.
  Click navigation uses the displayed pan/zoom transform.
- Manual W/A/S/D, arrow-key, and button controls work while autonomy is paused.
  A 0.55 s browser dead-man stops motion when commands stop arriving.
- Independent translation and rotation sliders adjust both manual and
  autonomous gait commands from 50% to 200%. They start at 100% after a mapper
  restart and do not alter collision distances, footprint, or dead-man timing.
- Detected objects are listed in the Objects tab and may be navigated to,
  renamed, or removed. Two distinct observations confirm a landmark.
- During autonomous mapping, a new tentative object starts `object_focus`:
  motion stops until it is confirmed, missed twice, or times out after 15 s.
  Mapping then replans and continues. Rejected candidates may be detected again.
- Empty-scene YOLO inference does not delay the next exploration burst. Once a
  YOLO detection is visible, the controller holds for a settled depth/TF
  observation before continuing or entering `object_focus`.
- RGB failure does not disable depth/LiDAR-protected manual driving. On this
  robot a stalled RGB/UVC stream has required a full robot reboot.

## Depth and LiDAR policy

Native `/camera/depth/image_raw` is used for collision sensing. Registered
depth plus `/camera/depth_registered/camera_info` supplies temporary low-object
planner cells and also projects YOLO detections into the map.

- The upper depth band is accepted with at least 200 valid pixels and at least
  2% coverage. Less coverage causes `FAULT: insufficient valid depth in upper
  band` only when there is no recent valid decision. Brief post-motion sparse
  frames reuse the last valid depth decision for at most 1.5 seconds while
  LiDAR remains active; sustained sparsity still fails closed.
- Close-depth exclusion is disabled (`_depth_ignore_at_or_below:=0.0`). Upper
  returns, floor bumps, and the synthetic dropout/too-close result remain stop
  inputs, and valid depth still uses the 0.60 m stop threshold.
- LiDAR remains an independent fallback and stops at 0.45 m with the 0.30 m
  body radius. Depth still cannot guarantee detection of reflective,
  transparent, out-of-view, or invalid geometry. Change
  `_depth_ignore_at_or_below` only with physical testing.
- If floor calibration fails three times, the controller continues in clearly
  reported `upper-only` mode; floor-bump detection is then unavailable.
- The registered-depth planner layer is independent of that per-row floor
  calibration. It rejects points below 0.025 m as floor and above 0.50 m, keeps
  only clustered cells, and expires observations after 30 s. Each new
  observation refreshes the cell's hold. On the web map, LiDAR walls are black
  and depth-only obstacles are magenta; cells confirmed by both remain black.

## Web interface

The dashboard contains live RGB, a robot-centered SLAM map, telemetry,
pause/resume, camera recovery, route cancellation, map clearing, and manual
drive controls.
The Objects tab shows confirmed and pending landmarks. Map and object routes
always use known-free, footprint-inflated planner space and remain subject to
live sensor stops.

`CLEAR MAP` stops and pauses the robot, resets Hector and the planner, clears
object landmarks, and starts a fresh stationary survey. Hector's installed
`/reset_map` callback performs the reset but returns empty/default Trigger
fields, so `success=False` alone is not evidence of failure.

## Startup

The robot is `hiwonder@192.168.137.223`; the project is
`/home/hiwonder/room_mapper`. Do not store the SSH password in this project.

```bash
# LiDAR (after reboot, /start_scan may also be required)
roslaunch jethexa_peripherals lidar.launch publish_tf:=false

# Hector and the TF bridges
roslaunch ~/room_mapper/hex_map.launch

# Safe dry run
cd ~/room_mapper
python3 room_mapper.py

# Real controller, initially paused
python3 room_mapper.py _drive:=true _start_paused:=true
```

### Non-production RTAB-Map experiment

`visual_slam.launch` replaces `hex_map.launch`; never run the two together. It
is retained for research comparison but failed the expected mapping behavior
during its first supervised motion pulse. Do not use it for autonomous motion.
Start LiDAR with `publish_tf:=false`, then use:

```bash
roslaunch ~/room_mapper/visual_slam.launch

cd ~/room_mapper
python3 room_mapper.py \
  _drive:=false _start_paused:=true \
  _map_topic:=/visual_map \
  _pose_type:=tf \
  _map_frame:=map \
  _pose_frame:=jethexa/base_link \
  _reset_service:=/visual_slam/reset \
  _reset_service_type:=empty
```

Before any supervised drive test, verify `/visual_map`, `/visual_slam/info`, and
`/visual_slam/odom`, confirm the TF chain
`map -> visual_odom -> jethexa/base_link`, and place several known low obstacles
in view while checking that they appear occupied in `/visual_map`.

RTAB-Map's `localization_pose` output is inactive while building a map. The
mapper therefore uses `_pose_type:=tf` to read the live map-to-base transform.
RTAB-Map also publishes `/visual_map` only when the grid changes, so the mapper
uses `/visual_slam/info` as its default five-second liveness heartbeat whenever
`_map_topic:=/visual_map`. Hector retains the original `/map` freshness check.

After reboot, if `/scan` has a publisher but no messages:

```bash
rosservice call /start_scan '{}'
```

The robot publishes `std_msgs/Float32` voltage on `/voltage`. The vendor alarm
cutoff is 9.8 V and the configured full-pack reference is 12.6 V. The dedicated
`battery_low_alarm` launch includes are disabled in both controller launch
variants; `/voltage` and the website's visual low-battery warning remain active.
Recoverable `.pre_room_mapper_alarm_disable_20261012` backups sit beside both
edited launch files.

Use `_reset_map:=false` only for deliberate continuation of a still-valid map.
A reboot or Hector restart creates a fresh in-memory map.

## Fault recovery

- `insufficient valid depth in upper band`: verify depth frame rate and valid
  coverage. A live publisher alone does not prove usable depth pixels.
- RGB publisher present but no frames: while autonomy is paused, press
  `RECOVER CAMERA` on the web interface, or run `./recover_rgb_camera.sh`, and
  verify `/camera/rgb/image_raw`. The recovery restarts only
  the RGB node and USB function while preserving depth, Hector, and the mapper.
  If it produces no frame, reboot the robot, then restore LiDAR, Hector, and
  the mapper; call `/start_scan` if needed.
- `SLAM LOST` or stationary pose drift: stop the mapper, reset/restart Hector,
  and start a fresh map. Do not continue routing on the invalid grid.
- TF timeout: verify
  `jethexa/base_link -> jethexa/body_link -> body_link -> laser_link` and do not
  create a second parent for `laser_link`.

## Files and validation

- `room_mapper.py`: ROS node, safety, motion, recovery, object-focus workflow.
- `map_explore.py`: 10 cm planning grid, inflation, frontiers, point routes.
- `map_web.py`: dashboard, map transforms, manual/object/navigation APIs.
- `object_mapping.py`: YOLO decoding, TF projection, landmark tracking.
- `depth_mapping.py`: registered-depth projection for temporary planner cells.
- `hex_map.launch`: Hector and required TF bridges.
- `visual_slam.launch`: experimental RTAB-Map RGB-D/LiDAR replacement for Hector.
- `RUN_SHEET.txt`: operator checklist.
- `HANDOFF.md`: current deployment state for the next maintainer.
- `DECISIONS.md`: chronological architecture and validation log.

The updated suite contains 142 tests and does not publish physical
movement during test execution:

```bash
cd ~/room_mapper
python3 -m unittest discover -v
```

All 142 tests passed natively on PT. The current live stack uses Hector, not
RTAB-Map, with one drive-enabled `AUTO PAUSED` mapper. The operator confirmed
that two observed resumptions were caused by pressing `RESUME AUTO`. RGB,
native and registered depth, LiDAR, `/slam_out_pose`, `/map`, the temporary depth planner
layer, battery telemetry, and the web interface are live. The supervising
operator confirmed that the Hector pulse moved correctly and the map looked
substantially better; dry-run mode was then disabled without resuming autonomy.
