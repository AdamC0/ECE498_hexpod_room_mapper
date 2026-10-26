# JetHexa Room Mapper

Autonomous room mapping for the Hiwonder JetHexa:

`YDLIDAR G4 -> Hector pose/map + RGB-D obstacle layer -> frontier planner -> JetHexa controller`

The deployed system also provides native-depth collision sensing, RGB/YOLO
object mapping, click-to-navigate, manual driving, recovery behaviors, and an
offline web dashboard at `http://192.168.137.223:8000`.

## Reliability fixes implemented; deployment pending (2026-10-25)

The repository now implements the three approved reliability priorities, but
the robot still runs the earlier 163-test build. Do not treat these changes as
deployed or physically validated yet.

- Registered-depth projection now emits occupied and positively observed-free
  cells. A temporary magenta obstacle clears only after two consecutive valid
  clear observations. Any current low-surface evidence wins, missing/invalid/
  occluded views fall back to the 30 s TTL, and cells within 0.60 m of the
  timestamped camera origin are never ray-cleared. Hector occupancy and native
  depth collision decisions are untouched. The defaults are configurable with
  `_depth_map_clear_observations:=2` and
  `_depth_map_protected_range:=0.60`.
- Each ready stationary endpoint captures at most one immutable synchronized
  RGB/depth/intrinsics/TF snapshot before route departure. Acquisition may add
  at most `_object_snapshot_grace:=1.0` seconds; it never waits for inference.
  One inference may run while one newest endpoint snapshot remains pending.
  Status reports captured, processed, replaced, and pending counts.
- Objects, paths, robot pose, and requested/adjusted goals now share the exact
  active pan/zoom transform. Marker glyph sizes remain readable in pixels.

All 176 host tests pass, including synthetic removed/occluded/invalid depth,
slow-detector queueing, map-reset invalidation, and 50%/100%/200% panned map
views. Robot-native, no-drive, sensor-timing, and supervised motion gates in
`RUN_SHEET.txt` remain pending.

## Smooth routing and classroom detection update (deployed; partially motion-validated)

The repository now attacks the observed move-for-seconds/wait-for-about-20-
seconds cadence at its two causes. Frontier selection considers reachable
information gain and normally chooses a 1.5-3.0 m route rather than the first
0.5 m frontier. Endpoint survey defaults are reduced from six seconds/five
scans to two seconds/three scans. A synchronized RGB/depth/TF snapshot is still
captured only while stopped and settled, but slow YOLO inference may finish and
commit that snapshot after the next safe route has begun. Tentative objects no
longer enter the 15-second blocking focus phase by default. Set
`_object_focus_enabled:=true` to restore the old workflow.

Object detection now prefers `models/yolo26n-objv1-150.onnx` with its matching
`.names` file and validates it with a blank warm-up inference. This official
Objects365 vocabulary includes desk, chair, stool, whiteboard/blackboard,
projector, printer, laptop, books, backpacks, and many other classroom objects.
It does not include a podium/lectern class; the same sidecar-label pipeline
accepts a later room-specific fine-tuned checkpoint, and those class names are
already configured for the classroom confidence threshold.
Classroom classes use a 0.30 confidence threshold; all other supported classes
remain enabled at 0.45. Three distinct settled captures now confirm a landmark,
and class-aware NMS prevents overlapping desk/chair boxes from suppressing one
another. If the preferred model is absent, incompatible, or its selected CUDA
backend fails, startup automatically falls back to CPU and then to the retained
`models/yolov5n.onnx` model. Existing model files are never overwritten.

Prepare the preferred model on a workstation (not the robot), then copy both
output files into `/home/hiwonder/room_mapper/models/`:

```bash
python3 -m pip install ultralytics
python3 prepare_yolo_model.py
```

The exporter materializes a final broadcast constant so the generated model
runs on PT's legacy OpenCV 4.5.5 DNN implementation without changing ONNX
Runtime output. Use `validate_yolo_model.py` on the target before activation.

The full update was deployed on 2026-10-25 and all 163 tests pass both through
the workstation shim runner and natively on PT. The patched Objects365 model is
active on CPU; no-drive inference measured about 5.8-6.9 s. Repeated stationary
captures exercised synchronized mapping and three-observation confirmation.
Directly supervised 50% manual strafe/rotation and a short 31-degree clicked
route were subsequently confirmed physically correct by the operator. Tests of
the beyond-45-degree turn-first behavior, diagonal obstacle gate, and detector
quality remain pending. LiDAR/depth fusion and collision thresholds were not
changed.

The UI, planner logs, and navigation-validation report intentionally do not
show a map-completion percentage. The occupancy grid covers a configured
working area rather than a known room boundary, so its known-cell fraction is
not a meaningful measure of room completion. Exploration continues to use
reachable frontiers and known-free space directly.

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
- Active paths use continuous translation and turning without timed observation
  stops. Heading progress is checked during turns. Recovery retains bounded
  motion; stationary surveys at route endpoints require three scans over two seconds.
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
- Manual W/A/S/D, arrow-key, and button controls work while autonomy is paused
  in the last deployed revision.
  A 0.55 s browser dead-man stops motion when commands stop arriving.
- Independent translation and rotation sliders adjust both manual and
  autonomous gait commands from 50% to 200%. They start at 100% after a mapper
  restart and do not alter collision distances, footprint, or dead-man timing.
- Detected objects are listed in the Objects tab and may be navigated to,
  renamed, or removed. Three distinct settled observations confirm a landmark.
- During stationary surveys, tentative objects remain nonblocking by default.
  The legacy `object_focus` behavior is opt-in; when enabled, motion stops until
  the object is confirmed, missed twice, or the 15-second limit expires.
- YOLO inference and detections do not interrupt active paths. Object mapping
  uses stationary observations at stops; depth and LiDAR still block obstacles.
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

## Omnidirectional motion update (deployed; initial physical validation complete)

The repository controller now uses JetHexa's body-relative travel direction
instead of reducing every route segment to turn-then-forward motion. Route
following crab-steers continuously in 5 degree increments up to 45 degrees
left or right; larger errors retain the measured in-place turn and heading
progress guard. Directional swept-body LiDAR clearance is checked before a
crab command. The 45 degree limit deliberately keeps autonomous translation
near the forward depth camera's view rather than claiming blind-side low-object
coverage.

The paused manual drive pad exposes full left/right strafe, with Q/E and the
left/right arrow keys reserved for rotation. Manual driving still bypasses
autonomous obstacle gates and therefore requires direct sight, key-release/
browser-blur stop handling, and the 0.55 second server dead-man.

This revision is deployed and included in the 163-test native pass. On
2026-10-25 the operator confirmed correct left/right strafe without noticeable
yaw or forward motion, correct in-place Q/E rotation without lateral motion,
and continuous travel on a short known-free clicked route about 31 degrees from
the current heading. The operator also confirmed that physical movement matched
the map. Beyond-45-degree turn-first and diagonal obstacle-gate trials remain
pending. The Objects tab shows confirmed and pending landmarks. Map and object
routes always use known-free, footprint-inflated planner space and remain
subject to live sensor stops.

`CLEAR MAP` stops and pauses the robot, resets Hector and the planner, clears
object landmarks, and starts a fresh stationary survey. Hector's installed
`/reset_map` callback performs the reset but returns empty/default Trigger
fields, so `success=False` alone is not evidence of failure.

## Startup

The robot is `hiwonder@192.168.137.223`; the project is
`/home/hiwonder/room_mapper`. Do not store the SSH password in this project.

For the normal production startup, run the launcher on the robot:

```bash
cd ~/room_mapper
./start_room_mapper.sh
```

It restores `/scan` when necessary, starts Hector if it is not already live,
and starts exactly one drive-enabled mapper with autonomy paused. If Hector is
already live, its current map is preserved; pass `--fresh-map` to deliberately
reset it. The launcher refuses to run alongside visual SLAM, verifies
`AUTO PAUSED` and command `stop`, and writes logs to
`~/.room_mapper_logs/`. It never resumes autonomous motion.

From a Windows checkout, double-click `start_room_mapper.cmd` to run the same
launcher over SSH and open the web dashboard. SSH may prompt for the robot
password if key authentication is not configured; the password is not stored.

The equivalent manual sequence is retained below for troubleshooting.

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
- `prepare_yolo_model.py`: workstation-only ONNX export and label-sidecar tool.
- `validate_yolo_model.py`: target OpenCV warm-up and timing check.
- `depth_mapping.py`: registered-depth projection for temporary planner cells.
- `hex_map.launch`: Hector and required TF bridges.
- `visual_slam.launch`: experimental RTAB-Map RGB-D/LiDAR replacement for Hector.
- `RUN_SHEET.txt`: operator checklist.
- `HANDOFF.md`: current deployment state for the next maintainer.
- `DECISIONS.md`: chronological architecture and validation log.

The repository suite contains 177 tests and does not publish physical movement
during test execution. On a workstation without ROS, the import-only shim
runner exercises the same suite without creating a ROS node or hardware link:

```bash
python host_test_runner.py
```

The deployment gate remains a robot-native run against the target ROS message
types and OpenCV build:

```bash
cd ~/room_mapper
python3 -m unittest discover -v
```

All 177 tests pass both through the host shim and natively on PT. The current
live stack uses Hector, not RTAB-Map, with one drive-disabled `AUTO PAUSED`
mapper and command `stop`. RGB,
native and registered depth, LiDAR, `/slam_out_pose`, `/map`, the temporary
depth planner layer, battery telemetry, and the web interface are live. The
preferred Objects365 model is active on CPU after its legacy-OpenCV broadcast
rewrite. The reliability deployment reduced worst-case synthetic registered-
depth projection from 0.347 s to 0.085 s mean on PT; successful live updates
add and clear temporary cells, although strict image-time TF lookup can report
transient misses between Hector updates. This deployment used no physical
movement; measured-object and supervised-motion validation remain separate and
pending.
