# JetHexa Room Mapper

Autonomous room mapping for the Hiwonder JetHexa:

`YDLIDAR G4 -> Hector pose/map + RGB-D obstacle layer -> frontier planner -> JetHexa controller`

The deployed system also provides native-depth collision sensing, RGB/YOLO
object mapping, click-to-navigate, manual driving, recovery behaviors, and an
offline web dashboard at `http://192.168.137.223:8000`.

## Dynamic depth/LiDAR occupancy fusion (deployed; measured-object gate pending)

The planner and web map now maintain a session-persistent fused occupancy
layer downstream of Hector. Six settled registered-depth observations promote
an unconfirmed magenta cell to persistent black occupancy, but only while a
timestamp-aligned settled LiDAR scan is also available. Four consecutive
observations in which both valid depth rays and distinct LiDAR rays traverse a
cell can remove a promoted obstacle or suppress stale Hector occupancy in the
fused planner/display map. A current occupied endpoint from either sensor
immediately vetoes clearing and restores a suppressed cell; neighboring cells
are also protected against 10 cm grid-quantization disagreement.

Hector's `/map`, scan input, pose, and update policy are unchanged. Suppression
is a downstream fused-map decision and never writes into Hector. Persistent
depth cells and Hector suppressions last only for the current mapping session
and reset with `CLEAR MAP` or mapper restart. Missing, stale, moving, invalid,
occluded, or unsynchronized observations cannot promote or jointly clear a
cell. The defaults are configurable with
`_depth_map_promotion_observations:=6`,
`_depth_map_joint_clear_observations:=4`, and
`_depth_lidar_sync_tolerance:=0.35`.

All 211 host and robot-native tests pass, including synthetic LiDAR ray
projection, promotion, one-sensor rejection, occupied-neighbor veto,
moved-object clearing, reappearance, reset, and map-color cases. In a paused
drive-disabled live check, 25 cells promoted and remained after the old 30 s
TTL while no Hector cells were suppressed. Sensor rates remained about 12 Hz
for scan/pose and 30 Hz for RGB/native/registered depth. The exact revision is
now running drive-enabled but `AUTO PAUSED` with command `stop`; no motion was
commanded. Measured-object clearing and physical-motion validation remain
pending.

## Reliability and object-validation behavior (2026-10-26)

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
- Each ready stationary endpoint captures a bounded burst of up to five
  distinct synchronized RGB/depth/intrinsics/TF snapshots before route
  departure. Acquisition may add at most `_object_snapshot_grace:=1.0`
  seconds; it never waits for inference. The asynchronous queue is bounded to
  five pending snapshots and discards its oldest entry on overflow. The worker
  begins inference only after that endpoint's acquisition window closes.
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

YOLO is endpoint-triggered, not a continuous video detector. Each ready
stationary survey admits one bounded five-frame synchronized RGB/depth/TF
burst. Only distinct sensor timestamps count. After that burst, remaining
paused at the same endpoint runs latest-frame display-only preview inference;
those preview frames never enter landmark validation. Another validation burst
is admitted only at a new survey endpoint.
Consequently, `_object_hz` limits queued processing and legacy focus recapture
frequency—it is not a continuous-preview frame rate. This prevents duplicate
frames from consuming inference resources or being treated as additional
mapping observations. A future continuous preview/test mode must keep repeated
display-only detections separate from landmark-confirmation observations.

### Optional TensorRT acceleration

CPU remains the production default and rollback path. On PT, build the
device-specific FP16 engine in the isolated, gitignored trial directory:

```bash
cd ~/room_mapper
python3 prepare_tensorrt_engine.py models/yolo26n-objv1-150.onnx \
  --output models/gpu_trial/yolo26n-objv1-150-fp16.engine
```

The engine metadata binds it to the exact ONNX checksum. Enable it explicitly
with `./start_room_mapper.sh --yolo-backend tensorrt_fp16` or the ROS parameter
`_yolo_backend:=tensorrt_fp16`. Engine load, checksum, warm-up, or later CUDA
execution failure permanently switches that mapper process to the same model
on CPU; the retained YOLOv5n rollback remains the final fallback. Running the
launcher without the option restores CPU selection.

Use `benchmark_yolo_backends.py` on fixed saved frames before enabling the
mapper. With all ROS perception processes stopped, the initial PT trial on
three non-classroom reference images measured 1.010 s CPU versus 0.400 s
TensorRT FP16 median inference (nine samples each). All 11 detections matched;
minimum box IoU was 0.979 and maximum confidence difference was 0.023. This is
standalone performance/parity evidence, not classroom detector-quality or
full-stack sensor-rate validation.

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
  rotation is constrained and rear LiDAR swept-path clearance is available.
  Reverse recovery lasts one 0.5 s gait cycle, then reassesses; required travel
  clearance scales with translation speed while retaining the 0.45 m circular
  LiDAR envelope and a 0.02 m margin. This applies to mapping and clicked/object
  routes. Cancel Route stops and pauses a route.
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
- Detected objects are listed in the Objects tab. Pending objects may be
  renamed or removed but cannot be navigation targets. They appear as hollow
  amber map markers with the current validation reason; confirmed landmarks
  are solid. Expected classroom
  classes require three qualifying detections in four fresh frames (each at
  least 0.50 confidence and median at least 0.65). Every other model class
  remains eligible but requires four in five (each at least 0.60 and median at
  least 0.75) with tighter box and map-position agreement. Depth coverage,
  surface consistency, and inlier gates apply before either tier can create an
  observation. Two consecutive same-view misses remove a pending candidate;
  confirmed landmarks use only qualifying hits for their stored coordinate,
  remain fixed for the mapping session, and clear with the map.
- Stationary preview uses a single latest-frame queue slot and never changes
  validation evidence. Camera boxes clear immediately when motion begins and
  expire after one second without a new result, so an old box cannot remain
  over unrelated live video.
- During stationary surveys, tentative objects remain nonblocking by default.
  The legacy `object_focus` behavior is opt-in; when enabled, motion stops until
  the object is confirmed, missed twice, or the 15-second limit expires.
- YOLO inference and detections do not interrupt active paths. Object mapping
  uses stationary observations at stops; depth and LiDAR still block obstacles.
- RGB failure does not disable depth/LiDAR-protected manual driving. On this
  robot a stalled RGB/UVC stream has required a full robot reboot.

## Depth and LiDAR policy

Native `/camera/depth/image_raw` is used for collision sensing. Registered
depth plus `/camera/depth_registered/camera_info` supplies low-object planner
cells and also projects YOLO detections into the map.

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
  calibration. It rejects points below 0.025 m as floor and above 0.50 m and
  keeps only clustered cells. Unconfirmed observations use the existing 30 s
  hold and render magenta. Repeated settled observations with synchronized
  LiDAR promote them to session-persistent black cells. Only repeated positive
  free rays from both sensors can clear persistent fused occupancy. This can
  render stale Hector occupancy white downstream without changing `/map`.

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
reset it. A Hector node that responds but does not publish `/slam_out_pose`, or
a stale `hex_map.launch` without a live Hector node, is stopped and restarted
with a fresh map. The launcher refuses to run alongside visual SLAM, verifies
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
- `prepare_tensorrt_engine.py`: target-only isolated FP16 engine builder.
- `benchmark_yolo_backends.py`: same-frame CPU/TensorRT timing and parity report.
- `validate_yolo_model.py`: target OpenCV/TensorRT warm-up and timing check.
- `depth_mapping.py`: registered-depth projection for temporary planner cells.
- `lidar_mapping.py`: settled LiDAR occupied/free ray projection for fusion.
- `hex_map.launch`: Hector and required TF bridges.
- `visual_slam.launch`: experimental RTAB-Map RGB-D/LiDAR replacement for Hector.
- `RUN_SHEET.txt`: operator checklist.
- `HANDOFF.md`: current deployment state for the next maintainer.
- `DECISIONS.md`: chronological architecture and validation log.

The repository suite contains 211 tests and does not publish physical movement
during test execution. On a workstation without ROS, the import-only shim
runner exercises the same suite without creating a ROS node or hardware link:

```bash
python host_test_runner.py
```

To repeat the target-native regression suite against the robot's ROS message
types and OpenCV build:

```bash
cd ~/room_mapper
python3 -m unittest discover -v
```

All 204 tests pass through the host shim and natively on PT. The paused live
preview validation results are recorded in `HANDOFF.md` and `RUN_SHEET.txt`.
A drive-disabled,
map/depth-disabled mapper startup reported the preferred model on
`tensorrt_fp16`; a second startup with missing engine metadata automatically
reported the same model on `cpu`. The current production mapper was later
restarted explicitly on `tensorrt_fp16` and remains `AUTO PAUSED` with command
`stop`. After restoring the Astra depth stack, one full-stack stationary GPU
snapshot completed in 3.07 s with two tentative mapped detections. LiDAR,
Hector pose, RGB, native depth, and registered depth remained near 12, 12, 30,
30, and 30 Hz respectively in the post-inference sample. The deployed decoder
now vectorizes class selection and threshold filtering across all 8,400
Objects365 predictions. Three paused full-stack TensorRT restarts measured
1.12, 0.50, and 0.47 s (0.50 s median), versus 2.64-3.47 s immediately before
the change; post-change LiDAR, pose, RGB, native depth, and registered depth
remained near 12, 12, 30, 30, and 30 Hz. This is paused pipeline evidence, not
a detector-quality or motion validation. The preferred
Objects365 ONNX and retained YOLOv5n remain unchanged. The earlier
reliability deployment reduced worst-case synthetic registered-
depth projection from 0.347 s to 0.085 s mean on PT; successful live updates
add and clear temporary cells, although strict image-time TF lookup can report
transient misses between Hector updates. This deployment used no physical
movement; measured-object and supervised-motion validation remain separate and
pending.
