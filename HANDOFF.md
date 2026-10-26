# JetHexa Room Mapper - Agent Handoff

Updated: 2026-10-25

## Live checkpoint

Deployed 2026-10-25: PT now runs the rear-swept-clearance revision through
`5dca682`. All 180 tests pass on the Windows host and natively on PT. Exactly one
drive-enabled mapper is `AUTO PAUSED` with command `stop`; Hector is the only
SLAM stack. Registered-depth projection averages 0.085 s on PT after the
packed-key optimization, down from 0.347 s in the failed first no-drive gate.
The live layer has added temporary cells and reported confirmed clears. Strict
0.5 s image-time TF lookup still alternates between successful updates and
transient misses at the current Hector TF cadence; watch this during the
measured low-object test. RGB, depth, LiDAR, pose, map, web, detector, and
battery telemetry are live. The current rear geometry provides 0.134 m of swept
travel before entering the 0.45 m envelope, versus 0.060 m required at 100%
translation or 0.100 m at 200%. Native depth recalibrated upper-only, so the
previous dropout alarm cleared. No motion was commanded during this deployment;
the 0.5 s autonomous reverse remains pending direct supervised validation.

On 2026-10-25, directly supervised 50% tests on the preceding revision physically
confirmed clean left/right strafe, clean in-place Q/E rotation, and continuous
diagonal travel on a short clicked route about 31 degrees off heading. The
operator reported that all movement was correct and matched the map. During
later route activity the operator intentionally moved both speed sliders to
200%; these were not unexplained controller changes. The agent then cancelled
the active route and left the mapper `AUTO PAUSED` with command `stop`. Do not
infer completion of the still-pending beyond-45-degree or obstacle-gate trials.

## Reliability update deployed - physical validation pending

The operator reported three issues: mapped object markers do not follow map
zoom, old magenta registered-depth obstacles remain after an object is moved,
and fast exploration can leave too few settled object captures for confirmed
landmarks. All three priorities in `RELIABILITY_FIX_PLAN.md` are implemented in
separate commits and are now deployed to PT:

1. `a201e37` adds conservative free-space evidence to the temporary registered-depth
   planner layer. Clear a magenta cell only after two consecutive valid depth
   observations disprove it. Preserve all cells within at least 0.60 m of the
   registered-depth camera because close geometry may be beneath its useful
   view, even while the robot faces the cell. Determine distance from the
   timestamped camera-to-map transform. Invalid, occluded, out-of-view, and
   behind-surface cells continue to rely on the 30 s TTL. Never clear Hector's
   LiDAR occupancy or the native-depth collision state.
2. `5d82570` captures one immutable synchronized RGB/depth/intrinsics/TF snapshot as each
   stationary survey becomes ready, then process it asynchronously. The
   operator accepted one active inference plus the newest pending snapshot,
   three distinct observations for confirmation, and up to 1.0 s of endpoint
   grace to acquire synchronized data. Exploration must not wait for inference.
3. `6cd0253` passes the active map scale to every world-to-screen conversion so
   markers retain the correct position through zoom and pan.

The combined host and native suite has 177 passing tests. Python compilation
also passes. No-drive evidence covers target projection cost, live depth-layer
updates, sensor rates, one endpoint snapshot/inference, multi-scale map HTTP
rendering, and battery telemetry. It does not replace the pending measured
low-object clearing/near-field test, multiple busy-detector endpoint test,
visual marker check, detector-quality trial, or supervised motion. The
near-field depth protection is a safety requirement, not optional tuning.

The first post-deployment boot had USB enumeration error `-71`, no LiDAR
device, and no RGB/depth frames. The documented in-place RGB recovery could
not find the USB function, so PT was rebooted with no mapper or SLAM process
running. After reboot and `/start_scan`, LiDAR was about 12 Hz, Hector pose
about 10 Hz, RGB about 29-30 Hz, native depth about 31-36 Hz, and registered
depth about 29-31 Hz. Native depth is again upper-only; the independent
registered-depth planner overlay is live. Battery was about 11.90 V / 75%.

The exporter now materializes YOLO26's final `[1,4,8400]` scale tensor instead
of relying on broadcasting that PT's OpenCV 4.5.5 mishandles. ONNX Runtime
produces bit-identical output before and after the rewrite. The preferred
365-label model now passes target warm-up, is reported by live status as active
on CPU, and measured about 5.8-6.9 s per stationary inference. Repeated paused
captures mapped six tracks and confirmed two after three observations. Treat
that as pipeline evidence, not a detector-quality judgment. Labeled-scene
quality and a measured two-second endpoint-timing check remain pending; the
initial omnidirectional motion cases are recorded above.

Current object-mapping behavior: per-frame object detections are working, but
objects are not currently being added to the map because detections are not
registering across enough distinct settled frames to satisfy landmark
validation. A visible per-frame detection is therefore not evidence that an
object has been confirmed or mapped. Confirmation requires three distinct
settled captures; investigate capture/track registration and validation before
changing detector thresholds or treating this as a detector failure.

Repository-only update on 2026-10-24: removed the fixed-grid map-known
percentage from web status, planner logs, and navigation validation. The room's
total extent is unknown, so the fraction was not a valid completion measure.
Frontier planning and occupancy data are unchanged. Test count is now 160;
robot-native validation remains pending.

The workstation validation is now reproducible through `host_test_runner.py`,
which supplies import-only ROS/message shims and runs all 160 tests without a
ROS node or hardware connection. Four detector-startup regressions verify
missing-preferred fallback, CUDA-to-CPU retry, warm-up failure rollback, and
clean disablement when both models are unavailable. The run sheet's object
handling summary now matches the nonblocking, three-capture default.

Repository-only update on 2026-10-24: autonomous frontier routes now prefer
information-rich goals 1.5-3.0 m away, endpoint surveys default to two seconds
and three settled scans, and YOLO completion no longer blocks the next route.
Image-time synchronized RGB/depth/TF snapshots can commit after motion resumes.
Blocking `object_focus` is off by default but reversible with
`_object_focus_enabled:=true`. Landmark confirmation is three distinct settled
captures, retained for five minutes, rather than two immediate observations.

The preferred detector is now an externally prepared
`yolo26n-objv1-150.onnx` plus `.names` sidecar. It covers the 365 Objects365
classes, including classroom-specific desk, whiteboard/blackboard, projector,
printer, stool, and school-supply classes absent from COCO. Classroom classes
use a lower 0.30 threshold while other classes retain 0.45; class-aware NMS
keeps overlapping classes. Startup warm-up validates the model/backend and
automatically falls back to CPU and then the unchanged `yolov5n.onnx`. The
model artifacts are not committed and must be prepared with
`prepare_yolo_model.py`, then copied beside the fallback model.

The operator reports that the previously tested object-mapping revision placed
landmarks nearly perfectly and did not produce duplicate landmarks or false
detections. Its observed weaknesses were missed objects and slow inference.
Do not change depth projection, landmark association, or confirmation solely in
response to that report. The Objects365/nonblocking revision is deployed;
labeled-scene detector quality remains untested.

If the next robot tests still show missed objects or unacceptable inference
delay, the following are optional directions for investigation, not required or
approved changes:

- Benchmark the preferred and fallback models on labeled images from the robot,
  recording per-class precision/recall and end-to-end inference time.
- Tune per-class confidence thresholds from those results while preserving the
  previously observed absence of false detections.
- Fine-tune a small model on actual classroom-camera images, especially missed,
  small, distant, occluded, podium, and lectern examples.
- Compare CPU, CUDA FP16, and, if compatible with PT, TensorRT FP16 execution
  before considering a larger model. Measure camera, ROS, and controller rates
  as well as detector latency.
- During the settled survey, choose the sharpest of a small set of candidate
  frames for one inference. Keep any pending-work queue bounded to the newest
  useful settled observation so slow inference cannot accumulate stale work.
- Consider a selective high-resolution or tiled second pass only for views in
  which the normal 640-pixel pass finds nothing. This may improve small-object
  recall but can increase latency.
- If misses are caused by camera coverage rather than classification, consider
  aiming endpoint captures toward unexplored space or taking additional bounded
  views. Any new body rotation requires the normal supervised motion validation.
- Restrict the enabled production vocabulary to useful classroom landmarks if
  that makes lower class-specific thresholds acceptable without adding clutter.

These possibilities intentionally leave the currently successful map-point and
deduplication behavior unchanged unless new evidence identifies a regression in
those areas. None should be described as necessary until robot measurements
support it.

## Jetson Nano GPU YOLO evaluation plan (planning only)

### Goal and current findings

Determine whether GPU inference is faster than the current CPU path while
preserving detection results and healthy mapper sensor rates. Keep the
experiment optional and reversible. If GPU performance is slower than the
non-GPU test, less reliable, or harms sensor rates, stop it and restore CPU
configuration and artifacts.

The deployed `yolo26n-objv1-150.onnx` model runs through OpenCV DNN on CPU,
with measured stationary inference around 5.8-6.9 seconds per frame. PT has
JetPack CUDA 10.2, cuDNN 8.2, TensorRT 8.2.1, TensorRT Python bindings, and
PyCUDA installed. Its OpenCV 4.5.5 build has no CUDA modules and reports zero
CUDA devices. OpenCV's CUDA constants therefore do not indicate GPU support;
requesting `cuda_fp16` falls back to CPU after warm-up failure.

TensorRT is the primary candidate. First check its ONNX parser/tooling and
operator support. Record JetPack, TensorRT, CUDA, cuDNN versions, precision,
model checksums, memory pressure, and temperature. Do not assume the graph will
build or that FP16 will be faster on Nano's Maxwell GPU. PT reported about 1 GB
available memory during inspection, and CPU/GPU share system memory.

### Test sequence

1. Freeze the CPU reference. Keep the production ONNX model and `.names` file
   unchanged; record checksums. Capture fixed, representative robot RGB frames
   including empty scenes and labeled classroom objects. Run CPU inference on
   these same frames and record warm-up, per-frame latency, median, 90th
   percentile, worst time, detections, labels, confidence, boxes, memory, and
   CPU use. Record baseline `/scan`, `/map`, `/slam_out_pose`, RGB, native-depth,
   and registered-depth rates with the normal paused mapper active.
2. Build an isolated candidate while the robot is stationary and autonomy
   paused. Use a standalone benchmark with `_drive:=false` or no mapper
   connection. Put TensorRT engine and trial configuration under a separate
   directory such as `models/gpu_trial/`; never overwrite production ONNX,
   labels, CPU model, or current artifacts. Try FP16 only if supported; use
   FP32 as a separate comparison if needed. If engine generation fails,
   document the unsupported operation. Do not rewrite, quantize, or simplify
   the production model without a separate output comparison.
3. If a runtime selector is needed, make it explicitly opt-in (for example,
   `tensorrt_fp16`), keep CPU as the default, and fall back to CPU when engine
   load, warm-up, or inference fails. Keep TensorRT dependencies and engine
   paths out of standard CPU startup.
4. Run CPU and GPU on the exact same saved frames. Compare warm-up and
   steady-state latency (median, 90th percentile, worst), matched detections,
   box overlap, confidence differences, misses, and added detections. Report
   empty-scene false positives separately from labeled-object recall. Monitor
   GPU/CPU load, memory, temperature, throttling, and failures with available
   Jetson telemetry such as `tegrastats`.
5. With the mapper paused and drive disabled, check sensor rates and endpoint
   capture behavior while inference runs. Call the trial successful only if it
   is reliably faster than the 5.8-6.9 s CPU baseline on the same workload,
   preserves acceptable detection parity, and causes no memory pressure,
   thermal throttling, inference failures, or unhealthy ROS rates. GPU
   utilization alone is not a success measure.

### Reversibility and rollback

- Keep all engines, calibration data, and trial settings in a separate,
  removable GPU-trial directory. Never replace the CPU ONNX model or labels.
- Keep CPU as the default. Enable GPU only with an explicit parameter during a
  paused trial.
- Commit code changes separately from model artifacts and keep CPU startup
  covered. Roll back code by reverting the GPU-backend commit, then restart the
  mapper with CPU selected.
- If GPU performance is worse than the non-GPU run, stop the benchmark,
  terminate the GPU process, restart the standard CPU mapper in `AUTO PAUSED`
  mode, and verify status reports `(cpu)`. Remove only the isolated trial
  engine after checking its exact path. Preserve the known-good ONNX and label
  files.
- After rollback, recheck CPU inference, ROS sensor rates, and mapper status.
  Do not resume autonomy during benchmarking or rollback.

Require an isolated benchmark, output comparison, CPU fallback check, and
paused/no-drive live soak before any physical-use decision. If these pass, keep
GPU execution opt-in until it has been observed during ordinary stationary
surveys. Any movement test is separate and directly supervised; GPU
benchmarking itself must not command movement.

The Jetson GPU software stack is present, but the deployed OpenCV runtime
cannot use it. Whether TensorRT can build this model, whether FP16 helps, and
whether GPU output matches CPU detections remain untested. The CPU path remains
the baseline and rollback target until measurements support a change.

Ten regressions were added (160 total). All 49 directly runnable ROS-free tests
pass, the remaining 111 controller tests pass under import-only ROS message
shims, and Python compilation passes on the Windows host. This is not a
substitute for a native ROS run. Nothing in this update has been deployed or
physically validated. The last
verified robot state below remains authoritative. LiDAR/depth fusion, safety
distances, and manual controls are unchanged.

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
- The deployed controller can crab-steer active paths up to 45 degrees and
  provides supervised manual left/right strafe. Left/right strafe, Q/E rotation,
  and a 31-degree clicked route have been physically validated; larger-bearing
  and obstacle-gate cases remain pending.
- Depth is the forward primary signal; LiDAR is an independent 0.45 m
  emergency fallback and supplies rear/rotation clearance.
- Web UI provides robot-following zoomable/pannable map, click navigation,
  route cancellation, manual driving, RGB, telemetry, and object management.
- Independent 50-200% translation and rotation sliders scale manual and
  autonomous gait output without changing any sensor or planning threshold.
- The web UI includes a header battery percentage badge with voltage and
  charging details plus clear stale/unavailable states.
- The deployed update requires three distinct settled captures for a
  landmark. Inference finishes asynchronously and does not enter blocking
  `object_focus` unless the legacy mode is explicitly enabled.
- Object capture remains stationary; inference completion may occur during the
  next route because image-time TF and registered depth were snapshotted.

## Depth/navigation update

- Removed the 0.50 m close-depth exclusion and added directional
  left/center/right depth clearance.
- Recovery follows the selected route's turn direction when that side remains
  clear, then falls back to combined depth/LiDAR side clearance.
- A front obstacle that persists through 2.5 seconds of turning escalates to a
  0.5 s reverse escape only when its speed-scaled travel plus 0.02 m fits in a
  rear LiDAR swept-circle path retaining the 0.45 m emergency envelope. This
  replaces the over-conservative fixed 0.65 m radial-sector cutoff. Floor-bump
  and depth-dropout alarms still prohibit autonomous reversing.
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
- object confirmation: 3 distinct settled observations
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

Use the checked-in production launcher for routine startup:

```bash
cd ~/room_mapper
./start_room_mapper.sh
```

It verifies ROS, restores LiDAR scanning when needed, rejects visual SLAM,
starts Hector only when absent, and starts exactly one drive-enabled mapper in
`AUTO PAUSED` with command `stop`. A live Hector map is preserved by default;
`--fresh-map` explicitly resets it. Logs are written outside the repository at
`~/.room_mapper_logs/`. The Windows `start_room_mapper.cmd` wrapper invokes
this script over SSH and opens the dashboard after successful verification.

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
