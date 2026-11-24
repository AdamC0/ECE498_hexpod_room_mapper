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

Robot Passwords: hiwonder

Web UI: `http://192.168.137.223:8000`

Real motion requires
direct supervision. Prefer `_drive:=true _start_paused:=true` and confirm only
one mapper process exists.

## Current architecture

- YDLIDAR G4 raw `/scan` feeds Hector, which owns pose and the persistent map.
- Registered depth adds low-obstacle cells to a fused planner copy. Three
  settled observations paired with LiDAR promote a temporary cell for the
  mapping session; four joint free observations can remove it or suppress a
  stale Hector-black cell downstream. It never writes into Hector.
  `visual_slam.launch` is non-production and must remain
  drive-disabled if used for research comparison. Never run both SLAM stacks.
- `map_explore.py` models the approximately 0.33 m full leg span as a 0.17 m
  radius and adds a 0.13 m outside planning margin.
- If no frontier route is reachable but the live 0.22 m circular rotation
  footprint is clear, the mapper may make up to eight zero-stride resurvey
  turns with settled replanning. It never drives a raw-only route.
- `room_mapper.py` handles continuous path gait commands, bounded recovery,
  sensor safety, omnidirectional manual control, clicked routes, and autonomous
  object verification. During autonomous frontier travel only, a moving YOLO
  detection may request a bounded stop to verify a possible novel object.
- `map_web.py` provides RGB, a robot-following zoomable map, click navigation,
  Cancel Route, manual W/S drive, A/D strafe, Q/E rotation, independent 50-200%
  translation/rotation sliders, object management, and a battery badge.
- Object mapping uses YOLO, synchronized registered depth, camera intrinsics,
  and timestamped TF. The deployed revision requires three distinct
  settled captures and prefers the Objects365 classroom model with automatic
  fallback to the retained COCO YOLOv5n model. Moving captures are RGB-only
  previews and never update semantic tracks or planner occupancy.
- The safe launcher defaults to TensorRT FP16. Engine initialization or runtime
  failure automatically falls back to CPU for that process; explicit CPU
  startup remains available with `--yolo-backend cpu`.
- The safe launcher defaults geometric registered depth to the CUDA sidecar and
  fails closed if its image or camera-info stream is unavailable. The planner
  consumes this geometry at 5 Hz; native depth remains independent for the
  immediate collision check. Explicit rollback is available with
  `--registered-depth-backend driver`.
- The production depth launch uses the driver's 640x480@15 mode and omits
  unused processing/point-cloud nodelets via
  `patches/halley_depth_cpu_reduction.patch`. Native and CUDA-registered raw
  depth remain required; do not remove or throttle either below the safety
  policy.

## Current safety/calibration facts

- LiDAR emergency stop: 0.45 m.
- Depth stop: 0.60 m.
- Close-depth exclusion: disabled (0.00 m); close/dropout returns remain stops.
- Depth validity: 200 pixels and 2% upper-band coverage.
- Depth planner layer: 0.15-2.0 m occupied range, 0.025-0.50 m height. Every
  cell has a 10 s minimum hold from its latest occupied hit. A temporary cell
  that never earns promotion ages out 60 s after its latest hit; repeated hits
  restart the limit. Promoted cells still require positive joint clear evidence
  and otherwise persist for the mapping session.
- Brief upper-depth sparsity reuses the last valid result for at most 1.5 s. A
  calibrated current lower-floor region can then provide bump/dropout sensing
  with LiDAR and the fused planner; without usable upper or lower-floor depth,
  sustained sparsity still faults.
- Map pose jump: 0.25 m / 20 degrees, confirmed by two consistent samples.
- Turn progress: require at least two new, recent Hector poses and evaluate
  cumulative heading across three bursts. A total below 3 degrees pauses
  autonomy and retains the map; turn failure alone is not localization loss.
- Stationary synchronized captures are the only source of semantic-map
  evidence. Moving YOLO preview is display/recording-only, and inference
  completion remains nonblocking. The legacy object-focus mode remains opt-in
  with its 15 s maximum and two-miss limit.
- During autonomous frontier travel only, a moving detection can stop the
  robot for close-object verification: settled registered depth must place it
  within 3.0 m, the hold is bounded to 15 s, and a novel candidate requests
  five distinct settled frames. Manual and clicked/object routes are excluded.
- Persistent front blockage escalates from turning to a 0.5 s reverse after
  2.5 s only when the speed-scaled reverse distance plus 0.02 m fits inside a
  LiDAR swept-circle path that retains the 0.45 m emergency envelope.
  Floor-bump and depth-dropout alarms prohibit reverse.
- Active paths use continuous translation and turning with live heading-progress
  checks. Repeated turn-progress loss stops and pauses with the map retained.
  Only a confirmed pose discontinuity schedules automatic Hector reset.
- Do not report a map completion/known percentage. The configured occupancy
  grid is not the unknown room boundary; use frontier availability and direct
  map evidence instead.

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

Sparse-depth resurvey, the RGB/registered-depth overlay, and corrected physical
footprint calibration are deployed at commit `2150407`. The operator clarified
that the complete leg span is about 0.33 m, so planning now uses a 0.17 m body
radius plus 0.13 m outside margin and a 0.22 m turn envelope; the independent
0.45 m LiDAR emergency stop is unchanged. All 306 host and robot-native tests
pass. Live 0%, 45%, and 100% overlay frames were verified. One mapper is
`AUTO PAUSED`/stop with Hector and CUDA registration only, matching hashes, and
no deployment motion. A subsequent supervised run issued eight bounded turns
and stopped when depth remained invalid; the operator physically confirmed
rotation, while Hector x/y stayed within about 0.02 m. It is again `AUTO
PAUSED`/stop. The occupied/unknown-footprint live veto check remains pending.

Repository cleanup and operator motion diagnosis are deployed at commits
`2415f9a` and `56f1f50`. Runtime modules remain at the root, tests are under
`tests/`, offline utilities under `tools/`, and historical plans/images under
`docs/`. Robot-side backups, loose revisions, logs, diagnostics, snapshots,
and caches were removed while models and recordings were preserved. All 293
host and robot-native tests pass. The dashboard reports a prioritized
`Why not moving: [CODE] detail` and retains explicit `Command: stop` launcher
proof. One mapper is `AUTO PAUSED`/stop with Hector and CUDA registration only;
RTAB-Map is stopped. Runtime hashes match and no motion was commanded.

CUDA geometric registration is the mapping default as of 2026-11-21. The safe
launcher starts and verifies it, keeps driver registration as an explicit
rollback, raises temporary obstacle insertion from 2 Hz to 5 Hz, and stops
autonomy if registered planner depth is stale for more than one second. All 288
host and robot-native tests pass; target shell syntax and catkin rebuild pass
with existing clock-skew warnings. A no-option safe launch started fresh Hector
and left one mapper `AUTO PAUSED`/stop with CUDA depth near 15.65-15.69 Hz,
native collision depth primary, active fused cells, and TensorRT at 0.20-0.43 s.
No motion occurred. Failure, soak, labeled alignment, and directly supervised
obstacle-motion validation remain pending.

The generalized clear-footprint resurvey is deployed at commit `0ee8135`. All
279 host and robot-native tests pass. When no frontier route is reachable, the
mapper may use one of eight bounded zero-stride turns only if live LiDAR/depth
show the 0.35 m rotation footprint clear; occupied footprints still hold and
raw-only routes are never driven. Hector was preserved and one TensorRT mapper
is `AUTO PAUSED`/stop. No deployment motion was commanded; supervised motion
validation remains pending.

The fresh cumulative turn-progress fix is deployed at commit `38d0614`, with
the TensorRT-default launcher follow-up at `f04602d`. All 277 host and
robot-native tests pass. Turn windows require two new recent Hector poses, and
three weak windows pause only when their cumulative rotation remains below 3
degrees. A no-option safe launch selected TensorRT, preserved Hector, and left
one mapper `AUTO PAUSED`/stop; no deployment motion was commanded. Supervised
normal-turn and deliberately ineffective-turn checks remain pending.

The large-room Hector and clicked-route fix is deployed at commit `c2f45df` and
has 261 passing Windows-host and robot-native tests. It uses a Hector-only
0.50 m minimum range, three multiresolution levels, and a one-scan queue;
controller collision LiDAR remains raw and unchanged.
`/odom/filtered` has no publisher and the advertised IMU topics produced no
samples, so no unverified pose prior is enabled. Clicked/object goals now
survive fused-path replanning and directional-LiDAR recovery instead of falling
through to frontier exploration. Live parameters verified 0.5/3/1 on a fresh
Hector frame; stationary ten-second drift was under 1 mm / about 0.04 degrees.
Exactly one TensorRT mapper is AUTO PAUSED/stop with Hector only. No deployment
motion occurred; supervised open-room and clicked-route checks remain pending.

The fused-frontier resurvey fix is deployed at commit `0816ea5` from
2026-11-09. All 257 tests pass on the Windows host and natively on PT. When the
fused depth layer disconnects all routes but raw Hector still has a reachable
frontier, the mapper may perform up to eight zero-stride resurvey turns. Each
turn requires a clear 0.35 m rotation footprint and is followed by settled
depth/LiDAR evidence; the raw-only route is never driven. The robot holds if
the footprint is unsafe or the bounded attempts do not reopen a fused route.
Exactly one TensorRT mapper is `AUTO PAUSED` with command `stop`, Hector is the
only SLAM stack, and no deployment motion was commanded. Supervised live
follow-up then reopened a fused route and issued spin/forward commands; the
operator physically observed the robot, reported proper behavior, and marked
the test successful. Keep that physical confirmation separate from unit-test
and SLAM-telemetry evidence.

The deployed baseline is commit `3e4c185` (documented by `3ec1299`) from
2026-11-02. All 253 tests passed on the Windows host and natively on PT; the
same 253-test host suite passed again during the 2026-11-08 documentation
audit. Exactly one TensorRT mapper was last verified `AUTO PAUSED` with command
`stop` on a freshly reset Hector frame, and Hector was the only SLAM stack.
RGB, native/registered depth, LiDAR, pose, map, depth fusion, YOLO, and the web
UI were live. This is deployment and no-motion evidence: supervised
close-object, active turn-loss reset, sparse-depth motion, and other pending
physical checks in `RUN_SHEET.txt` remain outstanding.

The current baseline adds autonomous-frontier close-object verification,
map-retaining pause after repeated turn-progress loss, lower-floor depth
fallback for a sparse upper image, a 10 s minimum depth-cell hold followed by
positive-clear removal, automatic pose-jump reset, session-static landmarks,
moving display-only YOLO preview, and downloadable annotated video recording.

### Earlier validation checkpoints

The stale-magenta recovery candidate had 215 passing host tests. It lowered
only the unconfirmed-cell TTL to 5 s and made `RESUME` reject an invalid SLAM
map with an explicit `CLEAR MAP` instruction. It was superseded by the current
10 s minimum-hold/positive-clear policy and later deployments.

The low-leg follow-up was deployed on 2026-10-26 at commit `f13f296`. All 214
tests pass on the Windows host and natively on PT. The promotion threshold is
three settled LiDAR-paired depth observations, and active routes recheck their
next 0.80 m against the fused planner map. In a paused no-motion check the live
layer reported 3 temporary and 28 persistent depth cells. Exactly one
drive-enabled mapper is `AUTO PAUSED` with command `stop`; Hector is the only
SLAM stack. Directly supervised table/chair-leg motion validation is pending.

The rear-swept-clearance update was deployed on 2026-10-25. All 180 tests pass
both under the checked-in Windows shim runner and natively on PT. Exactly one
drive-enabled mapper is `AUTO PAUSED` with command `stop`; Hector is the only
SLAM stack. RGB, native/registered depth, LiDAR,
TF, `/slam_out_pose`, `/map`, the temporary depth planner layer, battery
telemetry, and the web UI are live. No motion command or physical movement
occurred during this deployment validation.

The live rear scan measured 0.134 m of swept travel before entering the 0.45 m
envelope; the new speed-scaled requirement is 0.060 m at 100% and 0.100 m at
200%. This is no-motion sensor evidence. The 0.5 s autonomous reverse recovery
still requires direct supervised physical validation.

The Objects365 exporter now materializes a final broadcast constant for PT's
legacy OpenCV 4.5.5 DNN implementation. The patched preferred model is active
on CPU and measured about 5.8-6.9 s per stationary inference. Repeated paused
captures mapped six tracks and confirmed two after three observations; this is
pipeline evidence, not detector-quality validation. Physical strafe/diagonal
motion, longer frontier motion, endpoint timing, and labeled-scene detector
quality remain pending supervised tests. Native depth is upper-only, while the
independent registered-depth planner overlay is active.

`/voltage` is `std_msgs/Float32`. The dedicated vendor `battery_low_alarm` node
is persistently excluded from both controller launch variants; launch backups
are stored beside them. Do not disable voltage telemetry or the website's
visual low warning.

After a reboot, RGB/depth may recover but LiDAR may require:

```bash
rosservice call /start_scan '{}'
```

If RGB has a publisher but no frames, use the web UI's `RECOVER CAMERA` button
while autonomy is paused, or run `recover_rgb_camera.sh`, then verify actual RGB
messages. This recovery has restored RGB
without interrupting depth, Hector, or the mapper. If it fails, reboot the
robot and restore LiDAR, Hector, and the mapper. If SLAM jumps or drifts while
stationary, discard the map and start a fresh Hector frame.

Always separate unit-test success, no-drive live evidence, SLAM-reported motion,
and physically observed robot motion in reports.
