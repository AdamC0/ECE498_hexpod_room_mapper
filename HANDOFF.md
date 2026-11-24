# JetHexa Room Mapper - Agent Handoff

## Registered-depth alignment candidate - 2026-11-23

The operator observed the CUDA registered overlay shifted up and right. This
can affect more than display: the registered stream also supplies semantic
depth lookup and low-obstacle planner projection. A diagnostic capture paired
raw RGB and registered depth within 11.1 ms and still showed the displacement,
so a display-only transform was rejected. The candidate applies `x=-12 px` and
`y=+10 px` to the projection principal point before publishing the shared
registered stream. Native collision depth is independent and unchanged. The
offsets are bounded ROS launch parameters, and a reusable synchronized capture
tool is under `tools/`. All 307 host tests pass; target build, robot-native
tests, paused visual alignment, and labeled planner alignment remain pending.

## Variable-opacity RGB/depth overlay deployed - 2026-11-23

The camera card adds an `OVERLAY` view and 0-100% opacity slider. Unlike the
standalone native-depth diagnostic, the overlay uses the CUDA registered-depth
image already aligned to RGB, avoiding a misleading pixelwise blend of the
native depth optical frame. Only valid registered pixels are blended; RGB stays
visible through holes, and the 0.2-4.0 m legend remains on-image. This is
display-only: YOLO, collision depth, fused planning, and RGB recording inputs
are unchanged. All 306 host and robot-native tests pass. Live 0%, 45%, and 100%
frames showed the intended opacity change, retained RGB through invalid holes,
and kept the legend visible. Runtime hashes match the workstation.

## Sparse-depth recovery turn deployed; motion check pending - 2026-11-23

The operator reported a clear physical area while autonomy remained stopped on
`insufficient valid depth in upper band`; live status showed 3.00 m forward
LiDAR clearance and only about 8% registered-depth coverage. The candidate may
make up to eight zero-stride resurvey turns, settling and rechecking native
depth after every burst. Missing depth is never labeled clear: each turn needs
the live LiDAR nearest range outside the 0.30 m circular rotation envelope and
the complete intersecting footprint to be `FREE` in the fused planner map.
Unknown, Hector-occupied, temporary-depth, or persistent-depth cells prohibit
the turn. Translation remains stopped throughout recovery. All 306 host and
robot-native tests pass. Commit `fa0bd50` is deployed with one mapper
`AUTO PAUSED`/stop, Hector and CUDA registration only, and matching runtime
hashes. Directly supervised motion remains pending.

The subsequent directly supervised run entered sparse-depth recovery. Status
reported eight bounded turns, registered-depth coverage remained about
5.1-8.7%, and the controller stopped after the configured limit. The operator
physically confirmed that the robot rotated. Hector-reported x/y stayed within
about 0.02 m, which is telemetry rather than physical translation evidence.
The robot was then returned to `AUTO PAUSED` with `Command: stop`; depth remains
invalid in the present view.

## Live native-depth dashboard view deployed - 2026-11-23

The camera card now has RGB and DEPTH buttons. DEPTH serves a colorized view of
the native collision stream (not the registered planner stream) with a fixed
0.2-4.0 m on-image legend: near is red, far is blue, and invalid/no-return is
black. RGB remains the default, retains YOLO boxes, and remains the only video
recording source. Switching views changes display only and does not alter any
sensor, safety, mapping, or recording input. All 299 host and robot-native tests
pass. Commit `39e8260` is deployed; the live page exposes both controls, RGB
and depth JPEGs are fresh, and the depth rendering visibly showed current
geometry, invalid black regions, and the 0.2-4.0 m legend. Runtime hashes match.
One mapper is `AUTO PAUSED` with `Command: stop`, Hector and CUDA registration
are the only production stack, and no motion was commanded.

After that paused validation, autonomy was resumed externally while deployment
documentation was being synchronized. Telemetry showed an ordinary clear-depth
translation and a Hector pose change from about `(7.35, -2.43, -114.3 deg)` to
`(7.39, -3.33, -102.7 deg)` before the agent noticed and issued PAUSE. The final
state is again `AUTO PAUSED`/stop. This was not an agent-commanded or physically
observed test and did not encounter a floor bump, so it is not validation of the
new retreat behavior.

## Bounded floor-bump retreat deployed; motion check pending - 2026-11-23

Live inspection of a reported stuck case showed a real 0.33-0.34 m native-depth
return from a chair's low horizontal base. The controller correctly classified
it as a floor bump, and rear LiDAR reported about 4.53 m of swept clearance,
but the prior policy categorically vetoed reverse for every floor bump. Its
status then incorrectly said 4.53 m was below the 0.10 m requirement.

The candidate permits only an immediate 0.5 s backtrack over a straight-forward
footprint observed clear within 1.5 s, with no more than 0.12 m pose separation,
no more than 5 degrees heading change, and the existing speed-scaled rear LiDAR
sweep. Depth dropout remains an unconditional reverse veto. The reverse clock
now starts when the back gait actually begins instead of expiring during the
stop/settle interval. Status reports the actual veto when recovery is unsafe.
All 299 host and robot-native tests pass. Commit `213ca8b` is deployed with
matching runtime hashes. The safe launcher preserved Hector and CUDA
registration and started exactly one mapper `AUTO PAUSED` with `Command: stop`;
no motion was commanded. Directly supervised floor-bump escape motion remains
pending. The restarted mapper currently reports upper-only native depth because
the present view could not learn enough floor rows, so that paused observation
does not validate the floor-bump motion path.

## RoomMapper domain split - 2026-11-22

The former 3,000-plus-line controller is now a 392-line composition root.
Initialization, sensor/SLAM ingestion, object processing, motion/safety, and web
control live in focused `mapper_*` modules. The public `RoomMapper` methods and
single controller-state owner are preserved, so existing ROS callbacks and
tests use the same interface. The launcher fails closed if any new runtime
module is missing. A regression test caps the facade at 500 lines and verifies
that representative methods are owned by the intended domain modules.

All 294 Windows-host and robot-native tests pass, production modules compile,
and both shell scripts pass Bash syntax validation. The no-option safe launcher
preserved healthy Hector and CUDA processes, verified the CUDA image and camera
info streams, and started exactly one mapper `AUTO PAUSED` with `Command: stop`.
The active mapper parameters select the CUDA registered image/camera-info,
5.0 Hz planner projection, and TensorRT FP16. Native and CUDA depth measured
about 15.6 and 15.8 Hz. One Hector process and one CUDA registrar were present,
with no RTAB-Map process. Workstation and deployed hashes match; seven model
files and three recording files remain. No movement was commanded.

## Cleanup, structure, and motion diagnosis deployed - 2026-11-22

Robot-side historical clutter was removed after an explicit inventory. The
project fell from 73 MB to 58 MB; the retained size is primarily `models/`
(49 MB) and operator-created `recordings/` (7 MB). Current source, launchers,
tests, and maintained documentation remain. Tests now live under `tests/`,
offline utilities under `tools/`, and historical plans/images under `docs/`.
No `backup_*`, loose `.bak`/`.pre_*`, root log/diagnostic, snapshot, or cache
artifacts remain. GitHub is the recovery source for the deployed code.

The web status now reports a stable `Why not moving: [CODE] detail` diagnosis
for pause, stale sensors, SLAM/map failures, startup guard, depth health,
object verification, missing routes, and active motion. Runtime-essential
`Command: stop` remains visible; scan-integration counters, model paths/backend,
and object queue internals were removed. `map_web.py` delegates static content,
recording lifecycle, and diagnosis to `web_dashboard.py`,
`video_recording.py`, and `controller_status.py` respectively.

All 293 host and robot-native tests pass, production Python compiles, and both
shell scripts pass Bash syntax validation. Global `-W error` is not usable on
the target because ROS Melodic itself calls deprecated `logging.warn`; the
normal native suite passes and exposes those external warnings. The initial
safe launch failed closed because the old stop proof had lived in a removed
developer-counter line. Commit `56f1f50` restored the explicit `Command: stop`
contract and added launcher coverage. The final launch verified one mapper
`AUTO PAUSED`/stop, one Hector process, one CUDA registrar, no RTAB-Map, live
dashboard/object inference, matching runtime hashes, and retained recording
metadata. No motion was commanded or physically observed.

## Default CUDA geometric depth registration - 2026-11-21

The Jetson has CUDA 10.2 and an `sm_53` GPU even though `nvcc` is not on the
default shell `PATH` (`/usr/local/cuda/bin/nvcc`). The earlier conclusion that
there was no CUDA configuration switch remains correct, but a standalone
registration node is implemented under `cuda_depth_registration/` and is now
the mapper's default registered-depth source.

Live inspection also corrected an assumption about the deployed stream. With
the vendor launch's `depth_registration=false`,
`/camera/depth_registered/image_raw` is a `32FC1` conversion/remap of the same
native depth frame, not RGB-registered depth: it carries the depth optical
frame and depth intrinsics. Dynamically enabling the driver's hardware mode
changed the stream to the RGB optical frame and RGB-family intrinsics at the
same 15.5-15.8 Hz rate, but the Astra exposes only one depth mode, so that
setting removes the independent native stream required for collision sensing.
It was restored to `false` immediately after the camera-only check.

The new node subscribes to native `16UC1` depth and both camera-info topics,
loads the factory right-to-left extrinsics from `/camera/get_camera_params`,
inverts that transform in CUDA, applies raw-RGB distortion, and resolves
collisions with an `atomicMin` nearest-depth z-buffer. It preserves the input
stamp and publishes a separate `16UC1` candidate in the RGB optical frame at
`/camera/depth_registered_cuda/image_raw`.

All 284 host tests pass. The package built successfully in the target catkin
workspace against ROS Melodic and CUDA 10.2. A stopped-mapper trial sustained
about 15.6-15.9 Hz, 0.44-0.57 ms kernel time, about 36 MB RSS, and about 5.4%
CPU over roughly 2,500 frames. A conservative two-neighbor/50 mm GPU hole fill
then measured 0.68-0.79 ms and raised candidate valid coverage from about
52.9% to 53.6%; the unregistered native alias was about 69.9%. The current
alias differs geometrically as expected (final 30-frame joint-pixel median
12 mm, p95 421 mm, valid-mask IoU 0.57), so those figures are diagnostic rather
than acceptance parity. The trial node was stopped and no motion was commanded.

An added one-shot benchmark runs the identical transform, distortion,
nearest-depth z-buffer, and hole-fill algorithm on one persistent-buffer CPU
implementation and CUDA using the same captured 640x480 frame. Three runs of
10 warmups plus 100 measured iterations produced CPU medians of 29.6071,
29.3034, and 29.0879 ms; CUDA end-to-end medians (including copies) of 1.6596,
1.6460, and 1.6758 ms; and CUDA kernel medians of 0.7120, 0.7042, and 0.7102 ms.
End-to-end speedup was 17.84x, 17.80x, and 17.36x. Every run had valid-mask IoU
1.000 and median/p95 error 0 mm, so the timing comparison did not trade output
parity for speed. The benchmark publishes nothing and no motion was commanded.

The operator clarified that the goal is faster geometric obstacle insertion
for walls, chair legs, boxes, and similar collision geometry, not faster YOLO
labels. `room_mapper.py` therefore defaults its registered image and calibration
to the CUDA namespace, and geometric planner projection now runs at 5 Hz rather
than 2 Hz. A first accepted frame is immediately a temporary blocking cell, so
the software scheduling bound falls from about 0.5 s to about 0.2 s; the three
settled observations still govern session promotion only.

`start_room_mapper.sh` starts and verifies the CUDA node before the mapper and
fails closed if either CUDA image or camera-info output is missing. Use
`--registered-depth-backend driver` only for explicit rollback. Native
`/camera/depth/image_raw` remains unchanged for the 0.60 m collision stop. The
288-test host and robot-native suites pass, robot shell syntax passes, and the
target CUDA package rebuild succeeds with the existing clock-skew warnings. A
no-option safe launch started a fresh Hector frame and exactly one mapper in
`AUTO PAUSED` with command `stop`. CUDA depth published near 15.65-15.69 Hz,
native collision depth was `PRIMARY`, the fused layer reported live temporary
and persistent cells, and TensorRT inference reported 0.20-0.43 s. No motion
was commanded. Labeled alignment, TensorRT-contention soak, failure injection,
thermal validation, and directly supervised geometric-obstacle motion remain
pending.

Once autonomy is active, registered planner depth older than one second now
requests `stop` and enters `FAULT`; the mapper no longer continues on a stale
CUDA fused layer. Paused capture and manual dead-man behavior are unchanged.

## Depth pipeline CPU reduction - 2026-11-16

Live load sampling found all four CPU cores around 75-100%, with the Astra
nodelet manager near 58%, the mapper near one core, and Hector near one core.
The GPU was idle outside TensorRT bursts. The mapper is the only subscriber to
the native and registered raw depth images; both XYZ and XYZRGB point clouds
had no subscribers. The installed OpenCV 4.5.5 has no CUDA modules, so there is
no safe configuration switch that moves the ROS Melodic depth graph to GPU.

The deployed vendor patch disables unused RGB/depth processing and point-cloud
nodelets and selects the Astra driver's 640x480@15 mode (`depth_mode=68`) rather
than 640x480@30 (`69`). The original vendor file is recoverable at
`Halley.launch.backup_pre_63924a7`. After a camera-stack restart, only
`/camera/driver` was loaded; native and registered depth remained near
15.5-15.8 Hz, synchronized RGB/depth inference resumed, and depth sensing was
primary. Nodelet-manager CPU measured about 16%, versus about 58% before the
change (the reversible trial measured 32-35%). All 280 host and robot-native
tests pass. Exactly one TensorRT mapper is `AUTO PAUSED`/`stop`, Hector and its
map were preserved, and no motion was commanded. Full-reboot persistence and
supervised motion/depth-stop validation remain pending.

## Clear-footprint route resurvey - 2026-11-16

Live autonomy was active with command `stop` and `No reachable frontier` even
though current LiDAR reported 3.00 m. The raw map contained 506 frontier cells,
but only six walkable cells were connected to the robot; start clearance was
0.439 m against the 0.43 m planning requirement. The existing resurvey turn was
restricted to cases where raw Hector had a route and only the fused layer
blocked it, so this pinched raw-map case could never collect a new view.

The candidate generalizes the same bounded zero-stride resurvey: any failed
frontier plan may rotate when the live 0.35 m LiDAR/depth rotation footprint is
clear. It remains limited to eight turns with settling and replanning after
each; an occupied footprint still holds, and raw-only routes remain diagnostic
and are never driven. All 279 host and robot-native tests pass. Commit
`0ee8135` is deployed after backup to `backup_pre_0ee8135`, with matching
controller/test hashes. The safe TensorRT restart preserved Hector and left one
mapper `AUTO PAUSED` with command `stop`; no motion was commanded. Supervised
physical validation remains pending.

## TensorRT launcher default - 2026-11-16

Per operator request, `start_room_mapper.sh` now selects `tensorrt_fp16` by
default. Engine load, checksum, warm-up, or runtime inference failures retain
the existing permanent fallback to the same preferred model on CPU, followed
by the retained YOLOv5n model fallback if necessary. Explicit CPU startup
remains available with `--yolo-backend cpu`.

All 277 tests pass on the Windows host and natively on PT. Commit `f04602d` is
deployed after backup to `backup_pre_f04602d`; a no-option launcher restart
reported `tensorrt_fp16`, preserved the running Hector process, and left one
mapper `AUTO PAUSED` with command `stop`. No motion was commanded.

## False autonomous-turn pause correction - 2026-11-16

The operator reported repeated map resets without a visible pose jump. Live
inspection found the robot unexpectedly autonomous and issuing translation, so
it was immediately paused; no further motion was commanded. ROS history showed
multiple resets explicitly caused by `repeated turns made insufficient heading
progress`, plus a marginal single-message `pose jumped 0.26 m / 4 deg` reset.
The former conflated an ineffective/blocked physical turn with localization
loss, while the latter had no temporal confirmation.

Commit `371279a` first changed that condition to a map-retaining pause. The
operator then confirmed that the robot was physically rotating and the map was
stable, but the pause recurred. The detector sampled each fixed 0.5-second
window independently and could classify delayed Hector callbacks or three slow
sub-3-degree windows as a physical stall.

The new candidate requires at least two new, recent Hector poses before it
classifies a turn window. It measures cumulative heading change across the
three bursts, so three slow windows that total at least 3 degrees clear the
stall counter. Three evidence-backed bursts totaling less than 3 degrees still
pause and retain the map. All 277 Windows-host and robot-native tests pass under
warnings-as-errors. Commit `38d0614` is deployed with matching runtime hashes;
the safe restart preserved Hector and commanded no motion. Supervised physical
validation remains pending.

## Bounded unconfirmed depth cells - 2026-11-16

Live paused diagnosis found 96 temporary and 73 persistent depth cells. The
registered-depth layer admitted a brief clustered hit immediately, but the
positive production minimum hold disabled its legacy TTL forever. A cell that
never repeated enough to promote could therefore accumulate for the entire
session even when it was only transient depth noise. The live camera/map also
showed real chair casters and people mixed into the magenta layer, so clearing
the layer wholesale or weakening persistent clearing was rejected.

The candidate retains immediate blocking and the 10 s minimum hold. A cell
that remains unconfirmed now expires 60 s after its latest occupied hit;
another hit restarts the limit, and promotion still makes it session-persistent
until four synchronized depth/LiDAR free observations clear it. Valid measured
surfaces beyond the 2.0 m occupied radius may prove only the nearer ray free and
never create a farther obstacle endpoint. `tools/diagnose_depth_projection.py`
provides a read-only live comparison. All 272 host and robot-native tests pass
under `-W error`. Commit `6f07812` is deployed with matching runtime hashes and
an exact pre-change backup under `backup_pre_6f07812`.

The first preserved Hector frame drifted 0.065 m / 3.69 degrees in a 15 s
stationary check and changed 43 raw map cells, independently explaining visual
depth/map misalignment. It was discarded per the safety rule. The fresh frame
varied 0.018 m / 0.61 degrees and changed zero raw cells over 15 s. Exactly one
TensorRT mapper remains `AUTO PAUSED` with command `stop`, Hector is the only
SLAM stack, and no motion was commanded. After the fresh reset, temporary cells
fell from 4 to 2 in the final delayed sample while repeated current chair/person
geometry promoted from 15 to 19 persistent cells. A controlled 60 s transient
and promoted-cell retention scene check remains pending.

## Object landmark reliability deployed - 2026-11-16

Offline review found that `_object_merge_distance:=0.65` was also used for
track association even though confirmation accepts only 0.25 m expected-class
position spread, 0.20 m other-class spread, or the configured large-object
tolerance. Two nearby same-class objects could therefore feed one track that
could never confirm; confirmed tracks and manual suppression could also claim
a neighboring instance. Association and suppression now use the applicable
confirmation tolerance capped by the existing merge distance.

Registered-depth sampling still uses the protected central crop and unchanged
quality gates, but now returns the representative pixel of its coherent depth
inliers for back-projection rather than always using the YOLO box center. Clear
cross-model synonyms normalize TVs, boards, shelves/cabinets, trash bins, and
luggage to the established policy labels. Per-inference status explicitly
counts boxes rejected for insufficient depth, unreliable depth, or missing
image-time map TF. Confidence thresholds, three-hit/eight-frame validation,
box overlap, settled capture, synchronization, and map-trust gates are
unchanged.

All 269 Windows-host and robot-native tests pass under `-W error`; production
Python compiles. Commit `f15327d` was deployed with byte-matching hashes after
the eight previous files were backed up under `backup_pre_f15327d`. Post-reboot
LiDAR required the documented `/start_scan` service. A fresh Hector frame and
exactly one TensorRT mapper are live `AUTO PAUSED` with command `stop`; no
RTAB-Map process exists and no motion was commanded. Scan/pose measured about
12.0/10.9-12.0 Hz, RGB/native/registered depth about 27-31 Hz, and `/voltage`
reported 12.62 V / 100%.

The stopped live view processed 32 of 33 snapshots with no replacements. It
reported two detections, zero mapped, and the new explicit reason `2
insufficient registered depth`. This is deployment and diagnostic-path
evidence, not a labeled detector/placement validation. Run the remaining scene
checks in `RUN_SHEET.txt` before claiming improved real-object accuracy.

## Large-room Hector and clicked-route fix deployed - 2026-11-09

Live diagnosis found `/odom/filtered` has no publisher. `/imu/raw` and
`/imu/filtered` have registered publishers but produced no samples during a
12-second rate check, so no unverified pose prior is enabled. The candidate
instead applies the hardware-backed Hector tuning: exclude the verified
0.34-0.40 m moving body/leg returns with `laser_min_dist=0.50`, use three
multiresolution scan-matching levels, and reduce the subscriber queue to one.
Raw `/scan` remains continuous and the controller's collision path remains
unfiltered.

The clicked-route stop was a separate state-machine bug. A fused-prefix or
directional-LiDAR invalidation cleared the selected path and changed its phase
to generic frontier planning. The candidate retains the requested click,
replans around new fused occupancy, and performs bounded recovery for a
directional LiDAR block. It pauses with an explicit no-detour message only when
the selected goal truly has no reachable known-free route.

Commit `c2f45df` passes all 261 tests on the Windows host and natively on PT.
The nine changed files were backed up under `backup_pre_c2f45df` and copied
with matching hashes. Hector was deliberately restarted on a fresh frame so
the new parameters took effect; live ROS parameters verified `0.5 / 3 / 1` for
minimum range, multiresolution levels, and scan queue. Scan/pose were about
12.1/12.3 Hz and all camera streams about 29-31 Hz. Stationary ten-second drift
was under 1 mm and about 0.04 degrees. Exactly one TensorRT mapper is
`AUTO PAUSED` with command `stop`, Hector is the only SLAM stack, and no
deployment motion was commanded. Supervised open-room and clicked-route motion
remain pending.

## Fused-frontier resurvey deployed - 2026-11-09

Live read-only diagnosis found autonomy healthy in `SEARCH/plan` but stopped on
`No reachable frontier`. The fused layer held 218 temporary and 89 persistent
depth cells, while a replay of the same raw Hector map found a 15-waypoint
frontier route. The camera showed nearby furniture, so bypassing the fused
layer would be unsafe.

The deployed revision distinguishes this condition with a diagnostic-only
raw-Hector probe. It never drives the probe route. When only retained depth
occupancy disconnects the frontier, it may execute up to eight zero-stride
turns, each requiring a clear 0.35 m rotation footprint and a settled
depth/LiDAR observation before replanning. It holds if rotation is unsafe or
the bounded attempts do not reopen a fused route.

Commit `0816ea5` passes all 257 tests on the Windows host and natively on PT.
The eight changed files were backed up under `backup_pre_0816ea5`, copied with
matching hashes, and the existing Hector frame was preserved. Exactly one
TensorRT mapper is `AUTO PAUSED` with command `stop`; Hector is the only SLAM
stack. Scan/pose measured about 12.1/10.2 Hz, RGB 30-32 Hz, native depth
28-31 Hz, and registered depth 29-33 Hz. Voltage recovered near 1 Hz and last
reported 11.68 V (about 67%). No motion was commanded during deployment, so
the bounded live resurvey and subsequent route reopening initially remained
pending direct supervision.

Supervised follow-up: the operator resumed autonomy in the reproduced scene.
The controller reopened a fused route, entered `SEARCH/go`, and issued spin and
forward commands while the normal depth/LiDAR gates remained active. The
operator physically observed the behavior, reported that it worked properly,
and explicitly marked the test successful. This is direct supervised physical
validation, separate from the 257-test results and SLAM-reported pose changes.

## Repository/documentation audit - 2026-11-08

The repository was clean and synchronized with `origin/main` at commit
`3ec1299`. No production-code changes have landed after the 2026-11-02
close-object/turn-loss deployment documented immediately below. The complete
Windows host suite passed again: 253 tests in `tools/host_test_runner.py`.

`README.md`, `RUN_SHEET.txt`, and the newest `DECISIONS.md` entries already
match the deployed feature set and outstanding supervised checks. `AGENTS.md`
was brought forward from its superseded October 26 "latest validation" section
to the 253-test `3e4c185` deployment, and its architecture/safety summary now
captures moving display-only YOLO, bounded autonomous close-object stops,
lower-floor sparse-depth fallback, automatic turn-loss reset, and the 10-second
depth-cell minimum hold. This audit did not connect to the robot, deploy files,
issue motion commands, or change the last live-state evidence. Treat the
November 2 paused/no-motion observations below as the latest robot evidence.

## Close-object verification and turn-loss reset deployed - 2026-11-02

The approved autonomous-only policy uses moving YOLO as a stop request, then
requires settled registered depth inside 3.0 m. A candidate matching a track
that existed before the stop resumes immediately. A novel candidate captures
five distinct settled synchronized RGB/depth/TF observations; the stop lasts
at most 15 seconds, after which frozen observations may finish through the
normal capture-time-TF pipeline. Failed/known class-and-image-third candidates
receive a 20-second cooldown. Manual drive and `user_go` clicked/object routes
are excluded.

The live `SLAM map invalid; automatic reset failed or is unavailable` report
was traced to repeated insufficient turn progress: that path invalidated SLAM
but, unlike pose jumps, did not queue the reset. Both turn-progress loss paths
now schedule the same one-shot automatic reset. Actual service failures are
stored and reported separately. All 253 workstation tests pass and production
Python compiles. Robot-native validation, deployment, and supervised motion
validation were pending. The live robot was stopped in FAULT during diagnosis.

Deployment follow-up: all 253 tests passed natively after sourcing the JetHexa
ROS workspace. Commit `3e4c185` was copied with matching hashes after backup to
`~/room_mapper/backup_pre_3e4c185/`. The invalid Hector frame was explicitly
reset. Exactly one TensorRT mapper is now AUTO PAUSED with command `stop`, pose
near the fresh origin, and Hector is the only SLAM stack. RGB, depth, LiDAR,
pose, map, registered-depth fusion, YOLO, and the web UI are live. No physical
motion was commanded. Supervised close-object and active turn-loss checks are
still pending.

## Sparse-upper lower-floor fallback deployed - 2026-11-02

Live read-only sampling found 286/286 native-depth frames with zero valid
pixels throughout both the primary and widened upper image, despite a normal
RGB classroom view. Current depth existed only around source rows 285-469,
mostly floor. The old paused status looked ready because collision evaluation
does not run while paused; resuming would fault immediately.

Floor calibration now accepts a row with 10% rather than 30% central coverage,
while still requiring enough rows in at least 70% of calibration frames. A
calibrated, currently valid lower-floor region may supply bump/dropout sensing
with LiDAR and retained planner cells when the upper band is sparse. Complete
lower dropout remains a 0.30 m stop, and no usable floor model preserves the
existing upper-sparsity fault. All 250 tests pass on the host and natively on
PT. Commit `d03e7a6` was deployed after backup to
`~/room_mapper/backup_pre_d03e7a6/`, preserving the fresh Hector frame. The
production mapper learned 41 usable rows; an independent live-frame evaluation
learned 38 and returned `ready (lower-floor + LiDAR; upper sparse)` with clear
range. Exactly one mapper remains paused/stopped with Hector only. No physical
motion was commanded; supervised motion, bump, and dropout checks are pending.

## Ten-second depth-cell hold deployed - 2026-11-02

Temporary and promoted depth-sourced planner cells now retain the timestamp of
their latest occupied observation. Each hit restarts a hard 10-second hold.
Temporary clear observations and joint depth/LiDAR clear evidence are ignored
during that interval and cannot pre-accumulate. Afterward, the normal two-clear
temporary or four-clear joint policy must complete; time alone does not remove
an out-of-view cell. This is configured by `_depth_map_minimum_hold:=10`.
The 2026-11-16 bounded-unconfirmed policy above supersedes that rule for cells
that never promote; the positive-clear-only rule still applies to promoted cells.

Planner/Hector reset, mapper restart, and automatic pose-jump recovery still
clear the state immediately. The legacy TTL path remains only for direct
library callers that explicitly select a zero minimum hold; production
configuration requires at least one second. All 247 tests pass on the host and
natively on PT. Commit `81a3c28` was deployed after backup to
`~/room_mapper/backup_pre_81a3c28/`, with an explicit fresh Hector reset and one
paused TensorRT mapper. Live status showed `10 s minimum hold; clear evidence
required`. No motion was commanded. A measured-object timing check is pending.

## Automatic pose-jump reset deployed - 2026-11-02

Crossing the existing 0.25 m / 20 degree pose-jump threshold now marks the map
untrusted, stops motion, and schedules exactly one automatic reset from the
serialized controller loop. Successful recovery clears Hector, the fused
planner and its persistent depth cells, routes, object snapshots, and semantic
landmarks. It then requires a fresh map and the existing five-second pose guard.
Autonomy is restored only if it was active before the jump; paused/manual
sessions remain paused. Manual Clear Map still always pauses.

Reset-service failure clears the pending request and leaves the robot paused
with an invalid map, avoiding an unbounded retry loop. All 245 Windows-host
tests pass and production Python compiles. All 245 tests also pass natively on
PT. Commit `52f177e` was deployed after backup to
`~/room_mapper/backup_pre_52f177e/` and an explicit fresh-map startup. A paused
synthetic 1.00 m pose jump triggered exactly one automatic reset, returned the
live pose to approximately `(0,0)`, left zero persistent/temporary depth cells,
and correctly remained `AUTO PAUSED` with command `stop`. Hector is the only
SLAM stack and exactly one mapper is running. No physical motion was commanded;
active-session conditional resume remains pending supervised validation.

## Static landmark retention deployed - 2026-11-02

Confirmed tracks whose canonical class is in `_object_static_classes` are now
session-persistent and ignore observation-based YOLO absence removal. They
remain normal solid map markers and show `STATIC` in the Objects tab. Static
status begins only after normal three-hit confidence/depth/geometry
confirmation; pending furniture retains the five-miss and 300-second expiry
paths. Manual Remove, Clear Map, mapper restart, and SLAM invalidation still
clear static landmarks.

Defaults cover chair, desk/table variants, cabinet/shelf, bench, stool, couch,
bed, blackboard/whiteboard, sink, oven, refrigerator, toilet, power outlet,
clock, podium, and lectern. The consequence is deliberate: moved furniture
leaves its old semantic marker until explicit removal or map clearing. Planner
occupancy is unchanged and remains governed by Hector/depth fusion. All 242
tests pass on the Windows host and natively on PT. Commit `1e857cb` was deployed
after backing up the changed files under
`/home/hiwonder/room_mapper/backup_pre_1e857cb`. Exactly one drive-enabled
mapper was restored `AUTO PAUSED` with command `stop`; Hector remained the only
SLAM stack. The restarted scene reported no mapped detections, so a live
confirmed-furniture retention check remains pending. No deployment motion was
commanded.

## Moving YOLO preview deployed - 2026-11-02

YOLO now admits bounded RGB-only snapshots while motion is commanded. These
results are presentation-only for the live camera and video recorder: no depth
sampling, TF projection, tracker update, confirmation, or absence evidence is
possible. The result is displayed only when its captured motion generation
still matches the controller, preventing a slow inference from appearing after
a stop or state transition. At most one moving snapshot waits in the queue;
newer motion frames replace that slot, preventing a backlog. Stale and
duplicate RGB frames are rejected.

The transition into motion still clears stationary boxes, but fresh moving
detections are no longer suppressed by the web renderer. Stationary settled
captures remain validation-eligible and are unchanged. All 239 host and
robot-native tests pass and production Python compiles.

Commit `80317b7` is deployed with matching hashes after backup to
`~/room_mapper/backup_pre_80317b7/`. The launcher preserved Hector and restored
exactly one drive-enabled TensorRT mapper `AUTO PAUSED` with command `stop`.
The paused pipeline processed fresh stationary detections normally; this is
deployment evidence only. No motion was commanded or physically observed, so
live moving-box cadence, recorded moving overlays, and unchanged track counts
during actual movement still require a directly supervised labeled check.

## RGB + YOLO video recording deployed - 2026-11-02

The web camera card adds start, stop, and download controls for a robot-local
640x480, 10 FPS annotated recording. The recording renderer contains raw RGB
plus only fresh YOLO boxes, labels, confidence, and sampled range; it omits the
live page's LiDAR/depth line, paused banner, map, and other UI overlays. Files
are timestamped in `recordings/`; MP4V/MP4 is preferred with MJPEG/AVI fallback.
The newest completed recording remains downloadable after page refresh or
mapper restart.

Recording refuses missing RGB, concurrent starts, and less than 256 MB free.
Stop and mapper shutdown release the writer before the file becomes
downloadable. Downloads are streamed in 1 MB chunks. All 235 host and
robot-native tests pass and production Python compiles.

Commit `1ec981b` is deployed with matching hashes; the previous files are in
`~/room_mapper/backup_pre_1ec981b/`. The live target selected MP4V and created
`yolo_20230302_220404_384.mp4`: 640x480, 10 FPS, 104 readable frames, about
3.17 MB. Downloading through `/recording/download` produced the same SHA-256
as the stored file. An extracted real frame contained the clean RGB scene with
no LiDAR/depth or pause overlay. Exactly one TensorRT mapper remained
`AUTO PAUSED` with command `stop`, Hector was preserved, and no motion was
commanded or physically observed. The root filesystem had about 3.8 GB free
before the short recording; recordings are retained, so storage still requires
operator management.

## Depth-hole object-removal correction deployed - 2026-11-02

The operator reported stale person and skateboard markers directly ahead. Live
projection placed them near pixels `(146,174)` and `(217,327)`, safely inside
the 640x480 inset, but both 9x9 registered-depth patches had zero valid pixels.
The old rule therefore supplied no clear-view evidence. The stale person also
received age refreshes from a different same-class box overlapping the old
pixel direction.

Chosen: when a saved point is central and YOLO has no matching box, invalid
local depth now counts as semantic absence; a valid nearer return still pauses
removal as occluded. This never clears planner occupancy and automatic marker
removal remains rediscoverable. Overlapping same-class boxes only reset the old
track when their sampled depth is absent or agrees within 0.25 m / 25%.
Out-of-view and positively blocked tracks keep the age timer. All 232 host and
robot-native tests pass and production Python compiles. Commit `89bc4aa` is
deployed; the previous files are in `~/room_mapper/backup_pre_89bc4aa/`.

The production launcher preserved the live Hector frame and restored exactly
one drive-enabled TensorRT mapper `AUTO PAUSED` with command `stop`. The old
near-field person (about 0.76 m) and skateboard (about 0.80 m) session markers
were absent after restart; a newly observed person at a different map location
confirmed from fresh captures. This proves stale-session cleanup and healthy
operation, not a controlled live five-miss reproduction, which remains pending.
No motion was commanded or physically observed.

## Pending-object clear-view removal deployed - 2026-11-02

Pending candidates now use the same five eligible clear-view misses as
confirmed landmarks. A new private absence counter is independent of the
rolling confirmation evidence, preventing confirmation misses from
accidentally accelerating removal. Same-class boxes at the projected point
reset absence and refresh pending `last_seen` even if depth quality prevents a
new map observation. At this checkpoint, out-of-view, edge, invalid-depth, and
nearer-occluded frames added no absence evidence; the later depth-hole
correction documented above changed central invalid-depth misses to count. The
existing 300 s pending age timer remains the fallback for non-observable
conditions. Automatic removal still permits rediscovery.

All 231 host and robot-native tests pass and production Python compiles. Commit
`a271bae` is deployed with matching hashes after a recoverable backup to
`~/room_mapper/backup_pre_a271bae/`. The launcher preserved the existing Hector
frame and restored exactly one drive-enabled TensorRT mapper `AUTO PAUSED` with
command `stop`. Live JSON showed two confirmation misses but zero absence
misses for one pending track, confirming the counters are independent in the
deployed pipeline. LiDAR/pose measured about 12.14/12.05 Hz and
RGB/native/registered depth about 29.60/28.49/29.83-30.42 Hz. Camera and map
endpoints returned HTTP 200 and battery was about 90%. Controlled five-miss,
occlusion, age-fallback, and rediscovery scene checks remain pending. No motion
was commanded or physically observed.

## Visibility-aware confirmed-object removal deployed - 2026-11-02

Confirmed landmarks now retain Z as well as map X/Y. Each fresh synchronized
stationary result projects that saved point into the captured RGB/depth view.
A YOLO miss counts only inside the central 80% of the image when registered
depth is valid and reaches the expected object range. A nearer depth return
marks the view occluded; at this checkpoint invalid depth, missing TF, and
edge/out-of-view points added no absence evidence. The later depth-hole
correction documented above changed central invalid-depth misses to count. A
matching YOLO box or mapped observation resets the counter. Five consecutive
eligible misses remove the landmark, including a
renamed one, without adding manual suppression, so later rediscovery remains
possible. Pending-track behavior is unchanged.

All 228 host and robot-native tests pass and production Python compiles.
Commit `eefe879` is deployed with matching hashes after a recoverable backup to
`~/room_mapper/backup_pre_eefe879/`. The launcher preserved the existing Hector
frame and restored exactly one drive-enabled TensorRT mapper `AUTO PAUSED` with
command `stop`. Live object JSON exposes `z` and `absence_misses`; the normal
paused pipeline confirmed one track after restart. LiDAR/pose measured about
12.05/11.97 Hz and RGB/native/registered depth about 29.85/29.54/29.87-30.05
Hz. Camera/map endpoints returned HTTP 200 and battery was about 91%. This is
native-test, deployment, and no-motion pipeline evidence. Controlled occlusion,
five-miss removal, reset-on-detection, and rediscovery scene checks remain
pending. No motion was commanded or physically observed.

## Live-map false-positive, duplicate, and clearing candidate - 2026-11-01

Follow-up diagnosis after the operator physically repositioned the paused robot
reproduced the clearing failure: 57 persistent cells remained across six
settled updates, 3-4 joint-free cells were reported, but no persistent cell
accumulated evidence. A read-only status diagnostic now separates total
depth/LiDAR free cells and persistent-cell overlap before and after joint
agreement and the occupied guard. All 223 host tests pass; paused native
deployment is pending. Do not classify the operator-reported pose change as
SLAM drift.

Live diagnostic deployment then showed why: immediately after mapper restart,
the current scene promoted 39 cells; depth marked all 39 currently occupied,
LiDAR marked 11 free, and 9 had pre-veto paired agreement. A registered-depth
sample showed a real background wall at roughly 0.7-0.9 m plus nearer furniture,
not floor leakage. Raising the height cutoff from 0.025 to 0.15 m reduced only
39 to 36 occupied cells. The stale removed-object cell can sit immediately in
front of the newly visible wall cell, so the old 3x3 occupied guard vetoes it
forever even when both sensors mark the exact old cell free.

The 224-test fix lets exact free evidence from both sensors override only an
adjacent endpoint; an occupied return in the exact cell still wins immediately,
and one-cell-tolerant free agreement still retains the full neighbor guard.
Commit `9410ba4` passed all 224 tests on the host and natively on PT and is
deployed with one TensorRT mapper `AUTO PAUSED` on command `stop`. Live status
showed zero exact-free overlap with the 39-40 current persistent cells; all were
protected by current depth endpoints, consistent with the visible wall and
furniture. Three stale Hector-black cells were suppressed downstream after
exact joint-free evidence; raw Hector was unchanged. A controlled stationary
promote/remove check remains pending. Autonomy stays out of scope.

The operator reported two nearly coincident `power outlet` objects, a person's
leg confirmed as `chair` under the relaxed 0.30 policy, and removed persistent
depth obstacles sometimes failing to clear while visible. Live inspection
found outlet tracks at `(0.969, -0.041)` and `(0.975, -0.056)` m, only about
1.6 cm apart. The false chair was confirmed with about 0.459-0.506 confidence.
The fused layer showed 47 persistent cells but zero jointly cleared cells.

The 223-test candidate deduplicates same-frame outlet projections within
0.10 m, keeping only the highest-confidence observation. It canonicalizes
`backpack`, `suitcase`, and `luggage` as one luggage track. General expected
classes return to 0.50 minimum / 0.55 median confirmation; only luggage retains
the measured 0.30/0.33 override. The detector admission threshold is unchanged,
so weak detections can remain visible and pending without confirming.

Persistent clearing continues to require four positive settled depth/LiDAR
free observations and an immediate exact-cell occupied veto. Free-ray cells may
differ by one grid cell, and partial positive evidence remains usable for 5 s
across missing pairs; missing data itself adds no evidence. Status now exposes
joint-free cells and partial-evidence count/max. Raw Hector is never modified.
All 223 tests pass on the host and natively on PT. Commit `63d88cb` is deployed
with matching hashes. Exactly one drive-enabled TensorRT mapper is `AUTO PAUSED`
on command `stop`; Hector is the only SLAM stack. The live layer reported 34
persistent cells, one joint-free cell, and no partial clear candidate. The
post-restart view contained no object detections, so outlet deduplication,
false-chair rejection, and a measured remove-and-clear check remain pending.
No motion was commanded.

## Rolling endpoint object confirmation deployed - labeled check pending

The operator confirmed that objects should be able to reach confirmed state
while the robot remains stopped at one endpoint and accepted three strong hits
within eight fresh stationary observations. The current candidate makes the
single latest post-burst snapshot validation-eligible rather than display-only,
while retaining unique timestamps, synchronized RGB/depth, settled motion,
trusted SLAM, image-time TF, depth quality, confidence, box-overlap, and map-
position gates. Both class tiers require three hits in eight; non-priority
classes retain their higher confidence and tighter geometry thresholds.

The first labeled live check exposed a remaining threshold mismatch rather than
a capture or association failure. A stationary backlit backpack accumulated six
mapped observations with about 1 cm position standard deviation and 0.334-0.359
confidence, but the old expected-class gate required 0.50 per hit and 0.65
median even though the detector admits expected classes at 0.30. The follow-up
candidate aligns expected-class confirmation to 0.30 per hit and 0.33 median;
three-of-eight, depth, TF, overlap, and position gates remain. The other-class
0.60/0.75 policy is unchanged, so a simultaneous weak `sneakers` classification
does not receive the relaxed policy. Robot-native validation and deployment of
this threshold follow-up were completed at commit `f603ecf`; all 217 tests pass
on the host and natively on PT. Hector's live map was preserved and exactly one
TensorRT mapper was restarted `AUTO PAUSED` on command `stop`.

The initial post-restart frame remained strongly backlit and the foreground
shoe partly occluded the backpack. The detector then emitted only `sneakers`
and `leather shoes`, not `backpack`; both other-class tracks correctly remained
pending under the unchanged 0.60/0.75 gate. Repeat with the shoe removed and
the backpack front lit before evaluating the new expected-class confirmation
threshold. No motion was commanded.

The operator then explicitly selected a visible `luggage` object instead of the
backpack. It accumulated 25 mapped observations, about 2 mm position standard
deviation, and confidence up to 0.538, but remained in the other tier solely
because `luggage` was absent from the expected-class list. The current follow-up
adds exact Objects365 `luggage` and fallback COCO `suitcase` to the expected
tier. No unrelated-class threshold changes.

Commit `ecc4b60` passed all 217 host and robot-native tests and is deployed with
the live Hector map preserved. The paused `luggage` track progressed from 1/3
pending to confirmed after three qualifying observations at approximately
`(0.659, 0.240)` m. Maximum confidence was 0.439 and position standard deviation
about 0.0011 m. At the confirmation snapshot, the queue reported 31 captured /
30 processed, zero replacements, and 0.31 s inference. Exactly one TensorRT
mapper remained `AUTO PAUSED` on command `stop`; no motion was commanded. This
validates stationary confirmation for this scene, not broad detector quality.

Two adjacent detector misses no longer delete a pending track. Misses remain in
the rolling window and pending tracks still expire through the existing age
limit. The processing horizon is 45 s so the measured CPU fallback can drain a
five-frame burst, and the Objects tab exposes the exact reason on a separate
`WHY PENDING` line. All 216 tests pass on the host and natively on PT. Commit
`3b85e0c` is deployed after reconciling the robot's complete file set from its
previous `7c0c9df` deployment; the previous files are recoverably stored in
`~/room_mapper/backup_pre_3b85e0c/`.

The production launcher started a fresh Hector map and exactly one drive-enabled
TensorRT mapper `AUTO PAUSED` with command `stop`; RTAB-Map is absent. A paused
empty-scene check advanced 65 captured / 65 processed snapshots with zero
replacements and 0.28-0.62 s inference. Rates were about 12.04 Hz scan, 12.08 Hz
pose, 29.73 Hz RGB, 29.75 Hz native depth, and 29.68 Hz registered depth. The
page serves `WHY PENDING`. No object was detected in the current view, so a
paused labeled-object promotion check remains pending. No motion was commanded.

Updated: 2026-11-01

## Dynamic fused occupancy deployed - measured-object validation pending

The latest stale-magenta candidate lowers unconfirmed depth occupancy from a
30 s to a 5 s fallback lifetime; two positive clear views still remove it
sooner, while confirmed black occupancy remains session-persistent. `RESUME`
now refuses a latched invalid SLAM frame and directs the operator to `CLEAR MAP`.
All 215 host tests pass; robot-native validation and deployment are pending.

Follow-up live diagnosis found that promoted cells were being re-added to the
temporary render mask on their next depth hit, so magenta visually covered the
correct persistent black state. Commit `732ac55` removes persistent cells from
the temporary dictionaries after every fused observation and adds a
post-promotion-frame regression. After a fresh map, Hector TF arrived about
0.6 s behind registered depth; commit `d3ea8bb` permits up to 1.0 s TF age only
while stopped and settled, retaining 0.5 s during motion. The live paused map
then reported 5 temporary cells, 9 persistent black cells, and 1 joint-cleared
Hector cell; the rendered map visibly separated black and magenta cells.

At the operator's request, commit `e7cee65` also increases the planner's
outside leg margin by 0.10 m: the unchanged 0.30 m footprint plus 0.13 m margin
now requires 0.43 m center clearance from mapped obstacles.

The follow-up table/chair-leg report lowered promotion from six observations
to three and added continuous validation of the next 0.80 m of an active path
against the fused planner map. A newly projected obstacle now stops the active
command and returns the controller to planning instead of letting it continue
on the older path snapshot. Commit `f13f296` passes all 214 host and
robot-native tests and is deployed. Exactly one drive-enabled mapper remains
`AUTO PAUSED` with command `stop`; Hector is the only SLAM stack. A paused
no-motion check reported 3 temporary and 28 persistent depth cells without
fusion/TF errors.

The repository now promotes a temporary depth obstacle to session-persistent
black occupancy after three settled depth observations paired with fresh LiDAR.
Four consecutive settled observations must contain positive free rays from
both sensors before a promoted cell or stale Hector-black cell becomes free in
the downstream planner/display map. Any current occupied endpoint from either
sensor wins immediately, and a one-cell guard protects against projection-grid
disagreement. Missing, stale, moving, occluded, invalid, or unsynchronized
data cannot promote or jointly clear cells.

Raw Hector `/map`, localization, scan integration, and update parameters are
unchanged. Hector-black clearing is maintained only as a fused-map suppression
and disappears on `CLEAR MAP` or mapper restart. Unconfirmed depth cells retain
their two-observation depth clear behavior and use a 5 s TTL. The web map
uses magenta for unconfirmed depth and black for persistent fused occupancy.

The current 214-test suite passes on both the host and robot.
Synthetic tests cover LiDAR occupied/free projection, promotion, single-sensor
rejection, occupied-neighbor veto, joint clearing, immediate reappearance,
reset, and rendering. Commit `878267e` was deployed with matching runtime
hashes. A drive-disabled paused run promoted 25 current-scene cells; the same
25 remained beyond the former 30 s TTL, and no Hector-black cell was
suppressed. Rates were 12.17 Hz scan, 12.33 Hz pose, 30.0 Hz RGB, 29.67 Hz
native depth, and 29.5 Hz registered depth; temperature was about 32 C. No
fusion/TF errors were found. One drive-enabled TensorRT mapper was restored
`AUTO PAUSED` with command `stop`, preserving Hector's map. No movement was
commanded. A measured moved-object promotion/clear/reappearance trial and all
physical-motion checks remain pending.

## Live checkpoint

The approved object-visibility revision is deployed. It preserves
the conservative 3-of-4/4-of-5 validation gates but shows pending candidates as
hollow amber map markers, reports the exact pending reason in the Objects tab,
and runs display-only latest-frame inference while paused and settled. Preview
frames cannot create, confirm, move, or remove tracks. Boxes clear immediately
on motion and expire after one second without a new result. Transient preview
sync retries preserve the last completed inference status. All 204 tests pass
on the Windows host and natively on PT; production modules compile.

After deployment, the paused TensorRT mapper reported `preview: 0 detections`
with 0.29-0.68 s inference. In six seconds, counters advanced from 20 captured /
19 processed to 34 captured / 33 processed, with no queue backlog; the
validation track list stayed empty, confirming preview results do not create
landmarks. The web page served the pending-marker explanation. Under continuous
preview load, measured rates were 12.06 Hz scan, 12.05 Hz pose, 29.63 Hz RGB,
30.10 Hz native depth, and 30.12 Hz registered depth. Available RAM was 862 MB;
CPU/GPU temperatures were 35/31 C. One drive-enabled TensorRT mapper remains
`AUTO PAUSED` with command `stop`, Hector is the only SLAM stack, and no motion
was commanded. Runtime hashes matched the workstation. Labeled-scene detector
quality remains untested.

The stronger autonomous landmark-validation revision is deployed. All 199
tests pass on the Windows host and production modules compile. Each
settled endpoint now acquires at most five distinct synchronized frames within
the existing one-second grace; inference remains asynchronous and the queue is
bounded. Expected classroom classes require 3-of-4 strong, geometrically
consistent observations. Other Objects365 classes remain eligible but require
4-of-5 with higher confidence and tighter geometry. Poor depth support cannot
create an observation, two consecutive same-view misses discard a pending
candidate, pending objects cannot be navigation targets, and confirmed objects
use only qualifying coordinates and remain fixed until the map/session is
reset. Initial live validation exposed worker contention that admitted only two
frames; inference is now deferred until the acquisition window closes. Final
robot-native and no-drive live validation passed all 199 tests and acquired,
processed, and drained all five distinct frames with no replacement. The live
scene produced four geometrically consistent `sneakers` observations at 0.596
maximum confidence; it correctly remained pending because an other-tier hit
must be at least 0.60. A 0.359 `bench` observation also remained pending. This
is strong pipeline/gate evidence, not a labeled-scene detector-quality result.

Exactly one drive-enabled mapper remains `AUTO PAUSED` with command `stop` on
TensorRT FP16, with Hector as the only SLAM stack. Rates measured from 20
message timestamps were 12.04 Hz scan, 12.05 Hz pose, 29.77 Hz RGB, 29.83 Hz
native depth, and 29.89 Hz registered depth. Available RAM was 892 MB; CPU/GPU
temperatures were 34.0/30.5 C. Runtime file hashes match the workstation. No
motion command or physical movement occurred during deployment validation.
The production launcher briefly false-failed because it sampled the initial
paused status before the first stop command; it cleaned up its process, and the
same production command was then started under Bash and explicitly verified.

At the operator's request on 2026-10-26, YOLO post-processing was vectorized so
the 8,400 Objects365 predictions no longer perform per-row class selection and
threshold filtering in Python. Box conversion, allowed-class filtering,
class-specific thresholds, class-aware NMS, and output ordering are unchanged.
All 186 tests pass on the host and natively on PT. With the full production
stack live and clocks already locked at their maximums, three controlled paused
TensorRT restarts measured 1.12, 0.50, and 0.47 s inference (0.50 s median),
versus 2.64-3.47 s immediately before the change. A later admitted endpoint
completed in 0.22 s. Post-change rates were LiDAR 12.08 Hz, Hector pose 12.11
Hz, RGB 29.83 Hz, native depth 30.03 Hz, and registered depth 29.80 Hz.
Temperatures remained low, with CPU around 33 C and GPU around 29 C. The robot
has exactly one drive-enabled mapper on `tensorrt_fp16`, remains `AUTO PAUSED`
with command `stop`, and no gait command was issued. The operator manually
repositioned the robot during later observation, explaining the changed Hector
pose and additional endpoint captures. This is paused pipeline evidence, not
labeled-scene detector-quality or commanded physical-motion validation.

Default landmark validation admits one bounded burst of up to five distinct
immutable synchronized snapshots per ready stationary survey endpoint. After
that burst, a paused and settled robot runs latest-frame preview inference at
the bounded worker cadence. Preview frames are display-only and cannot inflate
landmark confirmation. Another validation burst requires a new endpoint-ready
event; repeated sensor timestamps are rejected.

The optional TensorRT FP16 backend is implemented with CPU as the unchanged
default. PT built the checksum-bound engine under `models/gpu_trial/` in
639.453 s. With ROS perception stopped, three saved non-classroom images and
nine timed samples per backend measured 1.010 s CPU median (1.036 s worst) and
0.400 s TensorRT median (0.410 s worst), about 2.5x faster. Both produced the
same 11 labels/detections; minimum matched box IoU was 0.979 and maximum
confidence delta was 0.023. This is standalone model evidence, not labeled
classroom quality or full-stack sensor-rate evidence. GPU remains explicit
opt-in, and checksum/load/warm-up/runtime failure switches permanently to CPU
for that process. The original ONNX and YOLOv5n artifacts were not replaced.
All 186 repository tests for that earlier revision passed on the host and natively on PT. A drive-disabled,
map/depth-disabled mapper reported the preferred model on `tensorrt_fp16` and
shut down cleanly. Repeating startup through a path with no required engine
metadata automatically selected the same preferred ONNX on CPU. Both checks
remained `AUTO PAUSED` with command `stop`. Full-stack sensor-rate, memory, and
thermal validation has a short paused full-stack pass; longer soak and labeled
classroom validation remain pending.

Deployed 2026-10-26: the production launcher now detects ROS node liveness
from an actual XML-RPC reply rather than `rosnode ping`'s unreliable exit code.
An injected, pingable Hector instance without pose output was removed; the
launcher then started the production Hector stack, received `/slam_out_pose`
and `/map`, and started exactly one mapper `AUTO PAUSED` with command `stop`.
A second launcher invocation preserved that single stopped mapper. All 180
tests pass natively on PT and on the Windows host. No motion command or
physical movement occurred during this validation.

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
landmarks. All three priorities in `docs/RELIABILITY_FIX_PLAN.md` are implemented in
separate commits and are now deployed to PT:

1. `a201e37` adds conservative free-space evidence to the temporary registered-depth
   planner layer. Clear a magenta cell only after two consecutive valid depth
   observations disprove it. Preserve all cells within at least 0.60 m of the
   registered-depth camera because close geometry may be beneath its useful
   view, even while the robot faces the cell. Determine distance from the
   timestamped camera-to-map transform. Invalid, occluded, out-of-view, and
   behind-surface cells continue to rely on the 5 s TTL. Never clear Hector's
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

The workstation validation is now reproducible through `tools/host_test_runner.py`,
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
`tools/prepare_yolo_model.py`, then copied beside the fallback model.

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

### Superseded landmark-confirmation proposal

This proposal is retained as history. It was superseded on 2026-10-26 by the
more conservative two-tier 3-of-4/4-of-5 policy described in the live
checkpoint and newest decision entry. No class confirms from one detection.

## Jetson Nano GPU YOLO evaluation plan (implemented; full-stack gate pending)

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
cannot use it. The isolated TensorRT engine now builds and the standalone
same-frame trial is faster with acceptable FP16 output parity. CPU remains the
default and rollback target until the GPU backend passes a paused, drive-
disabled full-stack soak with healthy sensor rates and representative labeled
classroom images.

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
- Landmark validation uses the bounded, confidence/depth/geometry-gated burst
  described in the live checkpoint. Inference finishes asynchronously and does
  not enter blocking `object_focus` unless legacy mode is explicitly enabled.
- Object-map capture remains stationary; inference completion may occur during
  the next route because image-time TF and registered depth were snapshotted.
  Separate RGB-only moving previews provide boxes without map evidence.

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

- estimated full leg span approximately `0.50 m`; `body_radius = 0.25 m`
- planner outside margin `0.13 m`; total mapped clearance `0.38 m`
- `waypoint_tolerance = 0.12 m`; thinned waypoint spacing about `0.20 m`
- independent `lidar_stop = 0.45 m`
- `depth_stop = 0.60 m`
- close-depth exclusion is disabled (`0.00 m`)
- upper-band depth validity: at least 200 pixels and 2% of the band
- transient sparse-depth grace: recent result for at most 1.5 s
- depth planner layer: 0.15-2.0 m occupied range, 0.025-0.50 m height, 10 s
  minimum hold; unconfirmed cells age out after 60 s, while promoted cells
  require positive joint-clear evidence
- object confirmation: expected 3-of-4; other classes 4-of-5, with confidence,
  depth, box, and map-position consistency gates
- object focus: 15 s or 2 distinct misses
- weak turn tolerance: require fresh pose evidence and pause with the map
  retained only when 3 bursts total less than 3 degrees
- pose jump guard: 0.25 m / 20 degrees across 2 consistent samples

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
`--fresh-map` explicitly resets it. A pingable Hector without live pose output,
or an orphaned `hex_map.launch`, is stopped and restarted with a fresh map.
Logs are written outside the repository at
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
python3 -m unittest discover -s tests -t . -v
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
- Native lower-floor coverage between 2% and 10% is treated as too sparse for
  floor-bump/dropout inference when the upper depth band is healthy; upper
  depth and the independent LiDAR stop remain active. Complete lower-depth
  loss still triggers the conservative 0.30 m dropout stop. Registered-depth
  frames below 10% valid coverage do not add or clear planner obstacles.
- Rebooting or resetting Hector loses the in-memory map. Object landmarks are
  cleared when map trust is lost or the map is reset.
- Map taps capture the displayed image's exact center/scale on pointer-down and
  submit navigation on pointer-up. Map refreshes preload and swap atomically, so
  the 800 ms refresh cycle cannot temporarily remove or mismatch click metadata.
