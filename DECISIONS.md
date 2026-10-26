# Room Mapper Decision Log

This file is intentionally part of the project. Add an entry whenever the
architecture, safety behavior, interface, or robot assumptions change.

## Initial decisions — 2026-10-11

### Goal: autonomous exploration, with manual assistance available

Chosen: the robot autonomously explores until no reachable frontier remains;
the web GUI can pause it or send it to a clicked location.

Considered: manual teleoperation-only mapping. Rejected because the old
project already demonstrated reliable autonomous walking and frontier-style
search, and the requested behavior is to walk around the room.

### Reuse Hector SLAM and the old TF bridge

Chosen: keep `hex_map.launch`, Hector SLAM, `/map`, and `/slam_out_pose`.

Considered: write a new occupancy-grid mapper or use the robot's full
navigation launch. Rejected: the old project already fixed the robot's
prefixed/unprefixed LiDAR TF issue, and the robot navigation launches can
start duplicate controllers.

### Separate, smaller frontier planner

Chosen: `map_explore.py` converts Hector's 5 cm map into a 10 cm planning grid,
inflates obstacles for the 0.25 m leg radius, and uses BFS to reach the
nearest free frontier cell.

Considered: copy the old object's search state machine wholesale. Rejected:
target detection, approach behavior, and remembered-object goals are unrelated
to room coverage and would make the new controller harder to understand.

### Preserve the old web GUI interaction model

Chosen: camera and map JPEG endpoints, updating browser images, status text,
pause/resume/clear controls, and map clicks that create a temporary safe
corridor.

Considered: ROS web tools or a separate desktop UI. Rejected because the old
project's built-in Python server works offline on the robot and is already
familiar.

### Layered LiDAR and depth safety

Chosen: retain LiDAR as the primary scan/planning sensor and add the depth
camera as a conservative central-view obstacle check. The node learns a
per-image-row floor profile while stationary, then flags elevated depth returns
and floor bumps. Missing or stale depth data stops the robot while depth mode
is enabled.

Considered: use depth as a full replacement for LiDAR or fuse both sensors
into Hector SLAM. Rejected for now: the depth camera is useful for collision
checks but its registration/calibration and field of view are installation
specific; Hector continues to consume `/scan`.

### Dry-run remains the default

Chosen: movement is disabled unless `_drive:=true` is supplied.

Reason: this is inherited from the old project's safest demonstrated workflow
and lets sensor/map behavior be checked before servo movement.

## Implementation checkpoint — 2026-10-11

Created the independent project files and retained the old topic names,
walking API, map resolution, body radius, and TF bridge. Offline syntax and
import checks were attempted, but this Windows workspace has only inaccessible
Microsoft Store Python launchers and no available linter. Source-level review
completed; robot validation has not occurred yet.

### Robot connection — 2026-10-11

The target robot is `hiwonder@192.168.137.223`. Documentation and the live
view log message use this address. The SSH password is not stored in the
repository; enter it interactively.

Connection attempt from the development workspace failed before SSH
authentication: TCP port 22 and ICMP to `192.168.137.223` timed out. The
workspace is therefore not currently on the robot's network, so no robot
process was started from here.

### Robot validation — 2026-10-11

The robot is reachable at `192.168.137.223`, and the deployed directory is
`~/room_mapper` (lowercase). The first mapper attempt was made before the
LiDAR launch had finished, so `/scan` and `laser_link` did not yet exist. After
starting `roslaunch jethexa_peripherals lidar.launch` and waiting for the
YDLIDAR G4 to report healthy scanning, the chain resolved correctly:
`jethexa/base_link -> jethexa/body_link -> body_link -> laser_link`.
Hector then published `/map`, and the dry-run node reported approximately 2%
map coverage while planning. The live status endpoint returned successfully
from port 8000. The deployed script's live-view address was corrected to
`192.168.137.223`.

The first real-drive attempt was started from `~/room_mapper`. It reached a
frontier and increased coverage to about 3%, then repeatedly entered
`HOLD -> AVOID -> SEARCH` for LiDAR readings between 0.18 m and 0.23 m. It was
stopped with Ctrl+C after roughly one minute to avoid continued maneuvering in
place. This needs a physical-position check and LiDAR self/obstacle diagnosis
before another drive run; no new navigation behavior was enabled automatically.

### Movement diagnosis — 2026-10-11

The robot's login banner identifies the sensor as `YDLIDAR_G4`, but the new
controller defaulted to `RPLIDAR_A1` when `LIDAR_TYPE` was absent from the
environment. That selected a 180-degree offset even though this robot's raw
angle 0 is forward. The resulting sector inversion explains the repeated
close-obstacle avoidance and circular movement despite open floor. The default
was changed to YDLIDAR/0 degrees; the startup log now prints the active offset.

The subsequent stationary scan showed a persistent approximately 0.342 m
front return in open space while the rear was approximately 1.14 m. This is
consistent with the front leg/body. The default self-filter was raised from
0.15 m to 0.35 m, following the old project's documented self-leg workaround.
The real-drive test still saw 0.34-0.38 m returns, so the robot-specific
default is now 0.50 m. This is a deliberate hardware calibration and should be
re-measured if the LiDAR mount or leg geometry changes.

### Fresh map per run — 2026-10-11

The robot retained the previous Hector map between attempts, so later runs
could plan against geometry from an earlier physical position. The controller
now calls Hector's `/reset_map` (`std_srvs/Trigger`) at startup by default.
`_reset_map:=false` is available for intentional continuation runs.

### Depth implementation checkpoint — 2026-10-11

Depth support is implemented locally but has not yet been deployed and
validated on the robot. The default topic is
`/camera/depth_registered/image_raw`; verify it with `rostopic list` and
`rostopic echo -n1` before a real-drive test. First validate in dry-run, watch
the floor-learning message, confirm the GUI depth status, and only then test
with `_drive:=true` under direct supervision. Tune `_depth_stop`,
`_depth_creep`, and the depth band parameters only from observed data.

## Depth validation and controller corrections - 2026-10-11

Deployed depth integration and `test_room_mapper.py` to the robot. Verified
the DABAI registered stream as 640 x 480 `32FC1` in metres. All 14 tests
passed and the stationary dry-run learned 30 floor rows; GUI status reported
about 0.77 m depth and 0.88 m front LiDAR. These changes have not yet been
tested during real walking.

The initial timed dry-run exposed a late timer command after shutdown's stop.
Added a controller lock and shutdown flag, with a regression test. Camera and
map JPEG endpoints and status all returned HTTP 200 during the final dry-run.

Chosen: retain the full 0.60 m depth threshold, fail closed on unusable depth
or stale mapping data, retry incomplete floor calibration, and treat a large
increase in missing floor pixels as a possible close object. Ported the floor
dropout concept from the old project's depth detector. Alternatives considered:
trusting a clear LiDAR scan to reduce the depth threshold, or interpreting
invalid depth as free space. Rejected because the LiDAR self-filter excludes
close objects and invalid depth can indicate an object inside camera range.

Corrected depth decoding for padding/endian order, delayed stop on entering
HOLD, and command deduplication after failed publication. Paths are discarded
when obstructed. Mapping mode now waits for fresh SLAM pose/map instead of
wandering without localization. The former depth_creep parameter is removed.

Inspected the robot's Hector reset callback: it calls slamProcessor->reset()
but never writes the Trigger response, explaining the false/empty reply.
Retained the working service and clarified logs rather than modifying vendor
SLAM code. Startup now aborts if the reset service cannot be called; silently
continuing with a previous map would violate the requested fresh-run behavior.

## Open questions

### Rotated map overlay diagnosis and local fix — 2026-10-11

Observed behavior: the map is initially correct, then Hector loses heading and
draws rotated copies of the room over the existing occupancy grid. The robot
was disconnected during this work, so no remote deployment or hardware test
was attempted.

The concrete configuration mismatch is that room_mapper ignored the robot's
verified 0.34–0.40 m body/leg returns with a 0.50 m self-filter, while Hector
accepted returns from 0.20 m. Moving legs could therefore become SLAM
landmarks. Hector also received scans continuously during gait and turning,
when a 2D scan is distorted by translation and body rocking. With no external
odometry and no loop closure, a wrong scan match permanently folds the map.

Chosen local changes: Hector minimum range 0.50 m, three multiresolution
levels, scan queue one, and `/scan_stable` input. The controller publishes
stable scans only while stopped after 0.65 seconds settling, and commands
motion in 0.45-second translation or 0.35-second turn bursts. It rejects pose
jumps over 0.50 m or 35 degrees and stops in FAULT. All values are parameters.

Considered: tuning Hector alone, automatically resetting after a jump, or
immediately using `/odom/filtered`/IMU as a pose prior. Tuning alone does not
remove motion-distorted scans. Automatic reset discards the map without
solving the cause. The robot's odometry topic and TF frame semantics were not
captured before disconnection, so enabling it blindly could create conflicting
TF publishers. When reconnected, inspect `/odom/filtered` headers and
`odom -> jethexa/base_link` TF; a valid independent yaw/odometry prior is the
next improvement if stationary scan matching still jumps.

Lower-band depth floor/dropout alarms are ignored only during commanded
zero-stride turns because body tilt was observed to cause these alarms and the
turn does not advance toward them. Upper-band depth and LiDAR remain active.

Added `OBJECT_MAPPING_PLAN.md`. YOLO object placement is deferred until room
mapping is repeatably stable. The planned method combines YOLO boxes,
registered depth, camera intrinsics, and timestamped camera-to-map TF, then
tracks repeated observations separately from the occupancy grid.

### Post-reboot deployment and dry-run — 2026-10-11

Reconnected after the user restarted the robot and deployed the stable-scan
changes. The first dry-run exposed a deterministic TF startup failure:
`lidar.launch`'s `body_link_to_laser_link` node collided by name after reboot,
leaving no `laser_link`. Chosen fix: start the peripheral launch with
`publish_tf:=false` and make `hex_map.launch` publish the verified 0.031 m
`body_link -> laser_link` mount under the unique name
`room_mapper_laser_mount`. This restored the full transform chain.

The current camera view had only one lower-band row with more than 30% valid
registered depth, so floor calibration could never complete. Chosen: after
three failed calibration windows, continue in explicitly reported upper-only
mode. Elevated-object depth protection remains active; floor-bump protection
is unavailable. Considered lowering the validity requirement, but sparse rows
may describe furniture rather than floor and would create an unsafe baseline.

All 18 tests passed on the robot. Final dry-run evidence: `/scan_stable` about
12 Hz, map known 2%, camera and map HTTP endpoints 200, no Hector TF timeout,
and stationary Hector drift over 10 seconds of 0.008 m / 0.23 degrees. A real
object in the upper depth band at about 0.56 m triggered dry-run HOLD/AVOID
commands. No walking was enabled. The dry-run was left running.

### First depth-enabled walking test - 2026-10-11

At the user's request, started the deployed mapper with `_drive:=true` and
default map reset. Sensor preflight passed and floor learning found 30 rows.
Walking produced repeated short depth-dropout/floor-bump HOLD/SEARCH cycles;
map-known fraction rose from 2% to 7%. Paused walking through `/pause`, leaving
the GUI/process running. Final status confirmed PAUSED, depth 0.45 m and
LiDAR 0.48 m. Do not classify all detections as false without inspecting the
scene. Chosen: pause for diagnosis rather than weaken sensor thresholds during
the run. This supersedes the earlier 'real walking untested' checkpoint.

- Should autonomous exploration stop based on map coverage, frontier absence,
  or an operator command/time limit?
- Validate depth thresholds while walking and physically isolate the suspected
  LiDAR body return before replacing the broad 0.50 m filter with a narrower
  geometric mask.

## Integrated navigation, UI, object, and depth update - 2026-10-12

This entry supersedes earlier operational defaults while retaining them above
as history.

### Routing and recovery

Chosen: model the full leg envelope as a 0.30 m radius circle and retain a
0.03 m planner margin. Paths now use approximately 0.20 m waypoint spacing and
0.12 m waypoint tolerance to reduce corner cutting. Blocked autonomous mapping
and selected routes stop first, rotate when the swept footprint is clear, or
reverse briefly when rotation is constrained and rear LiDAR clearance exists.
If neither is safe, the robot holds. Selected routes can be cancelled from the
web UI.

One weak 0.5-second turn burst is no longer enough to invalidate SLAM. The
controller retries and faults after three consecutive bursts with less than
3 degrees heading progress. The 0.25 m / 20 degree pose-jump guard remains.

### Web interface

Chosen: keep world geometry stable but render a moving viewport centered on
the current robot pose. Added wheel/button zoom, drag-to-pan, and a Center
button that restores 100% robot-following view. Click navigation includes the
displayed scale and offset, so zoomed/panned clicks retain correct world
coordinates. Added manual W/A/S/D and touch controls usable while autonomy is
paused, an Objects tab, object actions, and route cancellation.

### Object mapping and focus

YOLO/depth/TF object placement is deployed. Two distinct observations confirm
a landmark. During autonomous mapping, an unconfirmed detection enters
`object_focus`, stops motion, and gathers observations until confirmation, two
distinct misses, or a 15-second timeout. It then replans. Automatic exclusion
discards rather than suppresses the candidate; explicit user removal suppresses
nearby re-detection.

### Camera and fault recovery

The UVC RGB publisher twice remained registered without emitting frames. A
full robot reboot restored RGB and depth. After reboot, the LiDAR driver node
could remain present without scans; `/start_scan` restored `/scan`, after which
Hector and the mapper were restarted. A corrupted/drifting Hector frame was
reset rather than reused.

### Depth policy

Observed native upper-band validity varied around 4-9% in some scenes, making
the former 10% requirement oscillate into FAULT. Chosen: require at least 200
pixels and 2% coverage. Per explicit operator request, depth results at or
below 0.50 m are ignored, including elevated returns, floor bumps, and the
synthetic dropout/too-close result. Values above 0.50 m retain the 0.60 m stop.
LiDAR remains an independent 0.45 m emergency stop. This intentionally leaves
a close elevated-object blind region and is documented as a safety tradeoff.

Post-motion frames still occasionally fell below 2%. Chosen: keep the most
recent valid depth decision for a maximum 1.5-second grace period while LiDAR
remains active. Sustained sparsity, or sparsity before any valid result, still
fails closed. This was preferred to treating arbitrary missing depth as clear.

### Validation and live checkpoint

The deployed suite passed 117 tests. A no-drive live controller exercised
autonomous planning for six samples without recurrence of the sparse-depth
fault. The real controller was then restored with `_drive:=true` and
`_start_paused:=true`; final status was AUTO PAUSED, depth ready in upper-only
mode, LiDAR live, and one mapper process. Physical routing with the new temporal
grace still requires supervised observation after Resume.

## Battery telemetry and pending alarm configuration - 2026-10-12

The robot lost power during post-reboot service restoration and is currently
offline. No motion command was issued during that restoration attempt.

Chosen: add a prominent battery badge to the web header. The controller uses
type-erased subscriptions to `/battery`, `/battery_state`, and `/voltage`, then
decodes standard `BatteryState`, floating-point voltage, or integer millivolt
messages. A native `BatteryState.percentage` takes precedence; otherwise the
percentage is a clamped estimate between the vendor alarm's verified 9.8 V
cutoff and a configurable 12.6 V full default. The endpoint reports stale
readings unavailable after 10 s.
The UI visually distinguishes low, medium, and charging states and exposes
voltage/source in its tooltip.

The operator requested that the audible low-voltage alarm be disabled on the
next reconnect. The hardware/software source is not yet known, so no guessed
board command was added while offline. Reconnect work must identify the actual
battery topic, pack limits, and alarm source, then persistently disable only
the sound while retaining voltage telemetry and the visual low-battery warning.

Two battery unit tests bring the pending robot suite to 119 tests. All nine web
tests and source syntax checks pass locally; robot-side full-suite validation
and deployment remain pending.

### Deployment result

On reconnect, `/voltage` was confirmed as `std_msgs/Float32`. Its first samples
were 10.524 V and later 10.328 V. Both controller launch variants were backed
up and their `battery_low_alarm.launch` includes commented out; the running
alarm node was then stopped through ROS so its shutdown callback switched the
buzzer off. Voltage publishing continued and the alarm did not respawn.

All 119 tests passed on the robot. Hector and one mapper were restored with the
mapper `AUTO PAUSED`. The live battery endpoint reported 10.328 V / 18.9%, and
RGB, depth, LiDAR, SLAM pose, map, and web services were operational.

## Adjustable movement speed - 2026-10-12

The requested fixed speed increase was cancelled before implementation.
Chosen instead: independent web sliders for translation and rotation, each
bounded from 50% to 150% in 5% steps and reset to 100% at mapper startup.
Translation scales the forward/reverse stride; rotation scales the zero-stride
turn rate. Manual and autonomous commands share these settings. Collision
thresholds, the 0.30 m footprint, settling, burst timing, and browser dead-man
remain unchanged. Telemetry displays both active percentages.

The HTTP layer validates both bounds, and changing a slider invalidates the
cached gait command so a held manual direction picks up the new setting. All
122 tests passed on the robot. Live endpoint checks successfully changed and
read back both settings; an active browser then exercised the controls while
autonomy remained paused.

## Visual/depth navigation correction - 2026-10-19

The robot lost power during diagnosis, so every change in this entry is local
and pending robot-side validation.

Four direct causes were identified. First, the configured 0.50 m depth
exclusion discarded exactly the close elevated returns, floor bumps, and
dropout/too-close alarms needed for small-obstacle protection. Second, depth
was collapsed to one scalar, so it could stop translation but could not tell
recovery which side was clear. Third, recovery always selected the wider LiDAR
side and ignored the next route waypoint, explaining a left turn when a
selected door route required right. Fourth, map clicks were converted using
the newest robot pose even though the browser image could have been rendered
against an older robot-centered viewport.

Chosen controller changes: default close-depth exclusion is zero; retain
left/center/right depth clearance; react immediately to new close depth while
using history only to delay clearing; prefer the next route turn when that side
has safe combined clearance; and fall back to combined directional depth and
LiDAR. Chosen web change: stamp each map JPEG with its exact world center and
scale, have the browser return that snapshot with the click, and remove the
navigation confirmation dialog.

Per operator request, manual held commands now bypass map trust, sensor
freshness, collision, burst, settling, and turn-progress gates. Key release,
window blur, and the 0.55 s server dead-man remain because they terminate a
lost command rather than autonomously restricting an actively held command.
The translation and rotation slider maxima are raised to 200%.

Chosen visual SLAM path: add `visual_slam.launch` as an alternative to Hector,
not a simultaneous sidecar. RTAB-Map receives registered RGB-D and LiDAR,
uses visual odometry/loop closure with scan refinement, and publishes a 2-D
grid projected from depth so low obstacles can enter the planner map. The
controller now accepts `PoseWithCovarianceStamped`, configurable map frames,
and Trigger/Empty reset services. The candidate wiring is `/visual_map`,
`/visual_pose`, and `/visual_slam/reset`.

Higher depth emphasis should improve the reported low-obstacle failure because
it restores close live stops and makes low geometry available to the global
planner. It cannot compensate for bad camera extrinsics, missing/reflective
depth, incorrect ground segmentation, motion blur, or visual-odometry loss.
Those are explicit dry-run and supervised hardware validation items.

Validation: 128 tests pass locally with ROS message modules stubbed; Python
syntax and launch XML parse checks pass. RTAB-Map topic names, TF ownership,
camera calibration, resource load, map quality, and physical motion remain
unverified until the robot returns.

### Robot deployment and no-drive validation

PT returned on 2026-10-19. Native RGB and depth each ran near 30 Hz, LiDAR ran
near 12 Hz after `/start_scan`, and visual odometry ran near 3.1 Hz. The full
robot suite passes all 129 tests. No test or validation step commanded motion.

Two integration faults were found live. RTAB-Map initially inherited a stale
private `odom_frame_id` parameter, so both RTAB nodes now use
`clear_params="true"`. Directly synchronizing RGB, depth, odometry, and scan
also starved the graph callback. RTAB-Map now consumes rgbd_odometry's bundled
`odom_rgbd_image`; graph updates then ran near 0.9 Hz and the grid-map service
returned an 80 by 86 map at 0.05 m resolution containing free and occupied
cells. The installed reset service is `/visual_slam/reset`.

RTAB-Map does not publish `localization_pose` while it is in mapping mode. The
controller therefore gained `_pose_type:=tf`, reading the authoritative
`map -> visual_odom -> jethexa/base_link` chain. Stationary RGB/depth skew of
roughly 0.02-0.06 seconds and intermittent scan-time TF extrapolation warnings
remain live-test observations. Measured obstacle placement, map alignment,
five-minute resource observation, click-route behavior, and physical walking
are still required before calling the visual stack motion-validated.

### Initial stationary resource validation

The no-drive stack subsequently ran for more than seven minutes. Visual
odometry consumed about one CPU core/189 MB RSS, RTAB-Map about 16%/221 MB,
and the dry-run mapper about 63%/193 MB. During the mapper's three-minute
stationary sample, TF pose changed about 0.02 m and 0.1 degrees. `/visual_map`
contained 67 free and 79 occupied cells, the map endpoint returned HTTP 200,
and the speed endpoint exposed the requested 50-200% limits. The mapper stayed
AUTO PAUSED and no motion was commanded. Voltage fell to 10.016 V, so all
visual processes were stopped and charging became a prerequisite for motion.

### Stationary resource validation after recharge

A reboot-and-recharge retry completed more than five stationary minutes with
RGB, visual odometry, graph updates, TF, depth, and LiDAR live. RTAB-Map's grid
service contained 71 free and 76 occupied cells. The mapper received the map
topic and served a 9.6 KB rendered JPEG; its displayed coverage rounded below
one percent because the planner reports coverage over its larger fixed grid.
Visual odometry settled near 2.4 Hz and graph updates near 0.65 Hz under the
full mapper load. Approximate process CPU was 94% for visual odometry, 17% for
RTAB-Map, and 66% for the mapper. No controller fault or motion command occurred.

### Measured tall-box alignment - 2026-10-19

Operator reported a box 14.5 inches wide (0.3683 m) and 22 inches tall
(0.5588 m), with distances measured from body center to the nearest face.
All measurements used a paused mapper with drive disabled.

- At the reported 1.00 m placement, a front-face depth patch had median
  0.984 m and nearly 100% valid pixels. The configured camera transform adds
  0.065 m forward offset, giving approximately 1.049 m from body center.
  Occupied map cells corresponding to the face were near 1.041 m ahead.
- At the reported 0.75 m placement, three distinct depth frames each gave
  median 0.692 m (10th/90th percentiles 0.682/0.707 m), approximately 0.757 m
  from body center with the configured offset. Occupied cells transformed
  into the current base frame placed the new face around 0.79-0.80 m ahead.
- The map retained occupied cells around 1.04 m from the previous placement.
  Clearing moved obstacles remains unvalidated. These measurements do not
  validate the configured extrinsics independently or establish low-obstacle
  detection: this box is 0.559 m tall. A shorter object is still required.

Latest voltage sampled during the second placement was 10.069 V. One mapper
remained drive-disabled and AUTO PAUSED, with RTAB-Map running on the separate
`/tmp/room_mapper_box_validation.db` database. No walking was commanded.

### Continuation reachability check - 2026-10-19

Later read-only HTTP requests to the mapper's `/status` and `/battery.json`
endpoints could not connect from the development workstation. No command was
sent to the robot. The tall-box checkpoint remains the last-known state, not a
claim that those processes are still live; process, sensor, and battery state
must be checked again before further validation.

### Dry-run stack restored after SSH recovery - 2026-10-19

SSH connectivity was restored. No mapper or SLAM process was initially
running. RGB and depth were each near 30 Hz; `/scan` had a publisher but no
messages until the documented `/start_scan` service call restored LiDAR near
12 Hz. Battery telemetry was 12.411 V before the visual stack was started.

The deployed controller, planner, web, object-mapping, and visual launch files
matched the repository after normalizing line endings. RTAB-Map was started on
the fresh `/tmp/room_mapper_continue.db` database, followed by exactly one
mapper with `_drive:=false _start_paused:=true _reset_map:=false`. Hector was
not running. Visual odometry, a 0.05 m occupancy grid, map-to-base TF, and RTAB
graph updates near 1 Hz were live. The mapper reported AUTO PAUSED, localized,
depth ready in upper-only mode, and 0.97 m LiDAR clearance. The external web UI
and battery endpoint were reachable; the final battery sample was 12.307 V
(about 89.5%). No movement command or physical motion occurred.

### RTAB-Map stationary-grid liveness correction - 2026-10-19

The restored mapper later entered `FAULT: waiting for fresh SLAM pose and map`.
The occupancy grid was not empty: `/visual_map` contained a valid 39 by 45 grid
with 63 free and 35 occupied cells. During a 16-second sample, the grid emitted
no new messages because the stationary map had not changed, while
`/visual_slam/info` remained live near 0.84 Hz and visual odometry near 4 Hz.
The controller's fixed five-second grid-publication timeout therefore produced
the reported `map inactive` behavior even though RTAB-Map was healthy.

The mapper now defaults to `/visual_slam/info` as the mapping-process heartbeat
when `_map_topic:=/visual_map`. It still requires an occupancy grid to have been
received, a fresh pose, a trusted map, and a heartbeat no more than five seconds
old. If the heartbeat stops, it fails closed. Hector does not configure a
heartbeat and retains the existing `/map` publication-freshness requirement.
Object mapping uses the same liveness decision.

Two regression tests cover a live heartbeat with an unchanged grid and a stale
heartbeat with a recent grid. All 131 tests pass natively on the robot. In the
live dry run, the mapper remained localized and AUTO PAUSED after waiting more
than twice the former timeout, with no new grid publication required. It remains
drive-disabled; no motion command or physical walking occurred.

## Restore Hector pose/map with a temporary depth obstacle layer - 2026-10-19

The first supervised RTAB-Map motion pulse moved the robot forward correctly,
but the operator reported that mapping no longer behaved like the original
LiDAR version: the robot position and room-map creation were not being retained
as expected. The requested architecture is the proven LiDAR mapper plus depth
coverage for small objects close to the ground.

Chosen: restore `hex_map.launch` and Hector as the only authority for pose and
the persistent occupancy map. Registered depth is back-projected with its
camera calibration and timestamped `map -> camera_depth_optical_frame` TF.
Clustered points 0.025-0.50 m above the map floor and 0.15-2.0 m from the camera
become temporary occupied cells in the planner's copy of the Hector grid. They
expire after 2.5 s. The overlay never feeds Hector, so depth cannot move the
robot marker or deform the persistent LiDAR map. RTAB-Map remains available
only as a drive-disabled research experiment.

The implementation adds `depth_mapping.py`, preserves the LiDAR state
separately from the overlay in `map_explore.py`, and reports the live temporary
cell count in `/status`. Five new tests cover projection, height filtering,
expiry, and preservation across Hector updates. All 136 tests pass natively on
the robot.

Live evidence: Hector published `/slam_out_pose` near 12 Hz and `/map` was a
512 by 512 grid at 0.05 m. With one drive-disabled mapper, the status was
localized, showed about 4% known map coverage, and reported live temporary
depth cells. A subsequent directly supervised 50% speed, 0.55 s forward pulse
dead-man stopped normally. Hector reported a change from approximately
`(0.000, 0.006)` to `(0.027, 0.004)` m and about 0.9 degrees. This is SLAM
evidence only until the operator separately confirms what was physically seen.

The native depth floor-profile detector still reports upper-only mode on this
scene. The new registered-depth overlay is independent of that calibration,
but a measured short-object placement and removal/TTL check remain required.
Reflective, transparent, invalid, or out-of-view geometry can still be missed.

The supervising operator then confirmed that the Hector-based pulse moved the
robot correctly and that the map looked substantially better. At the operator's
request, the mapper was restarted with `_drive:=true _start_paused:=true` while
preserving the current Hector frame. Final live checks showed one mapper, one
Hector process, no RTAB process, localized stationary survey, approximately 4%
known map coverage, an updating depth layer, 1.11 m front LiDAR clearance, and
12.265 V (about 88%) battery telemetry. No resume or movement command was sent.

## In-place RGB/UVC recovery validation - 2026-10-19

The registered `/camera/uvc_color` node again stopped emitting RGB frames while
depth continued near 30 Hz. With Hector and exactly one drive-enabled mapper
running and `AUTO PAUSED`, `recover_rgb_camera.sh` stopped only the stale RGB
node, re-authorized the Orbbec `2bc5:0559` USB function, and launched a fresh
RGB node. Kernel logs briefly reported XUSB ring-expansion and URB allocation
errors, but RGB resumed at about 29-31 Hz. Depth, Hector, the mapper, the map,
and the temporary depth layer remained live. Battery voltage after recovery was
12.121 V. No movement command or physical motion occurred.

The recovery itself succeeded, but the script printed a false failure. Its
`rostopic hz | grep -q` validation ran under `set -o pipefail`; after `grep -q`
found the first rate line and closed the pipe, the producer's SIGPIPE status
could make the whole pipeline fail. The replacement check waits up to 30
seconds for the relaunched node, then holds one `rostopic echo` subscription
open for up to 90 seconds for an actual RGB message. This avoids both the
registration race and repeated subscriber churn while allowing for the delayed
USB recovery observed live. The script also sources the JetHexa workspace setup
itself; without that, a non-login SSH invocation could re-authorize USB but fail
to run `rosrun` or the validation commands. The final check directly verifies
the condition needed by downstream consumers. A full reboot remains the
fallback if the in-place recovery produces no frame. The separate vendor-driver
subscription patch remains undeployed and unvalidated.

During correction testing, short validation deadlines twice expired while the
XUSB driver was still reporting ring-expansion and URB allocation errors. RGB
subsequently recovered without a reboot and stabilized around 29-30 Hz; depth
remained around 29-34 Hz, Hector and exactly one drive-enabled mapper stayed
live, the mapper remained `AUTO PAUSED`, and battery voltage was 12.198 V. No
movement command or physical motion occurred. The longer single-subscription
check reflects that measured delayed-recovery behavior.

## Retain and distinguish low depth obstacles - 2026-10-19

The operator reported that the registered-depth obstacle layer works, but its
2.5-second cells disappeared after low objects entered the camera's close-range
blind area. Autonomous planning could then treat the cleared cell as traversable
and approach the object. The default `_depth_map_ttl` is increased to 30
seconds. Repeated observations refresh each cell independently. This retains a
low obstacle across many movement, settling, and replanning cycles while still
allowing moved objects to clear without modifying Hector's persistent map.

The planner now exposes a separate live depth-cell mask to the web renderer.
LiDAR occupancy remains black, depth-only occupied cells render magenta, and a
cell occupied by both sources remains black. This is a display distinction
only: both sources use the same occupied state, footprint inflation, and route
blocking behavior. The page includes the color key. Automated coverage checks
the 30-second expiry boundary, depth-mask expiry, magenta rendering, and LiDAR
color precedence. Physical low-object approach and removal/expiry validation
remain supervised follow-up work.

Deployment validation passed all 138 native tests without publishing movement.
The mapper was restarted with `_drive:=true _start_paused:=true` while retaining
the current Hector frame. Exactly one mapper and one Hector process were live,
with no RTAB-Map process. Live status reported 34 temporary obstacle cells and
the configured 30-second hold. The served map JPEG contained 768 magenta-like
pixels from the live depth layer. Depth remained ready in upper-only mode, the
mapper stayed `AUTO PAUSED`, and battery telemetry was 12.220 V. No movement
command or physical motion occurred. The longer expiry behavior with a placed
low object still requires direct supervised timing and approach validation.

RGB stalled after this deployment. The corrected `recover_rgb_camera.sh`
completed successfully and verified an actual frame before returning. RGB then
held about 28.7-30.4 Hz while depth remained about 26.3-31.6 Hz. Hector and the
single drive-enabled mapper stayed live, the mapper remained `AUTO PAUSED`, the
30-second depth layer continued updating, and battery voltage was 12.214 V. No
movement command or physical motion occurred during recovery or verification.

At the final post-push check, the mapper no longer reported `AUTO PAUSED`; it
was in `object_focus` with a stop command after recording six turn scans and a
SLAM heading near -21.4 degrees. This agent did not issue a resume command. A
`/pause` request was sent immediately.
The controller returned to `SEARCH/survey  AUTO PAUSED` with command `stop` and
exactly one mapper process. By then SLAM had recorded eleven turn scans and a
heading near -8.9 degrees. These are SLAM-reported changes only; no person
reported observing physical motion. The operator later confirmed this was the
first of two times they pressed `RESUME AUTO`. Treat the map frame as requiring
operator inspection before the next autonomous run.

A later check found the mapper outside the paused state again, now in
`SEARCH/go` with command `forward`. SLAM reported pose near `(0.60, -0.01)` m;
an immediate pause returned command `stop`, by which time SLAM reported about
`(0.74, 0.01)` m and -41.6 degrees with 28 turn scans. No physical observer was
present, so these remain SLAM-reported changes rather than confirmed physical
motion. The operator later confirmed pressing `RESUME AUTO` before both
observed resumptions. The mapper was therefore restarted with `_drive:=true`,
`_start_paused:=true`, and `_reset_map:=false`. Final checks showed exactly one
drive-enabled mapper, Hector as the sole SLAM process, `AUTO PAUSED`, and
command `stop`.

The web interface now provides a `RECOVER CAMERA` control for the recurring
registered RGB publisher-without-frames condition. The fixed `/recover-camera`
endpoint requires the mapper to be `AUTO PAUSED`, permits only one recovery at
a time, and runs the repository's `recover_rgb_camera.sh` with a 135-second
limit. The browser disables the button while the request is active and reports
the script's verified success or failure in the telemetry panel. The endpoint
does not accept a command or path from the browser.

The new endpoint was deployed and used to recover the current RGB failure. It
re-authorized USB function `1-2.1.3`, started `/camera/uvc_color`, and verified
a frame. Subsequent measurements reported RGB near 29.7 Hz, native depth near
29.7 Hz, and registered depth near 29.6 Hz. The temporary planner layer resumed
with 55 cells and the 30-second hold. The ROS-sourced native suite passes all
139 tests. Exactly one drive-enabled mapper remains `AUTO PAUSED` with command
`stop`; no movement command was issued during this recovery or validation.

## Empty-scene YOLO no longer delays exploration (2026-10-19)

Autonomous `send()` previously waited for every in-flight CPU YOLO inference
before starting the next bounded gait burst. Since inference takes multiple
seconds on PT, this created a long pause between exploration movements even
when the view contained no object. Exploration now proceeds through an
in-flight inference while the last YOLO result has no detections. If an
inference finds an object, its visible detection causes the next settled
observation to hold motion until depth/TF mapping completes; tentative objects
then retain the existing two-observation, two-miss, and 15-second focus rules.
The 0.65-second post-motion scan-settle interval is unchanged.

Robot-native regression coverage now verifies both branches: an empty-scene
inference permits the next burst, while an in-flight inference with a visible
object still holds motion. All 140 tests pass. Deployment was performed with
autonomy paused; no physical movement was requested for this change.

## Persistent corner blockage escalates to reverse (2026-10-19)

The existing recovery selected reverse immediately when the circular rotation
footprint was occupied, but selected turning without a time limit whenever
rotation was geometrically possible. This could leave the robot repeatedly
turning in a corner while its forward path remained blocked. A spin recovery
that remains front-blocked for 2.5 seconds now escalates to the existing
one-second reverse escape. The reverse still requires both rear and near-rear
LiDAR clearance of at least 0.65 m. It remains disabled when depth reports a
floor bump or dropout/too-close condition because the rear has no depth view.
Regression coverage verifies both escalation with clear rear space and refusal
to reverse when rear clearance is insufficient. All 142 native tests pass.

## Translation joins bounded bursts for smoother gait (2026-10-19)

Autonomous translation previously stopped after every one-second gait command
and waited 0.65 seconds before restarting, producing a pronounced stop/start
cadence. When the planner continues requesting the same forward or reverse
command, the controller now renews the translation window without issuing a
gait stop, up to a three-second continuous limit. It then performs the existing
0.65-second stationary interval for clean SLAM scans and YOLO/depth/TF
observations. Rotation remains limited to 0.50-second bursts so measured turn
progress is still checked after each burst. Sensor, pose, map, obstacle, web
pause, and route logic continue running at the 10 Hz controller rate and can
still issue an immediate stop.

Regression coverage verifies seamless renewal without another gait command and
the mandatory stop at the three-second limit. All 143 native tests pass. The
change was deployed with autonomy paused; smoothness still requires supervised
physical observation.

## Continuous path following (2026-10-19)

Active frontier and clicked routes now send continuous gait commands without
the three-second translation stop or half-second turn stop. Direction changes
are applied on the next controller tick. Turn progress is sampled every
half-second, with the existing three-failure SLAM guard. Obstacle and freshness
checks still precede path commands. Recovery remains bounded. YOLO detections
do not start object focus while a route is active; stationary endpoint surveys
remain available for mapping landmarks. This reduces opportunities for object
observations during travel. All 145 native tests pass; physical fluidity has
not been validated by an agent-commanded motion trial.

## Conservative omnidirectional path following (2026-10-20)

Vendor documentation and the official JetHexa source confirm that the six-leg
gait controller accepts a body-relative translation direction in radians: 0
is forward, pi/2 is left strafe, pi is reverse, and 3*pi/2 (equivalent to
-pi/2) is right strafe. The previous mapper exposed only forward, reverse, and
in-place rotation, so diagonal planner segments became turn-then-forward
sequences despite the hardware's holonomic gait capability.

References: the vendor's [standard motion control lesson](https://docs.hiwonder.com/projects/JetHexa/en/latest/docs/7_Standard_Motion_Control_Lesson.html),
[left/right example](https://github.com/Hiwonder/JetHexa/blob/jetson_nano/jethexa_tutorial/scripts/simple_control_05_move_left_right.py),
and [diagonal example](https://github.com/Hiwonder/JetHexa/blob/jetson_nano/jethexa_tutorial/scripts/simple_control_06_diagonally.py).

Chosen: make arbitrary-direction translation a controller command. Active
routes quantize heading error in 5 degree increments and crab-steer without a
stop up to 45 degrees left or right. Larger errors retain the calibrated
zero-stride turn and its half-second heading-progress sampling. Each crab
command also checks the LiDAR swept-body clearance in its actual travel
direction. The limit is intentionally conservative because registered/native
depth face forward and cannot guarantee blind-side low-obstacle detection.

Manual control exposes full lateral movement: A/D and on-screen buttons strafe
left/right; Q/E and the left/right arrow keys rotate. Manual motion retains the
existing explicit warning, browser release/blur handling, and 0.55-second
server dead-man, but still bypasses autonomous obstacle gates and requires
direct supervision.

Five regression cases cover diagonal command selection, the 45-degree turn
boundary, directional LiDAR clearance, vendor direction values, and manual
strafe dispatch. The suite now contains 150 tests. This Windows host lacks ROS,
and noninteractive SSH authentication to PT was unavailable, so robot-native
execution, deployment, and physical motion validation are still pending.

## Longer frontier routes and nonblocking surveys - 2026-10-24

The operator clarified that autonomous modes walk for only a few seconds, then
can remain stopped for roughly 20 seconds. Manual control is acceptable and is
not changed. The controller already had continuous path commands, but frontier
planning selected the first reachable frontier only 0.5 m away, the endpoint
survey required six seconds/five scans, and a tentative object could enter a
15-second blocking `object_focus`. Together these produced the reported cadence
even though individual path segments no longer used gait bursts.

Chosen: search the connected known-free region once and select a high-information
frontier normally 1.5-3.0 m away, retaining closer/farther fallback choices when
the known map cannot supply that band. Endpoint surveys now require two seconds
and three settled scans. YOLO still admits captures only while stationary and
settled, but synchronized RGB/depth, intrinsics, timestamped TF, and the motion
generation are snapshotted before inference. In default nonblocking mode, that
immutable observation may commit after a new route begins. Blocking focus is
disabled by default and remains reversible with `_object_focus_enabled:=true`.

LiDAR/depth fusion, obstacle distances, route footprint, recovery, turn-progress
checks, manual control, and the rule that object tracks never alter occupancy
geometry are unchanged. Robot deployment and physical smoothness validation
remain pending.

## Classroom-oriented YOLO profile with automatic rollback - 2026-10-24

COCO provides useful chair, book, backpack, laptop, keyboard, and dining-table
classes, but has no desk, podium, projector, printer, or blackboard/whiteboard
class. The official Objects365 vocabulary covers desk, chair, stool,
blackboard/whiteboard, projector, printer, school supplies, and many additional
indoor objects. Ultralytics publishes a 2.5-million-parameter YOLO26n Objects365
checkpoint (`yolo26n-objv1-150.pt`), making it a better ready-made classroom
candidate than simply increasing the size of the existing COCO model.
Objects365 still has no podium/lectern class. The external vocabulary and custom
checkpoint support deliberately leave that final class available for a later
room-specific fine-tune; podium and lectern are already classroom-priority
names when such a model is supplied.

Chosen: `prepare_yolo_model.py` exports that checkpoint to a static 640-pixel,
opset-12 ONNX file on a workstation and writes an ordered `.names` sidecar.
The robot continues to use its small OpenCV-DNN runner and does not install
PyTorch/Ultralytics. Startup performs a warm-up inference; an absent or
incompatible preferred model, label mismatch, or requested CUDA failure falls
back to CPU and then to the retained `yolov5n.onnx`. Artifacts are gitignored,
and the exporter refuses to overwrite them unless `--force` is explicit.

Classroom classes use a 0.30 admission threshold while other classes retain
0.45. Class-aware NMS fixes the previous behavior where an overlapping class
could suppress a desk/chair detection. Three distinct settled captures rather
than two confirm a landmark, and pending tracks live for five minutes to permit
confirmation at later route endpoints. All classes remain enabled unless the
operator sets `_object_classes`, so the broader model capability is preserved.

References: [Ultralytics Objects365 models and vocabulary](https://docs.ultralytics.com/datasets/detect/objects365/),
[Ultralytics ONNX export options](https://docs.ultralytics.com/modes/export/),
and [Jetson deployment guidance](https://docs.ultralytics.com/guides/nvidia-jetson/).
The repository has 160 tests. The 49 ROS-free tests pass directly, the 111
controller tests pass under the checked-in import-only ROS message shim, and
Python compilation passes on the Windows host. Native ROS tests, target inference
timing, detection quality, no-drive behavior, and supervised motion remain
pending on PT.

## Remove fixed-grid map completion percentage - 2026-10-24

The mapper reported the fraction of known cells in its configured 25.6 m
planning grid as a map-known percentage. Because the room boundary and true
total area are not known in advance, this number changes with the arbitrary
grid extent and is not a valid completion measure.

Chosen: remove the percentage from web status, autonomous planner logs, and the
bounded navigation-validation JSON. Remove the unused `ExploreMap.coverage()`
calculation and the validation script's `/map` subscription. Frontier selection,
known-free routing, occupancy rendering, and all LiDAR/depth behavior remain
unchanged. Historical percentages in earlier decision records remain only as
records of what prior builds displayed, not claims of room completion.

## Reproducible workstation release gate - 2026-10-24

The earlier workstation result used ad-hoc import-only ROS message shims, so a
fresh maintainer could run only the 49 ROS-free tests without reconstructing
that environment. The detector's startup fallback policy was also covered by
decoder tests and planned target checks, but not by direct unit regressions.

Chosen: retain `python3 -m unittest discover -v` as the robot-native gate and
add `host_test_runner.py` for no-ROS workstations. Its shims satisfy imports and
the simple message constructors used by tests; they do not start ROS or connect
to hardware. Four new tests cover a missing preferred model falling back to
the retained model, CUDA warm-up retrying the preferred model on CPU, an
incompatible preferred model rolling back after warm-up, and both models
missing disabling detection cleanly. The suite now has 160 tests: 49 run
directly without ROS and all 160 pass through the workstation shim runner.
Python compilation also passes. Native ROS execution, target OpenCV inference,
sensor behavior, and supervised motion remain robot-only validation gates.

## Conditional object-detector follow-ups recorded - 2026-10-25

The operator reports that the previously tested object-mapping revision placed
landmarks nearly perfectly and produced neither duplicate landmarks nor false
detections. Missed objects and inference delay were the observed problems. The
robot is currently unavailable, so no detector, projection, tracking, or survey
change has been selected or implemented from this report.

Chosen: preserve the current map projection, association, and confirmation
behavior while the pending Objects365/nonblocking revision awaits robot testing.
If those tests still reproduce missed detections or unacceptable delay, possible
evidence-driven follow-ups include: benchmark preferred and fallback models on
labeled robot-camera images; calibrate per-class thresholds; fine-tune a small
model on classroom images and missing classes; compare CPU, CUDA FP16, and
compatible TensorRT FP16 execution; select the sharpest settled survey frame;
bound queued inference work to the newest useful observation; use a selective
high-resolution or tiled second pass for otherwise empty views; improve endpoint
camera coverage; or narrow the production vocabulary before lowering selected
thresholds.

These are optional investigation paths, not requirements or approved
architecture changes. A larger model or unconditional high-resolution pass may
increase the existing delay, and lower thresholds may undo the observed lack of
false detections. Future comparisons must report recognition quality and
end-to-end latency separately from the already successful landmark placement.

## Deploy offline-tested routing/object revision - 2026-10-25

The 160-test revision was deployed to PT with a fresh Hector frame and one
drive-enabled mapper started with `_start_paused:=true`. The first native run
found one Python 3.6-only test incompatibility: `unittest.mock` call objects on
PT do not expose `.args` as on the workstation. The assertion now uses the
portable tuple form, and a repository `.gitattributes` policy forces LF text
exports so Windows-generated release archives keep robot shell scripts
executable. All 160 tests then passed both on the host and natively on PT.

The initial robot state had no `/dev/lidar` and no camera frames, with repeated
USB enumeration error `-71`. The documented RGB recovery could not find the
camera USB function. PT was rebooted while mapper and SLAM were stopped; after
boot and `/start_scan`, LiDAR ran near 12 Hz, Hector pose near 10 Hz, RGB near
29-30 Hz, native depth near 31-36 Hz, and registered depth near 29-31 Hz.
Exactly one mapper was left `AUTO PAUSED` with command `stop`, Hector was the
only SLAM process, the registered-depth overlay was live, and battery telemetry
was about 11.90 V / 75%. No motion command or physical movement occurred.

The preferred 365-label YOLO26 ONNX/sidecar pair was prepared at 640 pixels and
opset 12, checksum-verified, and installed without replacing `yolov5n.onnx`.
OpenCV 4.5.5 fails its blank warm-up forward pass with a DNN shape assertion,
so the designed startup rollback selects YOLOv5n. The fallback measured about
8-12.4 seconds per stationary inference. Results finishing near 12 seconds
correctly exceeded the processing-age guard and were shown but not mapped.
Objects365 recognition is therefore not active; physical motion, detector
quality, and endpoint timing remain pending supervised tests.

## Make Objects365 export compatible with target OpenCV - 2026-10-25

PT's OpenCV 4.5.5 loaded the simplified YOLO26 graph but failed its final
forward pass. Layer probing isolated the failure to the final box-scale
multiply: ONNX broadcasts `[1,4,8400] * [1,8400]`, while this OpenCV version
mishandles the rank difference. Lowering opset did not help, and disabling
simplification failed earlier on a broadcast Add.

Chosen: keep the official default grid output and opset 12, but have
`prepare_yolo_model.py` materialize the scale constant as `[1,4,8400]`. This
removes the unsupported broadcast and adds only about 100 KB. Randomized ONNX
Runtime comparison produced bit-identical output before and after rewriting.
`validate_yolo_model.py` now exercises the exact lightweight target runner and
can probe individual OpenCV layers. Three regressions cover the rewrite shape,
nonmatching shapes, and numeric label ordering; the suite now has 163 tests.

The patched artifact passed PT OpenCV warm-up and blank inference, then the
paused mapper reported `yolo26n-objv1-150.onnx (cpu)` with 5.8-6.9-second live
inference. Repeated stationary captures mapped six tracks and confirmed two
after three observations while LiDAR, RGB, and depth rates stayed live. This
validates model loading and the mapping pipeline, not recognition accuracy.
No movement command or physical motion occurred.

## Validate initial omnidirectional movement cases - 2026-10-25

With Hector as the only SLAM stack, exactly one drive-enabled mapper, autonomy
initially paused, and translation/rotation set to 50%, four directly supervised
manual dead-man pulses were performed. The operator confirmed that A strafed
left correctly without noticeable yaw, D strafed right without noticeable
forward motion or yaw, and Q/E rotated in place without lateral translation.
These are physical observations, separate from controller and Hector telemetry.

A known-free clicked target approximately 0.47 m away at a 31-degree bearing
was then requested; the planner selected a safe endpoint near `(0.35, 0.25)` m.
The operator confirmed continuous, entirely correct movement that matched the
map, satisfying the run sheet's 20-45-degree diagonal-route check. During later
route activity the operator intentionally moved both translation and rotation
sliders to 200%, explaining the observed slider changes. The active route was
subsequently cancelled and the mapper was left `AUTO PAUSED` with command
`stop`. No additional movement is authorized by this record.

Chosen: mark the left/right strafe, Q/E in-place rotation, and 20-45-degree
clicked-route checks complete. Keep the beyond-45-degree turn-first/SLAM-latch
test and diagonal LiDAR-obstacle gate test pending. These results do not validate
manual collision protection (manual movement bypasses autonomous obstacle
gates), low/reflective/transparent obstacles, or the upper-only depth system as
fully safe. The existing 163-test host and robot-native results are unchanged;
no code changed and tests were not rerun for this documentation-only update.

## Add an idempotent production startup launcher - 2026-10-25

Routine startup previously required several manual SSH commands and made it
easy to omit LiDAR recovery, use the wrong ROS workspace, or accidentally
start a duplicate mapper. The production `start_room_mapper.sh` launcher now
checks the ROS master, restores `/scan` when necessary, refuses to coexist
with visual SLAM, starts Hector only when absent, and starts exactly one mapper
with drive enabled but autonomy paused. It verifies the web status contains
`AUTO PAUSED` and command `stop`; failed mapper verification stops the mapper
it just launched. A live Hector map is preserved by default, while
`--fresh-map` is the explicit reset path. Runtime logs stay outside the
repository in `~/.room_mapper_logs/`.

The Windows `start_room_mapper.cmd` wrapper runs that launcher over SSH and
opens the dashboard only after verification succeeds. It stores no password
and never resumes autonomy or issues a motion command.

### Recover an unresponsive Hector stack during startup - 2026-10-26

The launcher previously treated the exit status of `rosnode ping -c 1` as
proof that a node was live. On PT's ROS Melodic installation that command
returns zero even when its output says the node is unknown, so an absent Hector
was misclassified as running and startup timed out waiting for
`/slam_out_pose`. Its diagnostic then incorrectly pointed to the launcher's
Hector log even though the launcher had not started the nonexistent process.

Chosen: define node liveness by the real XML-RPC reply text rather than the
unreliable command status, and require both a responsive Hector node and a
fresh pose message before preserving its map. If pose output is absent, stop
the owning `hex_map.launch` (or a manually started Hector node), wait for it to
exit, and start the checked-in production stack with a fresh map. Apply the
same recovery to an orphaned `hex_map.launch` whose Hector node is gone.
Shutdown first uses SIGINT, then explicitly stops only the four nodes owned by
`hex_map.launch` and escalates only the exact matching launch process if it
remains wedged. This recovery issues no gait command; the mapper still starts
drive-enabled, stopped, and `AUTO PAUSED`.

The fix was deployed and exercised against a deliberately started Hector node
that responded to ping but produced no pose because it lacked the production
TF stack. The launcher removed it, started `hex_map.launch`, received live
`/slam_out_pose` and `/map`, and started exactly one mapper `AUTO PAUSED` with
command `stop`. Re-running the launcher preserved the single stopped mapper.
All 180 tests pass on the host and natively on PT. This was startup and sensor
evidence only; no motion command or physical movement occurred.

## Implement the approved reliability plan - 2026-10-25

The three approved reliability priorities were implemented as separately
reviewable commits in their required order. `a201e37` extends registered-depth
projection with clustered valid-ray free evidence and clears a temporary
planner cell only after two consecutive clear observations. Current occupied
evidence vetoes clearing; invalid, occluded, out-of-view, and behind-surface
geometry cannot clear. The timestamped camera origin defines a protected
0.60 m near field, and the 30 s TTL remains the fallback. Clearing changes only
the temporary overlay, never Hector occupancy or native-depth collision state.

`5d82570` splits endpoint snapshot admission from inference. Every eligible
survey admits at most one immutable synchronized RGB/depth/intrinsics/TF/map-
generation observation, with at most 1.0 s acquisition grace. A single worker
runs inference while retaining only the newest pending endpoint snapshot;
captured, processed, replaced, and pending counts are reported. Map reset,
SLAM invalidation, shutdown, stale TF/result age, and repeated timestamps retain
explicit rejection paths. Three observations remain required, and default
exploration never waits for inference. The opt-in legacy focus workflow keeps
queued recapture support.

`6cd0253` removes the default scale from world-to-screen conversion and routes
path, robot, object, requested-goal, and adjusted-goal positions through the
active viewport transform. Marker glyph sizes remain fixed in pixels.

The host suite increased from 163 to 176 tests and passes through
`host_test_runner.py`; production modules compile. Synthetic coverage includes
removed/occluded/invalid depth sequences, protected near-field cells, LiDAR
preservation, route reopening, a blocked slow detector across later endpoints,
queue replacement/invalidation, and exact overlay positions at 25/50/100 px/m
with pan. These results are repository-only. PT remains on the earlier
163-test build; native tests, target performance, no-drive live evidence, and
physical validation are pending.

## Deploy and optimize the reliability revision - 2026-10-25

The reliability commits were deployed with `_drive:=false` and
`_start_paused:=true`. The first target benchmark averaged 0.347 s for a
synthetic 480x640 registered-depth projection. Under the live stack this
saturated a core and prevented successful image-time depth-map TF lookups, so
the no-drive gate failed before any motion was considered.

Chosen: preserve the exact occupied/free evidence and distinct-ray thresholds,
but encode signed row/column/ray tuples as one-dimensional integer keys before
deduplication. PT's older NumPy handles this substantially faster than
`unique(axis=0)`. Commit `bd3aee0` adds a signed-cell/distinct-ray regression.
All 177 tests pass on the host and natively on PT. The same target benchmark
then averaged 0.085 s (0.080-0.100 s across five runs).

The repeated no-drive run kept LiDAR near 12.1 Hz, `/map` near 1.2 Hz,
`/slam_out_pose` near 8.1 Hz, RGB near 30 Hz, native depth around 20-30 Hz, and
registered depth around 30-32 Hz. The preferred CPU model completed a
stationary inference in 6.61 s; snapshot status reported one captured and one
processed observation. The planner layer produced 23-26 temporary cells and
reported successful clears. Map endpoints returned HTTP 200 with correct
25/50/100 px/m and pan headers. Battery was 12.44 V / 94%.

Hector's current TF cadence means the strict 0.5 s image-time lookup can still
report transient misses between successful depth projections. The tolerance
was not relaxed without motion evidence. Measured low-object clearing,
near-field retention, multiple endpoint captures while inference is busy,
visual landmark alignment, detector quality, and all motion remain pending.
Exactly one drive-disabled mapper was left `AUTO PAUSED`; no gait command or
physical motion occurred.

## Replace fixed rear cutoff with a swept path - 2026-10-25

The operator reported that the 0.65 m rear-clearance requirement left the robot
stuck despite visible room. Autonomy was paused before diagnosis. The live scan
placed the limiting return about 0.60 m away at -150 degrees: only 0.516 m
rearward but 0.298 m lateral. The central rear corridor measured 0.688-0.771 m.
The old minimum over the entire 120-240 degree sector therefore treated a
corner return as though it were directly behind the robot.

Chosen: calculate how far the robot center can move backward before any LiDAR
point enters the circular 0.45 m emergency envelope. Require that swept-path
distance to exceed the predicted reverse travel at the active translation
speed plus 0.02 m. Shorten recovery from 1.0 s to one complete 0.5 s gait cycle,
then stop and reassess. A directly rear obstacle still blocks; a corner point
outside the swept circle no longer creates a false fixed-distance veto.
Floor-bump and depth-dropout alarms continue to prohibit autonomous reverse.

Three regressions cover corner geometry, speed-scaled path admission versus a
short direct path, and retention of the depth-dropout prohibition. All 180 host
tests pass and production Python compiles. Robot-native and physical validation
remain pending; the robot stayed paused and no motion was commanded.

The exact revision then passed all 180 tests natively on PT. One drive-enabled
mapper was restored `AUTO PAUSED` with command `stop`, preserving Hector's map.
The live scan produced 0.134 m of swept reverse travel before the 0.45 m
envelope. The speed-scaled requirement was 0.060 m at the reset 100% setting
and would be 0.100 m at 200%, so the observed corner geometry now passes. Native
depth recalibrated in upper-only mode and the earlier dropout alarm was no
longer present. These are software and no-motion sensor results; the new 0.5 s
reverse recovery remains pending direct supervised physical validation.

## Add opt-in TensorRT FP16 object inference with CPU rollback - 2026-10-26

OpenCV 4.5.5 on PT has no CUDA backend, but the installed TensorRT 8.2.1 and
PyCUDA 2019.1 stack can execute a device-specific engine. Chosen: retain CPU as
the default; add an explicit `tensorrt_fp16` selector, an isolated target-side
engine builder, checksum metadata tying the engine to the exact ONNX, and a
same-frame CPU/GPU benchmark. TensorRT and PyCUDA imports occur only on the GPU
path. Engine/checksum/load/warm-up failure retries the preferred ONNX on CPU,
then the retained YOLOv5n model. A CUDA failure during later inference
permanently replaces the detector with CPU for that mapper process and retries
the captured frame. Normal startup without the selector is the rollback path.

PT built `models/gpu_trial/yolo26n-objv1-150-fp16.engine` in 639.453 s without
overwriting either ONNX model. With the mapper, Hector, camera, and LiDAR
stopped at the operator's request, three saved non-classroom images produced a
1.010 s CPU median and 0.400 s TensorRT FP16 median across nine samples per
backend (worst 1.036 s and 0.410 s respectively). The backends returned the
same 11 detections; minimum matched box IoU was 0.979 and maximum confidence
difference was 0.023. One blank-frame TensorRT warm-up was 1.923 s. This proves
standalone acceleration and close numeric parity on those frames only. Labeled
classroom quality, empty-scene coverage, memory/thermal behavior, automatic
runtime fallback on PT, and paused drive-disabled full-stack sensor rates remain
gates before ordinary use. No movement was commanded.
All 185 repository tests pass through the host shim; robot-native execution of
this revision remains pending.
