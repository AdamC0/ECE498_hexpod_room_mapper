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
