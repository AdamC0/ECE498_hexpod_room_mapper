# Map, Depth-Layer, and Object-Capture Reliability Plan

Date: 2026-10-25

Status: proposed only; no behavior in this plan has been implemented or deployed.

## Goals and priority

Work proceeds from highest operational risk to lowest:

1. Conservatively clear disproven registered-depth obstacles.
2. Guarantee useful object snapshots without waiting for YOLO inference.
3. Keep mapped-object screen positions correct at every map zoom and pan.

The depth change is first because it alters the planner's collision geometry.
The object-capture change is second because it affects the mapper's core
mission and timing. The display-only zoom defect is third because it does not
change the planner's world coordinates or robot motion.

## Phase 0 - Baseline and regression fixtures

Before changing behavior:

- Reproduce the object-marker zoom error with synthetic world coordinates at
  50%, 100%, and 200% views, including a panned view.
- Add a synthetic depth sequence containing an obstacle, valid farther returns
  after its removal, an occluded obstacle, and invalid/dropout pixels.
- Add a fast-controller/slow-detector test that crosses several stationary
  survey endpoints while inference takes approximately the measured 5.8-6.9 s.
- Record current host test results. Deployment and robot-native results remain
  separate gates and will not be described as physical evidence.

## Priority 1 - Remove only depth obstacles disproved by valid observations

### Cause

`project_obstacle_cells()` currently returns occupied cells only, and
`ExploreMap.update_depth_obstacles()` can only add/refresh cells. A moved object
therefore stays magenta until its 30 s TTL expires even when the registered
depth camera is facing its old position and measures clear space.

### Design

- Extend registered-depth projection to produce two disjoint sets:
  `occupied_cells` and `observed_free_cells`.
- Derive free evidence only from valid depth rays. A ray may clear cells before
  its measured surface, with a range/noise margin, and only where that ray
  passes through the configured 0.025-0.50 m obstacle-height band.
- Never clear through a nearer return, an invalid/zero/NaN pixel, a dropout,
  outside the camera frustum, beyond the configured 2.0 m depth-map range, or
  behind the measured surface. Occupied evidence wins if a cell appears in
  both sets in one update.
- Require two consecutive valid clear observations by default before deleting
  a live depth-only cell. New occupied evidence resets its clear count.
- Apply clearing only to the temporary registered-depth overlay. It must never
  erase Hector/LiDAR occupancy, object landmarks, or the native-depth collision
  decision. The existing 30 s TTL remains the fallback for cells that are not
  positively re-observed.
- Expose the confirmation count as a bounded ROS parameter and report recent
  clears in status/logging so field behavior can be audited.

### Acceptance tests

- A removed low obstacle clears after the configured number of valid farther
  observations when its former cells are in view.
- A present, partly occluded, out-of-view, or invalid-depth obstacle is not
  cleared early.
- Simultaneous occupied evidence prevents clearing.
- LiDAR-black cells remain occupied when overlapping magenta cells clear.
- Planner inflation/path tests show the route opens only after the temporary
  cell is actually removed.

### Live validation gate

Deploy with `_drive:=false` first. Place and remove a matte, depth-visible test
object at measured positions and compare registered depth, magenta cells, and
LiDAR occupancy. Only then repeat with one mapper, Hector only, `_drive:=true
_start_paused:=true`, and direct supervision. Reflective, transparent,
out-of-view, and invalid geometry remain explicit limitations.

## Priority 2 - Capture at every useful stop without blocking on YOLO

### Cause

Snapshot admission and slow inference currently happen in the same worker.
The controller can finish its two-second/three-scan survey and begin the next
route before that worker polls the brief settled window. In that case no
immutable RGB/depth/TF observation exists for YOLO to finish later. In
addition, three distinct captures are required before a landmark is drawn, so
missed endpoint captures are especially visible during fast exploration.

### Design

- Split cheap snapshot admission from expensive inference. When a stationary
  survey becomes ready, the controller captures one fresh synchronized RGB,
  registered-depth, intrinsics, timestamped TF, map generation, and observation
  ID before leaving the endpoint.
- Give snapshot acquisition a short bounded grace period only when a fresh
  synchronized pair is not immediately available. The proposed maximum is
  1.0 s; the robot must never wait for the 5.8-6.9 s inference itself.
- Feed immutable snapshots to the existing detector thread through a bounded
  queue: one active inference plus one newest pending endpoint snapshot.
  Replace an older unprocessed pending snapshot rather than accumulating stale
  work, and expose captured/processed/replaced counters in status.
- Preserve image-time TF projection, map-generation checks, result-age checks,
  class thresholds, association distance, and three-distinct-observation
  confirmation. Do not lower confirmation merely to make markers appear.
- Allow default nonblocking inference to finish during travel. At later natural
  endpoints, capture additional distinct observations. Do not restore the
  legacy 15 s `object_focus` hold by default.
- If robot measurements show that three natural viewpoints are still uncommon,
  add a bounded confirmation-viewpoint preference at the next normal planning
  stop. It may acquire another snapshot but may not wait for inference or
  repeatedly trap exploration around one candidate.

### Acceptance tests

- Every eligible endpoint admits at most one distinct snapshot even when the
  detector is busy.
- A 7 s fake inference cannot make the controller miss later endpoint capture
  opportunities or block route start beyond the bounded acquisition grace.
- Queue replacement is deterministic and cannot recount frozen RGB/depth
  timestamps.
- Three distinct snapshots confirm a landmark at the image-time map position;
  motion after capture does not move it.
- Map reset, SLAM loss, stale TF, excessive result age, and shutdown invalidate
  queued work safely.
- A no-detection scene adds no recurring inference hold and does not materially
  restore the old stop-heavy exploration cadence.

### Live validation gate

Run paused/no-drive timing first and log endpoint-ready, capture, inference,
queue, and commit times. Then perform a supervised route through a labeled
scene and compare route cadence against the current build. Success requires
landmarks from three distinct settled viewpoints, no stale placement, and at
most the configured snapshot-acquisition delay per endpoint. Detector quality
and control-loop timing are reported separately.

## Priority 3 - Correct landmark rendering at all map scales

### Cause

`draw_map()` passes the selected scale to occupancy, path, and robot
conversions, but object landmarks call `to_screen()` without it and therefore
use the fixed 50 px/m default. Requested and adjusted goal markers have the same
omission.

### Design

- Pass the active render scale to every world-to-screen conversion for object,
  requested-goal, and adjusted-goal positions.
- Keep marker glyphs and labels approximately constant in pixel size for
  readability; only their world-coordinate screen position scales and pans.
- Centralize overlay conversion enough that future world-anchored layers
  cannot silently fall back to the default scale.

### Acceptance tests

- Pixel positions for a fixed world point match the exact viewport transform
  at minimum, default, and maximum zoom and under pan.
- Object, route, robot, requested goal, and adjusted goal remain co-located
  with the same map feature after repeated zoom/center operations.
- Click-to-navigate coordinate conversion remains unchanged and round-trips
  against the rendered headers.

## Integration, documentation, and release order

Each priority is implemented as a separately reviewable commit with its own
tests. After all host tests pass:

1. Update `README.md`, `HANDOFF.md`, `RUN_SHEET.txt`, and `DECISIONS.md` with
   the implemented defaults and explicit limitations.
2. Push the tested repository changes before robot deployment.
3. Deploy while paused, run the complete native suite, and perform no-drive
   sensor/timing checks.
4. Run any physical validation only with direct supervision and exactly one
   drive-enabled mapper. Keep Hector authoritative and RTAB-Map stopped.
5. Record unit tests, no-drive evidence, SLAM telemetry, and operator-observed
   physical behavior as separate evidence.

## Operator decisions requested before implementation

The proposed defaults are:

- Clear a magenta cell after two consecutive valid depth observations disprove
  it, while retaining the 30 s TTL when no valid view exists.
- Keep three distinct observations for landmark confirmation.
- Allow up to 1.0 s of extra endpoint dwell only to obtain a synchronized
  snapshot; never wait there for YOLO inference.

Implementation should not begin until the operator accepts these defaults or
selects alternatives.
