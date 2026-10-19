# Stationary survey changes — 2026-10-12

## Current deployed addendum (supersedes operational details below)

This file retains historical validation evidence. The current implementation
differs in several important ways:

- Hector consumes raw `/scan` continuously; `/scan_stable` is diagnostic.
- A viewpoint survey requires five scans over six seconds, but is not required
  between every short gait burst.
- The web viewport follows the robot and supports zoom, pan, and Center reset.
  Fixed-viewport statements below describe an earlier experiment.
- Planner clearance is a 0.30 m radius plus 0.03 m margin, with approximately
  0.20 m waypoint spacing and 0.12 m waypoint tolerance.
- One sub-3-degree turn retries; three consecutive weak turns invalidate SLAM.
- Autonomous mapping holds in `object_focus` until a tentative YOLO landmark
  is confirmed or excluded.
- Depth results at or below 0.50 m are ignored by operator request. LiDAR keeps
  a 0.45 m emergency stop.
- Brief sparse depth reuses the last valid decision for at most 1.5 seconds;
  sustained sparsity still faults. The deployed suite passed 117 tests.

At this documentation checkpoint, a six-sample no-drive autonomous trial did
not reproduce the sparse-depth fault. The real controller was restored paused.
Historical stationary-map results below remain evidence only for their stated
configurations.

> Historical experiment: the later comparison with the independently working
> Team-Red project showed that stationary-only input is incompatible with
> Hector on this no-odometry robot. Hector now consumes raw `/scan`
> continuously; stationary surveys remain application-level waypoint checks.

The robot and workspace source hashes matched before editing. Robot originals
are in `/home/hiwonder/room_mapper/backup_stationary_survey_20261012`.

Findings and changes:

- The existing scan gate rejected moving scans, but motion resumed immediately
  after settling with no guarantee of a newly processed stationary scan.
  `StationarySurvey` now requires a six-second dwell, five distinct admitted
  scans, and a pose timestamp at or beyond the fifth scan before the next burst.
- Removed the initial full-circle body turn. Survey first, then plan into
  known-free space toward a frontier. Hold if no such route is available.
- Reject unknown start-neighborhood routes, blocked diagonal corners, and goals
  closer than 0.50 m. A live trial exposed repeated 0.10 m goals already inside
  the controller's 0.30 m arrival tolerance.
- The web viewport is anchored at the first pose; walls no longer shift as the
  robot translates. This is a display change, not a replacement for localization.

Validation:

- All 78 robot-side regression tests passed after adding the known-free
  viewpoint fallback.
- A 15-second stationary read-only measurement received 180 admitted scans
  (12.04 Hz), 129 poses and 16 maps. Maximum displacement from the first pose
  was 0.0095 m and maximum heading difference was 0.347 degrees.
- All measured poses used `map`; map frame/dimensions/resolution stayed fixed.
  Zero occupancy cells changed between the first and last stationary snapshots.
- `/odom/filtered` was advertised but no message arrived during a five-second
  probe, so odometry was not enabled as a pose prior.

Limits:

After a robot reboot restored RGB at about 30 Hz, Hector reported 0.56 m and
31.8 degrees during a fresh-map window. These were not physically observed and
must not be treated as proof of robot travel. Map coverage grew from 2% to 6%
and 7,718 cells changed. A second stationary window changed no cells and drifted
only 0.0073 m and 0.34 degrees.

The live map then contained 110 safely connected cells but no reachable cells
directly adjoining a frontier. The new fallback selected a known-free viewpoint
0.64 m away with 253 unknown cells in its 1.5 m neighborhood. During a bounded
fallback trial, SLAM reported 0.20 m and 10.8 degrees and updated 1,240 cells,
and completed its post-motion 5/5 localized survey without a pose jump. Coverage
remained 6%, so that pass refined geometry rather than expanding known area.

Hector still handles scan alignment and can drift or misalign scans. The
existing jump guard cannot retract a bad scan already integrated by Hector.
Existing maps were retained by some diagnostic restarts; the fresh walking run
used the normal reset and did not inherit the earlier duplicated map.
Scan gating tracks this controller's commands, not motion from another program
or someone physically moving the robot. A full robot reboot restored the RGB
camera after an in-process UVC restart failed. Depth floor-bump/dropout alarms
repeatedly caused HOLD/AVOID during the fallback trial and limited travel. The
robot was paused after the bounded test with the web page still available.

Repeat the read-only measurement with `python3 check_stationary_mapping.py`.

## False-wall correction

The initial Hector probabilities strongly favored occupied evidence (0.90 hit
versus 0.40 clear), and its 0.20 m / 0.06 rad update thresholds prevented
repeated stationary scans from updating the grid. One false hit could therefore
block a route without later clear observations removing it.

The deployed configuration integrates every admitted stationary scan, balances
clear/hit evidence at 0.35/0.65, and requires 75% confidence before the planner
classifies a wall. On a fresh stationary map, later scans corrected 276 cells
in the first 15-second window and 111 in the next as the map converged. Connected
safe space increased from the prior run's 110 cells with zero reachable
frontiers to 1,203 cells with 179 frontiers.

In the following bounded drive window, SLAM reported 0.37 m and 5.4 degrees;
physical travel was not confirmed. It updated 5,316 cells without a pose-jump fault. After
the move, 1,201 safe cells remained connected and 103 frontiers remained
reachable. The map image was substantially cleaner, with continuous wall lines
instead of the prior dense cluster of persistent false obstacles. The robot was
paused after the test. All 79 robot-side tests passed.
# Localization correction after the extended trial

The extended 2026-10-12 trial initially appeared to report about 2.4 m of
travel and only a small heading change. Physical observation disproved those
SLAM values: the robot translated less than 0.1 m and rotated more than 360
degrees. Those trial distances and coverage changes must not be treated as
valid localization evidence.

Root cause: the stationary-only scan gate also hid every zero-stride turn from
Hector. With no incremental scans during rotation, Hector retained nearly the
old heading and misinterpreted later geometry as translation. The corrected
gate now suppresses translating scans, forwards pure in-place turn scans for
incremental yaw estimation, and excludes those turn scans from the stationary
survey count.

A bounded one-second, zero-stride turn after the correction produced 33 SLAM
pose samples. Heading changed from 0.0 to 25.1 degrees while apparent
translation was only 0.007 m (`(-0.099, 0.353)` to `(-0.105, 0.350)`). This is
the expected localization signature for an in-place turn. Autonomous drive
remains disabled pending a later supervised navigation run.

The following 45-second navigation test issued six approximately 0.4-second
motion bursts separated by the stationary survey dwell. It ended paused with
8% known coverage and a net SLAM displacement of 0.699 m. A subsequent
15-second stationary check was stable (0.024 m and 0.97 degrees maximum drift),
but the navigation trace contained one 0.487 m pose update. That is implausible
for a single burst and was only just below the old 0.50 m fault threshold, so
the run is not considered a clean localization pass. The default jump guards
are now 0.25 m and 20 degrees; exceeding either stops further mapping/motion.
Physical observation subsequently confirmed that the robot did not translate
and its displayed heading again failed to follow its rotation. Therefore the
0.699 m result is false translation and this navigation test failed. The
successful slow-turn diagnostic used rotation rate 0.10, whereas autonomous
turning used 0.35; the default is now the validated 0.10 rate with a minimum
three-degree post-turn heading-progress check. Autonomous drive stays disabled
until a human-observed run confirms both translation and rotation.

