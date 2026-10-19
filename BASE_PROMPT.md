# Base Prompt for the Next Agent

You are taking over the JetHexa room-mapping project. Read `HANDOFF.md`,
`README.md`, `RUN_SHEET.txt`, and the newest `DECISIONS.md` entry before making
changes. Keep documentation synchronized with deployed behavior.

## Version control

All completed project changes must be committed and pushed to the GitHub
repository at `https://github.com/AdamC0/ECE498_hexpod_room_mapper`. Before
committing, review the diff and repository status. Use a clear, accurate commit
message that describes the changes, and do not leave completed work only in the
local working tree.

Robot: `hiwonder@192.168.137.223`

Robot project: `/home/hiwonder/room_mapper`

Web UI: `http://192.168.137.223:8000`

Do not store or repeat the SSH password in project files. Real motion requires
direct supervision. Prefer `_drive:=true _start_paused:=true` and confirm only
one mapper process exists.

## Current architecture

- YDLIDAR G4 raw `/scan` feeds Hector continuously.
- `hex_map.launch` owns the required TF bridges.
- `map_explore.py` plans through known free space using a 0.30 m radius and
  0.03 m margin.
- `room_mapper.py` handles bounded gait bursts, recovery, sensor safety,
  manual control, clicked routes, and autonomous object focus.
- `map_web.py` provides RGB, a robot-following zoomable map, click navigation,
  Cancel Route, manual W/A/S/D, independent 50-150% translation/rotation
  sliders, object management, and a battery badge.
- Object mapping uses YOLO, synchronized registered depth, camera intrinsics,
  and timestamped TF. Two observations confirm a landmark.

## Current safety/calibration facts

- LiDAR emergency stop: 0.45 m.
- Depth stop: 0.60 m.
- Operator-requested depth exclusion: results <= 0.50 m are ignored.
- Depth validity: 200 pixels and 2% upper-band coverage.
- Brief sparsity: reuse last valid result for at most 1.5 s; sustained sparsity
  or no previous valid result still faults.
- Map pose jump: 0.25 m / 20 degrees.
- Turn progress: three consecutive bursts below 3 degrees invalidate SLAM.
- Object focus: 15 s maximum, two misses, then replan.

Ignoring close depth is a deliberate blind region. LiDAR protects its plane,
but elevated close objects can be missed. Do not describe this as fully safe.

## Latest validation

The robot is online with autonomy paused. All 122 tests passed on the robot. RGB,
depth, LiDAR, Hector, `/voltage`, map, and web UI are live. The battery endpoint
initially reported 10.328 V / 18.9%.

`/voltage` is `std_msgs/Float32`. The dedicated vendor `battery_low_alarm` node
is persistently excluded from both controller launch variants; launch backups
are stored beside them. Do not disable voltage telemetry or the website's
visual low warning.

After a reboot, RGB/depth may recover but LiDAR may require:

```bash
rosservice call /start_scan '{}'
```

If RGB has a publisher but no frames, a full robot reboot has been the reliable
recovery. After reboot restore LiDAR, Hector, and the mapper. If SLAM jumps or
drifts while stationary, discard the map and start a fresh Hector frame.

Always separate unit-test success, no-drive live evidence, SLAM-reported motion,
and physically observed robot motion in reports.
