# Object Mapping - Repository Design (2026-10-24; deployment pending)

The original object-mapping plan is implemented. This document describes the
current behavior rather than future work.

## Pipeline

1. YOLO runs on fresh RGB frames while the robot is stationary and settled.
2. A synchronized registered-depth image is sampled inside each bounding box.
3. `/camera/rgb/camera_info` intrinsics back-project the sample into the camera
   optical frame.
4. Timestamped TF transforms that point into `map`. Stale, disconnected, or
   cyclic TF is rejected.
5. `ObjectTracker` merges same-class observations within 0.65 m and retains
   count, confidence, position variance, display name, and last-seen time.
6. Three distinct settled RGB/depth captures confirm a map landmark. A pending
   track is retained for five minutes so different survey viewpoints can
   provide confirmation without stopping in place.

Native depth remains separate and is used for collision sensing. Registered
depth is used only for RGB-aligned object placement.

## Nonblocking autonomous workflow

YOLO only captures a synchronized observation while the robot is stationary,
settled, localized, and using a trusted fresh map. It snapshots the image-time
TF before inference. The next route may begin immediately; the worker can
finish and map the immutable snapshot during motion. This removes detector
latency from autonomous route selection without projecting a moving frame.

- Confirmation: the same class is placed consistently in three distinct
  settled captures, potentially from different route endpoints.
- A one-off false positive remains pending and expires; it never becomes a
  confirmed landmark merely because inference is slow.
- Already-confirmed landmarks do not repeatedly interrupt mapping.
- Set `_object_focus_enabled:=true` to restore the former two-miss/15-second
  blocking focus behavior for comparison or rollback.

The web status reports `object_focus` and explains whether a candidate was
added or excluded. The Objects tab displays pending and confirmed tracks and
allows navigation, rename, and removal. User removal suppresses the same class
near that location; automatic focus exclusion does not.

## Integrity rules

- Accept map positions only while stopped, settled, and using a fresh trusted
  SLAM pose/map.
- Use distinct synchronized RGB/depth timestamps; frozen frames cannot inflate
  a track count.
- Snapshot image-time TF before slow inference.
- Motion after a valid settled capture is allowed; the stored RGB, registered
  depth, camera intrinsics, timestamps, and image-time TF remain immutable.
- Discard placement if map trust, pose/map freshness, TF, synchronization, or
  result-age validation fails.
- Clear all landmarks after map reset or localization loss.
- Show camera boxes even when map placement is rejected, but do not turn them
  into landmarks.
- Object tracks never modify occupancy geometry or planner obstacles.

## Tunable parameters

```text
preferred model           yolo26n-objv1-150.onnx + matching .names
fallback model            yolov5n.onnx (COCO)
object_confidence          0.45 general / 0.30 classroom-priority
object_merge_distance      0.65 m
object_min_observations    3
object_track_max_age       300 s
object_focus_enabled       false
object_focus_seconds       15 s (legacy mode only)
object_focus_max_misses    2 (legacy mode only)
object_sync_slop           0.20 s
object_max_age             2.5 s admission
object_result_max_age      12 s completion
object_standoff            0.75 m
yolo_input_size            640
yolo_backend               cpu (cuda_fp16 is opt-in with CPU fallback)
opencv_threads             2
```

## Validation

The suite covers frozen/reused frames, timestamp and dimension checks, TF
validity, asynchronous completion after motion resumes, legacy focus rollback,
SLAM loss, slow inference, tracking/merge behavior, class-specific thresholds,
class-aware NMS, external vocabularies, YOLOv5/v8/raw and six-value end-to-end
outputs, confirmation, and non-suppressing discard. Robot-native validation of
the 2026-10-24 revision and the preferred model remains pending.
