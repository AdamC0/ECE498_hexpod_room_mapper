# Object Mapping - Deployed Design

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
6. Two distinct RGB/depth observations confirm a map landmark.

Native depth remains separate and is used for collision sensing. Registered
depth is used only for RGB-aligned object placement.

## Autonomous focus workflow

During autonomous mapping, the first mapped observation of an unconfirmed
candidate clears the current path and changes the phase to `object_focus`.
The robot remains stopped while the object worker gathers distinct settled
observations.

- Confirmation: the track reaches two observations, remains on the map, and
  exploration replans.
- Exclusion: the candidate is missed in two distinct inference results or the
  15-second focus deadline expires. The tentative track is discarded and
  exploration replans.
- A discarded candidate is not suppressed; it may be detected again later.
- Already-confirmed landmarks do not repeatedly interrupt mapping.
- Clicked/object navigation (`user_go`) does not start mapping focus.

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
- Discard map placement if motion generation changes during inference.
- Clear all landmarks after map reset or localization loss.
- Show camera boxes even when map placement is rejected, but do not turn them
  into landmarks.
- Object tracks never modify occupancy geometry or planner obstacles.

## Tunable parameters

```text
object_confidence          0.45
object_merge_distance      0.65 m
object_min_observations    2
object_focus_seconds       15 s
object_focus_max_misses    2
object_sync_slop           0.20 s
object_max_age             2.5 s admission
object_result_max_age      12 s completion
object_standoff            0.75 m
```

## Validation

The robot-side suite covers frozen/reused frames, timestamp and dimension
checks, TF validity, motion during inference, SLAM loss, slow inference,
tracking/merge behavior, confirmation, focus exclusion, and non-suppressing
discard. The complete deployed suite passed 116 tests on 2026-10-12.
