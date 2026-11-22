"""Stationary and moving object capture, inference, and verification policy."""

import math
import time

import rospy

from object_mapping import (backproject, canonical_object_label,
                            deduplicate_mapped_detections, landmark_visibility,
                            sample_box_depth, transform_point)


def object_loop(self):
    period = 1.0 / max(0.1, self.object_hz)
    next_run = 0.0
    while not rospy.is_shutdown() and not self.shutting_down:
        now = time.time()
        if now < next_run:
            time.sleep(min(.05, next_run - now))
            continue
        # Finish the short endpoint acquisition before starting inference.
        # TensorRT post-processing can otherwise contend for the GIL and
        # prevent the control timer from collecting the remaining fresh
        # frames inside the fixed one-second grace.
        endpoint_key = self.object_endpoint_key
        if (endpoint_key is not None and
                self.object_endpoint_done != endpoint_key):
            time.sleep(.02)
            continue
        verification = getattr(self, "novel_object_verification", None)
        if verification is not None and verification.get("acquiring", False):
            time.sleep(.02)
            continue
        with self.object_queue_lock:
            snapshot = (self.object_pending_snapshots.pop(0)
                        if self.object_pending_snapshots else None)
        if snapshot is None:
            time.sleep(.05)
            continue
        try:
            self.process_object_snapshot(snapshot)
        except Exception as exc:
            # process_object_snapshot() clears this flag for detector
            # failures and normal commits. Errors while validating depth
            # or updating tracks can also escape, so recover the worker
            # state here or preview capture will remain disabled forever.
            with self.control_lock:
                self.object_processing = False
            self.object_status = "processing failed: %s" % exc
            rospy.logerr_throttle(5, "Object processing failed: %s", exc)
        finally:
            with self.object_queue_lock:
                self.object_snapshots_processed += 1
            verification_id = snapshot.get("novel_verification_id")
            if verification_id is not None:
                with self.control_lock:
                    verification = getattr(self, "novel_object_verification", None)
                    if (verification is not None and
                            verification.get("id") == verification_id):
                        verification["processed"] += 1
            # Schedule from completion, not start. A slow detector must not
            # immediately begin another inference and monopolize the CPU.
            next_run = time.time() + period

def objects_ready(self, now):
    """Called with control/data locks; freshness alone does not prove SLAM accuracy."""
    return (not self.shutting_down and not self.commanded_moving and
            now >= self.settle_until and self.map_trusted and self.use_map and
            self.pose is not None and 0 <= now - self.t_pose <= self.slam_stale and
            self.map_is_fresh(now) and self.detector is not None)

def object_commit_ready(self, now):
    """Return whether a previously captured settled observation is still valid.

    Image-time TF and synchronized RGB/depth are snapshotted before
    inference. Movement after that capture does not alter their geometry,
    so slow inference may commit while a new route is already active.
    """
    return (not self.shutting_down and self.map_trusted and self.use_map and
            self.pose is not None and 0 <= now - self.t_pose <= self.slam_stale and
            self.map_is_fresh(now) and self.detector is not None)

def capture_object_snapshot(self):
    """Freeze one settled synchronized observation before slow inference."""
    with self.control_lock, self.data_lock:
        now = time.time()
        if not self.objects_ready(now):
            self.object_status = "waiting for settled robot and fresh, trusted SLAM"
            return None
        if (self.camera is None or self.registered_depth is None or
                self.camera_k is None or not self.camera_frame):
            self.object_status = "waiting for RGB, registered depth, or camera info"
            return None
        stamps = (self.camera_stamp, self.registered_depth_stamp)
        ros_now = rospy.Time.now().to_sec()
        if any(s <= 0 or not 0 <= ros_now - s <= self.object_max_age for s in stamps):
            self.object_status = "waiting for fresh RGB/depth timestamps"
            return None
        if (any(s <= old for s, old in zip(stamps, self.last_object_stamps)) or
                min(self.t_camera, self.t_registered_depth) < self.settle_until or
                min(stamps) < ros_now - (now - self.settle_until)):
            self.object_status = "waiting for a new settled RGB/depth pair"
            return None
        if abs(stamps[0] - stamps[1]) > self.object_sync_slop:
            self.object_status = "waiting for synchronized RGB/depth (skew %.3f s)" % \
                abs(stamps[0] - stamps[1])
            return None
        shape = self.camera.shape[:2]
        if (shape != self.registered_depth.shape[:2] or shape != self.camera_info_size or
                self.camera_frame.lstrip('/') != self.camera_info_frame):
            self.object_status = "RGB/depth/calibration dimensions or RGB frame mismatch"
            return None
        frame, depth = self.camera.copy(), self.registered_depth.copy()
        k, frame_id = self.camera_k, self.camera_frame
        self.last_object_stamps = stamps
        # Snapshot image-time TF before slow inference can evict its history.
        with self.object_lock:
            try:
                transform = self.tf_buffer.lookup(getattr(self, "map_frame", "map"), frame_id,
                                                  stamps[0], max_age=.5)
            except (KeyError, ValueError) as exc:
                transform = None
                rospy.logwarn_throttle(5, "Object map TF unavailable: %s", exc)
        return {
            "frame": frame,
            "depth": depth,
            "k": k,
            "frame_id": frame_id,
            "stamps": stamps,
            "transform": transform,
            "motion_generation": self.motion_generation,
            "map_generation": getattr(self, "map_generation", 0),
            "observation_id": stamps[0],
        }

def enqueue_object_snapshot(self, snapshot):
    """Queue bounded immutable observations and discard the oldest overflow."""
    with self.object_queue_lock:
        queue_limit = max(self.object_burst_frames,
                          getattr(self, "novel_object_frames", 5))
        if len(self.object_pending_snapshots) >= queue_limit:
            self.object_pending_snapshots.pop(0)
            self.object_snapshots_replaced += 1
        self.object_pending_snapshots.append(snapshot)
        self.object_snapshots_captured += 1

def capture_preview_snapshot(self, now):
    """Queue one fresh settled observation after the endpoint burst.

    Continued inference at the same stopped endpoint is validation-eligible.
    capture_object_snapshot() still enforces fresh synchronized timestamps,
    settled motion, trusted SLAM, and image-time TF, while one-at-a-time
    admission prevents a slow detector from accumulating a stale backlog.
    """
    if (not getattr(self, "object_preview_enabled", True) or
            now < getattr(self, "object_preview_next_capture", 0.0)):
        return False
    with self.object_queue_lock:
        busy = bool(self.object_pending_snapshots)
    if busy or self.object_processing:
        return False
    previous_status = getattr(self, "object_status", "waiting for preview")
    snapshot = self.capture_object_snapshot()
    if snapshot is None:
        # Preview retries are opportunistic. Preserve the last completed
        # inference result instead of presenting transient sync skew as a
        # stopped detector.
        self.object_status = previous_status
        return False
    snapshot["validation_group"] = (self.object_endpoint_key or
                                    ("stationary", getattr(self.survey,
                                                            "settled_at", 0.0)))
    self.enqueue_object_snapshot(snapshot)
    self.object_preview_next_capture = now + 1.0 / max(.1, self.object_hz)
    return True

def capture_motion_preview_snapshot(self, now):
    """Queue one latest RGB-only inference while the robot is moving.

    Moving observations are presentation-only. They deliberately omit
    registered depth, TF, and validation metadata so they can never update
    mapped landmarks or provide absence evidence.
    """
    if (not getattr(self, "object_preview_enabled", True) or
            not getattr(self, "commanded_moving", False) or
            getattr(self, "detector", None) is None or
            now < getattr(self, "object_motion_preview_next_capture", 0.0)):
        return False
    with self.data_lock:
        stamp = getattr(self, "camera_stamp", 0.0)
        frame = None if self.camera is None else self.camera.copy()
        camera_age = now - getattr(self, "t_camera", 0.0)
    if (frame is None or stamp <= 0.0 or
            stamp <= getattr(self, "object_motion_preview_stamp", 0.0) or
            not 0.0 <= camera_age <= getattr(self, "object_max_age", 2.5)):
        return False
    snapshot = {
        "frame": frame,
        "depth": None,
        "k": None,
        "stamps": (stamp, stamp),
        "transform": None,
        "motion_generation": self.motion_generation,
        "map_generation": getattr(self, "map_generation", 0),
        "observation_id": stamp,
        "validation_group": ("moving-preview",),
        "display_only": True,
    }
    self.object_motion_preview_stamp = stamp
    with self.object_queue_lock:
        moving_indexes = [index for index, item in
                          enumerate(self.object_pending_snapshots)
                          if item.get("display_only", False)]
        for index in reversed(moving_indexes):
            self.object_pending_snapshots.pop(index)
            self.object_snapshots_replaced += 1
        # Prioritize one current moving view ahead of retained stationary
        # mapping observations. Motion bursts are bounded, so stationary
        # evidence resumes afterward without making the live view wait for
        # the whole endpoint burst to drain.
        self.object_pending_snapshots.insert(0, snapshot)
        self.object_snapshots_captured += 1
    self.object_motion_preview_next_capture = now + 1.0 / max(.1, self.object_hz)
    return True

def novel_object_direction(box, image_width):
    """Bucket a moving candidate so failed views receive a local cooldown."""
    center = (float(box[0]) + float(box[2])) * .5
    if center < image_width / 3.0:
        return "left"
    if center > image_width * 2.0 / 3.0:
        return "right"
    return "center"

def request_novel_object_verification_locked(self, detections, frame, now):
    """Remember one moving-view candidate for the control timer to stop on."""
    if (not getattr(self, "novel_object_stop_enabled", True) or
            getattr(self, "novel_object_request", None) is not None or
            getattr(self, "novel_object_verification", None) is not None or
            getattr(self, "paused", True) or getattr(self, "phase", "") != "go" or
            not getattr(self, "commanded_moving", False)):
        return False
    cooldowns = getattr(self, "novel_object_cooldowns", {})
    expired = [key for key, deadline in cooldowns.items() if now >= deadline]
    for key in expired:
        cooldowns.pop(key, None)
    width = frame.shape[1]
    for detection in sorted(detections,
                            key=lambda item: float(item.get("confidence", 0.0)),
                            reverse=True):
        label = canonical_object_label(detection.get("label", ""))
        box = tuple(detection.get("box", ()))
        if not label or len(box) != 4:
            continue
        direction = self.novel_object_direction(box, width)
        key = (label.lower(), direction)
        if now < cooldowns.get(key, 0.0):
            continue
        self.novel_object_request = {
            "label": label, "box": box, "direction": direction,
            "key": key, "requested_at": now,
            "known_ids": {track["id"] for track in self.object_tracker.tracks},
        }
        self.object_status = ("moving preview found possible new %s; stopping for "
                              "settled depth verification" % label)
        return True
    return False

def begin_novel_object_verification(self, now):
    """Stop autonomous frontier travel for a requested moving candidate."""
    request = getattr(self, "novel_object_request", None)
    if request is None:
        return False
    self.novel_object_request = None
    if self.paused or self.phase != "go":
        return False
    self.send(("stop",))
    self.path = []
    self.novel_object_sequence = getattr(self, "novel_object_sequence", 0) + 1
    self.novel_object_verification = dict(
        request, id=self.novel_object_sequence, deadline=now + self.novel_object_timeout,
        next_capture=now, captured=0, processed=0, observations=set(),
        range_checked=False, acquiring=True)
    self.phase, self.phase_start = "novel_object_verify", now
    self.web_msg = ("Possible new %s detected while moving; stopped for settled "
                    "depth check and %d-frame verification" %
                    (request["label"], self.novel_object_frames))
    return True

def finish_novel_object_verification(self, now, message, cooldown=True):
    verification = getattr(self, "novel_object_verification", None)
    if verification is None:
        return
    if cooldown:
        self.novel_object_cooldowns[verification["key"]] = \
            now + self.novel_object_cooldown
    self.novel_object_verification = None
    self.path = []
    self.phase, self.phase_start = "plan", now
    self.web_msg = message

def capture_novel_object_snapshot(self, now):
    """Range-gate a candidate, then collect distinct settled observations."""
    verification = getattr(self, "novel_object_verification", None)
    if verification is None:
        return
    if now >= verification["deadline"]:
        self.finish_novel_object_verification(
            now, ("Close-object verification hold reached %.0f s after capturing "
                  "%d/%d frames; frozen observations may finish processing while "
                  "autonomy resumes" %
                  (self.novel_object_timeout, verification["captured"],
                   self.novel_object_frames)))
        return
    if verification["captured"] >= self.novel_object_frames:
        verification["acquiring"] = False
        if verification["processed"] >= self.novel_object_frames:
            self.finish_novel_object_verification(
                now, "%d close-object frames processed; autonomy resuming" %
                self.novel_object_frames)
        return
    if now < verification["next_capture"]:
        return
    snapshot = self.capture_object_snapshot()
    if snapshot is None:
        return
    observation = tuple(snapshot["stamps"])
    if observation in verification["observations"]:
        return
    if not verification["range_checked"]:
        sample = sample_box_depth(snapshot["depth"], verification["box"],
                                  min_pixels=self.object_depth_min_pixels,
                                  return_quality=True)
        if sample is None:
            self.finish_novel_object_verification(
                now, "Candidate had no reliable settled depth; autonomy resuming")
            return
        distance, u, v, quality = sample
        quality_ok = (
            quality["valid_pixels"] >= self.object_depth_min_pixels and
            quality["coverage"] >= self.object_depth_min_coverage and
            quality["mad"] <= self.object_depth_max_mad and
            quality["inlier_ratio"] >= self.object_depth_min_inlier_ratio)
        if not quality_ok:
            self.finish_novel_object_verification(
                now, "Candidate depth was unreliable; autonomy resuming")
            return
        if distance >= self.novel_object_range:
            self.finish_novel_object_verification(
                now, ("Candidate depth %.2f m is outside the %.2f m stop range; "
                      "autonomy resuming" % (distance, self.novel_object_range)))
            return
        transform = snapshot.get("transform")
        point = (transform_point(backproject(u, v, distance, snapshot["k"]),
                                 *transform) if transform is not None else None)
        if point is None:
            self.finish_novel_object_verification(
                now, "Candidate map transform was unavailable; autonomy resuming")
            return
        with self.object_lock:
            known = next((track for track in self.object_tracker.tracks
                          if track["id"] in verification["known_ids"] and
                          canonical_object_label(track["label"]).lower() ==
                          verification["label"].lower() and
                          math.hypot(track["x"] - point[0],
                                     track["y"] - point[1]) <=
                          self.object_merge_distance), None)
        if known is not None:
            self.finish_novel_object_verification(
                now, ("Settled %s matches mapped %s object %d; autonomy resuming" %
                      (verification["label"],
                       "confirmed" if known.get("confirmed") else "pending",
                       known["id"])))
            return
        verification["range_checked"] = True
        verification["distance"] = distance
    verification["observations"].add(observation)
    snapshot["validation_group"] = ("novel-object", verification["id"])
    snapshot["novel_verification_id"] = verification["id"]
    self.enqueue_object_snapshot(snapshot)
    verification["captured"] += 1
    verification["next_capture"] = now + self.object_burst_interval
    if verification["captured"] >= self.novel_object_frames:
        verification["acquiring"] = False
        self.object_status = ("captured %d/%d close-object frames at %.2f m; "
                              "processing" %
                              (verification["captured"], self.novel_object_frames,
                               verification["distance"]))

def capture_endpoint_snapshot(self, now):
    """Capture a short fresh-frame burst within the bounded endpoint grace."""
    if self.detector is None:
        return True
    key = (getattr(self.survey, "settled_at", 0.0),
           getattr(self.survey, "required_stamp", 0.0))
    if self.object_endpoint_done == key:
        return True
    if self.object_endpoint_key != key:
        self.object_endpoint_key = key
        self.object_endpoint_deadline = now + self.object_snapshot_grace
        self.object_endpoint_captured = 0
        self.object_endpoint_next_capture = now
        self.object_endpoint_observations = set()
    if now < self.object_endpoint_next_capture:
        return False
    snapshot = self.capture_object_snapshot()
    if snapshot is not None:
        observation = tuple(snapshot["stamps"])
        if observation in self.object_endpoint_observations:
            if now >= self.object_endpoint_deadline:
                self.object_endpoint_done = key
                self.object_status = (
                    "endpoint burst captured %d/%d distinct frames within %.2f s grace" %
                    (self.object_endpoint_captured, self.object_burst_frames,
                     self.object_snapshot_grace))
                return True
            return False
        self.object_endpoint_observations.add(observation)
        snapshot["validation_group"] = key
        self.enqueue_object_snapshot(snapshot)
        self.object_endpoint_captured += 1
        self.object_endpoint_next_capture = now + self.object_burst_interval
        if self.object_endpoint_captured >= self.object_burst_frames:
            self.object_endpoint_done = key
            self.object_endpoint_deadline = 0.0
            return True
    if now >= self.object_endpoint_deadline:
        self.object_endpoint_done = key
        self.object_status = ("endpoint burst captured %d/%d distinct frames within %.2f s grace" %
                              (self.object_endpoint_captured,
                               self.object_burst_frames,
                               self.object_snapshot_grace))
        return True
    return False

def process_objects(self):
    """Synchronously capture/process one pair for tests and legacy callers."""
    snapshot = self.capture_object_snapshot()
    if snapshot is not None:
        self.process_object_snapshot(snapshot)

def process_object_snapshot(self, snapshot):
    """Run YOLO and commit one previously frozen observation."""
    frame = snapshot["frame"]
    depth = snapshot["depth"]
    k = snapshot["k"]
    stamps = snapshot["stamps"]
    transform = snapshot["transform"]
    generation = snapshot["motion_generation"]
    map_generation = snapshot["map_generation"]
    observation_id = snapshot.get("observation_id", stamps[0])
    validation_group = snapshot.get("validation_group", ("legacy",))
    display_only = bool(snapshot.get("display_only", False))
    # Set this under the control lock so legacy focus mode can keep its
    # original inference hold without affecting default nonblocking mode.
    with self.control_lock:
        if self.shutting_down:
            return
        self.object_processing = True
    inference_started = time.time()
    try:
        try:
            detections = self.detector.detect(frame)
        except Exception as exc:
            if getattr(self, "active_yolo_backend", "") != "tensorrt_fp16":
                raise
            self.fallback_yolo_to_cpu(exc)
            detections = self.detector.detect(frame)
    except Exception:
        with self.control_lock:
            self.object_processing = False
        raise
    inference_seconds = time.time() - inference_started
    for detection in detections:
        detection["map_point"] = None
        detection["map_rejection"] = None
        if display_only:
            continue
        depth_min_pixels = getattr(self, "object_depth_min_pixels", 24)
        sample = sample_box_depth(depth, detection["box"],
                                  min_pixels=depth_min_pixels,
                                  return_quality=True)
        if sample is None:
            detection["map_rejection"] = "insufficient registered depth"
            continue
        distance, u, v, quality = sample
        detection["depth"] = distance
        detection["depth_quality"] = quality
        quality_ok = (
            quality["valid_pixels"] >= depth_min_pixels and
            quality["coverage"] >= getattr(self, "object_depth_min_coverage", .15) and
            quality["mad"] <= getattr(self, "object_depth_max_mad", .12) and
            quality["inlier_ratio"] >= getattr(
                self, "object_depth_min_inlier_ratio", .60))
        if not quality_ok:
            detection["map_rejection"] = "unreliable registered depth"
        elif transform is None:
            detection["map_rejection"] = "image-time map transform unavailable"
        else:
            detection["map_point"] = transform_point(backproject(u, v, distance, k),
                                                      *transform)
    # Serialize commit with motion, shutdown, and pose-loss track clearing.
    with self.control_lock, self.data_lock, self.object_lock:
        self.object_processing = False
        now = time.time()
        same_motion = generation == self.motion_generation
        stationary_display_current = same_motion and not self.commanded_moving
        if display_only:
            for detection in detections:
                detection["map_point"] = None
            self.object_detections = detections if same_motion else []
            self.object_detections_stamp = now if same_motion else 0.0
            self.object_status = ("moving preview: %d detections; %.2f s inference" %
                                  (len(detections), inference_seconds))
            if same_motion:
                self.request_novel_object_verification_locked(
                    detections, frame, now)
            return
        ros_now = rospy.Time.now().to_sec()
        capture_ages = tuple(ros_now - s for s in stamps)
        if not self.object_commit_ready(now):
            for detection in detections:
                detection["map_point"] = None
            if not self.shutting_down:
                self.object_detections = (detections if stationary_display_current else [])
                self.object_detections_stamp = (now if stationary_display_current else 0.0)
            self.object_status = ("discarded observation after %.2f s: "
                                  "SLAM/map became unavailable" % inference_seconds)
            return
        if map_generation != getattr(self, "map_generation", 0):
            for detection in detections:
                detection["map_point"] = None
            self.object_detections = (detections if stationary_display_current else [])
            self.object_detections_stamp = (now if stationary_display_current else 0.0)
            self.object_status = ("discarded observation after %.2f s: map generation "
                                  "changed" % inference_seconds)
            return
        if (getattr(self, "object_focus_enabled", False) and
                generation != self.motion_generation):
            for detection in detections:
                detection["map_point"] = None
            self.object_detections = (detections if stationary_display_current else [])
            self.object_detections_stamp = (now if stationary_display_current else 0.0)
            self.object_status = "discarded observation after %.2f s: motion changed" % \
                inference_seconds
            return
        if any(not 0 <= age <= self.object_result_max_age for age in capture_ages):
            for detection in detections:
                detection["map_point"] = None
            self.object_detections = (detections if stationary_display_current else [])
            self.object_detections_stamp = (now if stationary_display_current else 0.0)
            self.object_status = ("%d detections shown; map observation discarded after "
                                  "%.2f s: RGB/depth age %.2f/%.2f s exceeds %.2f s" %
                                  (len(detections), inference_seconds,
                                   capture_ages[0], capture_ages[1],
                                   self.object_result_max_age))
            return
        mapped = 0
        updated_tracks = []
        seen_track_ids = []
        for mapped_detection in deduplicate_mapped_detections(detections):
            detection = mapped_detection["detection"]
            point = detection["map_point"]
            track = self.object_tracker.update(
                mapped_detection["map_label"], point[0], point[1],
                detection["confidence"], now, observation_id=observation_id,
                validation_group=validation_group,
                box=detection["box"], z=point[2])
            if track is not None:
                updated_tracks.append(track)
                seen_track_ids.append(track["id"])
                mapped += 1
        eligible_misses = []
        for track in self.object_tracker.tracks:
            detected, clear_view = landmark_visibility(
                track, detections, depth, k, transform,
                image_margin=getattr(self, "object_removal_image_margin", .10),
                occlusion_margin=getattr(
                    self, "object_removal_occlusion_margin", .15))
            if detected:
                seen_track_ids.append(track["id"])
                # A same-class box at the saved projection proves the
                # candidate is still visible even when depth quality is
                # insufficient to remap it. Refresh the pending age timer
                # as well as resetting observation-based absence.
                track["last_seen"] = now
            elif clear_view:
                eligible_misses.append(track["id"])
        removed_tracks = self.object_tracker.finish_observation(
            validation_group, observation_id, seen_track_ids,
            eligible_track_ids=eligible_misses,
            miss_limit=getattr(self, "object_removal_misses", 5))
        self.object_detections = (detections if stationary_display_current else [])
        self.object_detections_stamp = (now if stationary_display_current else 0.0)
        confirmed = len(self.object_tracker.snapshot(True, now))
        removal_text = (", %d stale removed" % len(removed_tracks)
                        if removed_tracks else "")
        rejection_counts = {}
        for detection in detections:
            rejection = detection.get("map_rejection")
            if rejection:
                rejection_counts[rejection] = rejection_counts.get(rejection, 0) + 1
        rejection_text = ("; not mapped: " + ", ".join(
            "%d %s" % (count, reason)
            for reason, count in sorted(rejection_counts.items()))
            if rejection_counts else "")
        self.object_status = ("%d detections, %d mapped, %d confirmed%s; "
                              "%.2f s inference%s" %
                              (len(detections), mapped, confirmed, removal_text,
                               inference_seconds, rejection_text))
        self.update_object_focus_locked(updated_tracks, now)

def update_object_focus_locked(self, updated_tracks, now):
    """Start or resolve autonomous object focus while object_lock is held."""
    if not getattr(self, "object_focus_enabled", False):
        return
    focus_id = getattr(self, "object_focus_id", None)
    if focus_id is None:
        if (getattr(self, "paused", True) or
                getattr(self, "phase", "") in ("go", "user_go", "goal_reached",
                                                "goal_stopped", "goal_cancelled")):
            return
        candidate = next((track for track in updated_tracks
                          if not track.get("confirmed", False)), None)
        if candidate is None:
            return
        minimum = int(self.object_tracker._policy(candidate["label"])["hits"])
        self.object_focus_id = candidate["id"]
        self.object_focus_label = candidate["label"]
        self.object_focus_deadline = now + getattr(self, "object_focus_seconds", 15.0)
        self.object_focus_misses = 0
        self.object_focus_next_capture = now
        self.path = []
        self.phase, self.phase_start = "object_focus", now
        self.web_msg = "Focusing on possible %s (observation %d/%d)" % (
            candidate["label"], candidate["count"], minimum)
        return

    target = next((track for track in self.object_tracker.tracks
                   if track["id"] == focus_id), None)
    if target is not None and target.get("confirmed", False):
        self.finish_object_focus_locked(True, now)
        return
    if not any(track["id"] == focus_id for track in updated_tracks):
        self.object_focus_misses = getattr(self, "object_focus_misses", 0) + 1
    else:
        self.object_focus_misses = 0
    if self.object_focus_misses >= getattr(self, "object_focus_max_misses", 2):
        self.finish_object_focus_locked(False, now, "not seen consistently")

def finish_object_focus_locked(self, confirmed, now, reason=""):
    """Finish focus and resume frontier planning; object_lock must be held."""
    focus_id = getattr(self, "object_focus_id", None)
    label = getattr(self, "object_focus_label", None) or "object"
    if not confirmed and focus_id is not None:
        self.object_tracker.discard(focus_id)
    self.object_focus_id = None
    self.object_focus_label = None
    self.object_focus_deadline = 0.0
    self.object_focus_misses = 0
    self.object_focus_next_capture = 0.0
    self.path = []
    self.phase, self.phase_start = "plan", now
    self.web_msg = ("Added %s to the map; continuing mapping" % label if confirmed else
                    "Excluded possible %s (%s); continuing mapping" %
                    (label, reason or "focus timed out"))


