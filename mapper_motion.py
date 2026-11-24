"""Motion commands, safety gating, recovery, and path control.

Functions are attached explicitly to RoomMapper so controller state remains owned by
the facade while this safety-critical domain can be reviewed separately.
"""

import math
import time

import numpy as np
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger

from mapper_constants import AVOID, FAULT, HOLD, SEARCH


def swept_circle_clearance(points, direction, radius):
    """Return travel before 2-D points enter a moving circular envelope."""
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return float("inf")
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("Swept-clearance points must be finite x/y pairs")
    if not math.isfinite(direction) or not math.isfinite(radius) or radius <= 0:
        raise ValueError("Swept-clearance direction/radius is invalid")
    forward = points[:, 0] * math.cos(direction) + points[:, 1] * math.sin(direction)
    lateral = -points[:, 0] * math.sin(direction) + points[:, 1] * math.cos(direction)
    intersects = (forward > 0) & (np.abs(lateral) < radius)
    if not intersects.any():
        return float("inf")
    lateral = lateral[intersects]
    boundary = np.sqrt(np.maximum(0.0, radius * radius - lateral * lateral))
    return float(np.min(forward[intersects] - boundary))


def invalidate_slam(self, reason):
    """Stop trusting the current frame and schedule one bounded map reset."""
    self.map_trusted = False
    self.automatic_map_reset_pending = True
    self.automatic_map_reset_reason = str(reason)
    self.automatic_map_reset_failure = ""
    self.clear_object_tracks()
    self.web_msg = "SLAM LOST: %s; automatic map reset scheduled" % reason
    rospy.logerr(self.web_msg)

def clear_object_tracks(self):
    """Discard landmarks tied to a SLAM frame that is no longer valid."""
    # The attribute guard also keeps the callback independently testable
    # with lightweight RoomMapper.__new__ fixtures.
    if not hasattr(self, "object_lock"):
        return
    with self.object_lock:
        self.object_tracker.clear()
        self.object_detections = []
        self.object_detections_stamp = 0.0
    self.map_generation = getattr(self, "map_generation", 0) + 1
    if hasattr(self, "novel_object_request"):
        self.novel_object_request = None
    if hasattr(self, "novel_object_verification"):
        self.novel_object_verification = None
    if hasattr(self, "object_queue_lock"):
        with self.object_queue_lock:
            self.object_pending_snapshots = []

def pause_for_turn_progress(self, reason):
    """Stop autonomy after ineffective turns without discarding a valid map."""
    self.pending_spin_yaw = None
    self.pending_spin_pose_count = None
    self.spin_stall_count = 0
    self.spin_stall_start_yaw = None
    message = "Autonomy paused: %s; map retained" % reason
    self.stop_manual(message)
    rospy.logerr(message)

def start_turn_progress_window(self, now=None, reset_stalls=False):
    """Snapshot fresh-pose evidence for a commanded autonomous turn."""
    now = time.time() if now is None else now
    with self.data_lock:
        pose = self.pose
        pose_count = getattr(self, "pose_update_count", 0)
    self.pending_spin_yaw = pose[2] if pose is not None else None
    self.pending_spin_pose_count = pose_count
    if reset_stalls:
        self.spin_stall_count = 0
        self.spin_stall_start_yaw = None

def measured_turn_progress(self, now=None):
    """Return heading change only when Hector supplied enough fresh poses.

    A delayed pose callback must not turn physical motion into a reported
    zero-progress burst. ``None`` means the controller needs more pose
    evidence; the normal SLAM freshness gate remains responsible for
    stopping motion if that evidence does not arrive.
    """
    now = time.time() if now is None else now
    start = getattr(self, "pending_spin_yaw", None)
    if start is None:
        return None
    with self.data_lock:
        pose = self.pose
        pose_count = getattr(self, "pose_update_count", 0)
        pose_time = getattr(self, "t_pose", 0.0)
    start_count = getattr(self, "pending_spin_pose_count", None)
    enough_samples = (start_count is None or pose_count - start_count >= 2)
    max_age = max(.25, getattr(self, "spin_burst", .5))
    fresh = pose_time > 0.0 and 0.0 <= now - pose_time <= max_age
    if pose is None or not enough_samples or not fresh:
        return None
    return abs(math.atan2(math.sin(pose[2] - start),
                          math.cos(pose[2] - start)))

def record_turn_progress(self, progress):
    """Return True only for a confirmed cumulative turn stall."""
    threshold = getattr(self, "min_spin_progress", math.radians(3.0))
    if progress >= threshold:
        self.spin_stall_count = 0
        self.spin_stall_start_yaw = None
        return False
    if getattr(self, "spin_stall_count", 0) == 0:
        self.spin_stall_start_yaw = getattr(self, "pending_spin_yaw", None)
    self.spin_stall_count = getattr(self, "spin_stall_count", 0) + 1
    limit = getattr(self, "max_spin_stalls", 3)
    if self.spin_stall_count < limit:
        return False
    with self.data_lock:
        pose = self.pose
    aggregate_start = getattr(self, "spin_stall_start_yaw", None)
    aggregate = (abs(math.atan2(math.sin(pose[2] - aggregate_start),
                                math.cos(pose[2] - aggregate_start)))
                 if pose is not None and aggregate_start is not None else 0.0)
    if aggregate >= threshold:
        self.spin_stall_count = 0
        self.spin_stall_start_yaw = None
        return False
    return True

def send_actual(self, cmd):
    if cmd == self.last_cmd:
        return
    if not self.drive:
        self.last_cmd = cmd
        rospy.loginfo("[dry run] would send: %s", cmd)
        return
    try:
        translation_scale = getattr(self, "translation_speed_percent", 100.0) / 100.0
        rotation_scale = getattr(self, "rotation_speed_percent", 100.0) / 100.0
        if cmd[0] == "stop":
            self.robot.traveling(gait=0)
        elif cmd[0] in ("forward", "back", "translate", "strafe_left",
                        "strafe_right"):
            direction = {"forward": 0.0, "back": math.pi,
                         "strafe_left": math.pi / 2.0,
                         "strafe_right": -math.pi / 2.0}.get(
                             cmd[0], cmd[1] if len(cmd) > 1 else 0.0)
            self.robot.traveling(gait=1, stride=self.stride * translation_scale,
                                 height=self.step_height,
                                 direction=direction, rotation=0.0, time=self.step_time,
                                 steps=0, interrupt=True, relative_height=False)
        elif cmd[0] == "spin":
            self.robot.traveling(gait=1, stride=self.spin_stride, height=self.step_height,
                                 direction=0.0,
                                 rotation=cmd[1] * self.spin_rate * rotation_scale,
                                 time=self.step_time, steps=0, interrupt=True,
                                 relative_height=False)
        self.last_cmd = cmd
    except Exception as exc:
        self.last_cmd = None
        rospy.logerr("traveling() failed: %s", exc)
        raise

def set_motion(self, moving):
    moving = bool(moving)
    if moving != self.commanded_moving:
        self.motion_generation = getattr(self, "motion_generation", 0) + 1
        self.commanded_moving = moving
        if moving and hasattr(self, "object_lock"):
            with self.object_lock:
                self.object_detections = []
                self.object_detections_stamp = 0.0
            # A partial stopped-endpoint burst is no longer relevant once
            # motion begins and must not block moving preview inference.
            self.object_endpoint_done = getattr(self, "object_endpoint_key", None)
        elif not moving and hasattr(self, "object_queue_lock"):
            with self.object_queue_lock:
                self.object_pending_snapshots = [
                    item for item in self.object_pending_snapshots
                    if not item.get("display_only", False)]
        self.motion_pub.publish(Bool(moving))

def send(self, cmd, interrupt_object=False):
    """Apply motion in short bursts, then stand still for clean SLAM scans.

    Default object inference is nonblocking because image/depth/TF are
    frozen at endpoint capture. The opt-in legacy focus mode may still hold
    autonomous motion while a visible candidate is being confirmed.
    """
    now = time.time()
    if not self.drive:
        self.send_actual(cmd)
        return
    if cmd[0] == "stop":
        was_moving = self.commanded_moving
        self.motion_until = 0.0
        self.continuous_translation_start = 0.0
        self.send_actual(cmd)
        self.set_motion(False)
        if was_moving:
            self.settle_until = time.time() + self.scan_settle
            if hasattr(self, "survey"):
                with self.data_lock:
                    self.survey.reset(self.settle_until)
        return
    if self.commanded_moving:
        if now >= self.motion_until:
            object_hold = (getattr(self, "object_focus_enabled", False) and
                           getattr(self, "object_processing", False) and
                           bool(getattr(self, "object_detections", [])) and
                           not interrupt_object)
            translation_start = getattr(self, "continuous_translation_start", now)
            continuous_limit = getattr(self, "max_continuous_translation", 3.0)
            can_continue = (cmd == self.last_cmd and cmd[0] in
                            ("forward", "back", "translate") and
                            not object_hold and
                            now - translation_start < continuous_limit)
            if can_continue:
                remaining = continuous_limit - (now - translation_start)
                self.motion_until = now + min(self.move_burst, remaining)
                return
            self.send_actual(("stop",))
            self.set_motion(False)
            self.continuous_translation_start = 0.0
            self.settle_until = time.time() + self.scan_settle
            if hasattr(self, "survey"):
                with self.data_lock:
                    self.survey.reset(self.settle_until)
        return
    if now < self.settle_until:
        return
    # A complete stationary survey is required before choosing a new
    # viewpoint, not between every short gait burst.  Hector now receives
    # raw scans continuously, so pose tracking does not depend on this
    # application-level survey gate.
    spin_start = getattr(self, "pending_spin_yaw", None)
    if spin_start is not None:
        progress = self.measured_turn_progress(now)
        if progress is None:
            self.web_msg = "Waiting for fresh Hector poses to verify turn progress"
            return
        if progress < getattr(self, "min_spin_progress", math.radians(3.0)):
            if self.record_turn_progress(progress):
                self.pause_for_turn_progress(
                    "%d turn bursts made less than %.1f deg cumulative progress" %
                    (getattr(self, "max_spin_stalls", 3),
                     math.degrees(self.min_spin_progress)))
            else:
                self.web_msg = ("Turn made %.1f deg; checking cumulative progress "
                                "(%d/%d)" %
                                (math.degrees(progress), self.spin_stall_count,
                                 getattr(self, "max_spin_stalls", 3)))
            self.pending_spin_yaw = None
            self.pending_spin_pose_count = None
            return
        self.record_turn_progress(progress)
        self.pending_spin_yaw = None
        self.pending_spin_pose_count = None
    # CPU inference is slow on this robot. Keep exploring through an
    # empty-scene inference; if that inference finds something, its boxes
    # remain visible and the next settled observation is allowed to finish
    # before motion resumes. This avoids multi-second pauses between every
    # gait burst while preserving stable depth/TF observations for objects.
    object_in_view = bool(getattr(self, "object_detections", []))
    if (getattr(self, "object_focus_enabled", False) and
            getattr(self, "object_processing", False) and object_in_view and
            not interrupt_object):
        return
    if cmd[0] == "spin":
        self.start_turn_progress_window(now, reset_stalls=False)
    self.send_actual(cmd)
    self.set_motion(True)
    self.continuous_translation_start = (now if cmd[0] in
                                         ("forward", "back", "translate") else 0.0)
    self.motion_until = now + (self.spin_burst if cmd[0] == "spin" else self.move_burst)

def send_manual(self, cmd):
    """Send an operator-held command without autonomous motion gates.

    Manual mode intentionally bypasses map trust, sensor freshness,
    collision checks, burst timing, settling, and turn-progress checks.
    The browser key-up/blur handling and server dead-man remain active so a
    lost browser event still issues a stop.
    """
    self.motion_until = 0.0
    self.continuous_translation_start = 0.0
    self.pending_spin_yaw = None
    self.pending_spin_pose_count = None
    self.spin_stall_count = 0
    self.spin_stall_start_yaw = None
    self.send_actual(cmd)
    self.set_motion(cmd[0] != "stop")

def send_path(self, cmd):
    """Follow a sensor-checked route continuously, monitoring turn progress."""
    now = time.time()
    if now < self.settle_until and not self.commanded_moving:
        return
    previous = self.last_cmd
    if (previous is not None and previous[0] == "spin" and
            self.commanded_moving and now >= self.motion_until):
        progress = self.measured_turn_progress(now)
        if progress is None:
            self.web_msg = "Waiting for fresh Hector poses to verify turn progress"
            self.motion_until = now + self.spin_burst
        elif self.record_turn_progress(progress):
            self.pause_for_turn_progress(
                "%d turn bursts made less than %.1f deg cumulative progress" %
                (self.max_spin_stalls, math.degrees(self.min_spin_progress)))
            return
        else:
            self.start_turn_progress_window(now, reset_stalls=False)
            self.motion_until = now + self.spin_burst
    if cmd != previous or not self.commanded_moving:
        if cmd[0] == "spin":
            self.start_turn_progress_window(now, reset_stalls=True)
        else:
            self.pending_spin_yaw = None
            self.pending_spin_pose_count = None
            self.spin_stall_count = 0
            self.spin_stall_start_yaw = None
        self.motion_until = now + self.spin_burst
        # A direction change invalidates any pending stationary observation.
        self.motion_generation = getattr(self, "motion_generation", 0) + 1
    self.send_actual(cmd)
    self.set_motion(True)

def set_state(self, state, reason=""):
    if reason:
        self.state_reason = reason
    if state != self.state:
        rospy.loginfo(">>> %s -> %s %s", self.state, state, reason)
        self.state, self.state_start = state, time.time()

def route_turn_direction(self, pose):
    """Return the turn sign toward the next planned waypoint, if any."""
    if pose is None or not getattr(self, "path", None):
        return None
    tolerance = getattr(self, "waypoint_tolerance", 0.12)
    target = next(((wx, wy) for wx, wy in self.path
                   if math.hypot(wx - pose[0], wy - pose[1]) > tolerance), None)
    if target is None:
        return None
    wx, wy = target
    error = math.atan2(math.sin(math.atan2(wy - pose[1], wx - pose[0]) - pose[2]),
                       math.cos(math.atan2(wy - pose[1], wx - pose[0]) - pose[2]))
    if abs(error) <= math.radians(5.0):
        return None
    return 1.0 if error > 0 else -1.0

def open_side(self, lidar, pose=None):
    """Choose a turn using the route first, then LiDAR and depth clearance."""
    preferred = self.route_turn_direction(pose)
    depth = getattr(self, "depth_sectors", {})
    left = min(lidar["left"], depth.get("left", float("inf")))
    right = min(lidar["right"], depth.get("right", float("inf")))
    rotation_clearance = getattr(self, "body_radius", 0.25) + 0.05
    if preferred == 1.0 and left >= rotation_clearance:
        return preferred
    if preferred == -1.0 and right >= rotation_clearance:
        return preferred
    return 1.0 if left >= right else -1.0

def call_reset_service(self):
    """Reset Hector (Trigger) or RTAB-Map (Empty), as configured."""
    rospy.wait_for_service(self.reset_service, timeout=5.0)
    service_type = Empty if self.reset_service_type == "empty" else Trigger
    result = rospy.ServiceProxy(self.reset_service, service_type)()
    if self.reset_service_type == "trigger":
        # The installed Hector callback resets the map but leaves the
        # Trigger response at its default values (verified in source).
        rospy.loginfo("SLAM reset service returned (success=%s, message=%r)",
                      result.success, result.message)
    else:
        rospy.loginfo("SLAM reset service %s completed", self.reset_service)
    return result

def begin_recovery(self, lidar, rotation_blocked, rear_clear, reason, now,
                   pose=None):
    """Choose a safe escape shared by autonomous mapping and web routes."""
    self.avoid_dir = self.open_side(lidar, pose)
    self.avoid_forward_start = None
    self.recovery_start = now
    self.recovery_back_start = None
    if not rotation_blocked:
        self.recovery_action = "spin"
        detail = "turning toward open side"
    elif rear_clear:
        self.recovery_action = "back"
        detail = "rotation constrained; reversing to create clearance"
    else:
        self.recovery_action = None
        return False
    self.set_state(AVOID, reason + "; " + detail)
    self.web_msg = "Blocked: %s; %s" % (reason, detail)
    return True

def recent_forward_retreat_is_clear(self, now, pose, reverse_travel):
    """Allow a short backtrack only over a just-observed forward footprint.

    Native depth cannot see behind the robot. A floor bump may nevertheless
    be escaped by reversing over space the body occupied immediately before
    the stop, provided the current rear LiDAR sweep is also clear. This proof
    is deliberately unavailable for depth dropout and expires before it can
    become general blind-reverse permission.
    """
    proof = getattr(self, "recent_forward_clearance", None)
    if proof is None or pose is None:
        return False
    proof_time, proof_pose = proof
    max_age = getattr(self, "recent_forward_clearance_max_age", 1.5)
    if now - proof_time < 0.0 or now - proof_time > max_age:
        return False
    distance = math.hypot(pose[0] - proof_pose[0], pose[1] - proof_pose[1])
    heading = abs(math.atan2(math.sin(pose[2] - proof_pose[2]),
                             math.cos(pose[2] - proof_pose[2])))
    # The proof is refreshed during clear forward travel, so only normal
    # between-callback motion should separate it from the detected bump.
    return distance <= max(0.12, reverse_travel) and heading <= math.radians(5.0)

def depth_recovery_rotation_is_clear(self, lidar, pose):
    """Require live LiDAR and fused known-free evidence for a sparse-depth turn."""
    radius = getattr(self, "body_radius", 0.25) + 0.05
    if (getattr(self, "spin_stride", 0.0) != 0.0 or pose is None or
            lidar.get("nearest", 0.0) < radius):
        return False
    check = getattr(self.explorer, "rotation_footprint_clear", None)
    if not callable(check):
        return False
    with self.map_lock:
        return bool(check(pose, radius))

def handle_sparse_depth_recovery(self, lidar, pose, now):
    """Collect bounded new depth views without translating on unknown depth."""
    clear = self.depth_recovery_rotation_is_clear(lidar, pose)
    max_turns = getattr(self, "depth_recovery_max_turns", 8)
    attempts = getattr(self, "depth_recovery_attempts", 0)

    if self.phase == "depth_recovery_turn":
        if not clear:
            self.send(("stop",))
            self.set_state(FAULT, "sparse depth; rotation footprint is not verified clear")
            self.web_msg = "Depth recovery stopped: rotation footprint is not verified clear"
            return
        was_moving = self.commanded_moving
        self.send(("spin", self.avoid_dir), interrupt_object=True)
        if was_moving and not self.commanded_moving:
            self.phase, self.phase_start = "depth_recovery_settle", now
            self.web_msg = "Depth recovery turn complete; settling before depth recheck"
        else:
            self.web_msg = "Depth recovery: zero-stride resurvey turn %d/%d" % (
                attempts, max_turns)
        return

    if self.phase != "depth_recovery_settle":
        self.send(("stop",))
        self.phase, self.phase_start = "depth_recovery_settle", now
        self.set_state(SEARCH, "sparse depth; preparing bounded resurvey")
        self.web_msg = "Depth invalid; settling before a verified resurvey turn"
        return

    self.send(("stop",))
    if not self.survey.ready(now):
        self.web_msg = "Depth invalid; waiting for settled LiDAR/map evidence"
        return
    if attempts >= max_turns:
        self.set_state(FAULT, "%d depth resurvey turns did not recover valid depth" % max_turns)
        self.web_msg = "%d bounded depth resurvey turns found no valid view; holding" % max_turns
        return
    if not clear:
        self.set_state(FAULT, "sparse depth; rotation footprint is not verified clear")
        self.web_msg = "Depth invalid; rotation footprint is not verified clear; holding"
        return

    self.depth_recovery_attempts = attempts + 1
    self.avoid_dir = self.open_side(lidar, pose)
    self.phase, self.phase_start = "depth_recovery_turn", now
    self.set_state(SEARCH, "bounded turn to recover valid depth")
    self.web_msg = "Depth recovery: zero-stride resurvey turn %d/%d" % (
        self.depth_recovery_attempts, max_turns)
    self.send(("spin", self.avoid_dir), interrupt_object=True)

def start_look(self, now):
    # A 360-degree LiDAR already surveys the room without rotating the body.
    self.phase, self.phase_start, self.turned, self.last_yaw = "survey", now, 0.0, None

def start_navigation(self, x, y, label="selected point", standoff=0.0,
                     replanning=False):
    """Plan a web-requested route while preserving all control safety gates."""
    # Stop the previous autonomous burst before spending time planning.
    self.send(("stop",))
    with self.data_lock:
        pose = self.pose
    if (pose is None or not self.map_trusted or not self.explorer.have_map):
        self.stop_manual("Navigation unavailable: waiting for a trusted map and pose")
        return False
    with self.map_lock:
        path, actual = self.explorer.plan_to_point(pose, (x, y), standoff)
    if not path or actual is None:
        self.path = []
        self.nav_goal = None
        if replanning:
            self.phase = "goal_stopped"
            self.stop_manual(
                "Clicked route paused: a new fused obstacle leaves no reachable "
                "known-free detour")
        else:
            self.stop_manual("Navigation unavailable: no reachable known-free route")
        return False
    self.manual_cmd, self.manual_until = None, 0.0
    self.paused = False
    self.path = path
    self.user_goal = (float(x), float(y), str(label), float(standoff))
    self.nav_goal = actual
    self.state, self.state_start = SEARCH, time.time()
    self.phase, self.phase_start = "user_go", time.time()
    adjustment = math.hypot(actual[0] - x, actual[1] - y)
    action = "Replanned route to" if replanning else "Navigating to"
    self.web_msg = ("%s %s via safe goal %.2f, %.2f m%s" %
                    (action, label, actual[0], actual[1],
                     " (adjusted %.2f m)" % adjustment if adjustment > .15 else ""))
    return True

def plan(self, pose, now):
    with self.map_lock:
        path, goal = self.explorer.plan_frontier(pose)
        raw_probe = getattr(self.explorer, "plan_frontier_slam_only", None)
        raw_path, _ = ((None, None) if path or not callable(raw_probe)
                       else raw_probe(pose))
    self.frontier_blocked_by_fused = bool(raw_path)
    if path:
        self.frontier_resurvey_attempts = 0
        self.path = path
        self.phase, self.phase_start = "go", now
        rospy.loginfo("%s goal (%.1f, %.1f), %d waypoints",
                      self.explorer.last_plan_kind or "mapping", goal[0], goal[1],
                      len(path))
        return True
    if raw_path:
        rospy.logwarn_throttle(
            5, "Reachable Hector frontier is blocked by retained fused-depth occupancy")
    return False

def follow_path(self, pose):
    x, y, yaw = pose
    while (self.path and math.hypot(self.path[0][0] - x,
                                    self.path[0][1] - y) < self.waypoint_tolerance):
        self.path.pop(0)
    if not self.path:
        return None
    wx, wy = self.path[0]
    err = math.atan2(math.sin(math.atan2(wy-y, wx-x) - yaw),
                     math.cos(math.atan2(wy-y, wx-x) - yaw))
    max_crab = getattr(self, "max_crab_angle", math.radians(45.0))
    if abs(err) > max_crab:
        return "spin", 1.0 if err > 0 else -1.0
    deadband = getattr(self, "crab_deadband", math.radians(5.0))
    if abs(err) <= deadband:
        return ("forward",)
    step = getattr(self, "crab_angle_step", math.radians(5.0))
    direction = max(-max_crab, min(max_crab, round(err / step) * step))
    return "translate", direction

def command_lidar_clearance(lidar, cmd):
    """Return swept-body LiDAR clearance along a translation command."""
    if cmd is None or cmd[0] not in ("forward", "back", "translate",
                                     "strafe_left", "strafe_right"):
        return float("inf")
    direction = {"forward": 0.0, "back": math.pi,
                 "strafe_left": math.pi / 2.0,
                 "strafe_right": -math.pi / 2.0}.get(
                     cmd[0], cmd[1] if len(cmd) > 1 else 0.0)
    heads = np.asarray(lidar.get("heads", ()), dtype=np.float64)
    free = np.asarray(lidar.get("free", ()), dtype=np.float64)
    if not len(heads) or len(heads) != len(free):
        return float("inf")
    error = np.abs(np.arctan2(np.sin(heads - direction),
                              np.cos(heads - direction)))
    return float(free[int(np.argmin(error))])

def safe_step(self, _event):
    with self.control_lock:
        if self.shutting_down:
            return
        try:
            self.update_pose_from_tf()
            self.control_step()
        except Exception as exc:
            try:
                self.send(("stop",))
            except Exception:
                pass  # Keep the timer alive to retry the stop next cycle.
            rospy.logerr_throttle(2, "Controller error; stop requested: %s", exc)

def control_step(self):
    now = time.time()
    with self.data_lock:
        pose, lidar = self.pose, getattr(self, "lidar", None)
        depth = self.depth
        t_pose, t_scan, t_camera, t_depth = self.t_pose, self.t_scan, self.t_camera, self.t_depth
        registered_depth = getattr(self, "registered_depth", None)
        t_registered_depth = getattr(self, "t_registered_depth", 0.0)
    # Web safety controls must still work while a sensor or map is faulty.
    self.handle_web()
    if getattr(self, "automatic_map_reset_pending", False):
        reason = getattr(self, "automatic_map_reset_reason", "pose jump")
        # Clear the request before calling the service. A failed service
        # call leaves the map untrusted and stopped instead of creating an
        # unbounded reset loop in the controller timer.
        self.automatic_map_reset_pending = False
        self.reset_mapping(automatic=True, reason=reason)
        return
    if getattr(self, "commanded_moving", False):
        self.capture_motion_preview_snapshot(now)
    if getattr(self, "novel_object_request", None) is not None:
        if self.begin_novel_object_verification(now):
            return
    manual_cmd = getattr(self, "manual_cmd", None)
    if manual_cmd is not None and now > getattr(self, "manual_until", 0.0):
        self.stop_manual("Manual drive stopped: browser command timed out")
        return
    if manual_cmd is not None:
        command = ({"forward": ("forward",), "back": ("back",),
                    "strafe_left": ("strafe_left",),
                    "strafe_right": ("strafe_right",),
                    "left": ("spin", 1.0), "right": ("spin", -1.0)}
                   [manual_cmd])
        self.web_msg = ("Manual override: hold %s; map and obstacle gates bypassed; "
                        "release to stop" % manual_cmd)
        self.send_manual(command)
        return
    if self.paused and manual_cmd is None:
        self.send(("stop",))
        if (getattr(self, "detector", None) is not None and self.survey.ready(now) and
                self.objects_ready(now)):
            if self.capture_endpoint_snapshot(now):
                self.capture_preview_snapshot(now)
        return
    # The fused route planner must not silently continue after its
    # registered geometry source dies. Native depth remains the immediate
    # collision sensor, but autonomous mapping stops until aligned planner
    # depth recovers. Manual dead-man control keeps its existing behavior.
    if (getattr(self, "use_depth_map", False) and
            (registered_depth is None or now - t_registered_depth > 1.0)):
        self.send(("stop",))
        self.set_state(FAULT, "waiting for registered planner depth")
        return
    # RGB is used for display and YOLO, not collision sensing. A stalled
    # RGB stream must not disable depth/LiDAR-protected manual navigation.
    if (now - t_scan > 0.6 or lidar is None or
            (self.use_depth and (depth is None or now - t_depth > 1.0))):
        self.send(("stop",))
        missing = []
        if lidar is None or now - t_scan > 0.6: missing.append("LiDAR")
        if self.use_depth and (depth is None or now - t_depth > 1.0): missing.append("depth")
        self.set_state(FAULT, "waiting for " + ", ".join(missing))
        return
    mapping_ok = (self.use_map and pose is not None and now - t_pose <= self.slam_stale
                  and self.explorer.have_map and self.map_is_fresh(now)
                  and self.map_trusted)
    if self.use_map and not mapping_ok:
        self.send(("stop",))
        failure = getattr(self, "automatic_map_reset_failure", "")
        reason = (("SLAM map invalid; automatic reset failed: %s" % failure)
                  if not self.map_trusted and failure else
                  "SLAM map invalid; no automatic reset is pending"
                  if not self.map_trusted else
                  "waiting for fresh SLAM pose and map")
        self.set_state(FAULT, reason)
        return
    if self.drive and now < self.pose_guard_start:
        self.send(("stop",))
        self.set_state(FAULT, "waiting for startup pose guard")
        return
    if self.use_depth and self.floor_profile is None:
        self.send(("stop",))
        rospy.loginfo_throttle(2, "Learning depth-camera floor - keep the robot still...")
        return
    front = min(lidar["front"], lidar["free"][len(lidar["free"]) // 2])
    with self.data_lock:
        depth_near = self.depth_ahead(depth) if self.use_depth else float("inf")
    # Body tilt during an in-place turn can briefly make the learned floor
    # look like a bump/dropout. A zero-stride turn cannot drive forward
    # into it, and LiDAR still protects the rotating footprint.
    turning = (self.commanded_moving and self.last_cmd is not None and
               self.last_cmd[0] == "spin" and self.spin_stride == 0.0)
    if turning and self.depth_source in ("floor-bump", "depth dropout / too close"):
        depth_near = float("inf")
        self.depth_source = "floor ignored while turning"
    self.depth_history = (self.depth_history + [depth_near])[-5:]
    # React to a newly close depth return immediately; use the history only
    # to avoid clearing an established obstacle on one noisy frame.
    depth_near = min(depth_near, float(np.median(self.depth_history)))
    if (self.use_depth and
            self.depth_health == "insufficient valid depth in upper band"):
        self.handle_sparse_depth_recovery(lidar, pose, now)
        return
    if self.use_depth and not self.depth_health.startswith("ready"):
        self.send(("stop",))
        self.set_state(FAULT, self.depth_health)
        return
    if self.phase in ("depth_recovery_turn", "depth_recovery_settle"):
        self.send(("stop",))
        self.depth_recovery_attempts = 0
        self.set_state(SEARCH, "depth recovered after bounded resurvey")
        self.web_msg = "Depth recovered; settling and replanning"
        self.start_look(now)
        return
    if self.state == FAULT:
        self.set_state(SEARCH, "sensors recovered")
        self.start_look(now)
    if self.phase == "object_focus":
        self.send(("stop",))
        if (now >= getattr(self, "object_focus_next_capture", 0.0) and
                self.objects_ready(now)):
            snapshot = self.capture_object_snapshot()
            if snapshot is not None:
                self.enqueue_object_snapshot(snapshot)
            self.object_focus_next_capture = now + 1.0 / max(0.1, self.object_hz)
        if now >= getattr(self, "object_focus_deadline", 0.0):
            with self.object_lock:
                self.finish_object_focus_locked(False, now, "focus timed out")
        else:
            self.set_state(SEARCH, "holding still to verify detected object")
        return
    # Depth is the primary forward obstacle signal and gets the longer
    # stopping distance. LiDAR remains an independent close-range fallback
    # because SLAM already requires it and the camera has a limited FOV.
    depth_limit = self.depth_stop
    depth_blocked = depth_near < depth_limit
    lidar_emergency = front < self.lidar_stop
    blocked = depth_blocked or lidar_emergency
    # Forward clearance and in-place rotation are different geometries.
    # A zero-stride turn may safely look for an open route when an object
    # blocks translation, provided nothing intrudes into the circular
    # swept footprint of the body and legs.
    rotation_clearance = self.body_radius + 0.05
    rotation_blocked = (self.spin_stride != 0.0 or
                        lidar["nearest"] < rotation_clearance or
                        depth_near < rotation_clearance)
    rear = min(lidar.get("back", float("inf")),
               lidar.get("near_back", float("inf")))
    rear_path_clearance = lidar.get(
        "rear_path_clearance", rear - self.lidar_stop)
    speed_scale = getattr(self, "translation_speed_percent", 100.0) / 100.0
    reverse_travel = (getattr(self, "stride", 40.0) / 1000.0 * speed_scale *
                      self.reverse_seconds / max(getattr(self, "step_time", 0.5), 0.1))
    rear_clearance = reverse_travel + getattr(self, "reverse_clearance_margin", 0.02)
    if (not blocked and self.commanded_moving and self.last_cmd is not None and
            self.last_cmd[0] == "forward"):
        self.recent_forward_clearance = (now, tuple(pose))
    recent_retreat_clear = (
        self.depth_source == "floor-bump" and
        self.recent_forward_retreat_is_clear(now, pose, reverse_travel))
    # Rearward travel has LiDAR coverage but no rear-facing depth. A dropout
    # always vetoes reverse. A floor bump permits only an immediate, bounded
    # retreat over the forward footprint the body just occupied.
    rear_clear = (rear_path_clearance >= rear_clearance and
                  self.depth_source != "depth dropout / too close" and
                  (self.depth_source != "floor-bump" or recent_retreat_clear))
    reason = ("depth %.2f m (%s)" % (depth_near, self.depth_source)
              if depth_blocked else "LiDAR fallback %.2f m" % front)
    self.view = {"front": front, "nearest": lidar["nearest"], "blocked": blocked,
                 "depth": depth_near, "depth_limit": depth_limit, "depth_source": self.depth_source,
                 "depth_primary": True, "lidar_emergency": lidar_emergency}

    if self.phase == "novel_object_verify":
        self.send(("stop",))
        self.capture_novel_object_snapshot(now)
        if self.phase == "novel_object_verify":
            verification = self.novel_object_verification
            self.set_state(
                SEARCH, "holding for close-object verification %d/%d captured, %d processed" %
                (verification["captured"], self.novel_object_frames,
                 verification["processed"]))
        return

    if self.state == HOLD:
        self.send(("stop",))
        if not blocked:
            self.set_state(SEARCH, "path cleared")
        elif self.begin_recovery(lidar, rotation_blocked, rear_clear,
                                 "obstacle remains", now, pose):
            pass
        elif now - self.state_start > self.hold_seconds:
            if rear_path_clearance < rear_clearance:
                detail = "rear swept travel %.2f m is below %.2f m required" % (
                    rear_path_clearance, rear_clearance)
            elif self.depth_source == "depth dropout / too close":
                detail = "depth dropout prohibits blind reverse"
            elif self.depth_source == "floor-bump" and not recent_retreat_clear:
                detail = "floor bump has no fresh forward-footprint retreat proof"
            else:
                detail = "no verified recovery route"
            self.web_msg = "Blocked safely: rotation footprint occupied and " + detail
        return
    if self.state == AVOID:
        if self.recovery_action == "back":
            if not rear_clear:
                self.send(("stop",))
                self.set_state(HOLD, "rear clearance lost while reversing")
            elif getattr(self, "recovery_back_start", None) is None:
                self.send(("back",), interrupt_object=True)
                if (self.commanded_moving and self.last_cmd is not None and
                        self.last_cmd[0] == "back"):
                    self.recovery_back_start = now
                    self.web_msg = (
                        "Recovery: reversing briefly to create turning clearance")
                else:
                    self.web_msg = "Recovery: waiting to begin verified reverse"
            elif now - self.recovery_back_start < self.reverse_seconds:
                self.web_msg = "Recovery: reversing briefly to create turning clearance"
                self.send(("back",), interrupt_object=True)
            else:
                self.send(("stop",))
                self.recovery_action = None
                self.recovery_back_start = None
                self.set_state(HOLD, "reverse complete; reassessing clearance")
            return
        if rotation_blocked:
            if rear_clear:
                self.recovery_action = "back"
                self.recovery_back_start = None
                self.send(("back",), interrupt_object=True)
                if (self.commanded_moving and self.last_cmd is not None and
                        self.last_cmd[0] == "back"):
                    self.recovery_back_start = now
                self.web_msg = "Recovery: turn constrained; switching to reverse"
            else:
                self.send(("stop",))
                self.set_state(HOLD, "rotation and reverse paths occupied")
            return
        if (blocked and rear_clear and self.recovery_action == "spin" and
                now - self.recovery_start >= getattr(
                    self, "reverse_after_turn_seconds", 2.5)):
            self.recovery_action = "back"
            self.recovery_back_start = None
            self.web_msg = ("Recovery: front remained blocked after turning; "
                            "reversing to escape corner")
            self.send(("back",), interrupt_object=True)
            if (self.commanded_moving and self.last_cmd is not None and
                    self.last_cmd[0] == "back"):
                self.recovery_back_start = now
            return
        if (blocked and self.commanded_moving and self.last_cmd is not None and
                self.last_cmd[0] == "forward"):
            self.send(("stop",))
            return
        if not blocked and lidar["free"][len(lidar["free"]) // 2] > self.lidar_stop + 0.3:
            if getattr(self, "avoid_forward_start", None) is None:
                self.avoid_forward_start = now
            self.send(("forward",), interrupt_object=True)
            if now - self.avoid_forward_start > self.avoid_seconds:
                if self.phase == "user_go" and self.user_goal is not None:
                    x, y, label, standoff = self.user_goal
                    self.start_navigation(x, y, label, standoff)
                else:
                    self.set_state(SEARCH, "cleared obstacle")
                    self.start_look(now)
        else:
            self.avoid_forward_start = None
            self.send(("spin", self.avoid_dir), interrupt_object=True)
        return
    if blocked:
        # Terminate an existing translation before selecting an escape.
        # The subsequent turn/reverse command will begin after scan_settle.
        self.send(("stop",))
        if self.phase == "user_go":
            if self.begin_recovery(lidar, rotation_blocked, rear_clear, reason, now,
                                   pose):
                command = (("back",) if self.recovery_action == "back" else
                           ("spin", self.avoid_dir))
                self.send(command, interrupt_object=True)
            else:
                self.send(("stop",))
                self.set_state(HOLD, reason + "; no safe rotation or reverse route")
            return
        self.path = []
        self.phase = "plan"
        if not self.begin_recovery(lidar, rotation_blocked, rear_clear, reason, now,
                                   pose):
            self.send(("stop",))
            self.set_state(HOLD, reason + "; no safe rotation or reverse route")
        else:
            command = (("back",) if self.recovery_action == "back" else
                       ("spin", self.avoid_dir))
            self.send(command, interrupt_object=True)
        return

    if not mapping_ok:
        self.phase = "wander"
        if now - self.phase_start < self.look_seconds:
            self.send(("spin", 1.0))
        else:
            self.send(("forward",))
            if now - self.phase_start > self.look_seconds + 5:
                self.start_look(now)
        return

    if self.phase in ("look", "survey"):
        self.send(("stop",))
        if (self.survey.ready(now) and
                self.capture_endpoint_snapshot(now)):
            self.phase = "plan"
    elif self.phase == "plan":
        self.send(("stop",))
        if not self.plan(pose, now):
            attempts = getattr(self, "frontier_resurvey_attempts", 0)
            max_turns = getattr(self, "frontier_resurvey_max_turns", 8)
            fused_blocked = getattr(self, "frontier_blocked_by_fused", False)
            if attempts < max_turns and not rotation_blocked:
                self.frontier_resurvey_attempts = attempts + 1
                self.avoid_dir = self.open_side(lidar, pose)
                self.phase, self.phase_start = "frontier_resurvey_turn", now
                cause = ("Hector frontier blocked by retained depth cells"
                         if fused_blocked else
                         "No reachable route in the current map")
                self.web_msg = ("%s; safe in-place resurvey turn %d/%d" %
                                (cause, self.frontier_resurvey_attempts,
                                 max_turns))
                self.send(("spin", self.avoid_dir), interrupt_object=True)
            else:
                if fused_blocked:
                    reason = ("rotation footprint is not clear" if rotation_blocked else
                              "%d bounded resurvey turns found no safe route" % max_turns)
                    self.web_msg = (
                        "Hector frontiers remain blocked by fused obstacles; %s; "
                        "holding position" % reason)
                else:
                    reason = ("rotation footprint is not clear" if rotation_blocked else
                              "%d bounded resurvey turns found no route" % max_turns)
                    self.web_msg = ("No reachable frontier; %s; holding position" %
                                    reason)
                self.start_look(now)
    elif self.phase == "frontier_resurvey_turn":
        was_moving = self.commanded_moving
        self.send(("spin", self.avoid_dir), interrupt_object=True)
        if was_moving and not self.commanded_moving:
            self.phase, self.phase_start = "frontier_resurvey_settle", now
            self.web_msg = "Depth resurvey turn complete; collecting settled clear evidence"
    elif self.phase == "frontier_resurvey_settle":
        self.send(("stop",))
        if self.survey.ready(now):
            self.phase, self.phase_start = "plan", now
    elif self.phase in ("go", "user_go"):
        cmd = self.follow_path(pose)
        route_clearance = self.command_lidar_clearance(lidar, cmd)
        if hasattr(self, 'map_lock'):
            with self.map_lock:
                path_check = getattr(self.explorer, 'path_prefix_clear', None)
                fused_path_clear = (path_check(pose, self.path)
                                    if callable(path_check) else True)
        else:
            path_check = getattr(self.explorer, 'path_prefix_clear', None)
            fused_path_clear = (path_check(pose, self.path)
                                if callable(path_check) else True)
        if cmd is None:
            if self.phase == "user_go":
                label = self.user_goal[2] if self.user_goal else "selected goal"
                self.send(("stop",))
                self.paused = True
                self.path = []
                self.nav_goal = None
                self.phase = "goal_reached"
                self.web_msg = "Reached %s; autonomy paused" % label
            else:
                self.start_look(now)
        elif blocked:
            self.path = []
            self.set_state(HOLD, "planned path blocked (%s)" % reason)
            self.phase = "plan"
        elif not fused_path_clear:
            self.send(("stop",))
            if self.phase == "user_go" and self.user_goal is not None:
                x, y, label, standoff = self.user_goal
                self.start_navigation(x, y, label, standoff, replanning=True)
            else:
                self.path = []
                self.phase = "plan"
                self.set_state(HOLD, "planned path invalidated by fused obstacle map")
                self.web_msg = "Route replanning: newly mapped obstacle blocks the path"
        elif route_clearance < self.lidar_stop:
            self.send(("stop",))
            route_reason = "planned travel direction blocked by LiDAR %.2f m" % \
                route_clearance
            if self.phase == "user_go":
                if self.begin_recovery(lidar, rotation_blocked, rear_clear,
                                       route_reason, now, pose):
                    command = (("back",) if self.recovery_action == "back" else
                               ("spin", self.avoid_dir))
                    self.send(command, interrupt_object=True)
                else:
                    self.set_state(HOLD, route_reason +
                                   "; no safe rotation or reverse route")
                    self.stop_manual(
                        "Clicked route paused: directional LiDAR blockage has no "
                        "safe rotation or reverse recovery")
            else:
                self.path = []
                self.phase = "plan"
                self.set_state(HOLD, route_reason)
        elif now - self.phase_start > 45:
            if self.phase == "user_go":
                self.send(("stop",))
                self.paused = True
                self.path = []
                self.nav_goal = None
                self.phase = "goal_stopped"
                self.web_msg = "Goal navigation timed out; robot paused"
            else:
                self.phase = "plan"
        else:
            self.send_path(cmd)
    else:  # fallback wander
        self.send(("forward",))
        if now - self.phase_start > 5:
            self.start_look(now)

