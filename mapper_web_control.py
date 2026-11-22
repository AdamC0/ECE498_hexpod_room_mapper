"""Web commands and operator-facing runtime status for RoomMapper."""

import time

import rospy

from controller_status import diagnose_runtime, format_pose
from mapper_constants import SEARCH


def web_request(self, command):
    self.web_pending.append(command)

def stop_manual(self, message="Autonomy paused; manual drive available"):
    self.manual_cmd = None
    self.manual_until = 0.0
    self.paused = True
    self.novel_object_request = None
    self.novel_object_verification = None
    if self.phase == "novel_object_verify":
        self.phase, self.phase_start = "plan", time.time()
    self.send(("stop",))
    self.web_msg = message

def cancel_navigation(self):
    """Immediately stop and discard any web-selected route."""
    self.manual_cmd = None
    self.manual_until = 0.0
    self.paused = True
    self.path = []
    self.user_goal = None
    self.nav_goal = None
    self.novel_object_request = None
    self.novel_object_verification = None
    if self.phase == "user_go":
        self.phase = "goal_cancelled"
    self.send(("stop",))
    self.web_msg = "Route cancelled; autonomy paused"

def reset_mapping(self, automatic=False, reason=""):
    """Stop, clear Hector/local state, and begin a fresh stationary survey.

    A web-requested clear always pauses. An automatic pose-jump recovery
    remembers whether autonomy was active and restores that state only
    after resetting local state; the normal pose/map guards still prevent
    movement until the new SLAM frame is ready.
    """
    resume_autonomy = bool(automatic and not self.paused)
    self.manual_cmd = None
    self.manual_until = 0.0
    self.paused = True
    self.send(("stop",))
    try:
        self.call_reset_service()
    except Exception as exc:
        prefix = "Automatic map reset failed" if automatic else "CLEAR MAP failed"
        if automatic:
            self.automatic_map_reset_failure = str(exc)
        self.web_msg = "%s; robot remains paused: %s" % (prefix, exc)
        rospy.logerr(self.web_msg)
        return False
    now = time.time()
    with self.map_lock:
        self.explorer.reset()
        self.t_map = 0.0
        self.t_map_heartbeat = 0.0
    with self.data_lock:
        pose = self.pose
        self.settle_until = now + self.scan_settle
        self.survey.reset(self.settle_until)
        self.prev_pose = None
        self.pose_jump_candidate = None
        self.pose_jump_candidate_count = 0
        self.pose_guard_start = now + 5.0
        self.map_trusted = True
        self.automatic_map_reset_pending = False
        self.automatic_map_reset_reason = ""
        self.automatic_map_reset_failure = ""
        self.pending_spin_yaw = None
        self.pending_spin_pose_count = None
        self.spin_stall_count = 0
        self.spin_stall_start_yaw = None
        self.frontier_blocked_by_fused = False
        self.frontier_resurvey_attempts = 0
        # Any inference captured against the old map must fail its
        # generation check rather than repopulate cleared landmarks.
        self.motion_generation += 1
    self.path = []
    self.user_goal = None
    self.nav_goal = None
    self.object_focus_id = None
    self.object_focus_label = None
    self.object_focus_deadline = 0.0
    self.object_focus_misses = 0
    self.object_focus_next_capture = 0.0
    self.phase, self.phase_start = "survey", now
    self.state, self.state_start = SEARCH, now
    self.state_reason = "fresh map survey"
    # The old pose belongs to the discarded frame. Let the first pose in
    # the new frame establish the automatic reset's viewport center.
    self.web_map_center = None if automatic else (pose[:2] if pose else None)
    self.clear_object_tracks()
    self.paused = not resume_autonomy
    if automatic:
        mode = "autonomy will resume after fresh-map checks" if resume_autonomy \
            else "remaining paused"
        self.web_msg = ("SLAM pose jump recovered: map and landmarks reset (%s); %s" %
                        (reason or "threshold exceeded", mode))
        rospy.logwarn(self.web_msg)
    else:
        self.web_msg = "Map and object landmarks cleared; PAUSED for a fresh survey"
    return True

def handle_web(self):
    commands, self.web_pending = self.web_pending, []
    for cmd in commands:
        if cmd[0] == "pause":
            self.stop_manual("AUTONOMY PAUSED - manual drive remains available")
        elif cmd[0] == "cancel_route":
            self.cancel_navigation()
        elif cmd[0] == "resume":
            self.manual_cmd, self.manual_until = None, 0.0
            if not getattr(self, 'map_trusted', True):
                self.paused = True
                self.send(("stop",))
                self.web_msg = ("Cannot resume: SLAM map is invalid; use CLEAR MAP "
                                "to start a fresh frame")
            else:
                if self.phase in ("goal_reached", "goal_stopped", "goal_cancelled"):
                    self.phase = "plan"
                self.paused, self.web_msg = False, "Resumed"
        elif cmd[0] == "clear_map":
            self.reset_mapping()
        elif cmd[0] == "manual" and len(cmd) == 2:
            action = cmd[1]
            if action == "stop":
                self.stop_manual()
            elif action in ("forward", "back", "strafe_left", "strafe_right",
                            "left", "right"):
                if not self.drive:
                    self.stop_manual("Manual drive unavailable in dry-run mode")
                else:
                    # Manual motion always suspends autonomous exploration.
                    # The browser refreshes this deadline while held.
                    self.paused = True
                    self.manual_cmd = action
                    self.manual_until = time.time() + self.manual_deadman
                    self.web_msg = "Manual drive armed: " + action
        elif cmd[0] == "speed" and len(cmd) == 3:
            translation, rotation = float(cmd[1]), float(cmd[2])
            if (self.speed_min_percent <= translation <= self.speed_max_percent and
                    self.speed_min_percent <= rotation <= self.speed_max_percent):
                self.translation_speed_percent = translation
                self.rotation_speed_percent = rotation
                # Force the gait command to be refreshed if a slider changes
                # while a manual button is held.
                self.last_cmd = None
                self.web_msg = "Speed set: translation %.0f%%, rotation %.0f%%" % (
                    translation, rotation)
        elif cmd[0] == "navigate" and len(cmd) == 3:
            self.start_navigation(cmd[1], cmd[2])
        elif cmd[0] == "navigate_object" and len(cmd) == 2:
            with self.object_lock:
                objects = self.object_tracker.snapshot(False)
            target = next((item for item in objects if item["id"] == cmd[1]), None)
            if target is None:
                self.stop_manual("Object navigation failed: object no longer exists")
            elif not target.get("confirmed", False):
                self.stop_manual("Object navigation blocked: landmark is still pending")
            else:
                self.start_navigation(target["x"], target["y"],
                                      target.get("name", target["label"]),
                                      self.object_standoff)
        elif cmd[0] == "rename_object" and len(cmd) == 3:
            with self.object_lock:
                changed = self.object_tracker.rename(cmd[1], cmd[2])
            self.web_msg = ("Object renamed to " + cmd[2] if changed else
                            "Rename failed: object no longer exists or name is invalid")
        elif cmd[0] == "remove_object" and len(cmd) == 2:
            with self.object_lock:
                removed = self.object_tracker.remove(cmd[1])
            self.web_msg = ("Object removed from this map" if removed else
                            "Remove failed: object no longer exists")

def web_status(self):
    # Status is served from an HTTP worker thread and must derive its own
    # snapshot.  These values used to be calculated only by reset_mapping,
    # which made /status fail before the first web reset.
    with self.data_lock:
        pose, lidar = self.pose, getattr(self, "lidar", None)
    front = self.view.get("front", float("inf"))
    if lidar is not None:
        front = min(lidar["front"], lidar["free"][len(lidar["free"]) // 2])
    depth = self.view.get("depth", 99.0)
    depth_line = "Depth: %.2f m (%s), limit %.2f" % (
        depth, self.view.get("depth_source", "clear"), self.view.get("depth_limit", self.depth_stop)) if depth < 99 else "Depth: clear"
    if not self.use_depth:
        depth_line = "Depth: disabled"
    elif time.time() - self.t_depth > 1.0:
        depth_line = "Depth: waiting / stale"
    elif self.floor_profile is None:
        depth_line = "Depth: learning floor (%d/%d frames); %s" % (
            self.floor_frames, self.floor_frames_needed, self.depth_health)
    elif self.depth_health != "ready":
        depth_line = "Depth: " + self.depth_health
    survey_line = "Stationary survey: %d/%d scans, %s" % (
        min(self.survey.count, self.survey.required), self.survey.required,
        "localized" if self.survey.ready(time.time()) else "waiting for settled scans / SLAM")
    now = time.time()
    pose_line = format_pose(pose)
    manual_cmd = getattr(self, "manual_cmd", None)
    mode = "  MANUAL: %s" % manual_cmd.upper() if manual_cmd else ("  AUTO PAUSED" if self.paused else "")
    depth_map_line = ("Depth planner layer: " + self.depth_map_status
                      if self.use_depth_map else "Depth planner layer: disabled")
    snapshot = {
        "state": self.state,
        "phase": self.phase,
        "state_reason": getattr(self, "state_reason", ""),
        "web_message": self.web_msg,
        "paused": self.paused,
        "manual_cmd": manual_cmd,
        "commanded_moving": getattr(self, "commanded_moving", False),
        "last_cmd": self.last_cmd,
        "drive": self.drive,
        "use_map": self.use_map,
        "use_depth": self.use_depth,
        "use_depth_map": self.use_depth_map,
        "scan_stamp": self.t_scan,
        "depth_stamp": self.t_depth,
        "registered_depth_stamp": getattr(self, "t_registered_depth", 0.0),
        "pose_stamp": self.t_pose,
        "pose_available": pose is not None,
        "slam_stale": self.slam_stale,
        "map_trusted": self.map_trusted,
        "have_map": self.explorer.have_map,
        "map_fresh": self.map_is_fresh(now),
        "automatic_reset_pending": getattr(self, "automatic_map_reset_pending", False),
        "automatic_reset_reason": getattr(self, "automatic_map_reset_reason", ""),
        "automatic_reset_failure": getattr(self, "automatic_map_reset_failure", ""),
        "pose_guard_start": self.pose_guard_start,
        "floor_ready": self.floor_profile is not None,
        "depth_health": self.depth_health,
        "path_length": len(self.path),
    }
    diagnosis = diagnose_runtime(snapshot, now)
    command_name = self.last_cmd[0] if self.last_cmd else "none"
    return ("State: %s/%s%s\nCommand: %s\n%s\n%s\n"
            "Speed: translation %.0f%%, rotation %.0f%%\n"
            "%s\n%s\nLiDAR: %.2f m (emergency stop at %.2f)\n"
            "Objects: %s\n%s\nLast action: %s" % (
                self.state, self.phase, mode, command_name,
                diagnosis.line(), pose_line,
                self.translation_speed_percent, self.rotation_speed_percent,
                depth_line, depth_map_line, front, self.lidar_stop,
                self.object_status, survey_line, self.web_msg))

def web_battery(self):
    with self.data_lock:
        voltage = self.battery_voltage
        percentage = self.battery_percentage
        charging = self.battery_charging
        stamp = self.battery_stamp
        source = self.battery_source
    age = time.time() - stamp if stamp else None
    return {
        "available": bool(stamp and age <= 10.0),
        "percentage": percentage,
        "voltage": voltage,
        "charging": charging,
        "age": age,
        "source": source,
    }

def web_speed(self):
    return {
        "translation": self.translation_speed_percent,
        "rotation": self.rotation_speed_percent,
        "minimum": self.speed_min_percent,
        "maximum": self.speed_max_percent,
    }


