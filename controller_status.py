"""Operator-facing controller status without ROS or web-server dependencies."""

import math


class RuntimeStatus(object):
    """A stable, small status contract for the dashboard and tests."""

    def __init__(self, code, summary, moving=False):
        self.code = str(code)
        self.summary = str(summary)
        self.moving = bool(moving)

    def line(self):
        label = "Current action" if self.moving else "Why not moving"
        return "%s: [%s] %s" % (label, self.code, self.summary)


def _age(now, stamp):
    return max(0.0, now - stamp) if stamp else float("inf")


def diagnose_runtime(snapshot, now):
    """Return the highest-priority reason for the controller's motion state.

    ``snapshot`` is deliberately a plain dictionary.  The controller owns its
    mutable ROS state and publishes only the values needed for diagnosis.
    """
    manual = snapshot.get("manual_cmd")
    if manual:
        return RuntimeStatus(
            "MANUAL_ACTIVE", "manual %s command is active" % manual, moving=True)
    if snapshot.get("paused"):
        detail = snapshot.get("web_message") or "press RESUME AUTO to continue"
        return RuntimeStatus("AUTO_PAUSED", detail)
    if snapshot.get("automatic_reset_pending"):
        return RuntimeStatus(
            "MAP_RESET_PENDING",
            snapshot.get("automatic_reset_reason") or "waiting to reset the SLAM map")

    if (snapshot.get("use_depth_map") and
            _age(now, snapshot.get("registered_depth_stamp")) > 1.0):
        return RuntimeStatus(
            "REGISTERED_DEPTH_STALE",
            "planner depth is missing or older than 1.0 s")
    if _age(now, snapshot.get("scan_stamp")) > 0.6:
        return RuntimeStatus("LIDAR_STALE", "LiDAR is missing or older than 0.6 s")
    if (snapshot.get("use_depth") and
            _age(now, snapshot.get("depth_stamp")) > 1.0):
        return RuntimeStatus("DEPTH_STALE", "collision depth is missing or older than 1.0 s")

    if snapshot.get("use_map"):
        if not snapshot.get("map_trusted", True):
            failure = snapshot.get("automatic_reset_failure")
            detail = ("SLAM map is untrusted; reset failed: %s" % failure if failure else
                      "SLAM map is untrusted and must be reset")
            return RuntimeStatus("MAP_UNTRUSTED", detail)
        if not snapshot.get("pose_available"):
            return RuntimeStatus("POSE_UNAVAILABLE", "SLAM has not produced a pose")
        if _age(now, snapshot.get("pose_stamp")) > snapshot.get("slam_stale", 2.5):
            return RuntimeStatus("POSE_STALE", "SLAM pose updates are stale")
        if not snapshot.get("have_map"):
            return RuntimeStatus("MAP_UNAVAILABLE", "SLAM has not produced a map")
        if not snapshot.get("map_fresh"):
            return RuntimeStatus("MAP_STALE", "SLAM map heartbeat is stale")

    if snapshot.get("drive") and now < snapshot.get("pose_guard_start", 0.0):
        return RuntimeStatus("POSE_GUARD", "waiting for the startup pose stability guard")
    if snapshot.get("use_depth") and not snapshot.get("floor_ready"):
        return RuntimeStatus("LEARNING_FLOOR", "learning the depth-camera floor profile")
    depth_health = snapshot.get("depth_health", "ready")
    if snapshot.get("use_depth") and not depth_health.startswith("ready"):
        return RuntimeStatus("DEPTH_UNHEALTHY", depth_health)

    phase = snapshot.get("phase", "unknown")
    if phase in ("object_focus", "novel_object_verify"):
        return RuntimeStatus("OBJECT_VERIFICATION", "holding still to verify an object")

    last_cmd = snapshot.get("last_cmd")
    if snapshot.get("commanded_moving") and last_cmd and last_cmd[0] != "stop":
        return RuntimeStatus("MOTION_ACTIVE", "executing %s" % (last_cmd,), moving=True)
    if snapshot.get("state") == "FAULT":
        return RuntimeStatus(
            "CONTROLLER_FAULT", snapshot.get("state_reason") or "controller fault")
    if phase in ("plan", "goal_stopped"):
        return RuntimeStatus(
            "ROUTE_UNAVAILABLE",
            snapshot.get("web_message") or snapshot.get("state_reason") or
            "no reachable known-free route is available")
    if snapshot.get("path_length", 0) == 0 and phase in ("go", "user_go"):
        return RuntimeStatus("ROUTE_EMPTY", "the active route has no remaining waypoints")
    return RuntimeStatus(
        "SETTLED", snapshot.get("web_message") or "controller is settled and replanning")


def format_pose(pose):
    if pose is None:
        return "Pose: unavailable"
    return "Pose: x %.2f m, y %.2f m, heading %.1f deg" % (
        pose[0], pose[1], math.degrees(pose[2]))
