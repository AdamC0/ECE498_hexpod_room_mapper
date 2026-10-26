#!/usr/bin/env python3
"""Autonomous JetHexa room mapper.

The robot explores frontier cells in Hector SLAM's map. The depth camera is the
primary forward collision sensor; LiDAR supplies SLAM and an independent close
range emergency stop. Stationary RGB/depth observations place YOLO objects on
the map. Run without _drive:=true for a dry run.
"""

import math
import os
import struct
import threading
import time

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import BatteryState, CameraInfo, Image, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger

from depth_mapping import project_depth_cells
from map_explore import ExploreMap
from stationary_survey import StationarySurvey
from object_mapping import (ObjectTracker, TransformCache, YoloOnnxDetector, backproject,
                            decode_depth_image, load_labels, sample_box_depth,
                            transform_point)

try:
    from jethexa_controller import client
except ImportError:
    client = None


SEARCH, HOLD, AVOID, FAULT, DONE = "SEARCH", "HOLD", "AVOID", "FAULT", "DONE"


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


class RoomMapper:
    def __init__(self):
        rospy.init_node("room_mapper", anonymous=True, log_level=rospy.INFO)
        self.drive = bool(rospy.get_param("~drive", False))
        # Hardware validation should be able to bring the complete controller
        # online without racing to press PAUSE before the first motion command.
        self.start_paused = bool(rospy.get_param("~start_paused", False))
        self.use_map = bool(rospy.get_param("~use_map", True))
        self.reset_map = bool(rospy.get_param("~reset_map", True))
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.stable_scan_topic = rospy.get_param("~stable_scan_topic", "/scan_stable")
        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        # Native depth is substantially denser than RGB-registered depth on
        # this DABAI camera and is the correct stream for collision geometry.
        # Registered depth remains useful later for placing YOLO detections.
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.map_topic = rospy.get_param("~map_topic", "/map")
        # Hector republishes /map continuously, while RTAB-Map publishes its
        # occupancy grid only when the grid changes. Use RTAB-Map's graph info
        # as the liveness signal so an unchanged stationary map does not become
        # falsely stale, while a stopped mapping process still fails closed.
        default_map_heartbeat = ("/visual_slam/info"
                                 if self.map_topic == "/visual_map" else "")
        self.map_heartbeat_topic = str(rospy.get_param(
            "~map_heartbeat_topic", default_map_heartbeat)).strip()
        self.map_stale = float(rospy.get_param("~map_stale", 5.0))
        if not math.isfinite(self.map_stale) or not 1.0 <= self.map_stale <= 30.0:
            raise ValueError("map_stale must be between 1 and 30 seconds")
        self.pose_topic = rospy.get_param("~pose_topic", "/slam_out_pose")
        self.map_frame = str(rospy.get_param("~map_frame", "map")).lstrip("/")
        self.pose_frame = str(rospy.get_param(
            "~pose_frame", "jethexa/base_link")).lstrip("/")
        self.pose_type = str(rospy.get_param(
            "~pose_type", "pose_stamped")).strip().lower()
        if self.pose_type not in ("pose_stamped", "pose_with_covariance_stamped", "tf"):
            raise ValueError(
                "pose_type must be pose_stamped, pose_with_covariance_stamped, or tf")
        self.reset_service = str(rospy.get_param("~reset_service", "/reset_map"))
        self.reset_service_type = str(rospy.get_param(
            "~reset_service_type", "trigger")).strip().lower()
        if self.reset_service_type not in ("trigger", "empty"):
            raise ValueError("reset_service_type must be trigger or empty")
        self.web_port = int(rospy.get_param("~web_port", 8000))
        battery_topics = rospy.get_param(
            "~battery_topics", ["/battery", "/battery_state", "/voltage"])
        if isinstance(battery_topics, str):
            battery_topics = [battery_topics]
        self.battery_topics = [str(topic) for topic in battery_topics if str(topic)]
        self.battery_empty_voltage = float(rospy.get_param("~battery_empty_voltage", 9.8))
        self.battery_full_voltage = float(rospy.get_param("~battery_full_voltage", 12.6))
        if (not math.isfinite(self.battery_empty_voltage) or
                not math.isfinite(self.battery_full_voltage) or
                self.battery_full_voltage <= self.battery_empty_voltage):
            raise ValueError("battery voltage limits must be finite and full must exceed empty")
        self.manual_deadman = float(rospy.get_param("~manual_deadman", 0.55))
        if not math.isfinite(self.manual_deadman) or not 0.3 <= self.manual_deadman <= 2.0:
            raise ValueError("manual_deadman must be between 0.3 and 2.0 seconds")
        # The full leg envelope is modeled as a circular 30 cm footprint.
        # The planner adds map_margin outside this physical radius.
        self.body_radius = float(rospy.get_param("~body_radius", 0.30))
        if not math.isfinite(self.body_radius) or not 0.10 <= self.body_radius <= 0.50:
            raise ValueError("body_radius must be between 0.10 and 0.50 metres")
        self.lidar_stop = float(rospy.get_param("~lidar_stop", self.body_radius + 0.15))
        # The verified G4 installation sees persistent ~0.34-0.40 m returns from
        # the hexapod's own front leg/body in open space. Ignore that return;
        # farther obstacles remain visible and the map planner maintains body
        # clearance around mapped walls.
        # Keep collision returns inside the 0.40 m stop distance.  Hector has
        # its own 0.20 m minimum; 0.50 m here made nearby obstacles vanish.
        self.self_filter = float(rospy.get_param("~lidar_self_filter", 0.15))
        self.use_depth = bool(rospy.get_param("~use_depth", True))
        self.depth_stop = float(rospy.get_param("~depth_stop", 0.60))
        self.depth_ignore_at_or_below = float(rospy.get_param(
            "~depth_ignore_at_or_below", 0.0))
        if (not math.isfinite(self.depth_ignore_at_or_below) or
                not 0.0 <= self.depth_ignore_at_or_below < self.depth_stop):
            raise ValueError("depth ignore distance must be nonnegative and below depth_stop")
        self.floor_margin = float(rospy.get_param("~floor_margin", 0.25))
        self.floor_frames_needed = int(rospy.get_param("~floor_frames", 20))
        if self.floor_frames_needed < 1:
            raise ValueError("floor_frames must be positive")
        self.band_top = float(rospy.get_param("~depth_band_top", 0.20))
        self.band_split = float(rospy.get_param("~depth_band_split", 0.55))
        self.band_bottom = float(rospy.get_param("~depth_band_bottom", 0.95))
        self.band_left = float(rospy.get_param("~depth_band_left", 0.20))
        self.band_right = float(rospy.get_param("~depth_band_right", 0.80))
        if not (0 <= self.band_top < self.band_split < self.band_bottom <= 1 and
                0 <= self.band_left < self.band_right <= 1):
            raise ValueError("Invalid depth image bands")
        self.front_half = math.radians(float(rospy.get_param("~front_half_deg", 30)))
        self.side_max = math.radians(float(rospy.get_param("~side_max_deg", 100)))
        # This robot reports YDLIDAR_G4 at login. Its raw angle 0 points
        # forward, unlike the RPLIDAR convention used by some course images.
        # Keep the parameter override for other hardware, but make the
        # verified robot default safe when LIDAR_TYPE is not exported.
        lidar_type = os.environ.get("LIDAR_TYPE", "YDLIDAR_G4")
        default_offset = 180.0 if lidar_type.startswith("RPLIDAR") else 0.0
        self.yaw_offset = math.radians(float(rospy.get_param("~lidar_yaw_offset_deg", default_offset)))

        self.stride = float(rospy.get_param("~stride", 40.0))
        self.step_height = float(rospy.get_param("~step_height", 20.0))
        self.step_time = float(rospy.get_param("~step_time", 0.5))
        # The bounded hardware test tracked 0.10-rate rotation accurately.
        # The former 0.35 rate outran Hector and produced false translation.
        self.spin_rate = float(rospy.get_param("~spin_rate", 0.10))
        self.spin_stride = float(rospy.get_param("~spin_stride", 0.0))
        # JetHexa can translate in any body-relative direction.  Keep
        # autonomous crab steering inside the forward depth camera's useful
        # envelope; larger corrections still turn in place before translating.
        self.max_crab_angle = math.radians(float(rospy.get_param(
            "~max_crab_angle_deg", 45.0)))
        self.crab_angle_step = math.radians(float(rospy.get_param(
            "~crab_angle_step_deg", 5.0)))
        self.crab_deadband = math.radians(float(rospy.get_param(
            "~crab_deadband_deg", 5.0)))
        if (not math.isfinite(self.max_crab_angle) or
                not math.radians(15.0) <= self.max_crab_angle <= math.radians(60.0) or
                not math.isfinite(self.crab_angle_step) or
                not math.radians(1.0) <= self.crab_angle_step <= math.radians(15.0) or
                not math.isfinite(self.crab_deadband) or
                not 0.0 <= self.crab_deadband <= self.crab_angle_step):
            raise ValueError("crab steering angles are outside the supported range")
        self.translation_speed_percent = 100.0
        self.rotation_speed_percent = 100.0
        self.speed_min_percent = 50.0
        self.speed_max_percent = 200.0
        self.waypoint_tolerance = float(rospy.get_param("~waypoint_tolerance", 0.12))
        if (not math.isfinite(self.waypoint_tolerance) or
                not 0.05 <= self.waypoint_tolerance < self.body_radius):
            raise ValueError("waypoint_tolerance must be positive and smaller than body_radius")
        self.look_seconds = float(rospy.get_param("~look_seconds", 12.0))
        self.hold_seconds = float(rospy.get_param("~hold_seconds", 2.0))
        self.avoid_seconds = float(rospy.get_param("~avoid_seconds", 2.0))
        # Reverse only one full gait cycle, then stop and reassess. Required
        # rear clearance is computed from the swept LiDAR envelope below.
        self.reverse_seconds = float(rospy.get_param("~reverse_seconds", 0.5))
        self.reverse_clearance_margin = float(rospy.get_param(
            "~reverse_clearance_margin", 0.02))
        self.reverse_after_turn_seconds = float(rospy.get_param(
            "~reverse_after_turn_seconds", 2.5))
        if (not math.isfinite(self.reverse_seconds) or
                not 0.3 <= self.reverse_seconds <= 3.0 or
                not math.isfinite(self.reverse_clearance_margin) or
                not 0.0 <= self.reverse_clearance_margin <= 0.20 or
                not math.isfinite(self.reverse_after_turn_seconds) or
                not 1.0 <= self.reverse_after_turn_seconds <= 10.0):
            raise ValueError("reverse timing is outside the supported range")
        # One gait cycle is 0.50 s.  A shorter burst can stop before the feet
        # complete a step, which previously produced commands with no travel.
        self.move_burst = float(rospy.get_param("~move_burst", 1.0))
        self.spin_burst = float(rospy.get_param("~spin_burst", 0.50))
        self.max_continuous_translation = float(rospy.get_param(
            "~max_continuous_translation", 3.0))
        if (not math.isfinite(self.max_continuous_translation) or
                not 1.0 <= self.max_continuous_translation <= 10.0):
            raise ValueError("max_continuous_translation must be between 1 and 10 seconds")
        self.min_spin_progress = math.radians(float(rospy.get_param(
            "~min_spin_progress_deg", 3.0)))
        self.max_spin_stalls = int(rospy.get_param("~max_spin_stalls", 3))
        if not 2 <= self.max_spin_stalls <= 10:
            raise ValueError("max_spin_stalls must be between 2 and 10")
        self.scan_settle = float(rospy.get_param("~scan_settle", 0.65))
        self.survey = StationarySurvey(
            # Raw LiDAR already updates Hector continuously. This short gate
            # verifies a settled pose/scan and admits an RGB-depth snapshot;
            # YOLO completion no longer blocks the next route.
            float(rospy.get_param("~survey_seconds", 2.0)),
            int(rospy.get_param("~survey_scans", 3)))
        self.scan_max_age = float(rospy.get_param("~scan_max_age", 0.6))
        if not math.isfinite(self.scan_max_age) or self.scan_max_age <= 0:
            raise ValueError("scan_max_age must be finite and positive")
        # One short burst cannot plausibly move this platform half a metre.
        # Treat corrections larger than 25 cm / 20 deg as localization loss;
        # the former 50 cm threshold nearly admitted a bad 0.487 m update.
        self.max_pose_jump = float(rospy.get_param("~max_pose_jump", 0.25))
        self.max_yaw_jump = math.radians(float(rospy.get_param("~max_yaw_jump_deg", 20.0)))
        self.slam_stale = float(rospy.get_param("~slam_stale", 2.5))

        # Object detection is optional at runtime: mapping still starts if the
        # ONNX model has not been deployed.  Registered depth is intentionally
        # separate from the denser native depth used for collision avoidance.
        self.detect_objects = bool(rospy.get_param("~detect_objects", True))
        self.yolo_model = os.path.expanduser(rospy.get_param(
            "~yolo_model", "~/room_mapper/models/yolo26n-objv1-150.onnx"))
        self.yolo_fallback_model = os.path.expanduser(rospy.get_param(
            "~yolo_fallback_model", "~/room_mapper/models/yolov5n.onnx"))
        self.yolo_labels = os.path.expanduser(str(rospy.get_param("~yolo_labels", "")))
        self.yolo_input_size = int(rospy.get_param("~yolo_input_size", 640))
        self.yolo_backend = str(rospy.get_param("~yolo_backend", "cpu")).strip().lower()
        self.registered_depth_topic = rospy.get_param(
            "~registered_depth_topic", "/camera/depth_registered/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/rgb/camera_info")
        # Hector remains authoritative for pose and the persistent LiDAR map.
        # Registered depth contributes a retained planner obstacle layer for
        # objects below the laser plane. The hold covers the camera blind area
        # after the robot approaches, while remaining finite for moved objects.
        self.use_depth_map = bool(rospy.get_param("~use_depth_map", True))
        self.depth_map_camera_info_topic = rospy.get_param(
            "~depth_map_camera_info_topic", "/camera/depth_registered/camera_info")
        self.depth_map_hz = float(rospy.get_param("~depth_map_hz", 2.0))
        self.depth_map_decimation = int(rospy.get_param("~depth_map_decimation", 4))
        self.depth_map_range_min = float(rospy.get_param("~depth_map_range_min", 0.15))
        self.depth_map_range_max = float(rospy.get_param("~depth_map_range_max", 2.0))
        self.depth_map_height_min = float(rospy.get_param("~depth_map_height_min", 0.025))
        self.depth_map_height_max = float(rospy.get_param("~depth_map_height_max", 0.50))
        self.depth_map_min_points = int(rospy.get_param("~depth_map_min_points", 3))
        self.depth_map_ttl = float(rospy.get_param("~depth_map_ttl", 30.0))
        self.depth_map_clear_observations = int(rospy.get_param(
            "~depth_map_clear_observations", 2))
        self.depth_map_protected_range = float(rospy.get_param(
            "~depth_map_protected_range", self.depth_stop))
        if (not math.isfinite(self.depth_map_hz) or self.depth_map_hz <= 0 or
                not 1 <= self.depth_map_decimation <= 32 or
                not 0 < self.depth_map_range_min < self.depth_map_range_max or
                not 0 <= self.depth_map_height_min < self.depth_map_height_max or
                self.depth_map_min_points < 1 or
                not math.isfinite(self.depth_map_ttl) or self.depth_map_ttl <= 0 or
                not 1 <= self.depth_map_clear_observations <= 10 or
                not math.isfinite(self.depth_map_protected_range) or
                not self.depth_stop <= self.depth_map_protected_range <=
                    self.depth_map_range_max):
            raise ValueError("depth map parameters are outside safe bounds")
        self.object_hz = float(rospy.get_param("~object_hz", 2.0))
        self.rgb_processing_hz = float(rospy.get_param("~rgb_processing_hz", 10.0))
        self.depth_processing_hz = float(rospy.get_param("~depth_processing_hz", 15.0))
        self.registered_depth_processing_hz = float(rospy.get_param(
            "~registered_depth_processing_hz", 10.0))
        self.opencv_threads = int(rospy.get_param("~opencv_threads", 2))
        if (any(not math.isfinite(value) or value <= 0 for value in
                (self.rgb_processing_hz, self.depth_processing_hz,
                 self.registered_depth_processing_hz)) or self.opencv_threads < 1):
            raise ValueError("image processing rates and opencv_threads must be positive")
        # Structured-light depth is naturally sparse on dark, reflective, or
        # distant surfaces. Hundreds of current samples are sufficient for a
        # robust near-percentile check; requiring 10% coverage caused healthy
        # 4-9% frames to alternate between READY and FAULT.
        self.min_depth_valid_fraction = float(rospy.get_param(
            "~min_depth_valid_fraction", 0.02))
        self.min_depth_valid_pixels = int(rospy.get_param(
            "~min_depth_valid_pixels", 200))
        self.depth_sparse_grace = float(rospy.get_param("~depth_sparse_grace", 1.5))
        if (not math.isfinite(self.min_depth_valid_fraction) or
                not 0.005 <= self.min_depth_valid_fraction <= 0.25 or
                self.min_depth_valid_pixels < 30 or
                not math.isfinite(self.depth_sparse_grace) or
                not 0.2 <= self.depth_sparse_grace <= 5.0):
            raise ValueError("depth validity thresholds are outside safe bounds")
        self.object_sync_slop = float(rospy.get_param("~object_sync_slop", 0.20))
        self.object_max_age = float(rospy.get_param("~object_max_age", 2.5))
        # Admission age rejects backed-up sensor frames. Result age is a
        # separate bound because CPU-only YOLO can take several seconds after
        # a fresh, synchronized pair was captured. Motion generation and the
        # image-time TF snapshot keep that delayed observation map-safe.
        self.object_result_max_age = float(rospy.get_param(
            "~object_result_max_age", 12.0))
        self.object_snapshot_grace = float(rospy.get_param(
            "~object_snapshot_grace", 1.0))
        if (not math.isfinite(self.object_max_age) or self.object_max_age <= 0 or
                not math.isfinite(self.object_result_max_age) or
                self.object_result_max_age < self.object_max_age or
                not math.isfinite(self.object_snapshot_grace) or
                not 0.0 <= self.object_snapshot_grace <= 1.0):
            raise ValueError("object ages or snapshot grace are outside safe bounds")
        self.object_confidence = float(rospy.get_param("~object_confidence", 0.45))
        self.object_priority_confidence = float(rospy.get_param(
            "~object_priority_confidence", 0.30))
        priority_classes = rospy.get_param("~object_priority_classes", [
            "chair", "desk", "podium", "lectern", "stool", "bench", "dining table", "coffee table",
            "side table", "blackboard/whiteboard", "book", "backpack", "laptop",
            "keyboard", "mouse", "monitor/tv", "projector", "printer", "clock",
            "cabinet/shelf", "pen/pencil", "marker", "folder", "notepaper",
            "calculator", "trash bin can", "power outlet"])
        if isinstance(priority_classes, str):
            priority_classes = priority_classes.split(",")
        self.object_priority_classes = [str(item).strip() for item in priority_classes
                                        if str(item).strip()]
        object_classes = rospy.get_param("~object_classes", [])
        if isinstance(object_classes, str):
            object_classes = object_classes.split(",")
        self.object_classes = [str(item).strip() for item in object_classes
                               if str(item).strip()]
        if (not 0.01 <= self.object_priority_confidence <= self.object_confidence <= 0.99 or
                self.yolo_backend not in ("cpu", "cuda_fp16")):
            raise ValueError("YOLO confidence/backend parameters are invalid")
        self.object_merge_distance = float(rospy.get_param("~object_merge_distance", 0.65))
        self.object_min_observations = int(rospy.get_param("~object_min_observations", 3))
        self.object_track_max_age = float(rospy.get_param("~object_track_max_age", 300.0))
        if (not 1 <= self.object_min_observations <= 20 or
                not math.isfinite(self.object_track_max_age) or
                not 30.0 <= self.object_track_max_age <= 3600.0):
            raise ValueError("object confirmation/retention parameters are invalid")
        self.object_focus_enabled = bool(rospy.get_param("~object_focus_enabled", False))
        self.object_focus_seconds = float(rospy.get_param("~object_focus_seconds", 15.0))
        self.object_focus_max_misses = int(rospy.get_param("~object_focus_max_misses", 2))
        if (not math.isfinite(self.object_focus_seconds) or
                not 5.0 <= self.object_focus_seconds <= 60.0 or
                not 1 <= self.object_focus_max_misses <= 10):
            raise ValueError("object focus duration or miss limit is outside safe bounds")

        self.explorer = ExploreMap(
            robot_radius=self.body_radius,
            margin=float(rospy.get_param("~map_margin", 0.03)),
            occupied_threshold=int(rospy.get_param("~occupied_threshold", 75)),
            frontier_goal_distance=float(rospy.get_param(
                "~frontier_goal_distance", 1.50)),
            frontier_max_distance=float(rospy.get_param(
                "~frontier_max_distance", 3.0)))
        if self.use_map and self.reset_map:
            try:
                self.call_reset_service()
            except Exception as exc:
                raise RuntimeError("Could not reset SLAM map at run start: %s" % exc)
        self.map_lock = threading.Lock()
        self.data_lock = threading.Lock()
        self.control_lock = threading.RLock()
        self.shutting_down = False
        self.pose, self.lidar = None, None
        self.camera = None
        self.depth = None
        self.floor_profile = None
        self.floor_sum = None
        self.floor_count = None
        self.floor_zero_sum = None
        self.floor_zero_frac = 0.0
        self.floor_frames = 0
        self.floor_failures = 0
        self.floor_unavailable = False
        self.depth_source = ""
        self.depth_sectors = {"left": float("inf"), "center": float("inf"),
                              "right": float("inf")}
        self.depth_health = "waiting for depth"
        self.depth_history = []
        self.depth_sparse_since = None
        self.last_valid_depth_near = None
        self.last_valid_depth_source = ""
        self.last_valid_depth_sectors = dict(self.depth_sectors)
        self.last_valid_depth_time = 0.0
        self.scan_points = None
        self.t_scan = self.t_pose = self.t_camera = self.t_depth = self.t_map = 0.0
        self.t_map_heartbeat = 0.0
        self.path = []
        self.user_goal = None
        self.nav_goal = None
        self.object_standoff = float(rospy.get_param("~object_standoff", 0.75))
        if not math.isfinite(self.object_standoff) or not 0.4 <= self.object_standoff <= 2.0:
            raise ValueError("object_standoff must be between 0.4 and 2.0 metres")
        self.phase = "look"
        self.phase_start = time.time()
        self.state = SEARCH
        self.state_start = time.time()
        self.state_reason = "initializing"
        self.recovery_action = None
        self.recovery_start = 0.0
        self.last_yaw = None
        self.turned = 0.0
        self.avoid_dir = 1.0
        self.last_cmd = None
        self.motion_until = 0.0
        self.continuous_translation_start = 0.0
        self.last_scan_stamp = 0.0
        self.stable_stationary_scans = 0
        self.stable_turn_scans = 0
        self.rejected_motion_scans = 0
        self.pending_spin_yaw = None
        self.spin_stall_count = 0
        self.settle_until = time.time() + self.scan_settle
        self.survey.reset(self.settle_until)
        self.web_map_center = None
        self.commanded_moving = False
        self.motion_generation = 0
        self.map_generation = 0
        self.map_trusted = True
        self.prev_pose = None
        self.pose_guard_start = time.time() + 5.0
        self.paused = self.start_paused
        self.manual_cmd = None
        self.manual_until = 0.0
        self.web_pending = []
        self.web_msg = "Ready - mapping will begin after sensors and SLAM are online"
        self.view = {}
        self.battery_voltage = None
        self.battery_percentage = None
        self.battery_charging = None
        self.battery_stamp = 0.0
        self.battery_source = "waiting for battery telemetry"
        self.object_lock = threading.Lock()
        self.object_tracker = ObjectTracker(self.object_merge_distance,
                                            self.object_min_observations,
                                            self.object_track_max_age)
        self.object_detections = []
        self.registered_depth = None
        self.registered_depth_stamp = 0.0
        self.t_registered_depth = 0.0
        self.t_depth_map = 0.0
        self.depth_map_status = "waiting for registered depth"
        self.depth_map_last_cleared = 0
        self.depth_map_frame = ""
        self.depth_map_k = None
        self.depth_map_info_size = None
        self.camera_stamp = 0.0
        self.camera_frame = ""
        self.camera_k = None
        self.camera_info_size = None
        self.camera_info_frame = ""
        self.last_object_stamps = (0.0, 0.0)
        self.object_status = "disabled"
        self.detector = None
        self.active_yolo_model = ""
        self.active_yolo_backend = ""
        self.object_processing = False
        self.object_queue_lock = threading.Lock()
        self.object_pending_snapshot = None
        self.object_snapshots_captured = 0
        self.object_snapshots_processed = 0
        self.object_snapshots_replaced = 0
        self.object_endpoint_key = None
        self.object_endpoint_deadline = 0.0
        self.object_endpoint_done = None
        self.object_focus_id = None
        self.object_focus_label = None
        self.object_focus_deadline = 0.0
        self.object_focus_misses = 0
        self.object_focus_next_capture = 0.0
        self.tf_buffer = None
        self.tf_subscribers = []
        self.object_thread = None

        self.robot = None
        if self.drive:
            if client is None:
                raise RuntimeError("jethexa_controller not found - cannot drive")
            self.robot = client.Client(self)

        self.stable_scan_pub = rospy.Publisher(self.stable_scan_topic, LaserScan, queue_size=1)
        self.motion_pub = rospy.Publisher("/room_mapper/moving", Bool, queue_size=1, latch=True)
        self.motion_pub.publish(Bool(False))
        rospy.Subscriber(self.rgb_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        if self.use_depth:
            rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1, buff_size=2**24)
        if self.use_depth_map:
            self.ensure_tf_listener()
            rospy.Subscriber(self.registered_depth_topic, Image,
                             self.registered_depth_callback, queue_size=1, buff_size=2**24)
            rospy.Subscriber(self.depth_map_camera_info_topic, CameraInfo,
                             self.depth_map_camera_info_callback, queue_size=1)
        if self.use_map:
            rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
            if self.map_heartbeat_topic:
                rospy.Subscriber(self.map_heartbeat_topic, rospy.AnyMsg,
                                 self.map_heartbeat_callback, queue_size=1)
            if self.pose_type == "tf":
                self.ensure_tf_listener()
            else:
                pose_message = (PoseWithCovarianceStamped
                                if self.pose_type == "pose_with_covariance_stamped"
                                else PoseStamped)
                rospy.Subscriber(self.pose_topic, pose_message, self.pose_callback, queue_size=1)
        for topic in self.battery_topics:
            rospy.Subscriber(topic, rospy.AnyMsg, self.battery_callback,
                             callback_args=topic, queue_size=1)
        if self.detect_objects:
            self.start_object_detection()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.safe_step)

        if self.web_port:
            import map_web
            requested_web_port = self.web_port
            self.web_port = map_web.start(self, requested_web_port) or 0
            if self.web_port:
                rospy.loginfo("LIVE VIEW: open http://192.168.137.223:%d", self.web_port)
            else:
                rospy.logerr("Web server could not bind any port from %d through %d",
                             requested_web_port, min(requested_web_port + 9, 65535))
        rospy.loginfo("room_mapper started | %s | map=%s | scan=%s | depth=%s | LiDAR offset=%.0f deg",
                      "DRIVE" if self.drive else "DRY RUN (no walking)", self.use_map,
                      self.scan_topic, self.depth_topic if self.use_depth else "off",
                      math.degrees(self.yaw_offset))

    def battery_callback(self, raw, topic):
        """Accept common battery message types without requiring a fixed board driver."""
        message_type = (getattr(raw, "_connection_header", {}) or {}).get("type", "")
        voltage = percentage = None
        charging = None
        try:
            if message_type == "sensor_msgs/BatteryState":
                msg = BatteryState().deserialize(raw._buff)
                voltage = float(msg.voltage)
                if math.isfinite(msg.percentage) and msg.percentage >= 0:
                    percentage = 100.0 * msg.percentage
                charging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
            elif message_type in ("std_msgs/Float32", "std_msgs/Float64"):
                fmt = "<f" if message_type.endswith("Float32") else "<d"
                voltage = float(struct.unpack_from(fmt, raw._buff)[0])
            elif message_type in ("std_msgs/UInt16", "std_msgs/Int32"):
                fmt = "<H" if message_type.endswith("UInt16") else "<i"
                voltage = float(struct.unpack_from(fmt, raw._buff)[0])
                if voltage > 100.0:
                    voltage /= 1000.0
            else:
                self.battery_source = "%s uses unsupported %s" % (
                    topic, message_type or "message type")
                return
        except (ValueError, TypeError, struct.error) as exc:
            rospy.logwarn_throttle(5, "Bad battery telemetry on %s: %s", topic, exc)
            return
        if voltage is not None and (not math.isfinite(voltage) or voltage <= 0):
            return
        if percentage is None and voltage is not None:
            percentage = 100.0 * ((voltage - self.battery_empty_voltage) /
                                  (self.battery_full_voltage - self.battery_empty_voltage))
        if percentage is not None:
            percentage = max(0.0, min(100.0, percentage))
        with self.data_lock:
            self.battery_voltage = voltage
            self.battery_percentage = percentage
            self.battery_charging = charging
            self.battery_stamp = time.time()
            self.battery_source = topic

    def image_callback(self, msg):
        now = time.time()
        rate = getattr(self, "rgb_processing_hz", 0.0)
        if rate > 0 and now - getattr(self, "t_camera", 0.0) < 1.0 / rate:
            return
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            frame = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if msg.encoding != "bgr8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception as exc:
            rospy.logwarn_throttle(5, "Bad RGB image: %s", exc)
            return
        with self.data_lock:
            self.camera = frame.copy()
            self.t_camera = now
            self.camera_stamp = self.message_time(msg)
            self.camera_frame = getattr(getattr(msg, "header", None), "frame_id", "")

    @staticmethod
    def message_time(msg):
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        try:
            value = stamp.to_sec()
            return value if math.isfinite(value) and value > 0 else 0.0
        except Exception:
            return 0.0

    def start_object_detection(self):
        candidates = []
        for path in (self.yolo_model, self.yolo_fallback_model):
            if path and path not in candidates:
                candidates.append(path)
        failures = []
        cv2.setNumThreads(self.opencv_threads)
        thresholds = {label: self.object_priority_confidence
                      for label in self.object_priority_classes}
        for index, path in enumerate(candidates):
            if not os.path.isfile(path):
                failures.append("missing %s" % path)
                continue
            try:
                explicit_labels = self.yolo_labels if index == 0 else ""
                sidecar = explicit_labels or os.path.splitext(path)[0] + ".names"
                labels = load_labels(sidecar) if os.path.isfile(sidecar) else None
                backends = [self.yolo_backend]
                if self.yolo_backend != "cpu":
                    backends.append("cpu")
                last_error = None
                for backend in backends:
                    try:
                        detector = YoloOnnxDetector(
                            path, labels=labels, input_size=self.yolo_input_size,
                            confidence=self.object_confidence,
                            class_thresholds=thresholds,
                            allowed_labels=self.object_classes,
                            backend=backend)
                        detector.warmup()
                        self.detector = detector
                        self.active_yolo_model = path
                        self.active_yolo_backend = backend
                        break
                    except Exception as exc:
                        last_error = exc
                        self.detector = None
                if self.detector is None:
                    raise last_error
                self.ensure_tf_listener()
                if index:
                    rospy.logwarn("Preferred YOLO unavailable; using rollback model %s", path)
                break
            except Exception as exc:
                failures.append("%s: %s" % (path, exc))
                self.detector = None
        if self.detector is None:
            self.object_status = "startup failed: " + "; ".join(failures)
            rospy.logerr("Object detection disabled; %s", self.object_status)
            return
        if not self.use_depth_map:
            rospy.Subscriber(self.registered_depth_topic, Image,
                             self.registered_depth_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber(self.camera_info_topic, CameraInfo,
                         self.camera_info_callback, queue_size=1)
        self.object_status = ("%s (%s) ready; waiting for synchronized RGB, registered depth, "
                              "and camera info" %
                              (os.path.basename(self.active_yolo_model),
                               self.active_yolo_backend))
        self.object_thread = threading.Thread(target=self.object_loop, name="object-mapper")
        self.object_thread.start()

    def registered_depth_callback(self, msg):
        now = time.time()
        rate = getattr(self, "registered_depth_processing_hz", 0.0)
        if rate > 0 and now - getattr(self, "t_registered_depth", 0.0) < 1.0 / rate:
            return
        try:
            depth = decode_depth_image(msg)
        except Exception as exc:
            rospy.logwarn_throttle(5, "Bad registered depth image: %s", exc)
            return
        with self.data_lock:
            self.registered_depth = depth
            self.registered_depth_stamp = self.message_time(msg)
            self.t_registered_depth = now
        if self.use_depth_map and now - self.t_depth_map >= 1.0 / self.depth_map_hz:
            self.update_depth_obstacle_map(depth, msg)

    def depth_map_camera_info_callback(self, msg):
        if (len(msg.K) >= 9 and all(math.isfinite(value) for value in msg.K) and
                msg.K[0] > 0 and msg.K[4] > 0):
            with self.data_lock:
                self.depth_map_k = tuple(msg.K)
                self.depth_map_info_size = (msg.height, msg.width)

    def update_depth_obstacle_map(self, depth, msg):
        """Project low registered-depth geometry into Hector's planner frame."""
        now = time.time()
        with self.data_lock:
            k = self.depth_map_k
            size = self.depth_map_info_size
        if k is None or size != depth.shape:
            self.depth_map_status = "waiting for matching registered-depth calibration"
            return
        frame = getattr(getattr(msg, "header", None), "frame_id", "").lstrip('/')
        stamp = self.message_time(msg)
        if not frame or stamp <= 0:
            self.depth_map_status = "registered depth has no valid frame/time"
            return
        try:
            with self.object_lock:
                transform = self.tf_buffer.lookup(self.map_frame, frame, stamp, max_age=.5)
            occupied_cells, observed_free_cells = project_depth_cells(
                depth, k, transform, cell=self.explorer.cell,
                decimation=self.depth_map_decimation,
                range_min=self.depth_map_range_min, range_max=self.depth_map_range_max,
                height_min=self.depth_map_height_min, height_max=self.depth_map_height_max,
                min_points=self.depth_map_min_points,
                protected_range=self.depth_map_protected_range)
            with self.map_lock:
                cleared = self.explorer.update_depth_obstacles(
                    occupied_cells, now, self.depth_map_ttl,
                    observed_free_cells=observed_free_cells,
                    clear_confirmations=self.depth_map_clear_observations)
                count = self.explorer.depth_obstacle_count(now)
            self.t_depth_map = now
            self.depth_map_frame = frame
            self.depth_map_last_cleared = cleared
            self.depth_map_status = ("%d temporary obstacle cells (%.0f s hold); "
                                     "cleared %d this update" %
                                     (count, self.depth_map_ttl, cleared))
            if cleared:
                rospy.loginfo("Registered depth cleared %d disproven planner cells", cleared)
        except (KeyError, ValueError) as exc:
            self.depth_map_status = "projection unavailable: %s" % exc
            rospy.logwarn_throttle(5, "Depth obstacle map unavailable: %s", exc)

    def camera_info_callback(self, msg):
        with self.data_lock:
            self.camera_k = None
            if (len(msg.K) >= 9 and all(math.isfinite(v) for v in msg.K) and
                    msg.K[0] > 0 and msg.K[4] > 0):
                self.camera_k = tuple(msg.K)
                self.camera_info_size = (msg.height, msg.width)
                self.camera_info_frame = msg.header.frame_id.lstrip('/')

    def tf_callback(self, msg, is_static):
        with self.object_lock:
            for item in msg.transforms:
                t, q = item.transform.translation, item.transform.rotation
                self.tf_buffer.update(item.header.frame_id, item.child_frame_id,
                                      (t.x, t.y, t.z), (q.x, q.y, q.z, q.w),
                                      self.message_time(item), is_static)

    def ensure_tf_listener(self):
        """Create one shared TF cache for SLAM pose and object projection."""
        if self.tf_buffer is not None:
            return
        from tf2_msgs.msg import TFMessage
        self.tf_buffer = TransformCache()
        self.tf_subscribers = [
            rospy.Subscriber("/tf", TFMessage, self.tf_callback,
                             callback_args=False, queue_size=20),
            rospy.Subscriber("/tf_static", TFMessage, self.tf_callback,
                             callback_args=True, queue_size=20),
        ]

    def update_pose_from_tf(self):
        """Feed the controller RTAB-Map's map-to-base transform as a pose."""
        if self.pose_type != "tf" or self.tf_buffer is None:
            return
        stamp = rospy.Time.now()
        try:
            with self.object_lock:
                translation, quaternion = self.tf_buffer.lookup(
                    self.map_frame, self.pose_frame, stamp.to_sec(),
                    max_age=max(0.5, self.slam_stale))
        except (KeyError, ValueError) as exc:
            rospy.logwarn_throttle(5, "SLAM pose TF unavailable: %s", exc)
            return
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = translation
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = quaternion
        self.pose_callback(msg)

    def object_loop(self):
        period = 1.0 / max(0.1, self.object_hz)
        next_run = 0.0
        while not rospy.is_shutdown() and not self.shutting_down:
            now = time.time()
            if now < next_run:
                time.sleep(min(.05, next_run - now))
                continue
            with self.object_queue_lock:
                snapshot = self.object_pending_snapshot
                self.object_pending_snapshot = None
            if snapshot is None:
                time.sleep(.05)
                continue
            try:
                self.process_object_snapshot(snapshot)
            except Exception as exc:
                self.object_status = "processing failed: %s" % exc
                rospy.logerr_throttle(5, "Object processing failed: %s", exc)
            finally:
                with self.object_queue_lock:
                    self.object_snapshots_processed += 1
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
        """Queue one immutable observation, retaining only the newest pending one."""
        with self.object_queue_lock:
            if self.object_pending_snapshot is not None:
                self.object_snapshots_replaced += 1
            self.object_pending_snapshot = snapshot
            self.object_snapshots_captured += 1

    def capture_endpoint_snapshot(self, now):
        """Capture at most once per ready survey, with a bounded sensor grace."""
        if self.detector is None:
            return True
        key = (getattr(self.survey, "settled_at", 0.0),
               getattr(self.survey, "required_stamp", 0.0))
        if self.object_endpoint_done == key:
            return True
        if self.object_endpoint_key != key:
            self.object_endpoint_key = key
            self.object_endpoint_deadline = now + self.object_snapshot_grace
        snapshot = self.capture_object_snapshot()
        if snapshot is not None:
            self.enqueue_object_snapshot(snapshot)
            self.object_endpoint_done = key
            self.object_endpoint_deadline = 0.0
            return True
        if now >= self.object_endpoint_deadline:
            self.object_endpoint_done = key
            self.object_status = ("endpoint snapshot skipped after %.2f s acquisition grace" %
                                  self.object_snapshot_grace)
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
        # Set this under the control lock so legacy focus mode can keep its
        # original inference hold without affecting default nonblocking mode.
        with self.control_lock:
            if self.shutting_down:
                return
            self.object_processing = True
        inference_started = time.time()
        try:
            detections = self.detector.detect(frame)
        except Exception:
            with self.control_lock:
                self.object_processing = False
            raise
        inference_seconds = time.time() - inference_started
        for detection in detections:
            detection["map_point"] = None
            sample = sample_box_depth(depth, detection["box"])
            if sample is not None:
                distance, u, v = sample
                detection["depth"] = distance
                if transform is not None:
                    detection["map_point"] = transform_point(backproject(u, v, distance, k),
                                                              *transform)
        # Serialize commit with motion, shutdown, and pose-loss track clearing.
        with self.control_lock, self.data_lock, self.object_lock:
            self.object_processing = False
            now = time.time()
            ros_now = rospy.Time.now().to_sec()
            capture_ages = tuple(ros_now - s for s in stamps)
            if not self.object_commit_ready(now):
                for detection in detections:
                    detection["map_point"] = None
                if not self.shutting_down:
                    self.object_detections = detections
                self.object_status = ("discarded observation after %.2f s: "
                                      "SLAM/map became unavailable" % inference_seconds)
                return
            if map_generation != getattr(self, "map_generation", 0):
                for detection in detections:
                    detection["map_point"] = None
                self.object_detections = detections
                self.object_status = ("discarded observation after %.2f s: map generation "
                                      "changed" % inference_seconds)
                return
            if (getattr(self, "object_focus_enabled", False) and
                    generation != self.motion_generation):
                for detection in detections:
                    detection["map_point"] = None
                self.object_detections = detections
                self.object_status = "discarded observation after %.2f s: motion changed" % \
                    inference_seconds
                return
            if any(not 0 <= age <= self.object_result_max_age for age in capture_ages):
                for detection in detections:
                    detection["map_point"] = None
                self.object_detections = detections
                self.object_status = ("%d detections shown; map observation discarded after "
                                      "%.2f s: RGB/depth age %.2f/%.2f s exceeds %.2f s" %
                                      (len(detections), inference_seconds,
                                       capture_ages[0], capture_ages[1],
                                       self.object_result_max_age))
                return
            mapped = 0
            updated_tracks = []
            for detection in detections:
                point = detection["map_point"]
                if point is not None:
                    track = self.object_tracker.update(
                        detection["label"], point[0], point[1],
                        detection["confidence"], now, observation_id=observation_id)
                    if track is not None:
                        updated_tracks.append(track)
                        mapped += 1
            self.object_detections = detections
            confirmed = len(self.object_tracker.snapshot(True, now))
            self.object_status = ("%d detections, %d mapped, %d confirmed; %.2f s inference" %
                                  (len(detections), mapped, confirmed, inference_seconds))
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
            minimum = getattr(self.object_tracker, "min_observations", 2)
            candidate = next((track for track in updated_tracks
                              if track["count"] < minimum), None)
            if candidate is None:
                return
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
        minimum = getattr(self.object_tracker, "min_observations", 2)
        if target is not None and target["count"] >= minimum:
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

    def scan_callback(self, msg):
        if not msg.ranges:
            return
        stamp = self.message_time(msg)
        ros_now = rospy.Time.now().to_sec()
        if stamp <= 0 or not 0 <= ros_now - stamp <= self.scan_max_age:
            rospy.logwarn_throttle(5, "Ignoring LiDAR scan with missing, stale, or future timestamp")
            return
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
        angles = np.arctan2(np.sin(angles + self.yaw_offset), np.cos(angles + self.yaw_offset))
        valid = np.isfinite(ranges) & (ranges >= max(msg.range_min, self.self_filter)) & (ranges <= msg.range_max)

        def sector(mask):
            values = ranges[valid & mask]
            return float(values.min()) if values.size else float("inf")

        heads = np.radians(np.arange(-180, 180, 10))
        free = np.full(len(heads), 3.0)
        x, y = ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])
        for i, h in enumerate(heads):
            fwd = x * math.cos(h) + y * math.sin(h)
            side = -x * math.sin(h) + y * math.cos(h)
            hit = (fwd > 0) & (np.abs(side) < self.body_radius)
            if hit.any():
                free[i] = min(3.0, float(fwd[hit].min()))
        scan = {
            "front": sector(np.abs(angles) <= self.front_half),
            "left": sector((angles > self.front_half) & (angles <= self.side_max)),
            "right": sector((angles < -self.front_half) & (angles >= -self.side_max)),
            "back": sector(np.abs(angles) >= math.radians(150)),
            "near_front": sector(np.abs(angles) <= math.radians(60)),
            "near_back": sector(np.abs(angles) >= math.radians(120)),
            # Distance the robot center may translate backward before any
            # return enters the same 0.45 m envelope used by the emergency
            # LiDAR stop. Unlike a broad radial sector minimum, this correctly
            # handles objects near the rear corners of the circular footprint.
            "rear_path_clearance": swept_circle_clearance(
                np.column_stack((x, y)), math.pi,
                getattr(self, "lidar_stop", self.body_radius + 0.15)),
            "nearest": sector(np.ones(len(angles), dtype=bool)),
            "heads": heads, "free": free,
        }
        with self.data_lock:
            if stamp <= self.last_scan_stamp:
                return
            self.last_scan_stamp = stamp
            self.lidar = scan
            self.t_scan = time.time()
            self.scan_points = np.column_stack((x, y))
        # Translating scans are excluded. Pure zero-stride turns are different:
        # Hector needs their incremental geometry to estimate yaw; hiding a
        # full turn makes it invent translation when scans resume afterward.
        with self.control_lock, self.data_lock:
            now = time.time()
            if self.scan_ready_for_mapping(now, stamp, rospy.Time.now().to_sec()):
                self.stable_scan_pub.publish(msg)
                # Turning scans localize yaw but do not satisfy a stationary
                # survey requirement.
                if (hasattr(self, "survey") and not self.commanded_moving and
                        now >= self.settle_until):
                    self.survey.observe(stamp)
                    self.stable_stationary_scans += 1
                elif self.commanded_moving:
                    self.stable_turn_scans += 1
            elif self.commanded_moving:
                self.rejected_motion_scans += 1

    def scan_ready_for_mapping(self, now, stamp=None, ros_now=None):
        if self.shutting_down or not self.map_trusted:
            return False
        rotating_in_place = (self.commanded_moving and self.last_cmd is not None and
                             self.last_cmd[0] == "spin" and self.spin_stride == 0.0)
        if self.commanded_moving:
            return rotating_in_place
        if now < self.settle_until:
            return False
        if stamp is None:
            return True
        # LaserScan stamps identify the FIRST ray. Callback receipt after
        # settling is insufficient if acquisition began during motion/settling.
        return (stamp > 0 and 0 <= ros_now - stamp <= self.scan_max_age and
                stamp >= ros_now - (now - self.settle_until))

    def depth_callback(self, msg):
        now = time.time()
        rate = getattr(self, "depth_processing_hz", 0.0)
        if rate > 0 and now - getattr(self, "t_depth", 0.0) < 1.0 / rate:
            return
        try:
            if msg.encoding in ("16UC1", "mono16"):
                kind, scale = "u2", 0.001
            elif msg.encoding == "32FC1":
                kind, scale = "f4", 1.0
            else:
                raise ValueError("Unsupported depth encoding: " + msg.encoding)
            dtype = np.dtype((">" if msg.is_bigendian else "<") + kind)
            if (msg.height < 2 or msg.width < 2 or
                    msg.step < msg.width * dtype.itemsize or
                    len(msg.data) < msg.height * msg.step):
                raise ValueError("Invalid depth dimensions, stride, or buffer length")
            d = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                           strides=(msg.step, dtype.itemsize)).astype(np.float32) * scale
        except Exception as exc:
            rospy.logwarn_throttle(5, "Bad depth image: %s", exc)
            return
        d = d[::2, ::2].copy()
        d[~np.isfinite(d)] = 0.0
        with self.data_lock:
            if self.depth is not None and self.depth.shape != d.shape:
                self.floor_profile = self.floor_sum = self.floor_count = None
                self.floor_zero_sum = None
                self.floor_frames = 0
                self.floor_failures = 0
                self.floor_unavailable = False
            if self.floor_profile is None:
                self.learn_floor(d)
            self.depth = d
            self.t_depth = now

    def learn_floor(self, depth):
        """Learn per-row floor range while the robot is stationary."""
        h, w = depth.shape
        c0, c1 = int(self.band_left * w), int(self.band_right * w)
        cols = depth[:, c0:c1]
        if self.floor_sum is None:
            self.floor_sum = np.zeros(h, np.float32)
            self.floor_count = np.zeros(h, np.float32)
            self.floor_zero_sum = np.zeros(h, np.float32)
        for r in range(int(self.band_split * h), int(self.band_bottom * h)):
            values = cols[r][cols[r] > 0.1]
            self.floor_zero_sum[r] += 1.0 - values.size / float(cols.shape[1])
            if values.size > 0.3 * cols.shape[1]:
                self.floor_sum[r] += np.median(values)
                self.floor_count[r] += 1
        self.floor_frames += 1
        if self.floor_frames >= self.floor_frames_needed:
            with np.errstate(invalid="ignore", divide="ignore"):
                profile = self.floor_sum / self.floor_count
            profile[self.floor_count < self.floor_frames_needed * 0.7] = np.nan
            rows = np.isfinite(profile)
            needed = max(3, int(0.25 * (self.band_bottom - self.band_split) * h))
            if rows.sum() < needed:
                self.floor_failures += 1
                self.depth_health = "floor calibration failed: insufficient valid rows"
                rospy.logwarn_throttle(5, self.depth_health)
                if self.floor_failures >= 3:
                    # Registered depth can have almost no valid lower-image
                    # pixels in some scenes. Keep the upper-band obstacle
                    # check instead of blocking all mapping indefinitely.
                    self.floor_profile = np.full(h, np.nan, np.float32)
                    self.floor_unavailable = True
                    self.depth_health = "ready (upper-only; floor unavailable)"
                    rospy.logwarn("Depth continuing in upper-only mode; floor bumps are not detectable")
                    return
                self.floor_sum = self.floor_count = self.floor_zero_sum = None
                self.floor_frames = 0
                return
            self.floor_zero_frac = float((self.floor_zero_sum[rows] / self.floor_frames).mean())
            self.floor_profile = profile
            self.depth_health = "ready"
            rospy.loginfo("Depth floor learned: %d usable image rows", int(np.isfinite(profile).sum()))

    def depth_ahead(self, depth):
        """Return the nearest obstacle and retain left/center/right clearance.

        The directional values let recovery honor the planned turn instead of
        choosing from LiDAR alone, which cannot see many low obstacles.
        """
        if depth is None:
            self.depth_source = ""
            self.depth_sectors = {"left": float("inf"), "center": float("inf"),
                                  "right": float("inf")}
            return float("inf")
        h, w = depth.shape
        c0, c1 = int(self.band_left * w), int(self.band_right * w)
        split, bottom = int(self.band_split * h), int(self.band_bottom * h)
        top = depth[int(self.band_top * h):split, c0:c1]
        values = top[(top > 0.1) & np.isfinite(top)]
        min_valid = max(getattr(self, "min_depth_valid_pixels", 200),
                        int(getattr(self, "min_depth_valid_fraction", 0.02) * top.size))
        if values.size < min_valid:
            now = time.time()
            if getattr(self, "depth_sparse_since", None) is None:
                self.depth_sparse_since = now
            cached = getattr(self, "last_valid_depth_near", None)
            age = now - getattr(self, "last_valid_depth_time", 0.0)
            grace = getattr(self, "depth_sparse_grace", 1.5)
            if cached is not None and age <= grace:
                self.depth_health = "ready (brief sparse depth; using recent result)"
                self.depth_source = "recent " + (getattr(
                    self, "last_valid_depth_source", "") or "clear depth")
                self.depth_sectors = dict(getattr(
                    self, "last_valid_depth_sectors",
                    {"left": float("inf"), "center": float("inf"),
                     "right": float("inf")}))
                return cached
            self.depth_health = "insufficient valid depth in upper band"
            self.depth_source = "unknown"
            self.depth_sectors = {"left": float("inf"), "center": float("inf"),
                                  "right": float("inf")}
            return float("inf")
        self.depth_sparse_since = None
        self.depth_health = ("ready (upper-only; floor unavailable)"
                             if self.floor_unavailable else "ready")
        ignore_distance = getattr(self, "depth_ignore_at_or_below", 0.0)
        obstacle_values = values[values > ignore_distance]
        nearest = (float(np.percentile(obstacle_values, 10))
                   if obstacle_values.size >= 30 else float("inf"))
        source = "high" if nearest < 99 else ""
        sectors = {}
        names = ("left", "center", "right")
        edges = np.linspace(0, top.shape[1], 4, dtype=int)
        for index, name in enumerate(names):
            part = top[:, edges[index]:edges[index + 1]]
            part_values = part[(part > ignore_distance) & np.isfinite(part)]
            required = max(20, int(getattr(
                self, "min_depth_valid_fraction", 0.02) * part.size))
            sectors[name] = (float(np.percentile(part_values, 10))
                             if part_values.size >= required else float("inf"))
        if self.floor_profile is not None:
            low = depth[split:bottom, c0:c1]
            floor = self.floor_profile[split:bottom][:, None]
            valid_floor = np.isfinite(floor)
            valid = (low > 0.1) & valid_floor
            floor_pixels = np.broadcast_to(valid_floor, low.shape)
            if floor_pixels.any():
                zero_frac = float(np.count_nonzero(floor_pixels & ~valid)) / np.count_nonzero(floor_pixels)
                if zero_frac - self.floor_zero_frac > 0.30:
                    nearest, source = 0.30, "depth dropout / too close"
                    sectors["center"] = min(sectors["center"], nearest)
            if np.count_nonzero(valid) > 100:
                with np.errstate(invalid="ignore", divide="ignore"):
                    ratio = low / floor
                    shift = float(np.median(ratio[valid]))
                    bump = valid & (ratio < shift * (1.0 - self.floor_margin))
                if np.count_nonzero(bump) >= max(40, int(0.02 * np.count_nonzero(valid))):
                    bump_distance = float(np.percentile(low[bump], 10))
                    if bump_distance < nearest:
                        nearest, source = bump_distance, "floor-bump"
                    for index, name in enumerate(names):
                        section = bump[:, edges[index]:edges[index + 1]]
                        section_low = low[:, edges[index]:edges[index + 1]]
                        if np.count_nonzero(section) >= 10:
                            sector_bump = float(np.percentile(section_low[section], 10))
                            sectors[name] = min(sectors[name], sector_bump)
        if nearest <= ignore_distance:
            nearest = float("inf")
            source = "ignored depth <= %.2f m" % ignore_distance
        for name in names:
            if sectors[name] <= ignore_distance:
                sectors[name] = float("inf")
        self.depth_sectors = sectors
        self.last_valid_depth_near = nearest
        self.last_valid_depth_source = source
        self.last_valid_depth_sectors = dict(sectors)
        self.last_valid_depth_time = time.time()
        self.depth_source = source
        return nearest

    def map_callback(self, msg):
        i = msg.info
        with self.map_lock:
            self.explorer.update_from_slam(msg.data, i.width, i.height, i.resolution,
                                           i.origin.position.x, i.origin.position.y)
            self.t_map = time.time()

    def map_heartbeat_callback(self, _msg):
        """Record mapping-process liveness when occupancy updates are event-driven."""
        self.t_map_heartbeat = time.time()

    def map_is_fresh(self, now):
        """Require a recent grid update or the configured mapper heartbeat."""
        stamp = (self.t_map_heartbeat if self.map_heartbeat_topic else self.t_map)
        return stamp > 0.0 and 0.0 <= now - stamp <= self.map_stale

    def pose_callback(self, msg):
        stamped_pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        q = stamped_pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        pose = (stamped_pose.position.x, stamped_pose.position.y, yaw)
        now = time.time()
        with self.data_lock:
            if self.prev_pose is not None and now >= self.pose_guard_start and self.map_trusted:
                distance = math.hypot(pose[0] - self.prev_pose[0], pose[1] - self.prev_pose[1])
                turn = abs(math.atan2(math.sin(pose[2] - self.prev_pose[2]),
                                      math.cos(pose[2] - self.prev_pose[2])))
                if distance > self.max_pose_jump or turn > self.max_yaw_jump:
                    self.map_trusted = False
                    self.clear_object_tracks()
                    self.web_msg = ("SLAM LOST: pose jumped %.2f m / %.0f deg; restart mapper "
                                    "before continuing" % (distance, math.degrees(turn)))
                    rospy.logerr(self.web_msg)
            self.prev_pose = pose
            self.pose = pose
            self.t_pose = time.time()
            if hasattr(self, "survey"):
                self.survey.localize(self.message_time(msg))
            if getattr(self, "web_map_center", None) is None:
                self.web_map_center = pose[:2]

    def clear_object_tracks(self):
        """Discard landmarks tied to a SLAM frame that is no longer valid."""
        # The attribute guard also keeps the callback independently testable
        # with lightweight RoomMapper.__new__ fixtures.
        if not hasattr(self, "object_lock"):
            return
        with self.object_lock:
            self.object_tracker.clear()
            self.object_detections = []
        self.map_generation = getattr(self, "map_generation", 0) + 1
        if hasattr(self, "object_queue_lock"):
            with self.object_queue_lock:
                self.object_pending_snapshot = None

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
            with self.data_lock:
                pose = self.pose
            progress = (abs(math.atan2(math.sin(pose[2] - spin_start),
                                       math.cos(pose[2] - spin_start)))
                        if pose is not None else 0.0)
            self.pending_spin_yaw = None
            if progress < getattr(self, "min_spin_progress", math.radians(3.0)):
                self.spin_stall_count = getattr(self, "spin_stall_count", 0) + 1
                limit = getattr(self, "max_spin_stalls", 3)
                if self.spin_stall_count >= limit:
                    self.map_trusted = False
                    self.clear_object_tracks()
                    self.web_msg = ("SLAM LOST: %d consecutive turns made less than %.1f deg "
                                    "progress; robot remains stopped" %
                                    (self.spin_stall_count,
                                     math.degrees(self.min_spin_progress)))
                    rospy.logerr(self.web_msg)
                else:
                    self.web_msg = ("Turn made only %.1f deg progress; retrying (%d/%d)" %
                                    (math.degrees(progress), self.spin_stall_count, limit))
                return
            self.spin_stall_count = 0
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
            with self.data_lock:
                self.pending_spin_yaw = self.pose[2] if self.pose is not None else None
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
            start = getattr(self, "pending_spin_yaw", None)
            progress = (abs(math.atan2(math.sin(self.pose[2] - start),
                                      math.cos(self.pose[2] - start)))
                        if start is not None and self.pose is not None else 0.)
            self.spin_stall_count = (getattr(self, "spin_stall_count", 0) + 1
                                     if progress < self.min_spin_progress else 0)
            if self.spin_stall_count >= self.max_spin_stalls:
                self.send(("stop",))
                self.map_trusted = False
                self.clear_object_tracks()
                self.web_msg = "SLAM LOST: repeated turns made insufficient heading progress"
                return
            self.pending_spin_yaw = self.pose[2] if self.pose is not None else None
            self.motion_until = now + self.spin_burst
        if cmd != previous or not self.commanded_moving:
            self.pending_spin_yaw = (self.pose[2] if cmd[0] == "spin" and
                                     self.pose is not None else None)
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
        rotation_clearance = getattr(self, "body_radius", 0.30) + 0.05
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

    def start_look(self, now):
        # A 360-degree LiDAR already surveys the room without rotating the body.
        self.phase, self.phase_start, self.turned, self.last_yaw = "survey", now, 0.0, None

    def start_navigation(self, x, y, label="selected point", standoff=0.0):
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
        self.web_msg = ("Navigating to %s via safe goal %.2f, %.2f m%s" %
                        (label, actual[0], actual[1],
                         " (adjusted %.2f m)" % adjustment if adjustment > .15 else ""))
        return True

    def plan(self, pose, now):
        with self.map_lock:
            path, goal = self.explorer.plan_frontier(pose)
        if path:
            self.path = path
            self.phase, self.phase_start = "go", now
            rospy.loginfo("%s goal (%.1f, %.1f), %d waypoints",
                          self.explorer.last_plan_kind or "mapping", goal[0], goal[1],
                          len(path))
            return True
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

    @staticmethod
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
        # Web safety controls must still work while a sensor or map is faulty.
        self.handle_web()
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
                self.capture_endpoint_snapshot(now)
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
            reason = ("SLAM pose jump detected; mapper restart required" if not self.map_trusted
                      else "waiting for fresh SLAM pose and map")
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
        if self.use_depth and not self.depth_health.startswith("ready"):
            self.send(("stop",))
            self.set_state(FAULT, self.depth_health)
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
        # Rearward travel has LiDAR coverage but no depth-camera view of the
        # floor. Never reverse in response to a possible bump/dropout alarm.
        rear_clear = (rear_path_clearance >= rear_clearance and
                      self.depth_source not in ("floor-bump", "depth dropout / too close"))
        reason = ("depth %.2f m (%s)" % (depth_near, self.depth_source)
                  if depth_blocked else "LiDAR fallback %.2f m" % front)
        self.view = {"front": front, "nearest": lidar["nearest"], "blocked": blocked,
                     "depth": depth_near, "depth_limit": depth_limit, "depth_source": self.depth_source,
                     "depth_primary": True, "lidar_emergency": lidar_emergency}

        if self.state == HOLD:
            self.send(("stop",))
            if not blocked:
                self.set_state(SEARCH, "path cleared")
            elif self.begin_recovery(lidar, rotation_blocked, rear_clear,
                                     "obstacle remains", now, pose):
                pass
            elif now - self.state_start > self.hold_seconds:
                self.web_msg = ("Blocked safely: rotation footprint occupied and rear swept "
                                "travel %.2f m is below %.2f m required" %
                                (rear_path_clearance, rear_clearance))
            return
        if self.state == AVOID:
            if self.recovery_action == "back":
                if not rear_clear:
                    self.send(("stop",))
                    self.set_state(HOLD, "rear clearance lost while reversing")
                elif now - self.recovery_start < self.reverse_seconds:
                    self.web_msg = "Recovery: reversing briefly to create turning clearance"
                    self.send(("back",), interrupt_object=True)
                else:
                    self.send(("stop",))
                    self.recovery_action = None
                    self.set_state(HOLD, "reverse complete; reassessing clearance")
                return
            if rotation_blocked:
                if rear_clear:
                    self.recovery_action = "back"
                    self.recovery_start = now
                    self.send(("back",), interrupt_object=True)
                    self.web_msg = "Recovery: turn constrained; switching to reverse"
                else:
                    self.send(("stop",))
                    self.set_state(HOLD, "rotation and reverse paths occupied")
                return
            if (blocked and rear_clear and self.recovery_action == "spin" and
                    now - self.recovery_start >= getattr(
                        self, "reverse_after_turn_seconds", 2.5)):
                self.recovery_action = "back"
                self.recovery_start = now
                self.web_msg = ("Recovery: front remained blocked after turning; "
                                "reversing to escape corner")
                self.send(("back",), interrupt_object=True)
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
                self.web_msg = "No reachable frontier; holding position and surveying"
                self.start_look(now)
        elif self.phase in ("go", "user_go"):
            cmd = self.follow_path(pose)
            route_clearance = self.command_lidar_clearance(lidar, cmd)
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
            elif route_clearance < self.lidar_stop:
                self.send(("stop",))
                self.path = []
                self.phase = "plan"
                self.set_state(HOLD, "planned travel direction blocked by LiDAR %.2f m" %
                               route_clearance)
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

    def web_request(self, command):
        self.web_pending.append(command)

    def stop_manual(self, message="Autonomy paused; manual drive available"):
        self.manual_cmd = None
        self.manual_until = 0.0
        self.paused = True
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
        if self.phase == "user_go":
            self.phase = "goal_cancelled"
        self.send(("stop",))
        self.web_msg = "Route cancelled; autonomy paused"

    def reset_mapping(self):
        """Stop, clear Hector/local state, and begin a fresh stationary survey."""
        self.manual_cmd = None
        self.manual_until = 0.0
        self.paused = True
        self.send(("stop",))
        try:
            self.call_reset_service()
        except Exception as exc:
            self.web_msg = "CLEAR MAP failed; robot remains paused: %s" % exc
            rospy.logerr(self.web_msg)
            return
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
            self.pose_guard_start = now + 5.0
            self.map_trusted = True
            self.pending_spin_yaw = None
            self.spin_stall_count = 0
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
        self.web_map_center = pose[:2] if pose else None
        self.clear_object_tracks()
        self.web_msg = "Map and object landmarks cleared; PAUSED for a fresh survey"

    def handle_web(self):
        commands, self.web_pending = self.web_pending, []
        for cmd in commands:
            if cmd[0] == "pause":
                self.stop_manual("AUTONOMY PAUSED - manual drive remains available")
            elif cmd[0] == "cancel_route":
                self.cancel_navigation()
            elif cmd[0] == "resume":
                self.manual_cmd, self.manual_until = None, 0.0
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
        pose_line = ("Pose: x %.2f m, y %.2f m, heading %.1f deg" %
                     (pose[0], pose[1], math.degrees(pose[2]))) if pose else "Pose: unavailable"
        scan_line = "SLAM scans: stationary %d, turn %d, rejected motion %d; command %s" % (
            self.stable_stationary_scans, self.stable_turn_scans,
            self.rejected_motion_scans, self.last_cmd)
        manual_cmd = getattr(self, "manual_cmd", None)
        mode = "  MANUAL: %s" % manual_cmd.upper() if manual_cmd else ("  AUTO PAUSED" if self.paused else "")
        depth_map_line = ("Depth planner layer: " + self.depth_map_status
                          if self.use_depth_map else "Depth planner layer: disabled")
        active_model = getattr(self, "active_yolo_model", "")
        detector_line = ("Detector: %s (%s)" %
                         (os.path.basename(active_model),
                          getattr(self, "active_yolo_backend", "unknown"))
                         if active_model else "Detector: disabled")
        if hasattr(self, "object_queue_lock"):
            with self.object_queue_lock:
                queue_line = ("Object snapshots: captured %d, processed %d, replaced %d, "
                              "pending %s" %
                              (self.object_snapshots_captured,
                               self.object_snapshots_processed,
                               self.object_snapshots_replaced,
                               "yes" if self.object_pending_snapshot is not None else "no"))
        else:
            queue_line = "Object snapshots: unavailable"
        return "State: %s/%s%s\nReason: %s\n%s\n%s\nSpeed: translation %.0f%%, rotation %.0f%%\nDepth collision sensing: PRIMARY\n%s\n%s\nLiDAR fallback: %.2f m (emergency stop at %.2f)\n%s\nObjects: %s\n%s\n%s\nLast: %s" % (
            self.state, self.phase, mode, getattr(self, "state_reason", "unknown"), pose_line,
            scan_line, self.translation_speed_percent, self.rotation_speed_percent,
            depth_line, depth_map_line, front, self.lidar_stop,
            detector_line, self.object_status, queue_line, survey_line, self.web_msg)

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

    def shutdown(self):
        with self.control_lock:
            self.shutting_down = True
            if hasattr(self, "object_queue_lock"):
                with self.object_queue_lock:
                    self.object_pending_snapshot = None
            self.timer.shutdown()
            self.last_cmd = None
            try:
                self.send(("stop",))
            except Exception as exc:
                rospy.logerr("Shutdown stop publication failed: %s", exc)
        # The worker needs control_lock to reject any in-flight result.
        object_thread = getattr(self, "object_thread", None)
        if (object_thread is not None and object_thread.is_alive() and
                object_thread is not threading.current_thread()):
            object_thread.join(timeout=5.0)
        rospy.loginfo("room_mapper shutdown; no further controller commands")


if __name__ == "__main__":
    node = RoomMapper()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
