"""Construction and ROS wiring for the Room Mapper controller."""

import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool

import map_web
from map_explore import ExploreMap
from mapper_constants import SEARCH
from object_mapping import ObjectTracker
from stationary_survey import StationarySurvey

try:
    from jethexa_controller import client
except ImportError:
    client = None


def initialize_mapper(self):
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
    self.video_recording_dir = os.path.abspath(os.path.expanduser(str(
        rospy.get_param("~video_recording_dir", os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "recordings")))))
    self.video_recording_fps = float(rospy.get_param("~video_recording_fps", 10.0))
    if (not math.isfinite(self.video_recording_fps) or
            not 1.0 <= self.video_recording_fps <= 30.0):
        raise ValueError("video_recording_fps must be between 1 and 30")
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
    # The operator-estimated full leg-to-leg span is about 0.50 m, so model
    # its physical radius as 0.25 m.  The planner adds map_margin outside it.
    self.body_radius = float(rospy.get_param("~body_radius", 0.25))
    if not math.isfinite(self.body_radius) or not 0.10 <= self.body_radius <= 0.50:
        raise ValueError("body_radius must be between 0.10 and 0.50 metres")
    # Keep the independently validated 0.45 m emergency envelope; it is not
    # derived from the corrected physical radius.
    self.lidar_stop = float(rospy.get_param("~lidar_stop", 0.45))
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
    self.floor_row_min_fraction = float(rospy.get_param(
        "~floor_row_min_fraction", 0.10))
    if (self.floor_frames_needed < 1 or
            not math.isfinite(self.floor_row_min_fraction) or
            not 0.05 <= self.floor_row_min_fraction <= 0.50):
        raise ValueError("floor calibration parameters are invalid")
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
    self.recent_forward_clearance_max_age = float(rospy.get_param(
        "~recent_forward_clearance_max_age", 1.5))
    if (not math.isfinite(self.reverse_seconds) or
            not 0.3 <= self.reverse_seconds <= 3.0 or
            not math.isfinite(self.reverse_clearance_margin) or
            not 0.0 <= self.reverse_clearance_margin <= 0.20 or
            not math.isfinite(self.reverse_after_turn_seconds) or
            not 1.0 <= self.reverse_after_turn_seconds <= 10.0 or
            not math.isfinite(self.recent_forward_clearance_max_age) or
            not 0.5 <= self.recent_forward_clearance_max_age <= 3.0):
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
    self.pose_jump_confirmations = int(rospy.get_param(
        "~pose_jump_confirmations", 2))
    if not 2 <= self.pose_jump_confirmations <= 5:
        raise ValueError("pose_jump_confirmations must be between 2 and 5")
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
    self.yolo_engine = os.path.expanduser(rospy.get_param(
        "~yolo_engine",
        "~/room_mapper/models/gpu_trial/yolo26n-objv1-150-fp16.engine"))
    self.registered_depth_topic = rospy.get_param(
        "~registered_depth_topic", "/camera/depth_registered_cuda/image_raw")
    self.camera_info_topic = rospy.get_param(
        "~camera_info_topic", "/camera/rgb/camera_info")
    # Hector remains authoritative for pose and the persistent LiDAR map.
    # Registered depth contributes a retained planner obstacle layer for
    # objects below the laser plane. The hold covers the camera blind area
    # after the robot approaches, while remaining finite for moved objects.
    self.use_depth_map = bool(rospy.get_param("~use_depth_map", True))
    self.depth_map_camera_info_topic = rospy.get_param(
        "~depth_map_camera_info_topic",
        "/camera/depth_registered_cuda/camera_info")
    # CUDA registration supplies fresh aligned depth at camera rate. Five
    # planner updates per second bounds first-frame obstacle insertion to
    # about 0.2 s without running the heavier projection on every frame.
    self.depth_map_hz = float(rospy.get_param("~depth_map_hz", 5.0))
    self.depth_map_decimation = int(rospy.get_param("~depth_map_decimation", 4))
    self.depth_map_range_min = float(rospy.get_param("~depth_map_range_min", 0.15))
    self.depth_map_range_max = float(rospy.get_param("~depth_map_range_max", 2.0))
    self.depth_map_free_surface_max = float(rospy.get_param(
        "~depth_map_free_surface_max", 8.0))
    self.depth_map_height_min = float(rospy.get_param("~depth_map_height_min", 0.025))
    self.depth_map_height_max = float(rospy.get_param("~depth_map_height_max", 0.50))
    self.depth_map_min_points = int(rospy.get_param("~depth_map_min_points", 3))
    self.depth_map_min_valid_fraction = float(rospy.get_param(
        "~depth_map_min_valid_fraction", 0.10))
    self.depth_map_ttl = float(rospy.get_param("~depth_map_ttl", 5.0))
    self.depth_map_minimum_hold = float(rospy.get_param(
        "~depth_map_minimum_hold", 10.0))
    self.depth_map_unconfirmed_max_age = float(rospy.get_param(
        "~depth_map_unconfirmed_max_age", 60.0))
    self.depth_map_clear_observations = int(rospy.get_param(
        "~depth_map_clear_observations", 2))
    self.depth_map_promotion_observations = int(rospy.get_param(
        "~depth_map_promotion_observations", 3))
    self.depth_map_joint_clear_observations = int(rospy.get_param(
        "~depth_map_joint_clear_observations", 4))
    self.depth_map_joint_clear_evidence_ttl = float(rospy.get_param(
        "~depth_map_joint_clear_evidence_ttl", 5.0))
    self.depth_lidar_sync_tolerance = float(rospy.get_param(
        "~depth_lidar_sync_tolerance", 0.35))
    self.depth_map_settled_tf_max_age = float(rospy.get_param(
        "~depth_map_settled_tf_max_age", 1.0))
    self.lidar_fusion_decimation = int(rospy.get_param(
        "~lidar_fusion_decimation", 2))
    self.lidar_fusion_min_rays = int(rospy.get_param(
        "~lidar_fusion_min_rays", 2))
    self.depth_map_protected_range = float(rospy.get_param(
        "~depth_map_protected_range", self.depth_stop))
    if (not math.isfinite(self.depth_map_hz) or self.depth_map_hz <= 0 or
            not 1 <= self.depth_map_decimation <= 32 or
            not 0 < self.depth_map_range_min < self.depth_map_range_max or
            not self.depth_map_range_max <= self.depth_map_free_surface_max <= 20.0 or
            not 0 <= self.depth_map_height_min < self.depth_map_height_max or
            self.depth_map_min_points < 1 or
            not math.isfinite(self.depth_map_min_valid_fraction) or
            not .02 <= self.depth_map_min_valid_fraction <= .50 or
            not math.isfinite(self.depth_map_ttl) or self.depth_map_ttl <= 0 or
            not math.isfinite(self.depth_map_minimum_hold) or
            not 1.0 <= self.depth_map_minimum_hold <= 120.0 or
            not self.depth_map_minimum_hold <=
                self.depth_map_unconfirmed_max_age <= 600.0 or
            not 1 <= self.depth_map_clear_observations <= 10 or
            not 2 <= self.depth_map_promotion_observations <= 100 or
            not 2 <= self.depth_map_joint_clear_observations <= 100 or
            not math.isfinite(self.depth_map_joint_clear_evidence_ttl) or
            not 1.0 <= self.depth_map_joint_clear_evidence_ttl <= 30.0 or
            not math.isfinite(self.depth_lidar_sync_tolerance) or
            not .05 <= self.depth_lidar_sync_tolerance <= 1.0 or
            not math.isfinite(self.depth_map_settled_tf_max_age) or
            not .5 <= self.depth_map_settled_tf_max_age <= 2.0 or
            not 1 <= self.lidar_fusion_decimation <= 32 or
            not 1 <= self.lidar_fusion_min_rays <= 20 or
            not math.isfinite(self.depth_map_protected_range) or
            not self.depth_stop <= self.depth_map_protected_range <=
                self.depth_map_range_max):
        raise ValueError("depth map parameters are outside safe bounds")
    self.object_hz = float(rospy.get_param("~object_hz", 2.0))
    self.object_preview_enabled = bool(rospy.get_param(
        "~object_preview_enabled", True))
    self.object_detection_ttl = float(rospy.get_param(
        "~object_detection_ttl", 1.0))
    self.rgb_processing_hz = float(rospy.get_param("~rgb_processing_hz", 10.0))
    self.depth_processing_hz = float(rospy.get_param("~depth_processing_hz", 15.0))
    self.registered_depth_processing_hz = float(rospy.get_param(
        "~registered_depth_processing_hz", 10.0))
    self.opencv_threads = int(rospy.get_param("~opencv_threads", 2))
    if (any(not math.isfinite(value) or value <= 0 for value in
            (self.object_hz, self.rgb_processing_hz, self.depth_processing_hz,
             self.registered_depth_processing_hz)) or
            not .25 <= self.object_detection_ttl <= 5.0 or
            self.opencv_threads < 1):
        raise ValueError("image processing rates and opencv_threads must be positive")
    # Structured-light depth is naturally sparse on dark, reflective, or
    # distant surfaces. Hundreds of current samples are sufficient for a
    # robust near-percentile check; requiring 10% coverage caused healthy
    # 4-9% frames to alternate between READY and FAULT.
    self.min_depth_valid_fraction = float(rospy.get_param(
        "~min_depth_valid_fraction", 0.02))
    self.min_depth_valid_pixels = int(rospy.get_param(
        "~min_depth_valid_pixels", 200))
    self.floor_obstacle_min_valid_fraction = float(rospy.get_param(
        "~floor_obstacle_min_valid_fraction", 0.10))
    self.depth_sparse_grace = float(rospy.get_param("~depth_sparse_grace", 1.5))
    if (not math.isfinite(self.min_depth_valid_fraction) or
            not 0.005 <= self.min_depth_valid_fraction <= 0.25 or
            self.min_depth_valid_pixels < 30 or
            not math.isfinite(self.floor_obstacle_min_valid_fraction) or
            not self.min_depth_valid_fraction <=
                self.floor_obstacle_min_valid_fraction <= .50 or
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
        "~object_result_max_age", 45.0))
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
        "side table", "blackboard/whiteboard", "book", "backpack", "luggage",
        "suitcase", "laptop",
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
            self.yolo_backend not in ("cpu", "cuda_fp16", "tensorrt_fp16")):
        raise ValueError("YOLO confidence/backend parameters are invalid")
    self.object_merge_distance = float(rospy.get_param("~object_merge_distance", 0.65))
    self.object_min_observations = int(rospy.get_param("~object_min_observations", 3))
    self.object_track_max_age = float(rospy.get_param("~object_track_max_age", 300.0))
    self.object_removal_misses = int(rospy.get_param("~object_removal_misses", 5))
    self.object_removal_image_margin = float(rospy.get_param(
        "~object_removal_image_margin", 0.10))
    self.object_removal_occlusion_margin = float(rospy.get_param(
        "~object_removal_occlusion_margin", 0.15))
    self.object_burst_frames = int(rospy.get_param("~object_burst_frames", 5))
    self.object_burst_interval = float(rospy.get_param("~object_burst_interval", 0.18))
    self.novel_object_stop_enabled = bool(rospy.get_param(
        "~novel_object_stop_enabled", True))
    self.novel_object_range = float(rospy.get_param("~novel_object_range", 3.0))
    self.novel_object_frames = int(rospy.get_param("~novel_object_frames", 5))
    self.novel_object_timeout = float(rospy.get_param("~novel_object_timeout", 15.0))
    self.novel_object_cooldown = float(rospy.get_param("~novel_object_cooldown", 20.0))
    self.object_expected_hits = int(rospy.get_param("~object_expected_hits", 3))
    self.object_expected_window = int(rospy.get_param("~object_expected_window", 8))
    self.object_expected_min_confidence = float(rospy.get_param(
        "~object_expected_min_confidence", 0.50))
    self.object_expected_median_confidence = float(rospy.get_param(
        "~object_expected_median_confidence", 0.55))
    self.object_luggage_min_confidence = float(rospy.get_param(
        "~object_luggage_min_confidence", 0.30))
    self.object_luggage_median_confidence = float(rospy.get_param(
        "~object_luggage_median_confidence", 0.33))
    self.object_other_hits = int(rospy.get_param("~object_other_hits", 3))
    self.object_other_window = int(rospy.get_param("~object_other_window", 8))
    self.object_other_min_confidence = float(rospy.get_param(
        "~object_other_min_confidence", 0.60))
    self.object_other_median_confidence = float(rospy.get_param(
        "~object_other_median_confidence", 0.75))
    self.object_expected_position_tolerance = float(rospy.get_param(
        "~object_expected_position_tolerance", 0.25))
    self.object_other_position_tolerance = float(rospy.get_param(
        "~object_other_position_tolerance", 0.20))
    self.object_large_position_tolerance = float(rospy.get_param(
        "~object_large_position_tolerance", 0.50))
    self.object_expected_box_iou = float(rospy.get_param(
        "~object_expected_box_iou", 0.40))
    self.object_other_box_iou = float(rospy.get_param(
        "~object_other_box_iou", 0.50))
    self.object_depth_min_pixels = int(rospy.get_param(
        "~object_depth_min_pixels", 24))
    self.object_depth_min_coverage = float(rospy.get_param(
        "~object_depth_min_coverage", 0.15))
    self.object_depth_max_mad = float(rospy.get_param(
        "~object_depth_max_mad", 0.12))
    self.object_depth_min_inlier_ratio = float(rospy.get_param(
        "~object_depth_min_inlier_ratio", 0.60))
    large_classes = rospy.get_param("~object_large_classes", [
        "desk", "dining table", "coffee table", "side table",
        "blackboard/whiteboard", "monitor/tv", "cabinet/shelf"])
    if isinstance(large_classes, str):
        large_classes = large_classes.split(",")
    self.object_large_classes = [str(item).strip() for item in large_classes
                                 if str(item).strip()]
    static_classes = rospy.get_param("~object_static_classes", [
        "chair", "cabinet/shelf", "desk", "bench", "dining table", "stool",
        "couch", "coffee table", "side table", "bed", "blackboard/whiteboard",
        "sink", "oven", "refrigerator", "toilet", "power outlet", "clock",
        "podium", "lectern"])
    if isinstance(static_classes, str):
        static_classes = static_classes.split(",")
    self.object_static_classes = [str(item).strip() for item in static_classes
                                  if str(item).strip()]
    if (not 1 <= self.object_min_observations <= 20 or
            not math.isfinite(self.object_track_max_age) or
            not 30.0 <= self.object_track_max_age <= 3600.0 or
            not 2 <= self.object_removal_misses <= 20 or
            not math.isfinite(self.object_removal_image_margin) or
            not .05 <= self.object_removal_image_margin <= .30 or
            not math.isfinite(self.object_removal_occlusion_margin) or
            not .05 <= self.object_removal_occlusion_margin <= .50 or
            not 2 <= self.object_burst_frames <= 8 or
            not .05 <= self.object_burst_interval <= .5 or
            not math.isfinite(self.novel_object_range) or
            not .5 <= self.novel_object_range <= 8.0 or
            not 2 <= self.novel_object_frames <= 8 or
            not math.isfinite(self.novel_object_timeout) or
            not 5.0 <= self.novel_object_timeout <= 60.0 or
            not math.isfinite(self.novel_object_cooldown) or
            not 5.0 <= self.novel_object_cooldown <= 300.0 or
            not 1 <= self.object_expected_hits <= self.object_expected_window <= 8 or
            not 1 <= self.object_other_hits <= self.object_other_window <= 8 or
            not 0.01 <= self.object_expected_min_confidence <= \
                self.object_expected_median_confidence <= .99 or
            not 0.01 <= self.object_luggage_min_confidence <= \
                self.object_luggage_median_confidence <= .99 or
            not 0.01 <= self.object_other_min_confidence <= \
                self.object_other_median_confidence <= .99 or
            not .05 <= self.object_expected_position_tolerance <= .65 or
            not .05 <= self.object_other_position_tolerance <= .65 or
            not .05 <= self.object_large_position_tolerance <= 1.0 or
            not .05 <= self.object_expected_box_iou <= .95 or
            not .05 <= self.object_other_box_iou <= .95 or
            self.object_depth_min_pixels < 12 or
            not .01 <= self.object_depth_min_coverage <= 1.0 or
            not .01 <= self.object_depth_max_mad <= 1.0 or
            not .05 <= self.object_depth_min_inlier_ratio <= 1.0):
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
        margin=float(rospy.get_param("~map_margin", 0.13)),
        occupied_threshold=int(rospy.get_param("~occupied_threshold", 75)),
        frontier_goal_distance=float(rospy.get_param(
            "~frontier_goal_distance", 1.50)),
        frontier_max_distance=float(rospy.get_param(
            "~frontier_max_distance", 3.0)))
    self.frontier_resurvey_max_turns = int(rospy.get_param(
        "~frontier_resurvey_max_turns", 8))
    if not 1 <= self.frontier_resurvey_max_turns <= 12:
        raise ValueError("frontier_resurvey_max_turns must be between 1 and 12")
    self.depth_recovery_max_turns = int(rospy.get_param(
        "~depth_recovery_max_turns", 8))
    if not 1 <= self.depth_recovery_max_turns <= 12:
        raise ValueError("depth_recovery_max_turns must be between 1 and 12")
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
    self.recovery_back_start = None
    self.recent_forward_clearance = None
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
    self.pending_spin_pose_count = None
    self.pose_update_count = 0
    self.spin_stall_count = 0
    self.spin_stall_start_yaw = None
    self.frontier_blocked_by_fused = False
    self.frontier_resurvey_attempts = 0
    self.depth_recovery_attempts = 0
    self.settle_until = time.time() + self.scan_settle
    self.survey.reset(self.settle_until)
    self.web_map_center = None
    self.commanded_moving = False
    self.motion_generation = 0
    self.map_generation = 0
    self.map_trusted = True
    self.automatic_map_reset_pending = False
    self.automatic_map_reset_reason = ""
    self.automatic_map_reset_failure = ""
    self.prev_pose = None
    self.pose_jump_candidate = None
    self.pose_jump_candidate_count = 0
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
    expected_policy = {
        "hits": self.object_expected_hits,
        "window": self.object_expected_window,
        "minimum_confidence": self.object_expected_min_confidence,
        "median_confidence": self.object_expected_median_confidence,
        "position_tolerance": self.object_expected_position_tolerance,
        "large_position_tolerance": self.object_large_position_tolerance,
        "box_iou": self.object_expected_box_iou,
    }
    other_policy = {
        "hits": self.object_other_hits,
        "window": self.object_other_window,
        "minimum_confidence": self.object_other_min_confidence,
        "median_confidence": self.object_other_median_confidence,
        "position_tolerance": self.object_other_position_tolerance,
        "large_position_tolerance": min(
            self.object_large_position_tolerance,
            max(self.object_other_position_tolerance, .35)),
        "box_iou": self.object_other_box_iou,
    }
    class_policies = {
        "luggage": {
            "minimum_confidence": self.object_luggage_min_confidence,
            "median_confidence": self.object_luggage_median_confidence,
        },
    }
    self.object_tracker = ObjectTracker(self.object_merge_distance,
                                        self.object_min_observations,
                                        self.object_track_max_age,
                                        priority_labels=self.object_priority_classes,
                                        expected_policy=expected_policy,
                                        other_policy=other_policy,
                                        large_labels=self.object_large_classes,
                                        class_policies=class_policies,
                                        static_labels=self.object_static_classes)
    self.object_detections = []
    self.object_detections_stamp = 0.0
    self.registered_depth = None
    self.registered_depth_stamp = 0.0
    self.t_registered_depth = 0.0
    self.t_depth_map = 0.0
    self.depth_map_status = "waiting for registered depth"
    self.depth_map_last_cleared = 0
    self.depth_map_frame = ""
    self.depth_map_k = None
    self.depth_map_info_size = None
    self.lidar_fusion_occupied = []
    self.lidar_fusion_free = []
    self.lidar_fusion_stamp = 0.0
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
    self.object_pending_snapshots = []
    self.object_snapshots_captured = 0
    self.object_snapshots_processed = 0
    self.object_snapshots_replaced = 0
    self.object_endpoint_key = None
    self.object_endpoint_deadline = 0.0
    self.object_endpoint_done = None
    self.object_endpoint_captured = 0
    self.object_endpoint_next_capture = 0.0
    self.object_endpoint_observations = set()
    self.object_preview_next_capture = 0.0
    self.object_motion_preview_next_capture = 0.0
    self.object_motion_preview_stamp = 0.0
    self.novel_object_request = None
    self.novel_object_verification = None
    self.novel_object_sequence = 0
    self.novel_object_cooldowns = {}
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
