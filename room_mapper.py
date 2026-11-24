#!/usr/bin/env python3
"""Autonomous JetHexa room mapper.

The robot explores frontier cells in Hector SLAM's map. The depth camera is the
primary forward collision sensor; LiDAR supplies SLAM and an independent close
range emergency stop. Stationary RGB/depth observations place YOLO objects on
the map. Run without _drive:=true for a dry run.
"""

import os
import threading
import time  # Compatibility export for existing time-mocking integrations.

import cv2
import rospy
from sensor_msgs.msg import CameraInfo, Image

from mapper_constants import AVOID, DONE, FAULT, HOLD, SEARCH
from mapper_setup import initialize_mapper
from mapper_sensors import (
    battery_callback as sensors_battery_callback,
    image_callback as sensors_image_callback,
    message_time as sensors_message_time,
    registered_depth_callback as sensors_registered_depth_callback,
    depth_map_camera_info_callback as sensors_depth_map_camera_info_callback,
    update_depth_obstacle_map as sensors_update_depth_obstacle_map,
    camera_info_callback as sensors_camera_info_callback,
    tf_callback as sensors_tf_callback,
    ensure_tf_listener as sensors_ensure_tf_listener,
    update_pose_from_tf as sensors_update_pose_from_tf,
    scan_callback as sensors_scan_callback,
    update_lidar_fusion as sensors_update_lidar_fusion,
    scan_ready_for_mapping as sensors_scan_ready_for_mapping,
    depth_callback as sensors_depth_callback,
    learn_floor as sensors_learn_floor,
    depth_ahead as sensors_depth_ahead,
    map_callback as sensors_map_callback,
    map_heartbeat_callback as sensors_map_heartbeat_callback,
    map_is_fresh as sensors_map_is_fresh,
    pose_callback as sensors_pose_callback,
)
from mapper_objects import (
    object_loop as objects_object_loop,
    objects_ready as objects_objects_ready,
    object_commit_ready as objects_object_commit_ready,
    capture_object_snapshot as objects_capture_object_snapshot,
    enqueue_object_snapshot as objects_enqueue_object_snapshot,
    capture_preview_snapshot as objects_capture_preview_snapshot,
    capture_motion_preview_snapshot as objects_capture_motion_preview_snapshot,
    novel_object_direction as objects_novel_object_direction,
    request_novel_object_verification_locked as objects_request_novel_object_verification_locked,
    begin_novel_object_verification as objects_begin_novel_object_verification,
    finish_novel_object_verification as objects_finish_novel_object_verification,
    capture_novel_object_snapshot as objects_capture_novel_object_snapshot,
    capture_endpoint_snapshot as objects_capture_endpoint_snapshot,
    process_objects as objects_process_objects,
    process_object_snapshot as objects_process_object_snapshot,
    update_object_focus_locked as objects_update_object_focus_locked,
    finish_object_focus_locked as objects_finish_object_focus_locked,
)
from mapper_web_control import (
    web_request as web_web_request,
    stop_manual as web_stop_manual,
    cancel_navigation as web_cancel_navigation,
    reset_mapping as web_reset_mapping,
    handle_web as web_handle_web,
    web_status as web_web_status,
    web_battery as web_web_battery,
    web_speed as web_web_speed,
)
from mapper_motion import (
    invalidate_slam as motion_invalidate_slam,
    clear_object_tracks as motion_clear_object_tracks,
    pause_for_turn_progress as motion_pause_for_turn_progress,
    start_turn_progress_window as motion_start_turn_progress_window,
    measured_turn_progress as motion_measured_turn_progress,
    record_turn_progress as motion_record_turn_progress,
    send_actual as motion_send_actual,
    set_motion as motion_set_motion,
    send as motion_send,
    send_manual as motion_send_manual,
    send_path as motion_send_path,
    set_state as motion_set_state,
    route_turn_direction as motion_route_turn_direction,
    open_side as motion_open_side,
    call_reset_service as motion_call_reset_service,
    begin_recovery as motion_begin_recovery,
    recent_forward_retreat_is_clear as motion_recent_forward_retreat_is_clear,
    start_look as motion_start_look,
    start_navigation as motion_start_navigation,
    plan as motion_plan,
    follow_path as motion_follow_path,
    command_lidar_clearance as motion_command_lidar_clearance,
    safe_step as motion_safe_step,
    control_step as motion_control_step,
    swept_circle_clearance,
)
from object_mapping import TensorRtYoloDetector, YoloOnnxDetector, load_labels

try:
    from jethexa_controller import client
except ImportError:
    client = None



class RoomMapper:
    def __init__(self):
        initialize_mapper(self)

    battery_callback = sensors_battery_callback
    image_callback = sensors_image_callback
    message_time = staticmethod(sensors_message_time)
    registered_depth_callback = sensors_registered_depth_callback
    depth_map_camera_info_callback = sensors_depth_map_camera_info_callback
    update_depth_obstacle_map = sensors_update_depth_obstacle_map
    camera_info_callback = sensors_camera_info_callback
    tf_callback = sensors_tf_callback
    ensure_tf_listener = sensors_ensure_tf_listener
    update_pose_from_tf = sensors_update_pose_from_tf
    scan_callback = sensors_scan_callback
    update_lidar_fusion = sensors_update_lidar_fusion
    scan_ready_for_mapping = sensors_scan_ready_for_mapping
    depth_callback = sensors_depth_callback
    learn_floor = sensors_learn_floor
    depth_ahead = sensors_depth_ahead
    map_callback = sensors_map_callback
    map_heartbeat_callback = sensors_map_heartbeat_callback
    map_is_fresh = sensors_map_is_fresh
    pose_callback = sensors_pose_callback

    object_loop = objects_object_loop
    objects_ready = objects_objects_ready
    object_commit_ready = objects_object_commit_ready
    capture_object_snapshot = objects_capture_object_snapshot
    enqueue_object_snapshot = objects_enqueue_object_snapshot
    capture_preview_snapshot = objects_capture_preview_snapshot
    capture_motion_preview_snapshot = objects_capture_motion_preview_snapshot
    novel_object_direction = staticmethod(objects_novel_object_direction)
    request_novel_object_verification_locked = objects_request_novel_object_verification_locked
    begin_novel_object_verification = objects_begin_novel_object_verification
    finish_novel_object_verification = objects_finish_novel_object_verification
    capture_novel_object_snapshot = objects_capture_novel_object_snapshot
    capture_endpoint_snapshot = objects_capture_endpoint_snapshot
    process_objects = objects_process_objects
    process_object_snapshot = objects_process_object_snapshot
    update_object_focus_locked = objects_update_object_focus_locked
    finish_object_focus_locked = objects_finish_object_focus_locked

    web_request = web_web_request
    stop_manual = web_stop_manual
    cancel_navigation = web_cancel_navigation
    reset_mapping = web_reset_mapping
    handle_web = web_handle_web
    web_status = web_web_status
    web_battery = web_web_battery
    web_speed = web_web_speed

    invalidate_slam = motion_invalidate_slam
    clear_object_tracks = motion_clear_object_tracks
    pause_for_turn_progress = motion_pause_for_turn_progress
    start_turn_progress_window = motion_start_turn_progress_window
    measured_turn_progress = motion_measured_turn_progress
    record_turn_progress = motion_record_turn_progress
    send_actual = motion_send_actual
    set_motion = motion_set_motion
    send = motion_send
    send_manual = motion_send_manual
    send_path = motion_send_path
    set_state = motion_set_state
    route_turn_direction = motion_route_turn_direction
    open_side = motion_open_side
    call_reset_service = motion_call_reset_service
    begin_recovery = motion_begin_recovery
    recent_forward_retreat_is_clear = motion_recent_forward_retreat_is_clear
    start_look = motion_start_look
    start_navigation = motion_start_navigation
    plan = motion_plan
    follow_path = motion_follow_path
    command_lidar_clearance = staticmethod(motion_command_lidar_clearance)
    safe_step = motion_safe_step
    control_step = motion_control_step





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
                    detector = None
                    try:
                        detector = self.make_yolo_detector(path, labels, backend, thresholds)
                        detector.warmup()
                        self.detector = detector
                        self.active_yolo_model = path
                        self.active_yolo_backend = backend
                        break
                    except Exception as exc:
                        last_error = exc
                        close = getattr(detector, "close", None)
                        if close is not None:
                            close()
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

    def make_yolo_detector(self, path, labels, backend, thresholds):
        options = dict(labels=labels, input_size=self.yolo_input_size,
                       confidence=self.object_confidence,
                       class_thresholds=thresholds,
                       allowed_labels=self.object_classes)
        if backend == "tensorrt_fp16":
            return TensorRtYoloDetector(path, self.yolo_engine, **options)
        return YoloOnnxDetector(path, backend=backend, **options)

    def fallback_yolo_to_cpu(self, reason):
        """Permanently replace a failed optional GPU detector with CPU inference."""
        failed_backend = self.active_yolo_backend
        previous = self.detector
        failures = []
        candidates = [self.active_yolo_model]
        if self.yolo_fallback_model not in candidates:
            candidates.append(self.yolo_fallback_model)
        thresholds = {label: self.object_priority_confidence
                      for label in self.object_priority_classes}
        for index, path in enumerate(candidates):
            if not path or not os.path.isfile(path):
                failures.append("missing %s" % path)
                continue
            try:
                explicit_labels = self.yolo_labels if path == self.yolo_model else ""
                sidecar = explicit_labels or os.path.splitext(path)[0] + ".names"
                labels = load_labels(sidecar) if os.path.isfile(sidecar) else None
                detector = self.make_yolo_detector(path, labels, "cpu", thresholds)
                detector.warmup()
                self.detector = detector
                self.active_yolo_model = path
                self.active_yolo_backend = "cpu"
                close = getattr(previous, "close", None)
                if close is not None:
                    close()
                rospy.logerr("YOLO %s failed (%s); permanently switched to CPU model %s",
                             failed_backend, reason, path)
                return
            except Exception as exc:
                failures.append("%s: %s" % (path, exc))
        self.detector = None
        self.active_yolo_model = ""
        self.active_yolo_backend = ""
        close = getattr(previous, "close", None)
        if close is not None:
            close()
        raise RuntimeError("GPU inference failed and CPU rollback failed: " +
                           "; ".join(failures))


































































    def shutdown(self):
        recorder = getattr(self, "video_recorder", None)
        if recorder is not None:
            recorder.close()
        with self.control_lock:
            self.shutting_down = True
            if hasattr(self, "object_queue_lock"):
                with self.object_queue_lock:
                    self.object_pending_snapshots = []
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
        if object_thread is None or not object_thread.is_alive():
            detector = getattr(self, "detector", None)
            close = getattr(detector, "close", None)
            if close is not None:
                close()
        rospy.loginfo("room_mapper shutdown; no further controller commands")


if __name__ == "__main__":
    node = RoomMapper()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
