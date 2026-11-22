"""Sensor ingestion, depth/LiDAR fusion, and SLAM pose callbacks."""

import math
import struct
import time

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState

from depth_mapping import project_depth_cells
from lidar_mapping import project_lidar_cells
from map_explore import ExploreMap
from mapper_motion import swept_circle_clearance
from object_mapping import TransformCache, decode_depth_image


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


def message_time(msg):
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    try:
        value = stamp.to_sec()
        return value if math.isfinite(value) and value > 0 else 0.0
    except Exception:
        return 0.0


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
    free_surface_max = getattr(
        self, 'depth_map_free_surface_max', self.depth_map_range_max)
    valid_fraction = float(np.count_nonzero(
        np.isfinite(depth) & (depth >= self.depth_map_range_min) &
        (depth <= free_surface_max))) / max(1, depth.size)
    if valid_fraction < getattr(self, 'depth_map_min_valid_fraction', .10):
        # Sparse registered depth can turn isolated noise into retained
        # planner obstacles. Supplying no observation preserves existing
        # obstacles until trustworthy depth and LiDAR can clear them.
        self.depth_map_status = ("registered depth too sparse for planner "
                                 "update (%.1f%% valid)" % (100 * valid_fraction))
        return
    frame = getattr(getattr(msg, "header", None), "frame_id", "").lstrip('/')
    stamp = self.message_time(msg)
    if not frame or stamp <= 0:
        self.depth_map_status = "registered depth has no valid frame/time"
        return
    settled = (not getattr(self, 'commanded_moving', False) and
               now >= getattr(self, 'settle_until', 0.0))
    tf_max_age = (getattr(self, 'depth_map_settled_tf_max_age', 1.0)
                  if settled else .5)
    try:
        with self.object_lock:
            transform = self.tf_buffer.lookup(
                self.map_frame, frame, stamp, max_age=tf_max_age)
        occupied_cells, observed_free_cells = project_depth_cells(
            depth, k, transform, cell=self.explorer.cell,
            decimation=self.depth_map_decimation,
            range_min=self.depth_map_range_min, range_max=self.depth_map_range_max,
            height_min=self.depth_map_height_min, height_max=self.depth_map_height_max,
            min_points=self.depth_map_min_points,
            protected_range=self.depth_map_protected_range,
            free_surface_max=free_surface_max)
        with self.map_lock:
            lidar_stamp = getattr(self, 'lidar_fusion_stamp', 0.0)
            lidar_age = abs(stamp - lidar_stamp)
            paired = (settled and lidar_stamp > 0 and
                      lidar_age <= getattr(self, 'depth_lidar_sync_tolerance', .35) and
                      bool(getattr(self, 'lidar_fusion_occupied', []) or
                           getattr(self, 'lidar_fusion_free', [])))
            if paired:
                result = self.explorer.update_fused_observations(
                    occupied_cells, observed_free_cells,
                    getattr(self, 'lidar_fusion_occupied', []),
                    getattr(self, 'lidar_fusion_free', []),
                    now=now, ttl=self.depth_map_ttl,
                    temporary_clear_confirmations=self.depth_map_clear_observations,
                    promotion_confirmations=getattr(
                        self, 'depth_map_promotion_observations', 3),
                    joint_clear_confirmations=getattr(
                        self, 'depth_map_joint_clear_observations', 4),
                    joint_clear_evidence_ttl=getattr(
                        self, 'depth_map_joint_clear_evidence_ttl', 5.0),
                    minimum_hold=getattr(
                        self, 'depth_map_minimum_hold', 10.0),
                    unconfirmed_max_age=getattr(
                        self, 'depth_map_unconfirmed_max_age', 60.0))
                cleared = (result['cleared_temporary'] +
                           result['cleared_persistent'] +
                           result['suppressed_slam'])
            else:
                result = {'promoted': 0, 'cleared_persistent': 0,
                          'suppressed_slam': 0, 'joint_free': 0,
                          'joint_clear_candidates': 0,
                          'joint_clear_max_evidence': 0,
                          'depth_free': 0, 'lidar_free': 0,
                          'persistent_depth_free': 0,
                          'persistent_lidar_free': 0,
                          'persistent_joint_exact': 0,
                          'persistent_joint_agreement': 0,
                          'persistent_occupied_guard': 0,
                          'expired_temporary': 0}
                cleared = self.explorer.update_depth_obstacles(
                    occupied_cells, now, self.depth_map_ttl,
                    observed_free_cells=observed_free_cells,
                    clear_confirmations=self.depth_map_clear_observations,
                    minimum_hold=getattr(
                        self, 'depth_map_minimum_hold', 10.0),
                    unconfirmed_max_age=getattr(
                        self, 'depth_map_unconfirmed_max_age', 60.0))
                result['expired_temporary'] = getattr(
                    self.explorer, 'depth_obstacle_last_expired', 0)
            count = self.explorer.depth_obstacle_count(now)
            persistent = (self.explorer.persistent_obstacle_count()
                          if isinstance(self.explorer, ExploreMap) else 0)
            suppressed = (self.explorer.suppressed_slam_count()
                          if isinstance(self.explorer, ExploreMap) else 0)
        self.t_depth_map = now
        self.depth_map_frame = frame
        self.depth_map_last_cleared = cleared
        self.depth_map_status = (
            "%d temporary obstacle cells (%.0f s hold, %.0f s unconfirmed limit); "
            "cleared %d, expired %d this update; "
            "%d persistent depth, %d joint-cleared Hector cells; promoted %d; "
            "joint free %d, clear evidence %d cells/max %d; "
            "free D/L %d/%d, persistent D/L/exact/paired/guarded "
            "%d/%d/%d/%d/%d%s" %
            (count, self.depth_map_minimum_hold,
              getattr(self, 'depth_map_unconfirmed_max_age', 60.0),
              cleared, (result.get('expired_temporary', 0)
                        if isinstance(result.get('expired_temporary', 0),
                                      (int, float)) else 0),
              persistent, suppressed,
              result['promoted'], result['joint_free'],
              result['joint_clear_candidates'], result['joint_clear_max_evidence'],
              result['depth_free'], result['lidar_free'],
              result['persistent_depth_free'], result['persistent_lidar_free'],
              result['persistent_joint_exact'],
              result['persistent_joint_agreement'],
              result['persistent_occupied_guard'],
             "" if paired else "; waiting for settled synchronized LiDAR"))
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
    fusion_ready = False
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
                fusion_ready = True
            elif self.commanded_moving:
                self.stable_turn_scans += 1
        elif self.commanded_moving:
            self.rejected_motion_scans += 1
    if fusion_ready and getattr(self, "use_depth_map", False):
        self.update_lidar_fusion(msg)


def update_lidar_fusion(self, msg):
    """Cache occupied/free cells from one settled timestamped LiDAR scan."""
    frame = getattr(getattr(msg, "header", None), "frame_id", "").lstrip('/')
    stamp = self.message_time(msg)
    if not frame or stamp <= 0 or self.tf_buffer is None:
        return
    try:
        with self.object_lock:
            transform = self.tf_buffer.lookup(self.map_frame, frame, stamp, max_age=.5)
        occupied, observed_free = project_lidar_cells(
            msg.ranges, msg.angle_min, msg.angle_increment, transform,
            cell=self.explorer.cell,
            range_min=max(float(msg.range_min), self.self_filter),
            range_max=self.depth_map_range_max,
            decimation=self.lidar_fusion_decimation,
            min_rays=self.lidar_fusion_min_rays,
            yaw_offset=self.yaw_offset)
        with self.map_lock:
            self.lidar_fusion_occupied = occupied
            self.lidar_fusion_free = observed_free
            self.lidar_fusion_stamp = stamp
    except (KeyError, ValueError) as exc:
        rospy.logwarn_throttle(5, "LiDAR fusion projection unavailable: %s", exc)


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
        if values.size > getattr(self, 'floor_row_min_fraction', 0.10) * cols.shape[1]:
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
    upper_sparse = values.size < min_valid
    floor_rows_available = bool(
        self.floor_profile is not None and not self.floor_unavailable and
        np.isfinite(self.floor_profile[split:bottom]).any())
    if upper_sparse and not floor_rows_available:
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
    self.depth_health = ("ready (lower-floor + LiDAR; upper sparse)"
                         if upper_sparse else "ready")
    ignore_distance = getattr(self, "depth_ignore_at_or_below", 0.0)
    obstacle_values = values[values > ignore_distance]
    nearest = (float(np.percentile(obstacle_values, 10))
               if not upper_sparse and obstacle_values.size >= 30
               else float("inf"))
    source = "high" if nearest < 99 else (
        "lower-floor + LiDAR" if upper_sparse else "")
    sectors = {}
    names = ("left", "center", "right")
    edges = np.linspace(0, top.shape[1], 4, dtype=int)
    for index, name in enumerate(names):
        part = top[:, edges[index]:edges[index + 1]]
        part_values = part[(part > ignore_distance) & np.isfinite(part)]
        required = max(20, int(getattr(
            self, "min_depth_valid_fraction", 0.02) * part.size))
        sectors[name] = (float(np.percentile(part_values, 10))
                         if not upper_sparse and part_values.size >= required
                         else float("inf"))
    if self.floor_profile is not None:
        low = depth[split:bottom, c0:c1]
        floor = self.floor_profile[split:bottom][:, None]
        valid_floor = np.isfinite(floor)
        valid = (low > 0.1) & valid_floor
        floor_pixels = np.broadcast_to(valid_floor, low.shape)
        floor_pixel_count = np.count_nonzero(floor_pixels)
        valid_count = np.count_nonzero(valid)
        valid_fraction = (float(valid_count) / floor_pixel_count
                          if floor_pixel_count else 0.0)
        sparse_floor = (
            not upper_sparse and
            getattr(self, "min_depth_valid_fraction", .02) <= valid_fraction <
            getattr(self, "floor_obstacle_min_valid_fraction", .10))
        if sparse_floor:
            # A small but usable set of lower-image returns is common on
            # dark or reflective floors. It is not dense enough to infer a
            # floor bump or dropout, so retain upper depth plus LiDAR.
            self.depth_health = "ready (upper + LiDAR; floor too sparse)"
            if not source:
                source = "upper + LiDAR (floor sparse)"
        if floor_pixel_count and not sparse_floor:
            zero_frac = float(np.count_nonzero(floor_pixels & ~valid)) / np.count_nonzero(floor_pixels)
            if zero_frac - self.floor_zero_frac > 0.30:
                nearest, source = 0.30, "depth dropout / too close"
                sectors["center"] = min(sectors["center"], nearest)
        if valid_count > 100 and not sparse_floor:
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
                candidate = getattr(self, "pose_jump_candidate", None)
                if candidate is not None:
                    candidate_distance = math.hypot(
                        pose[0] - candidate[0], pose[1] - candidate[1])
                    candidate_turn = abs(math.atan2(
                        math.sin(pose[2] - candidate[2]),
                        math.cos(pose[2] - candidate[2])))
                    consistent = (
                        candidate_distance <= self.max_pose_jump * .5 and
                        candidate_turn <= self.max_yaw_jump * .5)
                else:
                    consistent = False
                count = (getattr(self, "pose_jump_candidate_count", 0) + 1
                         if consistent else 1)
                self.pose_jump_candidate = pose
                self.pose_jump_candidate_count = count
                required = getattr(self, "pose_jump_confirmations", 2)
                if count < required:
                    rospy.logwarn(
                        "Possible SLAM pose jump %.2f m / %.0f deg; "
                        "waiting for confirmation %d/%d",
                        distance, math.degrees(turn), count, required)
                    return
                self.pose_jump_candidate = None
                self.pose_jump_candidate_count = 0
                self.invalidate_slam(
                    "pose jumped %.2f m / %.0f deg in %d consistent samples" %
                    (distance, math.degrees(turn), required))
            else:
                self.pose_jump_candidate = None
                self.pose_jump_candidate_count = 0
        else:
            self.pose_jump_candidate = None
            self.pose_jump_candidate_count = 0
        self.prev_pose = pose
        self.pose = pose
        self.t_pose = time.time()
        self.pose_update_count = getattr(self, "pose_update_count", 0) + 1
        if hasattr(self, "survey"):
            self.survey.localize(self.message_time(msg))
        if getattr(self, "web_map_center", None) is None:
            self.web_map_center = pose[:2]

