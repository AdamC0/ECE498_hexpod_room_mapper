"""Run on the robot: python3 -m unittest -v test_room_mapper.

No ROS node is started and no motion commands are published.
"""
import math
import threading
import time
import unittest
import struct
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from room_mapper import RoomMapper, SEARCH, HOLD, AVOID, FAULT, swept_circle_clearance
from object_mapping import ObjectTracker
from std_srvs.srv import Empty


class MapperTests(unittest.TestCase):
    def setUp(self):
        logger = patch('room_mapper.rospy.logwarn_throttle')
        logger.start()
        self.addCleanup(logger.stop)
        error_logger = patch('room_mapper.rospy.logerr')
        error_logger.start()
        self.addCleanup(error_logger.stop)
        n = self.node = RoomMapper.__new__(RoomMapper)
        n.band_top, n.band_split, n.band_bottom = 0.2, 0.55, 0.95
        n.band_left, n.band_right = 0.2, 0.8
        n.floor_margin, n.floor_frames_needed = 0.15, 3
        n.floor_profile = n.floor_sum = n.floor_count = n.floor_zero_sum = None
        n.floor_frames, n.floor_zero_frac = 0, 0.0
        n.floor_failures, n.floor_unavailable = 0, False
        n.depth, n.depth_source, n.depth_health = None, '', 'waiting'
        n.depth_ignore_at_or_below = 0.0
        n.depth_sectors = {"left": float("inf"), "center": float("inf"),
                           "right": float("inf")}
        n.last_valid_depth_sectors = dict(n.depth_sectors)
        n.depth_history = []
        n.data_lock = threading.Lock()
        n.floor = np.full((100, 100), 2.0, np.float32)
        n.floor[55:95] = 1.0

    def calibrate(self):
        for _ in range(self.node.floor_frames_needed):
            self.node.learn_floor(self.node.floor)

    def controller(self):
        n = self.node
        self.calibrate()
        n.depth = n.floor.copy()
        n.pose, n.lidar = (0, 0, 0), dict(front=2., left=2., right=2., back=2.,
                                                near_back=2., nearest=2., free=np.ones(36)*2)
        n.t_pose = n.t_scan = n.t_camera = n.t_depth = n.t_map = time.time()
        n.t_map_heartbeat = 0.
        n.map_heartbeat_topic, n.map_stale = '', 5.
        n.use_map = n.use_depth = True
        n.explorer = SimpleNamespace(have_map=True)
        n.state, n.phase, n.phase_start = SEARCH, 'go', time.time()
        n.path, n.view, n.paused = [(1, 0)], {}, False
        n.manual_cmd, n.manual_until, n.manual_deadman = None, 0., .55
        n.depth_stop, n.lidar_stop, n.body_radius = 0.60, 0.40, 0.25
        n.reverse_seconds = .5
        n.reverse_clearance_margin = .02
        n.reverse_after_turn_seconds = 2.5
        n.stride, n.step_time = 40., .5
        n.translation_speed_percent = n.rotation_speed_percent = 100.
        n.slam_stale, n.map_trusted = 2.5, True
        n.drive, n.pose_guard_start = False, 0.
        n.commanded_moving, n.last_cmd = False, None
        n.spin_stride = 0.0
        n.waypoint_tolerance = .12
        n.handle_web, n.send, n.send_manual = Mock(), Mock(), Mock()
        n.send_path = Mock()
        return n

    def test_path_motion_continues_and_changes_direction_without_timed_stop(self):
        n = self.node
        n.drive, n.commanded_moving = True, True
        n.last_cmd, n.settle_until = ('forward',), 0.
        n.pose, n.motion_until = (0., 0., 0.), 1.
        n.spin_burst = .5
        n.send_actual, n.set_motion = Mock(), Mock()
        n.object_processing, n.object_detections = True, [{'label': 'chair'}]
        with patch('room_mapper.time.time', return_value=100.):
            n.send_path(('forward',))
            n.send_actual.assert_called_with(('forward',))
            n.send_path(('spin', 1.))
        self.assertEqual(n.send_actual.call_args_list,
                         [unittest.mock.call(('forward',)), unittest.mock.call(('spin', 1.))])

    def test_continuous_path_turn_faults_after_repeated_no_progress(self):
        n = self.node
        n.commanded_moving, n.last_cmd = True, ('spin', 1.)
        n.settle_until, n.motion_until = 0., 0.
        n.pending_spin_yaw, n.pose = 0., (0., 0., 0.)
        n.spin_burst, n.min_spin_progress, n.max_spin_stalls = .5, .05, 3
        n.send_actual, n.set_motion, n.send = Mock(), Mock(), Mock()
        n.clear_object_tracks = Mock()
        for now in (1., 2., 3.):
            with patch('room_mapper.time.time', return_value=now):
                n.send_path(('spin', 1.))
        n.send.assert_called_once_with(('stop',))
        self.assertFalse(n.map_trusted)

    def test_floor_and_body_bob_are_clear(self):
        self.calibrate()
        self.assertGreater(self.node.depth_ahead(self.node.floor), .6)
        self.assertGreater(self.node.depth_ahead(self.node.floor * .8), .6)

    def test_float_battery_voltage_is_converted_to_percentage(self):
        n = self.node
        n.battery_empty_voltage, n.battery_full_voltage = 9.8, 12.6
        n.battery_voltage = n.battery_percentage = n.battery_charging = None
        n.battery_stamp = 0.0
        n.battery_source = 'waiting'
        raw = SimpleNamespace(
            _connection_header={'type': 'std_msgs/Float32'},
            _buff=struct.pack('<f', 11.2))
        n.battery_callback(raw, '/voltage')
        self.assertAlmostEqual(n.battery_voltage, 11.2, places=4)
        self.assertAlmostEqual(n.battery_percentage, 50.0, places=3)
        self.assertEqual(n.battery_source, '/voltage')
        self.assertTrue(n.web_battery()['available'])

    def test_millivolt_battery_value_is_supported(self):
        n = self.node
        n.battery_empty_voltage, n.battery_full_voltage = 9.8, 12.6
        n.battery_voltage = n.battery_percentage = n.battery_charging = None
        n.battery_stamp = 0.0
        n.battery_source = 'waiting'
        raw = SimpleNamespace(
            _connection_header={'type': 'std_msgs/UInt16'},
            _buff=struct.pack('<H', 12600))
        n.battery_callback(raw, '/battery')
        self.assertEqual(n.battery_voltage, 12.6)
        self.assertEqual(n.battery_percentage, 100.0)

    def test_pending_object_cannot_start_navigation(self):
        n = self.node
        n.object_lock = threading.Lock()
        n.object_tracker = ObjectTracker(priority_labels=['chair'])
        n.object_tracker.update('chair', 1., 2., .75, time.time(), observation_id=1,
                                validation_group=('endpoint', 1),
                                box=(10, 10, 60, 80))
        n.web_pending = [('navigate_object', 1)]
        n.stop_manual, n.start_navigation = Mock(), Mock()
        n.handle_web()
        n.start_navigation.assert_not_called()
        n.stop_manual.assert_called_once_with(
            'Object navigation blocked: landmark is still pending')

    def test_close_elevated_obstacle_is_detected(self):
        self.calibrate()
        frame = self.node.floor.copy()
        frame[20:55, 20:80] = .50
        self.assertAlmostEqual(self.node.depth_ahead(frame), .50, places=5)

    def test_close_floor_bump_is_detected(self):
        self.calibrate()
        frame = self.node.floor.copy()
        frame[60:80, 40:60] = .4
        self.assertAlmostEqual(self.node.depth_ahead(frame), .4, places=5)
        self.assertEqual(self.node.depth_source, 'floor-bump')

    def test_floor_dropout_sentinel_is_detected(self):
        self.calibrate()
        frame = self.node.floor.copy()
        frame[55:95, 20:80] = 0
        self.assertAlmostEqual(self.node.depth_ahead(frame), .30, places=5)
        self.assertEqual(self.node.depth_source, 'depth dropout / too close')

    def test_invalid_calibration_retries(self):
        for _ in range(3):
            self.node.learn_floor(np.zeros((100, 100), np.float32))
        self.assertIsNone(self.node.floor_profile)
        self.calibrate()
        self.assertIsNotNone(self.node.floor_profile)

    def test_floor_calibration_falls_back_to_upper_only(self):
        for _ in range(self.node.floor_frames_needed * 3):
            self.node.learn_floor(np.zeros((100, 100), np.float32))
        self.assertTrue(self.node.floor_unavailable)
        self.assertIsNotNone(self.node.floor_profile)
        self.assertTrue(self.node.depth_health.startswith('ready'))

    def test_invalid_upper_depth_is_unknown(self):
        self.calibrate()
        self.node.depth_ahead(np.zeros((100, 100), np.float32))
        self.assertNotEqual(self.node.depth_health, 'ready')

    def test_brief_sparse_depth_reuses_recent_result_then_faults(self):
        n = self.node
        n.depth_sparse_grace = 1.5
        n.last_valid_depth_near = .55
        n.last_valid_depth_source = 'high'
        n.last_valid_depth_time = 100.
        n.depth_sparse_since = None
        empty = np.zeros((100, 100), np.float32)
        with patch('room_mapper.time.time', return_value=100.5):
            self.assertAlmostEqual(n.depth_ahead(empty), .55)
            self.assertTrue(n.depth_health.startswith('ready'))
        with patch('room_mapper.time.time', return_value=102.):
            self.assertGreater(n.depth_ahead(empty), .6)
            self.assertEqual(n.depth_health, 'insufficient valid depth in upper band')

    def test_sparse_but_sufficient_upper_depth_remains_usable(self):
        n = self.node
        n.floor_profile = np.full(240, np.nan, np.float32)
        n.floor_unavailable = True
        n.min_depth_valid_fraction = .02
        n.min_depth_valid_pixels = 200
        frame = np.zeros((240, 320), np.float32)
        # 625 current range samples are only 3.9% of the upper collision band,
        # but are ample for the robust tenth-percentile obstacle estimate.
        frame[55:80, 100:125] = .55
        self.assertAlmostEqual(n.depth_ahead(frame), .55, places=5)
        self.assertTrue(n.depth_health.startswith('ready'))

    def test_padded_big_endian_depth(self):
        raw = np.array([[1000, 2000, 999], [3000, 4000, 999]], dtype='>u2')
        msg = SimpleNamespace(encoding='16UC1', is_bigendian=True,
                              width=2, height=2, step=6, data=raw.tobytes())
        self.node.learn_floor = Mock()
        self.node.depth_callback(msg)
        self.assertAlmostEqual(float(self.node.depth[0, 0]), 1.0)

    def test_float_depth_nan_is_invalid(self):
        raw = np.array([[np.nan, 2], [3, 4]], dtype='<f4')
        msg = SimpleNamespace(encoding='32FC1', is_bigendian=False,
                              width=2, height=2, step=8, data=raw.tobytes())
        self.node.learn_floor = Mock()
        self.node.depth_callback(msg)
        self.assertEqual(self.node.depth[0, 0], 0)

    def test_obstacle_stops_in_same_cycle_even_with_clear_lidar(self):
        n = self.controller()
        n.depth[20:55, 20:80] = .55
        n.lidar['nearest'] = .29
        n.lidar['back'] = n.lidar['near_back'] = .30
        n.control_step()
        n.send.assert_called_with(('stop',))
        self.assertEqual(n.state, HOLD)
        self.assertEqual(n.path, [])

    def test_new_fused_obstacle_stops_and_replans_active_path(self):
        n = self.controller()
        n.explorer.path_prefix_clear = Mock(return_value=False)
        n.follow_path = Mock(return_value=('forward',))
        n.control_step()
        n.explorer.path_prefix_clear.assert_called_once_with(n.pose, [(1, 0)])
        n.send.assert_called_once_with(('stop',))
        self.assertEqual(n.state, HOLD)
        self.assertEqual(n.phase, 'plan')
        self.assertEqual(n.path, [])
        self.assertIn('newly mapped obstacle', n.web_msg)

    def test_manual_forward_bypasses_autonomous_pipeline_and_keeps_auto_paused(self):
        n = self.controller()
        n.manual_cmd, n.manual_until, n.paused = 'forward', time.time() + .5, True
        n.control_step()
        n.send_manual.assert_called_once_with(('forward',))
        n.send.assert_not_called()
        self.assertTrue(n.paused)

    def test_manual_forward_does_not_depend_on_rgb_stream(self):
        n = self.controller()
        n.manual_cmd, n.manual_until, n.paused = 'forward', time.time() + .5, True
        n.t_camera = 0.
        n.control_step()
        n.send_manual.assert_called_once_with(('forward',))

    def test_manual_forward_bypasses_depth_obstacle_gate(self):
        n = self.controller()
        n.manual_cmd, n.manual_until, n.paused = 'forward', time.time() + .5, True
        n.depth[20:55, 20:80] = .55
        n.control_step()
        n.send_manual.assert_called_once_with(('forward',))
        self.assertEqual(n.manual_cmd, 'forward')
        self.assertIn('gates bypassed', n.web_msg)

    def test_manual_forward_bypasses_stale_sensors_map_and_startup_guard(self):
        n = self.controller()
        n.manual_cmd, n.manual_until, n.paused = 'forward', time.time() + .5, True
        n.pose = n.lidar = n.depth = None
        n.t_pose = n.t_scan = n.t_depth = n.t_map = 0.
        n.map_trusted = False
        n.pose_guard_start = time.time() + 10.
        n.control_step()
        n.send_manual.assert_called_once_with(('forward',))
        n.send.assert_not_called()

    def test_manual_turn_is_allowed_when_only_forward_path_is_blocked(self):
        n = self.controller()
        n.manual_cmd, n.manual_until, n.paused = 'left', time.time() + .5, True
        n.depth[20:55, 20:80] = .55
        n.control_step()
        n.send_manual.assert_called_once_with(('spin', 1.0))

    def test_manual_strafe_reaches_omnidirectional_gait_command(self):
        n = self.controller()
        n.manual_cmd, n.manual_until, n.paused = 'strafe_left', time.time() + .5, True
        n.control_step()
        n.send_manual.assert_called_once_with(('strafe_left',))

    def test_autonomy_immediately_turns_away_from_front_obstacle(self):
        n = self.controller()
        n.depth[20:55, 20:80] = .55
        n.lidar['left'], n.lidar['right'] = 2., 1.
        n.control_step()
        self.assertEqual(n.state, AVOID)
        n.send.assert_called_with(('spin', 1.0), interrupt_object=True)

    def test_selected_route_is_retained_while_turning_around_obstacle(self):
        n = self.controller()
        n.phase = 'user_go'
        n.user_goal = (1., 0., 'desk', .75)
        n.nav_goal = (.5, 0.)
        n.depth[20:55, 20:80] = .55
        n.lidar['left'], n.lidar['right'] = 2., 1.
        n.control_step()
        self.assertEqual(n.state, AVOID)
        self.assertEqual(n.phase, 'user_go')
        self.assertEqual(n.user_goal, (1., 0., 'desk', .75))
        n.send.assert_called_with(('spin', 1.0), interrupt_object=True)

    def test_selected_route_turn_direction_overrides_wider_lidar_side(self):
        n = self.controller()
        n.phase = 'user_go'
        n.path = [(0., 0.), (0., -1.)]
        n.user_goal = (0., -1., 'door', 0.)
        n.nav_goal = (0., -1.)
        n.depth[20:55, 20:80] = .55
        n.lidar['left'], n.lidar['right'] = 2., 1.
        n.control_step()
        self.assertEqual(n.state, AVOID)
        self.assertEqual(n.avoid_dir, -1.)
        n.send.assert_called_with(('spin', -1.0), interrupt_object=True)

    def test_depth_sectors_choose_clearer_side_without_route_heading(self):
        n = self.controller()
        n.path = []
        n.depth_sectors = {'left': .45, 'center': .45, 'right': 1.5}
        n.lidar['left'], n.lidar['right'] = 2., 2.
        self.assertEqual(n.open_side(n.lidar, n.pose), -1.)

    def test_route_preference_does_not_turn_into_blocked_depth_side(self):
        n = self.controller()
        n.path = [(0., -1.)]
        n.depth_sectors = {'left': 1.5, 'center': .45, 'right': .25}
        n.lidar['left'], n.lidar['right'] = 2., 2.
        self.assertEqual(n.open_side(n.lidar, n.pose), 1.)

    def test_too_close_front_obstacle_uses_clear_reverse_route(self):
        n = self.controller()
        n.depth[20:55, 20:80] = .55
        n.lidar['nearest'] = .29
        n.control_step()
        self.assertEqual(n.state, AVOID)
        self.assertEqual(n.recovery_action, 'back')
        n.send.assert_called_with(('back',), interrupt_object=True)

    def test_selected_route_reverses_without_losing_goal(self):
        n = self.controller()
        n.phase = 'user_go'
        n.user_goal = (1., 0., 'desk', .75)
        n.nav_goal = (.5, 0.)
        n.depth[20:55, 20:80] = .55
        n.lidar['nearest'] = .29
        n.control_step()
        self.assertEqual(n.state, AVOID)
        self.assertEqual(n.phase, 'user_go')
        self.assertEqual(n.recovery_action, 'back')
        self.assertEqual(n.user_goal, (1., 0., 'desk', .75))
        n.send.assert_called_with(('back',), interrupt_object=True)

    def test_persistent_corner_block_escalates_turn_to_reverse(self):
        n = self.controller()
        n.depth[20:55, 20:80] = .55
        n.state = AVOID
        n.recovery_action = 'spin'
        n.avoid_dir = 1.0
        n.recovery_start = time.time() - 3.
        n.control_step()
        self.assertEqual(n.recovery_action, 'back')
        self.assertIn('escape corner', n.web_msg)
        n.send.assert_called_with(('back',), interrupt_object=True)

    def test_corner_escape_does_not_reverse_without_rear_clearance(self):
        n = self.controller()
        n.depth[20:55, 20:80] = .55
        n.lidar['back'] = n.lidar['near_back'] = .45
        n.state = AVOID
        n.recovery_action = 'spin'
        n.avoid_dir = -1.0
        n.recovery_start = time.time() - 3.
        n.control_step()
        self.assertEqual(n.recovery_action, 'spin')
        n.send.assert_called_with(('spin', n.avoid_dir), interrupt_object=True)

    def test_rear_corner_uses_swept_circle_instead_of_radial_cutoff(self):
        corner_angle = math.radians(-150.)
        corner = np.array([[.596 * math.cos(corner_angle),
                            .596 * math.sin(corner_angle)]])
        direct = np.array([[-.55, 0.]])
        self.assertGreater(swept_circle_clearance(corner, math.pi, .45), .17)
        self.assertAlmostEqual(swept_circle_clearance(direct, math.pi, .45), .10)

    def test_speed_scaled_rear_sweep_allows_corner_but_not_short_path(self):
        n = self.controller()
        n.translation_speed_percent = 200.
        n.depth[20:55, 20:80] = .55
        n.lidar['nearest'] = .29
        n.lidar['rear_path_clearance'] = .17
        n.control_step()
        self.assertEqual(n.recovery_action, 'back')

        n = self.controller()
        n.translation_speed_percent = 200.
        n.depth[20:55, 20:80] = .55
        n.lidar['nearest'] = .29
        n.lidar['rear_path_clearance'] = .09
        n.control_step()
        self.assertEqual(n.state, HOLD)
        self.assertIsNone(n.recovery_action)

    def test_depth_dropout_still_prohibits_swept_clear_reverse(self):
        n = self.controller()
        n.floor_zero_frac = 0.
        n.depth[55:95, 20:80] = 0.
        n.lidar['nearest'] = .29
        n.lidar['rear_path_clearance'] = 2.
        n.control_step()
        self.assertEqual(n.depth_source, 'depth dropout / too close')
        self.assertEqual(n.state, HOLD)
        self.assertIsNone(n.recovery_action)

    def test_manual_deadman_stops_expired_command(self):
        n = self.controller()
        n.manual_cmd, n.manual_until, n.paused = 'forward', time.time() - .1, True
        n.control_step()
        n.send.assert_called_with(('stop',))
        n.send_manual.assert_not_called()
        self.assertIsNone(n.manual_cmd)
        self.assertIn('timed out', n.web_msg)

    def test_manual_command_is_armed_while_autonomy_is_paused(self):
        n = RoomMapper.__new__(RoomMapper)
        n.paused, n.drive = True, True
        n.manual_cmd, n.manual_until, n.manual_deadman = None, 0., .55
        n.web_pending = [('manual', 'left')]
        n.send = Mock()
        n.handle_web()
        self.assertTrue(n.paused)
        self.assertEqual(n.manual_cmd, 'left')
        self.assertGreater(n.manual_until, time.time())
        n.send.assert_not_called()

    def test_resume_rejects_invalid_slam_map_and_requests_clear(self):
        n = RoomMapper.__new__(RoomMapper)
        n.paused, n.map_trusted = True, False
        n.manual_cmd, n.manual_until = None, 0.
        n.phase, n.web_msg = 'go', ''
        n.web_pending = [('resume',)]
        n.send = Mock()
        n.handle_web()
        self.assertTrue(n.paused)
        n.send.assert_called_once_with(('stop',))
        self.assertIn('CLEAR MAP', n.web_msg)

    def test_web_speed_command_updates_both_limits(self):
        n = RoomMapper.__new__(RoomMapper)
        n.web_pending = [('speed', 125., 80.)]
        n.speed_min_percent, n.speed_max_percent = 50., 200.
        n.translation_speed_percent = n.rotation_speed_percent = 100.
        n.last_cmd, n.web_msg = ('forward',), ''
        n.handle_web()
        self.assertEqual(n.translation_speed_percent, 125.)
        self.assertEqual(n.rotation_speed_percent, 80.)
        self.assertIsNone(n.last_cmd)

    def test_selected_navigation_goal_unpauses_with_safe_planner_path(self):
        n = RoomMapper.__new__(RoomMapper)
        n.data_lock, n.map_lock = threading.Lock(), threading.Lock()
        n.pose, n.map_trusted = (0., 0., 0.), True
        n.explorer = SimpleNamespace(
            have_map=True,
            plan_to_point=Mock(return_value=([(0., 0.), (.5, 0.)], (.5, 0.))))
        n.paused, n.manual_cmd, n.manual_until = True, None, 0.
        n.state, n.state_start = FAULT, 0.
        n.send = Mock()
        self.assertTrue(n.start_navigation(.5, 0.))
        n.send.assert_called_once_with(('stop',))
        self.assertFalse(n.paused)
        self.assertEqual(n.phase, 'user_go')
        self.assertEqual(n.state, SEARCH)
        self.assertEqual(n.nav_goal, (.5, 0.))

    def test_path_follower_does_not_skip_twenty_centimetre_corner_waypoint(self):
        n = RoomMapper.__new__(RoomMapper)
        n.path = [(.20, 0.), (.40, 0.)]
        n.waypoint_tolerance = .12
        self.assertEqual(n.follow_path((0., 0., 0.)), ('forward',))
        self.assertEqual(n.path[0], (.20, 0.))

    def test_path_follower_crab_steers_without_unnecessary_turn(self):
        n = RoomMapper.__new__(RoomMapper)
        n.path = [(1., 1.)]
        n.waypoint_tolerance = .12
        n.max_crab_angle = np.radians(45.)
        n.crab_angle_step = n.crab_deadband = np.radians(5.)
        command = n.follow_path((0., 0., np.radians(15.)))
        self.assertEqual(command[0], 'translate')
        self.assertAlmostEqual(command[1], np.radians(30.))

    def test_path_follower_turns_outside_depth_safe_crab_envelope(self):
        n = RoomMapper.__new__(RoomMapper)
        n.path = [(0., 1.)]
        n.waypoint_tolerance = .12
        n.max_crab_angle = np.radians(45.)
        self.assertEqual(n.follow_path((0., 0., 0.)), ('spin', 1.))

    def test_crab_command_uses_directional_swept_lidar_clearance(self):
        heads = np.radians(np.arange(-180., 180., 10.))
        free = np.full(len(heads), 2.)
        free[np.argmin(np.abs(heads - np.radians(40.)))] = .35
        lidar = {'heads': heads, 'free': free}
        self.assertAlmostEqual(
            RoomMapper.command_lidar_clearance(lidar, ('translate', np.radians(42.))), .35)

    def test_unreachable_navigation_goal_stays_paused(self):
        n = RoomMapper.__new__(RoomMapper)
        n.data_lock, n.map_lock = threading.Lock(), threading.Lock()
        n.pose, n.map_trusted = (0., 0., 0.), True
        n.explorer = SimpleNamespace(have_map=True,
                                     plan_to_point=Mock(return_value=(None, None)))
        n.send = Mock()
        n.stop_manual = Mock()
        self.assertFalse(n.start_navigation(4., 4.))
        n.send.assert_called_once_with(('stop',))
        n.stop_manual.assert_called_once()

    def test_cancel_navigation_stops_and_discards_route(self):
        n = RoomMapper.__new__(RoomMapper)
        n.manual_cmd, n.manual_until, n.paused = 'forward', 123., False
        n.path = [(1., 2.)]
        n.user_goal, n.nav_goal = (1., 2., 'desk'), (1., 1.5)
        n.phase, n.web_msg = 'user_go', ''
        n.send = Mock()
        n.cancel_navigation()
        self.assertTrue(n.paused)
        self.assertEqual(n.path, [])
        self.assertIsNone(n.user_goal)
        self.assertIsNone(n.nav_goal)
        self.assertEqual(n.phase, 'goal_cancelled')
        n.send.assert_called_once_with(('stop',))

    def test_floor_alarm_is_ignored_during_zero_stride_turn(self):
        n = self.controller()
        n.depth[60:80, 40:60] = .55
        n.commanded_moving, n.last_cmd = True, ('spin', 1.0)
        n.control_step()
        self.assertNotEqual(n.state, HOLD)
        self.assertEqual(n.depth_source, 'floor ignored while turning')

    def test_stale_pose_stops(self):
        n = self.controller()
        n.t_pose -= 3
        n.control_step()
        n.send.assert_called_with(('stop',))
        self.assertEqual(n.state, FAULT)

    def test_turn_with_translation_keeps_floor_protection(self):
        n = self.controller()
        n.depth[60:80, 40:60] = .55
        n.commanded_moving, n.last_cmd, n.spin_stride = True, ('spin', 1.), 10.
        n.control_step()
        n.send.assert_called_with(('stop',))
        self.assertEqual(n.state, HOLD)

    def test_zero_stride_turn_stops_inside_rotation_footprint(self):
        n = self.controller()
        n.depth[20:55, 20:80] = .55
        n.lidar['nearest'] = .29
        n.lidar['back'] = n.lidar['near_back'] = .30
        n.commanded_moving, n.last_cmd = True, ('spin', 1.)
        n.control_step()
        n.send.assert_called_with(('stop',))
        self.assertEqual(n.state, HOLD)

    def test_stale_map_stops(self):
        n = self.controller()
        n.t_map -= 6
        n.control_step()
        n.send.assert_called_with(('stop',))
        self.assertEqual(n.state, FAULT)

    def test_live_rtab_heartbeat_keeps_unchanged_map_active(self):
        n = self.controller()
        n.map_heartbeat_topic = '/visual_slam/info'
        n.t_map -= 60
        n.t_map_heartbeat = time.time()
        n.control_step()
        self.assertNotEqual(n.state, FAULT)

    def test_stale_rtab_heartbeat_stops_even_with_recent_grid(self):
        n = self.controller()
        n.map_heartbeat_topic = '/visual_slam/info'
        n.t_map_heartbeat = time.time() - 6
        n.control_step()
        n.send.assert_called_with(('stop',))
        self.assertEqual(n.state, FAULT)

    def test_stale_depth_stops(self):
        n = self.controller()
        n.t_depth -= 2
        n.control_step()
        n.send.assert_called_with(('stop',))
        self.assertEqual(n.state, FAULT)

    def test_failed_stop_is_retried(self):
        n = self.node
        n.drive, n.last_cmd = True, ('forward',)
        n.commanded_moving, n.motion_until = True, time.time() + 1
        n.settle_until, n.scan_settle = 0, .65
        n.motion_pub = Mock()
        n.robot = SimpleNamespace(traveling=Mock(side_effect=[RuntimeError('test failure'), None]))
        with self.assertRaises(RuntimeError):
            n.send(('stop',))
        n.send(('stop',))
        self.assertEqual(n.robot.traveling.call_count, 2)

    def test_shutdown_prevents_late_timer_command(self):
        n = self.controller()
        n.control_lock, n.shutting_down = threading.RLock(), False
        n.timer, n.control_step = Mock(), Mock()
        n.shutdown()
        n.safe_step(None)
        n.control_step.assert_not_called()
        n.timer.shutdown.assert_called_once_with()
        n.send.assert_called_once_with(('stop',))

    def test_motion_is_bursted_then_settled(self):
        n = self.node
        n.drive, n.last_cmd = True, None
        n.commanded_moving, n.motion_until = False, 0
        n.settle_until, n.scan_settle = 0, .65
        n.move_burst, n.spin_burst = .45, .35
        n.max_continuous_translation = 1.0
        n.stride, n.step_height, n.step_time = 40., 20., .5
        n.spin_stride, n.spin_rate = 0., .35
        n.motion_pub = Mock()
        n.robot = SimpleNamespace(traveling=Mock())
        n.send(('forward',))
        self.assertTrue(n.commanded_moving)
        n.motion_until = time.time() - .01
        n.continuous_translation_start = time.time() - 1.01
        n.send(('forward',))
        self.assertFalse(n.commanded_moving)
        self.assertEqual(n.last_cmd, ('stop',))
        n.send(('forward',))
        self.assertEqual(n.last_cmd, ('stop',))

    def test_matching_translation_renews_smoothly_until_continuous_limit(self):
        n = self.node
        n.drive, n.last_cmd = True, None
        n.commanded_moving, n.motion_until = False, 0.
        n.settle_until, n.scan_settle = 0., .65
        n.move_burst, n.spin_burst = 1., .5
        n.max_continuous_translation = 3.
        n.stride, n.step_height, n.step_time = 40., 20., .5
        n.spin_stride, n.spin_rate = 0., .35
        n.object_processing, n.object_detections = False, []
        n.motion_pub = Mock()
        n.robot = SimpleNamespace(traveling=Mock())
        with patch('room_mapper.time.time', return_value=10.):
            n.send(('forward',))
        self.assertTrue(n.commanded_moving)
        self.assertEqual(n.continuous_translation_start, 10.)

        n.motion_until = 11.
        with patch('room_mapper.time.time', return_value=11.1):
            n.send(('forward',))
        self.assertTrue(n.commanded_moving)
        self.assertAlmostEqual(n.motion_until, 12.1)
        self.assertEqual(n.robot.traveling.call_count, 1)

        n.motion_until = 13.
        with patch('room_mapper.time.time', return_value=13.1):
            n.send(('forward',))
        self.assertFalse(n.commanded_moving)
        self.assertEqual(n.last_cmd, ('stop',))

    def test_speed_percentages_scale_translation_and_rotation(self):
        n = self.node
        n.drive, n.last_cmd = True, None
        n.stride, n.step_height, n.step_time = 40., 20., .5
        n.spin_stride, n.spin_rate = 0., .10
        n.translation_speed_percent, n.rotation_speed_percent = 125., 150.
        n.robot = SimpleNamespace(traveling=Mock())
        n.send_actual(('forward',))
        self.assertEqual(n.robot.traveling.call_args[1]['stride'], 50.)
        n.last_cmd = None
        n.send_actual(('spin', 1.))
        self.assertAlmostEqual(n.robot.traveling.call_args[1]['rotation'], .15)

    def test_translation_command_uses_vendor_body_relative_direction(self):
        n = self.node
        n.drive, n.last_cmd = True, None
        n.stride, n.step_height, n.step_time = 40., 20., .5
        n.translation_speed_percent = n.rotation_speed_percent = 100.
        n.robot = SimpleNamespace(traveling=Mock())
        n.send_actual(('translate', np.radians(35.)))
        self.assertAlmostEqual(n.robot.traveling.call_args[1]['direction'],
                               np.radians(35.))
        n.last_cmd = None
        n.send_actual(('strafe_left',))
        self.assertAlmostEqual(n.robot.traveling.call_args[1]['direction'], np.pi / 2.)
        n.last_cmd = None
        n.send_actual(('strafe_right',))
        self.assertAlmostEqual(n.robot.traveling.call_args[1]['direction'], -np.pi / 2.)

    def test_manual_motion_interrupts_slow_object_inference(self):
        n = self.node
        n.drive, n.last_cmd = True, None
        n.commanded_moving, n.motion_until = False, 0
        n.settle_until, n.scan_settle = 0, .65
        n.move_burst, n.spin_burst = .45, .35
        n.stride, n.step_height, n.step_time = 40., 20., .5
        n.spin_stride, n.spin_rate = 0., .35
        n.motion_generation, n.object_processing = 3, True
        n.object_detections = [{'label': 'chair'}]
        n.object_focus_enabled = True
        n.motion_pub = Mock()
        n.robot = SimpleNamespace(traveling=Mock())

        n.send(('forward',))
        self.assertFalse(n.commanded_moving)
        n.send(('forward',), interrupt_object=True)
        self.assertTrue(n.commanded_moving)
        self.assertEqual(n.motion_generation, 4)

    def test_scan_gate_tracks_pure_turns_but_rejects_translation_and_settling(self):
        n = self.node
        n.shutting_down, n.map_trusted = False, True
        n.drive, n.spin_stride, n.settle_until = True, 0., 0
        n.commanded_moving, n.last_cmd = True, ('spin', 1.0)
        self.assertTrue(n.scan_ready_for_mapping(time.time()))
        n.last_cmd = ('forward',)
        self.assertFalse(n.scan_ready_for_mapping(time.time()))
        n.commanded_moving, n.settle_until = False, time.time() + 1
        self.assertFalse(n.scan_ready_for_mapping(time.time()))
        n.settle_until = 0
        self.assertTrue(n.scan_ready_for_mapping(time.time()))

    def test_drive_waits_until_startup_pose_guard_is_active(self):
        n = self.controller()
        n.drive, n.pose_guard_start = True, time.time() + 5.
        n.control_step()
        n.send.assert_called_once_with(('stop',))
        self.assertEqual(n.state, FAULT)

    def test_settling_starts_after_slow_stop_call_returns(self):
        n = self.node
        n.drive, n.last_cmd, n.commanded_moving = True, ('forward',), True
        n.scan_settle, n.motion_until, n.motion_generation = .65, 100., 0
        n.motion_pub = Mock()
        clock = [100.]
        n.robot = SimpleNamespace(traveling=Mock(side_effect=lambda **kw: clock.__setitem__(0, 100.4)))
        with patch('room_mapper.time.time', side_effect=lambda: clock[0]):
            n.send(('stop',))
        self.assertAlmostEqual(n.settle_until, 101.05)

    def test_pose_jump_marks_map_untrusted(self):
        n = self.node
        n.prev_pose = (0., 0., 0.)
        n.pose_guard_start, n.map_trusted = 0, True
        n.max_pose_jump, n.max_yaw_jump = .5, np.radians(35)
        n.web_msg = ''
        msg = SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=0., y=0.),
            orientation=SimpleNamespace(x=0., y=0., z=np.sin(np.radians(30)),
                                        w=np.cos(np.radians(30)))))
        n.pose_callback(msg)
        self.assertFalse(n.map_trusted)
        self.assertIn('SLAM LOST', n.web_msg)

    def test_covariance_stamped_visual_pose_is_accepted(self):
        n = self.node
        n.prev_pose, n.pose = None, None
        n.pose_guard_start, n.map_trusted = 0., True
        n.web_map_center, n.t_pose = None, 0.
        nested = SimpleNamespace(
            position=SimpleNamespace(x=1.25, y=-.75),
            orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
        msg = SimpleNamespace(pose=SimpleNamespace(pose=nested))
        n.pose_callback(msg)
        self.assertEqual(n.pose, (1.25, -.75, 0.))
        self.assertEqual(n.web_map_center, (1.25, -.75))

    def test_visual_slam_pose_can_be_read_from_tf(self):
        n = self.node
        n.pose_type, n.map_frame, n.pose_frame = 'tf', 'map', 'jethexa/base_link'
        n.slam_stale = 2.5
        n.object_lock = threading.Lock()
        n.tf_buffer = Mock()
        n.tf_buffer.lookup.return_value = (
            (1.25, -.75, 0.),
            (0., 0., np.sin(np.radians(15)), np.cos(np.radians(15))))
        n.prev_pose, n.pose = None, None
        n.pose_guard_start, n.map_trusted = 0., True
        n.web_map_center, n.t_pose = None, 0.
        n.survey = Mock()
        stamp = SimpleNamespace(to_sec=Mock(return_value=123.0))
        with patch('room_mapper.rospy.Time.now', return_value=stamp):
            n.update_pose_from_tf()
        self.assertEqual(n.pose[:2], (1.25, -.75))
        self.assertAlmostEqual(n.pose[2], np.radians(30))
        n.tf_buffer.lookup.assert_called_once_with(
            'map', 'jethexa/base_link', 123.0, max_age=2.5)

    def test_empty_reset_service_supports_rtabmap(self):
        n = self.node
        n.reset_service = '/visual_slam/reset'
        n.reset_service_type = 'empty'
        reset = Mock(return_value=SimpleNamespace())
        with patch('room_mapper.rospy.wait_for_service') as wait, \
                patch('room_mapper.rospy.ServiceProxy', return_value=reset) as proxy:
            n.call_reset_service()
        wait.assert_called_once_with('/visual_slam/reset', timeout=5.0)
        proxy.assert_called_once_with('/visual_slam/reset', Empty)
        reset.assert_called_once_with()

    def test_web_map_reset_stops_pauses_and_clears_local_state(self):
        n = RoomMapper.__new__(RoomMapper)
        n.paused, n.web_msg = False, ''
        n.send = Mock()
        n.map_lock, n.data_lock = threading.Lock(), threading.Lock()
        n.explorer = SimpleNamespace(reset=Mock())
        n.t_map, n.pose = 123., (1., 2., .3)
        n.scan_settle, n.settle_until = .65, 0.
        n.survey = SimpleNamespace(reset=Mock())
        n.prev_pose, n.pose_guard_start, n.map_trusted = (0., 0., 0.), 0., False
        n.motion_generation = 7
        n.path, n.phase, n.phase_start = [(3., 4.)], 'go', 0.
        n.state, n.state_start = FAULT, 0.
        n.web_map_center = (0., 0.)
        n.call_reset_service = Mock()
        n.clear_object_tracks = Mock()
        with patch('room_mapper.time.time', return_value=100.):
            n.reset_mapping()
        self.assertTrue(n.paused)
        n.send.assert_called_once_with(('stop',))
        n.explorer.reset.assert_called_once_with()
        self.assertEqual(n.path, [])
        self.assertEqual(n.phase, 'survey')
        self.assertEqual(n.state_reason, 'fresh map survey')
        self.assertEqual(n.web_map_center, (1., 2.))
        self.assertTrue(n.map_trusted)
        self.assertEqual(n.motion_generation, 8)
        self.assertEqual(n.state, SEARCH)
        n.call_reset_service.assert_called_once_with()
        n.clear_object_tracks.assert_called_once_with()

    def test_depth_map_reports_and_forwards_confirmed_clears(self):
        n = RoomMapper.__new__(RoomMapper)
        n.data_lock, n.object_lock, n.map_lock = (threading.Lock(),
                                                  threading.Lock(),
                                                  threading.Lock())
        n.depth_map_k = (100., 0., 0., 0., 100., 0., 0., 0., 1.)
        n.depth_map_info_size = (1, 1)
        n.depth_map_decimation = 4
        n.depth_map_range_min, n.depth_map_range_max = .15, 2.
        n.depth_map_height_min, n.depth_map_height_max = .025, .5
        n.depth_map_min_points = 3
        n.depth_map_protected_range = .6
        n.depth_map_clear_observations = 2
        n.depth_map_ttl = 30.
        n.map_frame = 'map'
        n.tf_buffer = Mock()
        n.tf_buffer.lookup.return_value = ((0., 0., .2), (0., 0., 0., 1.))
        n.explorer = Mock(cell=.1)
        n.explorer.update_depth_obstacles.return_value = 3
        n.explorer.depth_obstacle_count.return_value = 5
        n.message_time = Mock(return_value=99.)
        msg = SimpleNamespace(header=SimpleNamespace(frame_id='camera'))
        with patch('room_mapper.time.time', return_value=100.), \
                patch('room_mapper.project_depth_cells',
                      return_value=([(0, 1)], [(0, 2)])) as project, \
                patch('room_mapper.rospy.loginfo') as loginfo:
            n.update_depth_obstacle_map(np.ones((1, 1), np.float32), msg)
        project.assert_called_once()
        n.explorer.update_depth_obstacles.assert_called_once_with(
            [(0, 1)], 100., 30., observed_free_cells=[(0, 2)],
            clear_confirmations=2)
        self.assertEqual(n.depth_map_last_cleared, 3)
        self.assertIn('5 temporary obstacle cells', n.depth_map_status)
        self.assertIn('cleared 3 this update', n.depth_map_status)
        loginfo.assert_called_once()

    def test_depth_map_uses_only_settled_synchronized_lidar_for_fusion(self):
        n = RoomMapper.__new__(RoomMapper)
        n.data_lock, n.object_lock, n.map_lock = (threading.Lock(),
                                                  threading.Lock(),
                                                  threading.Lock())
        n.depth_map_k = (100., 0., 0., 0., 100., 0., 0., 0., 1.)
        n.depth_map_info_size = (1, 1)
        n.depth_map_decimation = 4
        n.depth_map_range_min, n.depth_map_range_max = .15, 2.
        n.depth_map_height_min, n.depth_map_height_max = .025, .5
        n.depth_map_min_points = 3
        n.depth_map_protected_range = .6
        n.depth_map_clear_observations = 2
        n.depth_map_promotion_observations = 3
        n.depth_map_joint_clear_observations = 4
        n.depth_lidar_sync_tolerance = .35
        n.depth_map_settled_tf_max_age = 1.
        n.depth_map_ttl = 30.
        n.commanded_moving, n.settle_until = False, 90.
        n.lidar_fusion_stamp = 99.1
        n.lidar_fusion_occupied = [(1, 1)]
        n.lidar_fusion_free = [(1, 2)]
        n.map_frame = 'map'
        n.tf_buffer = Mock()
        n.tf_buffer.lookup.return_value = ((0., 0., .2), (0., 0., 0., 1.))
        n.explorer = Mock(cell=.1)
        n.explorer.update_fused_observations.return_value = {
            'promoted': 2, 'cleared_temporary': 1,
            'cleared_persistent': 1, 'suppressed_slam': 1}
        n.explorer.depth_obstacle_count.return_value = 5
        n.message_time = Mock(return_value=99.)
        msg = SimpleNamespace(header=SimpleNamespace(frame_id='camera'))
        with patch('room_mapper.time.time', return_value=100.), \
                patch('room_mapper.project_depth_cells',
                      return_value=([(0, 1)], [(0, 2)])):
            n.update_depth_obstacle_map(np.ones((1, 1), np.float32), msg)
        n.explorer.update_fused_observations.assert_called_once_with(
            [(0, 1)], [(0, 2)], [(1, 1)], [(1, 2)],
            now=100., ttl=30., temporary_clear_confirmations=2,
            promotion_confirmations=3, joint_clear_confirmations=4)
        n.tf_buffer.lookup.assert_called_once_with(
            'map', 'camera', 99., max_age=1.)
        n.explorer.update_depth_obstacles.assert_not_called()
        self.assertEqual(n.depth_map_last_cleared, 3)
        self.assertIn('promoted 2', n.depth_map_status)


if __name__ == '__main__':
    unittest.main()
