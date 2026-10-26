"""Regression cases for scan admission, heading guards, and map presentation."""
import math
import threading
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np

from map_explore import ExploreMap, OCCUPIED
from map_web import draw_map, to_screen
from object_mapping import ObjectTracker
from room_mapper import RoomMapper
from stationary_survey import StationarySurvey


class ScanAdmissionTests(unittest.TestCase):
    def setUp(self):
        logger = patch('room_mapper.rospy.logwarn_throttle')
        logger.start()
        self.addCleanup(logger.stop)
        for target, value in (
                ('room_mapper.time.time', 100.),
                ('room_mapper.rospy.Time.now', NS(to_sec=lambda: 100.))):
            clock = patch(target, return_value=value)
            clock.start()
            self.addCleanup(clock.stop)
        n = self.node = RoomMapper.__new__(RoomMapper)
        n.control_lock, n.data_lock = threading.RLock(), threading.Lock()
        n.drive, n.map_trusted = True, True
        n.commanded_moving = n.shutting_down = False
        n.last_cmd, n.spin_stride = ('stop',), 0.
        n.settle_until = 99.9
        n.scan_max_age, n.last_scan_stamp = .6, 0.
        n.yaw_offset, n.self_filter, n.body_radius = 0., .15, .25
        n.front_half, n.side_max = math.radians(30), math.radians(100)
        n.stable_scan_pub = Mock()
        n.stable_stationary_scans = n.stable_turn_scans = n.rejected_motion_scans = 0
        n.t_scan, n.lidar = 0., None

    @staticmethod
    def scan(stamp=99.95):
        return NS(header=NS(stamp=NS(to_sec=lambda: stamp)), ranges=[2.]*360,
                  angle_min=-math.pi, angle_increment=2*math.pi/360,
                  range_min=.1, range_max=12.)

    def test_fault_stops_scan_integration_but_keeps_collision_updates(self):
        n = self.node
        n.map_trusted = False
        n.scan_callback(self.scan())
        n.stable_scan_pub.publish.assert_not_called()
        self.assertEqual(n.t_scan, 100.)
        self.assertAlmostEqual(n.lidar['front'], 2.)

    def test_pre_settle_capture_is_not_mapped_after_late_arrival(self):
        n = self.node
        n.scan_callback(self.scan(99.8))
        n.stable_scan_pub.publish.assert_not_called()
        self.assertEqual(n.t_scan, 100.)

    def test_fresh_post_settle_scan_is_forwarded_without_rewriting_geometry(self):
        msg = self.scan()
        self.node.scan_callback(msg)
        self.node.stable_scan_pub.publish.assert_called_once_with(msg)

    def test_collision_scan_keeps_returns_inside_stop_distance(self):
        msg = self.scan()
        msg.ranges[180] = .30
        self.node.scan_callback(msg)
        self.assertAlmostEqual(self.node.lidar['front'], .30, places=5)

    def test_stale_future_zero_and_nonfinite_scans_do_not_refresh_health(self):
        n = self.node
        for stamp in (98., 100.1, 0., float('nan')):
            with self.subTest(stamp=stamp):
                n.scan_callback(self.scan(stamp))
                n.stable_scan_pub.publish.assert_not_called()
                self.assertEqual(n.t_scan, 0.)

    def test_duplicate_and_out_of_order_scans_are_not_reintegrated(self):
        n = self.node
        n.scan_callback(self.scan(99.95))
        n.scan_callback(self.scan(99.95))
        n.scan_callback(self.scan(99.94))
        self.assertEqual(n.stable_scan_pub.publish.call_count, 1)

    def test_translation_still_suppresses_mapping(self):
        n = self.node
        n.commanded_moving, n.last_cmd = True, ('forward',)
        n.scan_callback(self.scan())
        n.stable_scan_pub.publish.assert_not_called()

    def test_zero_stride_turn_is_forwarded_for_incremental_yaw(self):
        n = self.node
        n.survey = Mock()
        n.commanded_moving, n.last_cmd = True, ('spin', 1.)
        n.scan_callback(self.scan())
        n.stable_scan_pub.publish.assert_called_once()
        n.survey.observe.assert_not_called()
        self.assertEqual(n.stable_turn_scans, 1)

    def test_recent_turn_scan_is_forwarded(self):
        n = self.node
        n.commanded_moving, n.last_cmd = True, ('spin', 1.)
        n.scan_callback(self.scan(99.8))
        n.stable_scan_pub.publish.assert_called_once()

    def test_dry_run_honors_initial_settling_and_map_fault(self):
        n = self.node
        n.drive, n.settle_until = False, 101.
        n.scan_callback(self.scan())
        n.stable_scan_pub.publish.assert_not_called()
        n.settle_until, n.map_trusted = 99., False
        n.scan_callback(self.scan(99.96))
        n.stable_scan_pub.publish.assert_not_called()

    def test_shutdown_never_forwards_a_scan(self):
        self.node.shutting_down = True
        self.node.scan_callback(self.scan())
        self.node.stable_scan_pub.publish.assert_not_called()

    def test_shutdown_cannot_interleave_between_gate_and_publish(self):
        n = self.node
        entered, release = threading.Event(), threading.Event()
        attempted, stopped = threading.Event(), threading.Event()
        order = []
        gate = n.scan_ready_for_mapping
        def delayed_gate(*args):
            allowed = gate(*args)
            entered.set()
            if not release.wait(2.):
                raise RuntimeError('test gate timed out')
            return allowed
        n.scan_ready_for_mapping = delayed_gate
        n.stable_scan_pub.publish.side_effect = lambda msg: order.append('publish')
        def stop():
            attempted.set()
            with n.control_lock:
                n.shutting_down = True
                order.append('stop')
                stopped.set()
        scanner = threading.Thread(target=n.scan_callback, args=(self.scan(),), daemon=True)
        stopper = threading.Thread(target=stop, daemon=True)
        scanner.start()
        try:
            self.assertTrue(entered.wait(2.))
            stopper.start()
            self.assertTrue(attempted.wait(2.))
            self.assertFalse(stopped.wait(.1))
        finally:
            release.set()
            scanner.join(2.)
            if stopper.ident is not None:
                stopper.join(2.)
        self.assertFalse(scanner.is_alive())
        self.assertFalse(stopper.is_alive())
        self.assertEqual(order, ['publish', 'stop'])


class MapPresentationTests(unittest.TestCase):
    def setUp(self):
        self.explorer = ExploreMap(size_m=4., cell=.1)
        self.raw = np.zeros((80, 80), np.int16)
        self.raw[20:60, 60] = 100
        self.raw[20, 20:60] = 100

    def update(self, data):
        self.explorer.update_from_slam(data.ravel(), 80, 80, .05, -2., -2.)

    def test_map_updates_replace_old_geometry_instead_of_overlaying(self):
        self.update(self.raw)
        self.assertTrue((self.explorer.state == OCCUPIED).any())
        self.update(np.zeros_like(self.raw))
        self.assertFalse((self.explorer.state == OCCUPIED).any())

    def test_single_low_confidence_hit_does_not_become_planner_wall(self):
        tentative = np.zeros_like(self.raw)
        tentative[20:60, 60] = 70
        self.update(tentative)
        self.assertFalse((self.explorer.state == OCCUPIED).any())
        confirmed = tentative.copy()
        confirmed[20:60, 60] = 80
        self.update(confirmed)
        self.assertTrue((self.explorer.state == OCCUPIED).any())

    def test_repeated_updates_do_not_move_or_accumulate_walls(self):
        self.update(self.raw)
        expected = self.explorer.state.copy()
        for _ in range(4):
            self.update(self.raw)
        np.testing.assert_array_equal(self.explorer.state, expected)

    def test_robot_yaw_changes_marker_and_heading_not_wall_pixels(self):
        self.update(self.raw)
        n = NS(explorer=self.explorer, pose=(0., 0., 0.), map_lock=threading.Lock(),
               object_lock=threading.Lock(), path=[], view={}, body_radius=.25,
               object_tracker=ObjectTracker())
        before = draw_map(n)
        n.pose = (0., 0., math.radians(30))
        after = draw_map(n)
        yy, xx = np.indices(before.shape[:2])
        outside_marker_and_label = (((xx-210)**2 + (yy-210)**2 > 30**2) &
                                    (yy > 30))
        np.testing.assert_array_equal(before[outside_marker_and_label],
                                      after[outside_marker_and_label])
        self.assertTrue(np.any(before != after))

    def test_translation_keeps_robot_centered_and_moves_world_geometry(self):
        self.update(self.raw)
        n = NS(explorer=self.explorer, pose=(0., 0., 0.), map_lock=threading.Lock(),
               object_lock=threading.Lock(), path=[], view={}, body_radius=.25,
               web_map_center=(0., 0.), object_tracker=ObjectTracker())
        before = draw_map(n)
        n.pose = (.6, .4, .7)
        after = draw_map(n)
        before_walls = np.all(before == 0, axis=2)
        after_walls = np.all(after == 0, axis=2)
        # Moving +0.6 m x and +0.4 m y pans stationary world geometry by
        # +30 rows and +20 columns at the default 50 px/m scale.
        expected = before_walls[:-30, :-20]
        shifted = after_walls[30:, 20:]
        yy, xx = np.indices(expected.shape)
        # Ignore the fixed heading label and both locations at which the
        # centered robot marker can cover otherwise stationary wall pixels.
        compare = ((yy > 30) &
                   ((xx-210)**2 + (yy-210)**2 > 35**2) &
                   ((xx-190)**2 + (yy-180)**2 > 35**2))
        self.assertLess(np.mean(expected[compare] != shifted[compare]), .003)
        # The robot itself remains at the center of the moving viewport.
        self.assertFalse(np.all(before[210, 210] == 0))
        self.assertFalse(np.all(after[210, 210] == 0))

    def test_all_world_overlays_share_zoomed_and_panned_transform(self):
        self.update(self.raw)
        point = (1.2, -.4)
        tracker = NS(snapshot=Mock(return_value=[
            {'id': 1, 'label': 'chair', 'name': 'Chair',
             'x': point[0], 'y': point[1]}]))
        n = NS(explorer=self.explorer, pose=(point[0], point[1], 0.),
               map_lock=threading.Lock(), object_lock=threading.Lock(),
               path=[point], view={}, body_radius=.25,
               object_tracker=tracker,
               user_goal=(point[0], point[1], 'requested', 0.),
               nav_goal=point)
        center = (.3, -.2)
        for scale in (25., 50., 100.):
            with self.subTest(scale=scale), \
                    patch('map_web.cv2.circle') as circle, \
                    patch('map_web.cv2.polylines') as polylines, \
                    patch('map_web.cv2.drawMarker') as marker:
                draw_map(n, scale=scale, center=center)
            expected = to_screen(point[0], point[1], center[0], center[1], scale)

            path_call = next(call for call in polylines.call_args_list
                             if call.args[3] == (230, 140, 40))
            self.assertEqual(tuple(path_call.args[1][0][0]), expected)
            body_call = next(call for call in circle.call_args_list
                             if call.args[3] == (30, 210, 230))
            object_call = next(call for call in circle.call_args_list
                               if call.args[3] == (0, 165, 255))
            self.assertEqual(body_call.args[1], expected)
            self.assertEqual(object_call.args[1], expected)
            self.assertEqual(object_call.args[2], 6)
            self.assertEqual(marker.call_args_list[0].args[1], expected)
            self.assertEqual(marker.call_args_list[0].args[4], 16)
            self.assertEqual(marker.call_args_list[1].args[1], expected)
            self.assertEqual(marker.call_args_list[1].args[4], 18)


class HectorLaunchTests(unittest.TestCase):
    def test_no_odometry_hector_receives_continuous_raw_scans(self):
        root = ET.parse(str(Path(__file__).with_name('hex_map.launch'))).getroot()
        args = {node.attrib['name']: node.attrib.get('default')
                for node in root.findall('arg')}
        hector = next(node for node in root.findall('node')
                      if node.attrib.get('name') == 'hector_mapping')
        params = {node.attrib['name']: node.attrib.get('value')
                  for node in hector.findall('param')}
        self.assertEqual(args['scan_topic'], 'scan')
        self.assertEqual(params['scan_topic'], '$(arg scan_topic)')
        self.assertEqual(params['laser_min_dist'], '0.20')
        self.assertEqual(params['map_update_distance_thresh'], '0.2')
        self.assertEqual(params['map_update_angle_thresh'], '0.06')


class SurveyMotionTests(unittest.TestCase):
    def test_next_burst_only_waits_for_scan_settle_not_full_survey(self):
        n = RoomMapper.__new__(RoomMapper)
        n.drive = n.use_map = True
        n.commanded_moving = False
        n.data_lock = threading.Lock()
        n.settle_until = 100.
        n.survey = StationarySurvey(1., 3)
        n.survey.reset(100.)
        n.send_actual, n.set_motion = Mock(), Mock()
        n.move_burst, n.spin_burst = 1., .35
        with patch('room_mapper.time.time', return_value=102.):
            n.send(('forward',))
            n.send_actual.assert_called_once_with(('forward',))

    def test_motion_waits_for_inference_when_object_is_in_view(self):
        n = RoomMapper.__new__(RoomMapper)
        n.drive = n.use_map = True
        n.commanded_moving = False
        n.data_lock = threading.Lock()
        n.settle_until = 0.
        n.survey = StationarySurvey(1., 1)
        n.survey.reset(0.)
        n.survey.observe(1.)
        n.survey.localize(1.)
        n.object_processing = True
        n.object_detections = [{'label': 'chair'}]
        n.object_focus_enabled = True
        n.send_actual, n.set_motion = Mock(), Mock()
        n.move_burst, n.spin_burst = .45, .35
        with patch('room_mapper.time.time', return_value=2.):
            n.send(('forward',))
            n.send_actual.assert_not_called()
            n.object_processing = False
            n.send(('forward',))
            n.send_actual.assert_called_once_with(('forward',))

    def test_default_nonblocking_detection_never_holds_autonomous_motion(self):
        n = RoomMapper.__new__(RoomMapper)
        n.drive = n.use_map = True
        n.commanded_moving = False
        n.data_lock = threading.Lock()
        n.settle_until = 0.
        n.object_processing = True
        n.object_detections = [{'label': 'chair'}]
        n.object_focus_enabled = False
        n.send_actual, n.set_motion = Mock(), Mock()
        n.move_burst, n.spin_burst = .45, .35
        with patch('room_mapper.time.time', return_value=2.):
            n.send(('forward',))
        n.send_actual.assert_called_once_with(('forward',))
        n.set_motion.assert_called_once_with(True)

    def test_motion_continues_through_empty_scene_inference(self):
        n = RoomMapper.__new__(RoomMapper)
        n.drive = n.use_map = True
        n.commanded_moving = False
        n.data_lock = threading.Lock()
        n.settle_until = 0.
        n.survey = StationarySurvey(1., 1)
        n.survey.reset(0.)
        n.object_processing = True
        n.object_detections = []
        n.send_actual, n.set_motion = Mock(), Mock()
        n.move_burst, n.spin_burst = .45, .35
        with patch('room_mapper.time.time', return_value=2.):
            n.send(('forward',))
        n.send_actual.assert_called_once_with(('forward',))
        n.set_motion.assert_called_once_with(True)

    def test_repeated_missing_heading_progress_faults_after_retries(self):
        n = RoomMapper.__new__(RoomMapper)
        n.drive = n.use_map = True
        n.commanded_moving = False
        n.data_lock = threading.Lock()
        n.settle_until = 0.
        n.survey = StationarySurvey(1., 1)
        n.survey.reset(0.)
        n.survey.observe(1.)
        n.survey.localize(1.)
        n.pose = (0., 0., math.radians(1.))
        n.pending_spin_yaw = 0.
        n.min_spin_progress = math.radians(3.)
        n.max_spin_stalls = 3
        n.spin_stall_count = 0
        n.map_trusted = True
        n.web_msg = ''
        n.clear_object_tracks = Mock()
        n.send_actual, n.set_motion = Mock(), Mock()
        n.move_burst, n.spin_burst = .45, .5
        with patch('room_mapper.time.time', return_value=2.), \
                patch('room_mapper.rospy.logerr'):
            n.send(('forward',))
            self.assertTrue(n.map_trusted)
            self.assertIn('retrying (1/3)', n.web_msg)
            n.pending_spin_yaw = 0.
            n.send(('forward',))
            self.assertTrue(n.map_trusted)
            self.assertIn('retrying (2/3)', n.web_msg)
            n.pending_spin_yaw = 0.
            n.send(('forward',))
        n.send_actual.assert_not_called()
        self.assertFalse(n.map_trusted)
        self.assertIn('3 consecutive turns', n.web_msg)


if __name__ == '__main__':
    unittest.main()
