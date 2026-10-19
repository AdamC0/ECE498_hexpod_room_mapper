"""Synthetic RGB/depth/TF integration; no ROS node or real detector required."""
import math
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from object_mapping import ObjectTracker, TransformCache
from room_mapper import RoomMapper


class ObjectPipelineTests(unittest.TestCase):
    def setUp(self):
        for target in ('room_mapper.time.time',):
            mocked = patch(target, return_value=100.0)
            mocked.start()
            self.addCleanup(mocked.stop)
        clock = patch('room_mapper.rospy.Time.now',
                      return_value=SimpleNamespace(to_sec=lambda: 100.0))
        clock.start()
        self.addCleanup(clock.stop)
        n = self.node = RoomMapper.__new__(RoomMapper)
        n.control_lock = threading.RLock()
        n.data_lock = threading.Lock()
        n.object_lock = threading.Lock()
        n.shutting_down = n.commanded_moving = False
        n.motion_generation = 0
        n.settle_until = 98.0
        n.use_map = n.map_trusted = True
        n.pose = (1., 2., math.pi / 2)
        n.t_pose = n.t_map = n.t_camera = n.t_registered_depth = 99.9
        n.t_map_heartbeat = 0.0
        n.map_heartbeat_topic, n.map_stale = '', 5.0
        n.slam_stale = n.object_max_age = 2.5
        n.object_result_max_age = 12.0
        n.object_sync_slop = .2
        n.camera_stamp = n.registered_depth_stamp = 99.9
        n.last_object_stamps = (0., 0.)
        n.camera = np.zeros((100, 100, 3), np.uint8)
        n.registered_depth = np.full((100, 100), 2., np.float32)
        n.camera_k = (100., 0., 50., 0., 100., 50., 0., 0., 1.)
        n.camera_info_size = (100, 100)
        n.camera_frame = n.camera_info_frame = 'optical'
        n.tf_buffer = TransformCache()
        n.tf_buffer.update('map', 'base', (1, 2, 0),
                           (0, 0, math.sin(math.pi/4), math.cos(math.pi/4)), 99.9)
        n.tf_buffer.update('base', 'camera', (.065, 0, .07), (0, 0, 0, 1), 0, True)
        n.tf_buffer.update('camera', 'optical', (0, 0, 0), (-.5, .5, -.5, .5), 0, True)
        n.object_tracker = ObjectTracker()
        n.object_detections = []
        n.detector = SimpleNamespace(detect=Mock(side_effect=self.detections))

    @staticmethod
    def detections(_frame):
        return [dict(label='chair', confidence=.9, box=(20, 20, 80, 80))]

    def test_three_distinct_frames_confirm_known_map_position(self):
        n = self.node
        for stamp in (99.9, 99.95, 100.):
            n.camera_stamp = n.registered_depth_stamp = stamp
            n.process_objects()
        tracks = n.object_tracker.snapshot(True, 100.)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0]['count'], 3)
        np.testing.assert_allclose(n.object_detections[0]['map_point'],
                                   (1., 4.065, .07), atol=1e-6)

    def test_auto_mapping_focuses_until_candidate_is_confirmed(self):
        n = self.node
        n.object_tracker = ObjectTracker(min_observations=2)
        n.paused, n.phase, n.path = False, 'go', [(2., 3.)]
        n.phase_start = 90.
        n.object_focus_seconds, n.object_focus_max_misses = 15., 2
        n.object_focus_id = n.object_focus_label = None
        n.object_focus_deadline, n.object_focus_misses = 0., 0
        n.process_objects()
        self.assertEqual(n.phase, 'object_focus')
        self.assertEqual(n.path, [])
        self.assertIn('Focusing on possible chair', n.web_msg)
        n.camera_stamp = n.registered_depth_stamp = 100.
        n.process_objects()
        self.assertEqual(n.phase, 'plan')
        self.assertEqual(len(n.object_tracker.snapshot(True, 100.)), 1)
        self.assertIn('Added chair', n.web_msg)

    def test_auto_mapping_excludes_candidate_after_distinct_misses(self):
        n = self.node
        n.object_tracker = ObjectTracker(min_observations=2)
        n.paused, n.phase, n.path = False, 'go', [(2., 3.)]
        n.phase_start = 90.
        n.object_focus_seconds, n.object_focus_max_misses = 15., 2
        n.object_focus_id = n.object_focus_label = None
        n.object_focus_deadline, n.object_focus_misses = 0., 0
        n.detector.detect.side_effect = [self.detections(None), [], []]
        for stamp in (99.9, 99.95, 100.):
            n.camera_stamp = n.registered_depth_stamp = stamp
            n.process_objects()
        self.assertEqual(n.phase, 'plan')
        self.assertEqual(n.object_tracker.tracks, [])
        self.assertIn('Excluded possible chair', n.web_msg)

    def test_frozen_pair_is_not_recounted(self):
        n = self.node
        for _ in range(4):
            n.process_objects()
        self.assertEqual(n.detector.detect.call_count, 1)
        self.assertEqual(n.object_tracker.snapshot(True, 100.), [])
        self.assertEqual(n.object_tracker.tracks[0]['count'], 1)

    def test_reused_depth_is_not_a_new_observation(self):
        n = self.node
        n.process_objects()
        n.camera_stamp = 100.
        n.process_objects()
        self.assertEqual(n.detector.detect.call_count, 1)

    def test_stale_future_and_missing_timestamps_are_rejected(self):
        n = self.node
        for stamp in (90., 101., 0., float('nan')):
            with self.subTest(stamp=stamp):
                n.camera_stamp = n.registered_depth_stamp = stamp
                n.process_objects()
                n.detector.detect.assert_not_called()

    def test_unsynchronized_pair_is_rejected(self):
        self.node.registered_depth_stamp = 99.
        self.node.process_objects()
        self.node.detector.detect.assert_not_called()

    def test_missing_or_stale_slam_is_rejected(self):
        n = self.node
        for field, value in (('t_pose', 90.), ('t_map', 90.), ('use_map', False),
                             ('pose', None), ('map_trusted', False)):
            with self.subTest(field=field):
                old = getattr(n, field)
                setattr(n, field, value)
                n.process_objects()
                n.detector.detect.assert_not_called()
                setattr(n, field, old)

    def test_dimensions_and_calibration_frame_must_match(self):
        n = self.node
        for field, value in (('camera_info_size', (50, 50)),
                             ('camera_info_frame', 'another_camera'),
                             ('registered_depth', np.ones((50, 50)))):
            with self.subTest(field=field):
                old = getattr(n, field)
                setattr(n, field, value)
                n.process_objects()
                n.detector.detect.assert_not_called()
                setattr(n, field, old)

    def test_pre_settle_images_are_rejected(self):
        self.node.t_camera = 97.
        self.node.process_objects()
        self.node.detector.detect.assert_not_called()

    def test_delayed_pre_settle_capture_is_rejected_even_if_received_now(self):
        n = self.node
        n.camera_stamp = n.registered_depth_stamp = 97.9
        n.process_objects()
        n.detector.detect.assert_not_called()

    def test_motion_during_inference_discards_result_even_after_stop(self):
        n = self.node
        n.motion_pub = Mock()
        def infer(frame):
            n.set_motion(True)
            n.set_motion(False)
            return self.detections(frame)
        n.detector.detect.side_effect = infer
        n.process_objects()
        self.assertEqual(n.object_tracker.tracks, [])
        self.assertEqual(len(n.object_detections), 1)
        self.assertIsNone(n.object_detections[0]['map_point'])

    def test_slam_loss_during_inference_cannot_repopulate_tracks(self):
        n = self.node
        n.prev_pose = (1., 2., math.pi / 2)
        n.pose_guard_start = 0.
        n.max_pose_jump, n.max_yaw_jump = .5, math.radians(35)
        def infer(frame):
            n.pose_callback(SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=10., y=2.),
                orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
            return self.detections(frame)
        n.detector.detect.side_effect = infer
        n.process_objects()
        self.assertFalse(n.map_trusted)
        self.assertEqual(n.object_tracker.tracks, [])

    def test_shutdown_during_inference_discards_result(self):
        n = self.node
        n.timer, n.send = Mock(), Mock()
        def infer(frame):
            n.shutdown()
            return self.detections(frame)
        n.detector.detect.side_effect = infer
        n.process_objects()
        n.send.assert_called_once_with(('stop',))
        self.assertEqual(n.object_tracker.tracks, [])

    def test_slow_inference_can_commit_snapshot_while_robot_stays_still(self):
        n = self.node
        with patch('room_mapper.rospy.Time.now', side_effect=[
                SimpleNamespace(to_sec=lambda: 100.),
                SimpleNamespace(to_sec=lambda: 104.)]):
            n.process_objects()
        self.assertEqual(len(n.object_tracker.tracks), 1)

    def test_result_beyond_processing_age_is_shown_but_not_mapped(self):
        n = self.node
        with patch('room_mapper.rospy.Time.now', side_effect=[
                SimpleNamespace(to_sec=lambda: 100.),
                SimpleNamespace(to_sec=lambda: 113.)]):
            n.process_objects()
        self.assertEqual(n.object_tracker.tracks, [])
        self.assertEqual(len(n.object_detections), 1)
        self.assertIsNone(n.object_detections[0]['map_point'])

    def test_stale_tf_keeps_camera_boxes_without_map_points(self):
        n = self.node
        n.tf_buffer.dynamic.clear()
        n.tf_buffer.update('map', 'base', (1, 2, 0), (0, 0, 0, 1), 90.)
        n.process_objects()
        self.assertEqual(len(n.object_detections), 1)
        self.assertIsNone(n.object_detections[0]['map_point'])
        self.assertEqual(n.object_tracker.tracks, [])

    def test_tf_is_snapshotted_before_inference(self):
        n = self.node
        def infer(frame):
            n.tf_buffer.dynamic.clear()
            return self.detections(frame)
        n.detector.detect.side_effect = infer
        n.process_objects()
        np.testing.assert_allclose(n.object_detections[0]['map_point'],
                                   (1., 4.065, .07), atol=1e-6)

    def test_two_detections_in_one_frame_cannot_confirm_one_track(self):
        n = self.node
        n.detector.detect.side_effect = lambda frame: self.detections(frame) * 2
        n.process_objects()
        self.assertEqual(n.object_tracker.tracks[0]['count'], 1)


if __name__ == '__main__':
    unittest.main()
