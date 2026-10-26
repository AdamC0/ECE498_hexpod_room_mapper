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
        n.map_generation = 0
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
        n.object_detections_stamp = 0.0
        n.object_processing = False
        n.object_queue_lock = threading.Lock()
        n.object_pending_snapshots = []
        n.object_snapshots_captured = 0
        n.object_snapshots_processed = 0
        n.object_snapshots_replaced = 0
        n.object_snapshot_grace = 1.0
        n.object_burst_frames = 5
        n.object_burst_interval = .18
        n.object_endpoint_key = None
        n.object_endpoint_deadline = 0.0
        n.object_endpoint_done = None
        n.object_endpoint_captured = 0
        n.object_endpoint_next_capture = 0.0
        n.object_endpoint_observations = set()
        n.object_preview_enabled = True
        n.object_preview_next_capture = 0.0
        n.object_detection_ttl = 1.0
        n.survey = SimpleNamespace(settled_at=98., required_stamp=99.9,
                                   localize=Mock())
        n.object_focus_enabled = False
        n.object_hz = 2.0
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

    def test_expected_class_validation_confirms_from_same_settled_view(self):
        n = self.node
        n.object_tracker = ObjectTracker(priority_labels=['chair'])
        for stamp in (99.9, 99.95, 100.):
            n.camera_stamp = n.registered_depth_stamp = stamp
            snapshot = n.capture_object_snapshot()
            snapshot['validation_group'] = ('endpoint', 1)
            n.process_object_snapshot(snapshot)
        tracks = n.object_tracker.snapshot(True, 100.)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0]['tier'], 'expected')

    def test_poor_depth_support_cannot_create_validation_track(self):
        n = self.node
        n.object_tracker = ObjectTracker(priority_labels=['chair'])
        n.registered_depth[:] = 0.
        n.registered_depth[49:52, 49:52] = 2.
        n.process_objects()
        self.assertEqual(n.object_tracker.tracks, [])
        self.assertIsNone(n.object_detections[0]['map_point'])

    def test_auto_mapping_focuses_until_candidate_is_confirmed(self):
        n = self.node
        n.object_tracker = ObjectTracker(min_observations=2)
        n.paused, n.phase, n.path = False, 'survey', [(2., 3.)]
        n.phase_start = 90.
        n.object_focus_seconds, n.object_focus_max_misses = 15., 2
        n.object_focus_enabled = True
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
        n.paused, n.phase, n.path = False, 'survey', [(2., 3.)]
        n.phase_start = 90.
        n.object_focus_seconds, n.object_focus_max_misses = 15., 2
        n.object_focus_enabled = True
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

    def test_endpoint_admits_bounded_fresh_frame_burst(self):
        n = self.node
        for index in range(5):
            stamp = 99.90 + index * .02
            n.camera_stamp = n.registered_depth_stamp = stamp
            self.assertEqual(n.capture_endpoint_snapshot(100. + index * .2),
                             index == 4)
        self.assertEqual(len(n.object_pending_snapshots), 5)
        self.assertTrue(all(item['validation_group'] == (98., 99.9)
                            for item in n.object_pending_snapshots))
        self.assertEqual(n.object_snapshots_captured, 5)
        self.assertEqual(n.object_snapshots_replaced, 0)
        self.assertTrue(n.capture_endpoint_snapshot(101.))
        self.assertEqual(n.object_snapshots_captured, 5)

    def test_endpoint_does_not_count_repeated_sensor_timestamps(self):
        n = self.node
        self.assertFalse(n.capture_endpoint_snapshot(100.))
        self.assertFalse(n.capture_endpoint_snapshot(100.2))
        self.assertEqual(n.object_snapshots_captured, 1)
        self.assertEqual(len(n.object_pending_snapshots), 1)

    def test_worker_waits_until_endpoint_acquisition_closes(self):
        n = self.node
        n.object_endpoint_key = ('endpoint', 1)
        n.object_endpoint_done = None
        n.enqueue_object_snapshot(n.capture_object_snapshot())
        inferred = threading.Event()

        def detect(frame):
            inferred.set()
            n.shutting_down = True
            return self.detections(frame)

        n.detector.detect.side_effect = detect
        with patch('room_mapper.rospy.is_shutdown', return_value=False,
                   create=True):
            worker = threading.Thread(target=n.object_loop)
            worker.start()
            self.assertFalse(inferred.wait(.1))
            n.object_endpoint_done = n.object_endpoint_key
            self.assertTrue(inferred.wait(1.))
            worker.join(1.)
            self.assertFalse(worker.is_alive())

    def test_stationary_preview_is_latest_only_and_never_validates(self):
        n = self.node
        n.object_tracker = ObjectTracker(priority_labels=['chair'])
        self.assertTrue(n.capture_preview_snapshot(100.))
        self.assertEqual(len(n.object_pending_snapshots), 1)
        self.assertTrue(n.object_pending_snapshots[0]['display_only'])
        self.assertFalse(n.capture_preview_snapshot(100.))
        snapshot = n.object_pending_snapshots.pop(0)
        n.process_object_snapshot(snapshot)
        self.assertEqual(n.object_tracker.tracks, [])
        self.assertEqual(len(n.object_detections), 1)
        self.assertEqual(n.object_detections_stamp, 100.)
        self.assertIn('preview:', n.object_status)

    def test_preview_retry_preserves_last_completed_status(self):
        n = self.node
        n.object_status = 'preview: 2 detections; 0.30 s inference'
        def no_snapshot():
            n.object_status = 'waiting for sync'
            return None
        n.capture_object_snapshot = Mock(side_effect=no_snapshot)
        self.assertFalse(n.capture_preview_snapshot(100.))
        self.assertEqual(n.object_status,
                         'preview: 2 detections; 0.30 s inference')

    def test_motion_immediately_clears_detection_overlay(self):
        n = self.node
        n.motion_pub = Mock()
        n.object_detections = self.detections(None)
        n.object_detections_stamp = 100.
        n.set_motion(True)
        self.assertEqual(n.object_detections, [])
        self.assertEqual(n.object_detections_stamp, 0.)

    def test_slow_inference_queue_keeps_five_newest_snapshots(self):
        n = self.node
        started, release = threading.Event(), threading.Event()
        def slow_detect(frame):
            started.set()
            if not release.wait(2.):
                raise RuntimeError('test inference timed out')
            return self.detections(frame)
        n.detector.detect.side_effect = slow_detect
        active = n.capture_object_snapshot()
        worker = threading.Thread(target=n.process_object_snapshot, args=(active,))
        worker.start()
        self.assertTrue(started.wait(1.))
        try:
            snapshots = []
            for index in range(6):
                item = dict(active)
                item['stamps'] = (99.90 + index * .01,) * 2
                item['observation_id'] = item['stamps'][0]
                snapshots.append(item)
                n.enqueue_object_snapshot(item)
            self.assertEqual(n.object_snapshots_captured, 6)
            self.assertEqual(n.object_snapshots_replaced, 1)
            self.assertEqual(len(n.object_pending_snapshots), 5)
            self.assertAlmostEqual(n.object_pending_snapshots[0]['stamps'][0], 99.91)
            self.assertAlmostEqual(n.object_pending_snapshots[-1]['stamps'][0], 99.95)
        finally:
            release.set()
            worker.join(2.)
        self.assertFalse(worker.is_alive())

    def test_snapshot_acquisition_grace_is_bounded(self):
        n = self.node
        n.capture_object_snapshot = Mock(return_value=None)
        self.assertFalse(n.capture_endpoint_snapshot(100.))
        self.assertFalse(n.capture_endpoint_snapshot(100.9))
        self.assertTrue(n.capture_endpoint_snapshot(101.))
        self.assertEqual(n.capture_object_snapshot.call_count, 3)
        self.assertIn('captured 0/5 distinct frames within 1.00 s grace', n.object_status)

    def test_map_generation_change_purges_and_invalidates_snapshot(self):
        n = self.node
        snapshot = n.capture_object_snapshot()
        n.enqueue_object_snapshot(snapshot)
        self.assertEqual(len(n.object_pending_snapshots), 1)
        n.clear_object_tracks()
        self.assertEqual(n.object_pending_snapshots, [])
        self.assertEqual(n.map_generation, 1)
        n.process_object_snapshot(snapshot)
        self.assertEqual(n.object_tracker.tracks, [])
        self.assertIn('map generation changed', n.object_status)

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

    def test_motion_after_capture_commits_map_result_but_clears_overlay(self):
        n = self.node
        n.motion_pub = Mock()
        def infer(frame):
            n.set_motion(True)
            n.set_motion(False)
            return self.detections(frame)
        n.detector.detect.side_effect = infer
        n.process_objects()
        self.assertEqual(len(n.object_tracker.tracks), 1)
        self.assertEqual(n.object_detections, [])
        self.assertEqual(n.object_detections_stamp, 0.)

    def test_legacy_focus_mode_discards_result_if_motion_changes(self):
        n = self.node
        n.object_focus_enabled = True
        n.motion_pub = Mock()
        def infer(frame):
            n.set_motion(True)
            n.set_motion(False)
            return self.detections(frame)
        n.detector.detect.side_effect = infer
        n.process_objects()
        self.assertEqual(n.object_tracker.tracks, [])
        self.assertEqual(n.object_detections, [])

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
        n.object_pending_snapshots = [{'queued': True}]
        def infer(frame):
            n.shutdown()
            return self.detections(frame)
        n.detector.detect.side_effect = infer
        n.process_objects()
        n.send.assert_called_once_with(('stop',))
        self.assertEqual(n.object_pending_snapshots, [])
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
        self.assertEqual(len(n.object_tracker.tracks), 2)
        self.assertEqual([track['count'] for track in n.object_tracker.tracks], [1, 1])


if __name__ == '__main__':
    unittest.main()
