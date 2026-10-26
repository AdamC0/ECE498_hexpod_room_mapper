import hashlib
import json
import math
import os
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

from object_mapping import (COCO_CLASSES, ObjectTracker, TensorRtYoloDetector,
                            TransformCache, YoloOnnxDetector, backproject,
                            decode_depth_image, load_labels, sample_box_depth,
                            transform_point)


class ObjectMappingTests(unittest.TestCase):
    def test_padded_depth_decode(self):
        raw = np.array([[1000, 2000, 999], [3000, 4000, 999]], dtype='<u2')
        msg = SimpleNamespace(encoding='16UC1', is_bigendian=False, width=2,
                              height=2, step=6, data=raw.tobytes())
        depth = decode_depth_image(msg)
        np.testing.assert_allclose(depth, [[1, 2], [3, 4]])

    def test_depth_sampling_ignores_box_edges_and_invalid_pixels(self):
        depth = np.zeros((100, 100), np.float32)
        depth[30:70, 30:70] = 2.25
        result = sample_box_depth(depth, (0, 0, 100, 100), min_pixels=20)
        self.assertAlmostEqual(result[0], 2.25)
        self.assertEqual(result[1:], (50.0, 50.0))

    def test_depth_sampling_falls_back_near_center_for_sparse_surface(self):
        depth = np.zeros((100, 100), np.float32)
        depth[35:38, 45:49] = 1.75
        result = sample_box_depth(depth, (10, 10, 90, 90), min_pixels=12)
        self.assertAlmostEqual(result[0], 1.75)

    def test_depth_sampling_reports_surface_quality(self):
        depth = np.full((100, 100), 2.0, np.float32)
        distance, _, _, quality = sample_box_depth(
            depth, (10, 10, 90, 90), min_pixels=24, return_quality=True)
        self.assertEqual(distance, 2.0)
        self.assertGreaterEqual(quality['valid_pixels'], 24)
        self.assertEqual(quality['coverage'], 1.0)
        self.assertEqual(quality['mad'], 0.0)
        self.assertEqual(quality['inlier_ratio'], 1.0)

    def test_backproject_and_transform(self):
        k = (100, 0, 50, 0, 100, 40, 0, 0, 1)
        point = backproject(60, 20, 2, k)
        np.testing.assert_allclose(point, (.2, -.4, 2.0))
        # 90 degree yaw followed by a translation.
        q = (0, 0, math.sin(math.pi/4), math.cos(math.pi/4))
        mapped = transform_point(point, (1, 2, 0), q)
        np.testing.assert_allclose(mapped, (1.4, 2.2, 2.0), atol=1e-6)

    def test_transform_cache_chains_static_and_dynamic_frames(self):
        cache = TransformCache()
        cache.update('map', 'base', (1, 2, 0), (0, 0, 0, 1), 10)
        cache.update('base', 'camera', (.2, 0, .5), (0, 0, 0, 1), 0, True)
        transform = cache.lookup('map', 'camera', 10)
        np.testing.assert_allclose(transform_point((0, 0, 1), *transform),
                                   (1.2, 2.0, 1.5), atol=1e-6)

    def test_tracker_requires_repeated_observations_and_merges(self):
        tracker = ObjectTracker(merge_distance=.5, min_observations=3)
        tracker.update('chair', 1.0, 2.0, .8, 10)
        tracker.update('chair', 1.1, 2.0, .9, 11)
        self.assertEqual(tracker.snapshot(True, 11), [])
        tracker.update('chair', .9, 2.0, .85, 12)
        tracks = tracker.snapshot(True, 12)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0]['count'], 3)
        self.assertAlmostEqual(tracks[0]['x'], 1.0)

    def test_expected_class_confirms_after_three_high_quality_hits(self):
        tracker = ObjectTracker(priority_labels=['chair'])
        for observation, x in enumerate((1.00, 1.03, .98), 1):
            tracker.update('chair', x, 2.0, .72, observation,
                           observation_id=observation,
                           validation_group=('endpoint', 1),
                           box=(10, 10, 60, 80))
        track = tracker.snapshot(True, 3)[0]
        self.assertTrue(track['confirmed'])
        self.assertEqual(track['tier'], 'expected')

    def test_other_class_requires_four_stronger_hits(self):
        tracker = ObjectTracker(priority_labels=['chair'])
        for observation in range(1, 4):
            tracker.update('faucet', 1.0, 2.0, .82, observation,
                           observation_id=observation,
                           validation_group=('endpoint', 1),
                           box=(10, 10, 60, 80))
        self.assertEqual(tracker.snapshot(True, 3), [])
        tracker.update('faucet', 1.01, 2.0, .80, 4,
                       observation_id=4, validation_group=('endpoint', 1),
                       box=(10, 10, 60, 80))
        self.assertTrue(tracker.snapshot(True, 4)[0]['confirmed'])
        self.assertEqual(tracker.snapshot(True, 4)[0]['tier'], 'other')

    def test_high_count_does_not_override_low_confidence(self):
        tracker = ObjectTracker(priority_labels=['chair'])
        for observation in range(1, 4):
            tracker.update('chair', 1.0, 2.0, .60, observation,
                           observation_id=observation,
                           validation_group=('endpoint', 1),
                           box=(10, 10, 60, 80))
        self.assertEqual(tracker.snapshot(True, 3), [])
        pending = tracker.snapshot(False, 3)[0]
        self.assertIn('median confidence 0.60 is below 0.65',
                      pending['validation_reason'])

    def test_high_confidence_does_not_override_box_inconsistency(self):
        tracker = ObjectTracker(priority_labels=['chair'])
        boxes = ((10, 10, 60, 80), (100, 100, 150, 180), (10, 10, 60, 80))
        for observation, box in enumerate(boxes, 1):
            tracker.update('chair', 1.0, 2.0, .75, observation,
                           observation_id=observation,
                           validation_group=('endpoint', 1), box=box)
        self.assertEqual(tracker.snapshot(True, 3), [])
        pending = tracker.snapshot(False, 3)[0]
        self.assertIn('box overlap', pending['validation_reason'])
        self.assertEqual(pending['validation']['required_hits'], 3)

    def test_two_same_view_misses_remove_pending_candidate(self):
        tracker = ObjectTracker(priority_labels=['chair'])
        group = ('endpoint', 1)
        track = tracker.update('chair', 1.0, 2.0, .75, 1,
                               observation_id=1, validation_group=group,
                               box=(10, 10, 60, 80))
        tracker.finish_observation(group, 2, [])
        self.assertEqual(len(tracker.tracks), 1)
        tracker.finish_observation(group, 3, [])
        self.assertEqual(tracker.tracks, [])
        self.assertIsNotNone(track)

    def test_confirmed_track_persists_for_mapping_session(self):
        tracker = ObjectTracker(max_age=30, priority_labels=['chair'])
        for observation in range(1, 4):
            tracker.update('chair', 1.0, 2.0, .75, observation,
                           observation_id=observation,
                           validation_group=('endpoint', 1),
                           box=(10, 10, 60, 80))
        tracker.prune(1000)
        self.assertEqual(len(tracker.snapshot(True, 1000)), 1)

    def test_confirmation_uses_only_qualifying_hits_and_freezes_position(self):
        tracker = ObjectTracker(priority_labels=['chair'])
        group = ('endpoint', 1)
        tracker.update('chair', 1.60, 2.0, .40, 1,
                       observation_id=1, validation_group=group,
                       box=(10, 10, 60, 80))
        for observation, x in enumerate((1.00, 1.02, .98), 2):
            tracker.update('chair', x, 2.0, .75, observation,
                           observation_id=observation, validation_group=group,
                           box=(10, 10, 60, 80))
        track = tracker.snapshot(True, 4)[0]
        self.assertAlmostEqual(track['x'], 1.0)
        self.assertEqual(track['count'], 3)
        tracker.update('chair', 1.50, 2.0, .35, 5,
                       observation_id=5, validation_group=('endpoint', 2),
                       box=(10, 10, 60, 80))
        self.assertAlmostEqual(tracker.snapshot(True, 5)[0]['x'], 1.0)

    def test_tracker_rename_and_remove_are_persistent(self):
        tracker = ObjectTracker(merge_distance=.5, min_observations=1)
        track = tracker.update('chair', 1.0, 2.0, .8, 10)
        self.assertTrue(tracker.rename(track['id'], 'Reading chair'))
        tracker.update('chair', 1.1, 2.0, .9, 11)
        self.assertEqual(tracker.snapshot(True, 11)[0]['name'], 'Reading chair')
        self.assertTrue(tracker.remove(track['id']))
        self.assertEqual(tracker.snapshot(False, 12), [])
        self.assertIsNone(tracker.update('chair', 1.1, 2.0, .9, 13))
        self.assertIsNotNone(tracker.update('chair', 2.0, 2.0, .9, 14))

    def test_discard_does_not_suppress_future_detection(self):
        tracker = ObjectTracker(min_observations=2)
        track = tracker.update('chair', 1.0, 2.0, .8, 10)
        self.assertTrue(tracker.discard(track['id']))
        self.assertEqual(tracker.tracks, [])
        self.assertIsNotNone(tracker.update('chair', 1.0, 2.0, .8, 11))

    def test_tracker_rejects_invalid_display_names(self):
        tracker = ObjectTracker(min_observations=1)
        track = tracker.update('bottle', 0., 0., .8, 10)
        self.assertFalse(tracker.rename(track['id'], ''))
        self.assertFalse(tracker.rename(track['id'], 'x' * 41))

    def test_transform_inverse_round_trip(self):
        cache = TransformCache()
        cache.update('map', 'base', (1, 2, 0),
                     (0, 0, math.sin(math.pi/4), math.cos(math.pi/4)), 10)
        cache.update('base', 'camera', (.065, 0, .07), (0, 0, 0, 1), 0, True)
        point = (.2, -.3, 2.)
        mapped = transform_point(point, *cache.lookup('map', 'camera', 10))
        restored = transform_point(mapped, *cache.lookup('camera', 'map', 10))
        np.testing.assert_allclose(restored, point, atol=1e-6)

    def test_stale_disconnected_and_cyclic_tf_are_rejected(self):
        cache = TransformCache()
        cache.update('map', 'base', (0, 0, 0), (0, 0, 0, 1), 10)
        with self.assertRaises(KeyError):
            cache.lookup('map', 'base', 20)
        with self.assertRaises(KeyError):
            cache.lookup('map', 'unknown', 10)
        cache.update('base', 'map', (0, 0, 0), (0, 0, 0, 1), 10)
        with self.assertRaises(ValueError):
            cache.lookup('map', 'base', 10)

    def test_invalid_intrinsics_cannot_produce_nan_coordinates(self):
        with self.assertRaises(ValueError):
            backproject(50, 50, 2, (float('nan'), 0, 50, 0, 100, 50, 0, 0, 1))

    def test_yolov5_objectness_and_letterbox_decode(self):
        detector = YoloOnnxDetector.__new__(YoloOnnxDetector)
        detector.labels = list(COCO_CLASSES)
        detector.confidence, detector.nms = .4, .45
        output = np.zeros((1, 2, len(COCO_CLASSES) + 5), np.float32)
        output[0, :, :5] = (320, 320, 100, 200, .8)
        output[0, 0, 5 + COCO_CLASSES.index('chair')] = .9
        output[0, 1, 4] = .1
        output[0, 1, 5 + COCO_CLASSES.index('bottle')] = .99
        detections = detector._decode(output, 1., 0, 80, 640, 480)
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]['box'], (270, 140, 370, 340))
        self.assertAlmostEqual(detections[0]['confidence'], .72, places=6)

    def test_yolov8_output_decode(self):
        detector = YoloOnnxDetector.__new__(YoloOnnxDetector)
        detector.labels = list(COCO_CLASSES)
        detector.confidence, detector.nms = .4, .45
        output = np.zeros((1, len(COCO_CLASSES) + 4, 1), np.float32)
        output[0, :4, 0] = (320, 320, 100, 200)
        output[0, 4 + COCO_CLASSES.index('chair'), 0] = .9
        detections = detector._decode(output, 1.0, 0, 0, 640, 640)
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]['label'], 'chair')
        self.assertEqual(detections[0]['box'], (270, 220, 370, 420))

    def test_priority_threshold_and_class_aware_nms_keep_desk_and_chair(self):
        detector = YoloOnnxDetector.__new__(YoloOnnxDetector)
        detector.labels = ['chair', 'desk', 'car']
        detector.confidence, detector.nms = .45, .45
        detector.class_thresholds = {'chair': .30, 'desk': .30}
        detector.allowed_labels = None
        output = np.zeros((1, 7, 3), np.float32)
        output[0, :4, :] = np.array([[320], [320], [200], [200]], np.float32)
        output[0, 4, 0] = .40  # chair, accepted at classroom threshold
        output[0, 5, 1] = .40  # overlapping desk, retained by class-aware NMS
        output[0, 6, 2] = .40  # car, rejected at the general threshold
        detections = detector._decode(output, 1., 0, 0, 640, 640)
        self.assertEqual({item['label'] for item in detections}, {'chair', 'desk'})

    def test_vectorized_decode_preserves_allowed_classes_and_thresholds(self):
        detector = YoloOnnxDetector.__new__(YoloOnnxDetector)
        detector.labels = ['chair', 'desk', 'car']
        detector.confidence, detector.nms = .45, .45
        detector.class_thresholds = {'chair': .30, 'desk': .30}
        detector.allowed_labels = {'chair', 'desk'}
        output = np.zeros((1, 7, 4), np.float32)
        output[0, :4, :] = np.array([
            [80, 240, 400, 560], [80, 240, 400, 560],
            [40, 40, 40, 40], [40, 40, 40, 40]
        ], np.float32)
        output[0, 4, 0] = .31  # allowed chair above its priority threshold
        output[0, 5, 1] = .40  # allowed desk above its priority threshold
        output[0, 6, 2] = .95  # disallowed car despite high confidence
        output[0, 4, 3] = .29  # allowed chair below its priority threshold
        detections = detector._decode(output, 1., 0, 0, 640, 640)
        self.assertEqual([item['label'] for item in detections], ['chair', 'desk'])
        np.testing.assert_allclose(
            [item['confidence'] for item in detections], [.31, .40], rtol=0, atol=1e-6)

    def test_end_to_end_six_value_output_decode(self):
        detector = YoloOnnxDetector.__new__(YoloOnnxDetector)
        detector.labels = ['chair', 'desk', 'car']
        detector.confidence, detector.nms = .45, .45
        detector.class_thresholds = {}
        detector.allowed_labels = None
        output = np.array([[[10., 20., 110., 220., .9, 1.]]], np.float32)
        detections = detector._decode(output, 1., 0, 0, 640, 640)
        self.assertEqual(detections[0]['label'], 'desk')
        self.assertEqual(detections[0]['box'], (10, 20, 110, 220))

    def test_external_model_labels_are_loaded_in_order(self):
        handle, path = tempfile.mkstemp(suffix='.names')
        os.close(handle)
        self.addCleanup(lambda: os.path.exists(path) and os.unlink(path))
        with open(path, 'w', encoding='utf-8') as stream:
            stream.write('Chair\nDesk\nProjector\n')
        self.assertEqual(load_labels(path), ['Chair', 'Desk', 'Projector'])

    def test_tensorrt_engine_sidecar_binds_exact_model_and_engine(self):
        directory = tempfile.mkdtemp()
        self.addCleanup(lambda: __import__('shutil').rmtree(directory))
        model = os.path.join(directory, 'model.onnx')
        engine = os.path.join(directory, 'model.engine')
        with open(model, 'wb') as stream:
            stream.write(b'model-v1')
        with open(engine, 'wb') as stream:
            stream.write(b'engine-v1')
        def checksum(path):
            with open(path, 'rb') as source:
                return hashlib.sha256(source.read()).hexdigest()
        with open(engine + '.json', 'w', encoding='utf-8') as stream:
            json.dump({'precision': 'fp16', 'onnx_sha256': checksum(model),
                       'engine_sha256': checksum(engine)}, stream)
        detector = TensorRtYoloDetector.__new__(TensorRtYoloDetector)
        detector.engine_path = engine
        detector._validate_sidecar(model)
        with open(model, 'wb') as stream:
            stream.write(b'model-v2')
        with self.assertRaisesRegex(ValueError, 'different ONNX'):
            detector._validate_sidecar(model)


if __name__ == '__main__':
    unittest.main()
