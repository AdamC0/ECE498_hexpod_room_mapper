import math
import unittest
from types import SimpleNamespace

import numpy as np

from object_mapping import (COCO_CLASSES, ObjectTracker, TransformCache, YoloOnnxDetector,
                            backproject, decode_depth_image, sample_box_depth,
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


if __name__ == '__main__':
    unittest.main()
