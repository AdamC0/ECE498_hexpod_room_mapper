import unittest

from prepare_yolo_model import legacy_opencv_broadcast_shape, ordered_names


class PrepareYoloModelTests(unittest.TestCase):
    def test_legacy_opencv_box_scale_broadcast_gets_explicit_axis(self):
        self.assertEqual(
            legacy_opencv_broadcast_shape([1, 4, 8400], [1, 8400]),
            [1, 4, 8400],
        )

    def test_unrelated_broadcast_shapes_are_unchanged(self):
        self.assertIsNone(
            legacy_opencv_broadcast_shape([1, 4, 8400], [8400]),
        )
        self.assertIsNone(
            legacy_opencv_broadcast_shape([1, 4, 8400], [2, 8400]),
        )

    def test_label_order_uses_numeric_class_ids(self):
        self.assertEqual(ordered_names({1: "chair", 0: "desk"}),
                         ["desk", "chair"])


if __name__ == "__main__":
    unittest.main()
