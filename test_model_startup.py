"""Workstation-safe regression tests for detector startup and rollback."""
import unittest
from unittest.mock import Mock, patch

from room_mapper import RoomMapper


class ModelStartupTests(unittest.TestCase):
    def node(self):
        n = RoomMapper.__new__(RoomMapper)
        n.yolo_model = "/models/preferred.onnx"
        n.yolo_fallback_model = "/models/fallback.onnx"
        n.yolo_labels = ""
        n.yolo_input_size = 640
        n.yolo_backend = "cpu"
        n.object_confidence = .45
        n.object_priority_confidence = .30
        n.object_priority_classes = ["desk"]
        n.object_classes = []
        n.opencv_threads = 2
        n.use_depth_map = True
        n.registered_depth_topic = "/registered"
        n.camera_info_topic = "/camera_info"
        n.ensure_tf_listener = Mock()
        n.registered_depth_callback = Mock()
        n.camera_info_callback = Mock()
        n.object_loop = Mock()
        n.detector = None
        return n

    @patch("room_mapper.threading.Thread")
    @patch("room_mapper.rospy.Subscriber")
    @patch("room_mapper.cv2.setNumThreads")
    @patch("room_mapper.YoloOnnxDetector")
    def test_missing_preferred_model_uses_retained_fallback(
            self, detector_type, _threads, _subscriber, thread_type):
        n = self.node()
        detector = detector_type.return_value
        detector.warmup.return_value = None
        with patch("room_mapper.os.path.isfile",
                   side_effect=lambda path: path == n.yolo_fallback_model):
            n.start_object_detection()
        detector_type.assert_called_once_with(
            n.yolo_fallback_model, labels=None, input_size=640, confidence=.45,
            class_thresholds={"desk": .30}, allowed_labels=[], backend="cpu")
        detector.warmup.assert_called_once_with()
        self.assertIs(n.detector, detector)
        self.assertEqual(n.active_yolo_model, n.yolo_fallback_model)
        self.assertIn("fallback.onnx (cpu) ready", n.object_status)
        thread_type.return_value.start.assert_called_once_with()

    @patch("room_mapper.threading.Thread")
    @patch("room_mapper.rospy.Subscriber")
    @patch("room_mapper.cv2.setNumThreads")
    @patch("room_mapper.YoloOnnxDetector")
    def test_cuda_failure_retries_same_preferred_model_on_cpu(
            self, detector_type, _threads, _subscriber, _thread_type):
        n = self.node()
        n.yolo_backend = "cuda_fp16"
        cuda_detector, cpu_detector = Mock(), Mock()
        cuda_detector.warmup.side_effect = RuntimeError("unsupported CUDA target")
        detector_type.side_effect = [cuda_detector, cpu_detector]
        with patch("room_mapper.os.path.isfile",
                   side_effect=lambda path: path.endswith(".onnx")):
            n.start_object_detection()
        self.assertEqual(detector_type.call_args_list[0][1]["backend"], "cuda_fp16")
        self.assertEqual(detector_type.call_args_list[1][1]["backend"], "cpu")
        self.assertIs(n.detector, cpu_detector)
        self.assertEqual(n.active_yolo_model, n.yolo_model)
        self.assertEqual(n.active_yolo_backend, "cpu")

    @patch("room_mapper.threading.Thread")
    @patch("room_mapper.rospy.Subscriber")
    @patch("room_mapper.cv2.setNumThreads")
    @patch("room_mapper.YoloOnnxDetector")
    def test_invalid_preferred_model_rolls_back_after_warmup(
            self, detector_type, _threads, _subscriber, _thread_type):
        n = self.node()
        preferred, fallback = Mock(), Mock()
        preferred.warmup.side_effect = ValueError("unsupported output")
        detector_type.side_effect = [preferred, fallback]
        with patch("room_mapper.os.path.isfile",
                   side_effect=lambda path: path.endswith(".onnx")):
            n.start_object_detection()
        self.assertEqual([item[0][0] for item in detector_type.call_args_list],
                         [n.yolo_model, n.yolo_fallback_model])
        self.assertIs(n.detector, fallback)
        self.assertEqual(n.active_yolo_model, n.yolo_fallback_model)

    @patch("room_mapper.cv2.setNumThreads")
    def test_missing_models_disable_detection_without_starting_thread(self, _threads):
        n = self.node()
        with patch("room_mapper.os.path.isfile", return_value=False), \
                patch("room_mapper.threading.Thread") as thread_type:
            n.start_object_detection()
        self.assertIsNone(n.detector)
        self.assertIn("startup failed", n.object_status)
        self.assertIn("preferred.onnx", n.object_status)
        self.assertIn("fallback.onnx", n.object_status)
        thread_type.assert_not_called()


if __name__ == "__main__":
    unittest.main()
