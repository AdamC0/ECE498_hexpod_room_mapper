import unittest

from benchmark_yolo_backends import compare_results


class BenchmarkParityTests(unittest.TestCase):
    def test_small_fp16_differences_pass(self):
        cpu = [{"image": "frame.jpg", "detections": [
            {"label": "chair", "confidence": .51, "box": [10, 20, 110, 220]}]}]
        gpu = [{"image": "frame.jpg", "detections": [
            {"label": "chair", "confidence": .53, "box": [11, 20, 111, 221]}]}]
        comparison = compare_results(cpu, gpu)
        self.assertTrue(comparison["passed"])
        self.assertGreater(comparison["minimum_iou"], .95)

    def test_missing_or_materially_shifted_detection_fails(self):
        cpu = [{"image": "frame.jpg", "detections": [
            {"label": "chair", "confidence": .8, "box": [10, 20, 110, 220]}]}]
        missing = [{"image": "frame.jpg", "detections": []}]
        shifted = [{"image": "frame.jpg", "detections": [
            {"label": "chair", "confidence": .8, "box": [200, 20, 300, 220]}]}]
        self.assertFalse(compare_results(cpu, missing)["passed"])
        self.assertFalse(compare_results(cpu, shifted)["passed"])


if __name__ == "__main__":
    unittest.main()
