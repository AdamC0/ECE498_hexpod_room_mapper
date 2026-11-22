import unittest
import xml.etree.ElementTree as ET

from tests.support import PROJECT_ROOT


class CudaDepthRegistrationTests(unittest.TestCase):
    def setUp(self):
        self.root = PROJECT_ROOT / 'cuda_depth_registration'

    def test_compatibility_launch_keeps_cuda_topic_isolated(self):
        launch = ET.parse(str(self.root / 'launch' / 'cuda_depth_trial.launch')).getroot()
        args = {item.attrib['name']: item.attrib.get('default')
                for item in launch.findall('arg')}
        self.assertEqual(args['input_image'], '/camera/depth/image_raw')
        self.assertEqual(args['output_image'],
                         '/camera/depth_registered_cuda/image_raw')
        self.assertNotEqual(args['output_image'],
                            '/camera/depth_registered/image_raw')

    def test_production_launch_preserves_native_input(self):
        launch = ET.parse(
            str(self.root / 'launch' / 'cuda_depth_registration.launch')).getroot()
        args = {item.attrib['name']: item.attrib.get('default')
                for item in launch.findall('arg')}
        self.assertEqual(args['input_image'], '/camera/depth/image_raw')
        self.assertEqual(args['output_image'],
                         '/camera/depth_registered_cuda/image_raw')
        self.assertEqual(args['output_camera_info'],
                         '/camera/depth_registered_cuda/camera_info')

    def test_kernel_uses_z_buffer_and_factory_extrinsic_inverse(self):
        source = (self.root / 'src' / 'cuda_registration_kernel.cu').read_text(
            encoding='utf-8')
        self.assertIn('atomicMin', source)
        self.assertIn('R^T * (p_left - t)', source)
        self.assertIn('right_to_left_translation', source)
        self.assertIn('cudaEventElapsedTime', source)
        self.assertIn('neighbours >= 2', source)
        self.assertIn('maximum - minimum <= max_delta_mm', source)

    def test_node_preserves_stamp_and_uses_rgb_frame_and_calibration(self):
        source = (self.root / 'src' /
                  'cuda_depth_registration_node.cpp').read_text(encoding='utf-8')
        self.assertIn('output.header = message->header', source)
        self.assertIn('output.header.frame_id = rgb_info.header.frame_id', source)
        self.assertIn('GetCameraParams', source)
        self.assertIn('service.response.r2l_t[index] * 0.001f', source)

    def test_benchmark_times_equivalent_cpu_and_cuda_outputs(self):
        source = (self.root / 'src' /
                  'registration_benchmark_node.cpp').read_text(encoding='utf-8')
        self.assertIn('cpu.registerDepth', source)
        self.assertIn('gpu.registerDepth', source)
        self.assertIn('end_to_end_speedup', source)
        self.assertIn('valid_iou', source)
        self.assertIn('median_error_mm', source)


if __name__ == '__main__':
    unittest.main()
