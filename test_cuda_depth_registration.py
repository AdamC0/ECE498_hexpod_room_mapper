import unittest
from pathlib import Path
import xml.etree.ElementTree as ET


class CudaDepthRegistrationTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).with_name('cuda_depth_registration')

    def test_trial_topic_cannot_replace_production_by_default(self):
        launch = ET.parse(str(self.root / 'launch' / 'cuda_depth_trial.launch')).getroot()
        args = {item.attrib['name']: item.attrib.get('default')
                for item in launch.findall('arg')}
        self.assertEqual(args['input_image'], '/camera/depth/image_raw')
        self.assertEqual(args['output_image'],
                         '/camera/depth_registered_cuda/image_raw')
        self.assertNotEqual(args['output_image'],
                            '/camera/depth_registered/image_raw')

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


if __name__ == '__main__':
    unittest.main()
