import unittest
import xml.etree.ElementTree as ET

from tests.support import PROJECT_ROOT
from tools.fit_registered_depth_alignment import fit_alignment


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

    def test_production_launch_expands_projection_about_optical_centre(self):
        root = ET.parse(
            str(self.root / 'launch' / 'cuda_depth_registration.launch')).getroot()
        args = {item.attrib['name']: item.attrib.get('default')
                for item in root.findall('arg')}
        self.assertEqual(args['pixel_scale_x'], '1.05')
        self.assertEqual(args['pixel_scale_y'], '1.05')
        self.assertEqual(args['pixel_offset_x'], '0.0')
        self.assertEqual(args['pixel_offset_y'], '0.0')
        params = {item.attrib['name']: item.attrib.get('value')
                  for item in root.find('node').findall('param')}
        self.assertEqual(params['pixel_scale_x'], '$(arg pixel_scale_x)')
        self.assertEqual(params['pixel_scale_y'], '$(arg pixel_scale_y)')
        self.assertEqual(params['pixel_offset_x'], '$(arg pixel_offset_x)')
        self.assertEqual(params['pixel_offset_y'], '$(arg pixel_offset_y)')

        # A focal scale expands around the principal point rather than moving
        # both sides in the same direction like a constant pixel translation.
        cx, cy = 319.5, 239.5
        scale_x = float(args['pixel_scale_x'])
        scale_y = float(args['pixel_scale_y'])
        project = lambda u, v: (
            cx + scale_x * (u - cx), cy + scale_y * (v - cy))
        lower_left = project(80.0, 420.0)
        lower_right = project(560.0, 420.0)
        self.assertLess(lower_left[0], 80.0)
        self.assertGreater(lower_left[1], 420.0)
        self.assertGreater(lower_right[0], 560.0)
        self.assertGreater(lower_right[1], 420.0)
        self.assertEqual(project(cx, cy), (cx, cy))

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
        self.assertIn('rgb_info.K[0]) * pixel_scale_x_', source)
        self.assertIn('rgb_info.K[4]) * pixel_scale_y_', source)
        self.assertIn('rgb_info.K[2]) + pixel_offset_x_', source)
        self.assertIn('rgb_info.K[5]) + pixel_offset_y_', source)

    def test_correspondence_fit_recovers_scale_and_residual_offset(self):
        centre = (319.5, 239.5)
        matches = []
        for x, y in [(70., 60.), (550., 70.), (85., 410.), (565., 405.),
                     (210., 150.), (440., 330.)]:
            matches.append({
                'depth': [x, y],
                'rgb': [centre[0] + 1.04 * (x - centre[0]) - 2.5,
                        centre[1] + 1.06 * (y - centre[1]) + 1.75],
            })
        result = fit_alignment(matches, centre)
        self.assertAlmostEqual(result['pixel_scale_x'], 1.04, places=10)
        self.assertAlmostEqual(result['pixel_scale_y'], 1.06, places=10)
        self.assertAlmostEqual(result['pixel_offset_x'], -2.5, places=10)
        self.assertAlmostEqual(result['pixel_offset_y'], 1.75, places=10)
        self.assertLess(result['rmse_pixels'], 1e-9)

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
