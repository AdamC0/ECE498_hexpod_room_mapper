#include "cuda_depth_registration/cpu_registration.hpp"

#include <algorithm>
#include <cmath>
#include <climits>
#include <cstddef>
#include <vector>

namespace cuda_depth_registration {

bool CpuRegistrar::registerDepth(const std::uint16_t* input,
                                 int input_width,
                                 int input_height,
                                 std::uint16_t* output,
                                 int output_width,
                                 int output_height,
                                 const Calibration& calibration,
                                 float max_depth_metres,
                                 unsigned int hole_fill_max_delta_mm,
                                 std::string* error) {
  if (input == nullptr || output == nullptr || input_width <= 0 || input_height <= 0 ||
      output_width <= 0 || output_height <= 0 || max_depth_metres <= 0.0f ||
      calibration.depth_fx <= 0.0f || calibration.depth_fy <= 0.0f ||
      calibration.rgb_fx <= 0.0f || calibration.rgb_fy <= 0.0f) {
    if (error) *error = "invalid CPU registration input";
    return false;
  }

  const std::size_t output_count =
      static_cast<std::size_t>(output_width) * output_height;
  z_buffer_.assign(output_count, UINT_MAX);
  const float* r = calibration.right_to_left_rotation;
  const float* t = calibration.right_to_left_translation;
  const float* d = calibration.rgb_distortion;
  for (int v = 0; v < input_height; ++v) {
    for (int u = 0; u < input_width; ++u) {
      const std::uint16_t depth_mm = input[v * input_width + u];
      if (depth_mm == 0) continue;
      const float z_left = static_cast<float>(depth_mm) * 0.001f;
      if (z_left <= 0.0f || z_left > max_depth_metres) continue;
      const float x_left = (static_cast<float>(u) - calibration.depth_cx) *
                           z_left / calibration.depth_fx;
      const float y_left = (static_cast<float>(v) - calibration.depth_cy) *
                           z_left / calibration.depth_fy;
      const float px = x_left - t[0];
      const float py = y_left - t[1];
      const float pz = z_left - t[2];
      const float x_right = r[0] * px + r[3] * py + r[6] * pz;
      const float y_right = r[1] * px + r[4] * py + r[7] * pz;
      const float z_right = r[2] * px + r[5] * py + r[8] * pz;
      if (z_right <= 0.0f || z_right > max_depth_metres) continue;

      const float xn = x_right / z_right;
      const float yn = y_right / z_right;
      const float radius2 = xn * xn + yn * yn;
      const float radial = 1.0f + d[0] * radius2 + d[1] * radius2 * radius2 +
                           d[4] * radius2 * radius2 * radius2;
      const float distorted_x = xn * radial + 2.0f * d[2] * xn * yn +
                                d[3] * (radius2 + 2.0f * xn * xn);
      const float distorted_y = yn * radial + d[2] * (radius2 + 2.0f * yn * yn) +
                                2.0f * d[3] * xn * yn;
      const int output_u = static_cast<int>(std::lrint(
          calibration.rgb_fx * distorted_x + calibration.rgb_cx));
      const int output_v = static_cast<int>(std::lrint(
          calibration.rgb_fy * distorted_y + calibration.rgb_cy));
      if (output_u < 0 || output_u >= output_width ||
          output_v < 0 || output_v >= output_height) continue;
      const unsigned int registered_mm = static_cast<unsigned int>(
          std::min(z_right * 1000.0f, 65535.0f));
      unsigned int& destination = z_buffer_[output_v * output_width + output_u];
      destination = std::min(destination, registered_mm);
    }
  }

  for (int y = 0; y < output_height; ++y) {
    for (int x = 0; x < output_width; ++x) {
      const std::size_t index = static_cast<std::size_t>(y) * output_width + x;
      const unsigned int value = z_buffer_[index];
      if (value != UINT_MAX) {
        output[index] = static_cast<std::uint16_t>(value);
        continue;
      }
      unsigned int minimum = UINT_MAX;
      unsigned int maximum = 0;
      int neighbours = 0;
      const int offsets[4] = {-1, 1, -output_width, output_width};
      const bool allowed[4] = {
          x > 0, x + 1 < output_width, y > 0, y + 1 < output_height};
      for (int offset_index = 0; offset_index < 4; ++offset_index) {
        if (!allowed[offset_index]) continue;
        const unsigned int neighbour = z_buffer_[index + offsets[offset_index]];
        if (neighbour == UINT_MAX) continue;
        minimum = std::min(minimum, neighbour);
        maximum = std::max(maximum, neighbour);
        ++neighbours;
      }
      output[index] =
          (neighbours >= 2 && maximum - minimum <= hole_fill_max_delta_mm)
              ? static_cast<std::uint16_t>(minimum)
              : 0;
    }
  }
  return true;
}

}  // namespace cuda_depth_registration
