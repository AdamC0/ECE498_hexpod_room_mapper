#pragma once

#include "cuda_depth_registration/registration.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace cuda_depth_registration {

class CpuRegistrar {
 public:
  bool registerDepth(const std::uint16_t* input,
                     int input_width,
                     int input_height,
                     std::uint16_t* output,
                     int output_width,
                     int output_height,
                     const Calibration& calibration,
                     float max_depth_metres,
                     unsigned int hole_fill_max_delta_mm,
                     std::string* error);

 private:
  std::vector<unsigned int> z_buffer_;
};

}  // namespace cuda_depth_registration
