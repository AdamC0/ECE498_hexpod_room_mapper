#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace cuda_depth_registration {

struct Calibration {
  float depth_fx;
  float depth_fy;
  float depth_cx;
  float depth_cy;
  float rgb_fx;
  float rgb_fy;
  float rgb_cx;
  float rgb_cy;
  // Factory transform from the RGB/right camera into the depth/left camera.
  // Translation is converted from factory millimetres to metres by the node.
  float right_to_left_rotation[9];
  float right_to_left_translation[3];
  // ROS plumb_bob order: k1, k2, p1, p2, k3.
  float rgb_distortion[5];
};

class Registrar {
 public:
  Registrar();
  ~Registrar();

  Registrar(const Registrar&) = delete;
  Registrar& operator=(const Registrar&) = delete;

  bool registerDepth(const std::uint16_t* input,
                     int input_width,
                     int input_height,
                     std::uint16_t* output,
                     int output_width,
                     int output_height,
                     const Calibration& calibration,
                     float max_depth_metres,
                     unsigned int hole_fill_max_delta_mm,
                     float* elapsed_ms,
                     std::string* error);

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace cuda_depth_registration
