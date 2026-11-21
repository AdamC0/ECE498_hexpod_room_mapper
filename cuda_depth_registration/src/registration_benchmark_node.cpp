#include "cuda_depth_registration/cpu_registration.hpp"
#include "cuda_depth_registration/registration.hpp"

#include <astra_camera/GetCameraParams.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace cuda_depth_registration {
namespace {

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double position = fraction * static_cast<double>(values.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double weight = position - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

}  // namespace

class BenchmarkNode {
 public:
  BenchmarkNode() : private_node_("~") {
    private_node_.param<std::string>("input_image", input_image_,
                                     "/camera/depth/image_raw");
    private_node_.param<std::string>("depth_camera_info", depth_info_topic_,
                                     "/camera/depth/camera_info");
    private_node_.param<std::string>("rgb_camera_info", rgb_info_topic_,
                                     "/camera/rgb/camera_info");
    private_node_.param<std::string>("calibration_service", calibration_service_,
                                     "/camera/get_camera_params");
    private_node_.param("iterations", iterations_, 100);
    private_node_.param("warmup_iterations", warmup_iterations_, 10);
    private_node_.param("max_depth_metres", max_depth_metres_, 8.0f);
    int hole_fill_delta = 50;
    private_node_.param("hole_fill_max_delta_mm", hole_fill_delta, 50);
    if (iterations_ < 5 || iterations_ > 1000 || warmup_iterations_ < 1 ||
        warmup_iterations_ > 100 || hole_fill_delta < 0 || hole_fill_delta > 500) {
      throw std::runtime_error("invalid benchmark iteration or hole-fill parameter");
    }
    hole_fill_max_delta_mm_ = static_cast<unsigned int>(hole_fill_delta);
    calibration_client_ = node_.serviceClient<astra_camera::GetCameraParams>(
        calibration_service_, true);
    depth_info_subscriber_ = node_.subscribe(
        depth_info_topic_, 1, &BenchmarkNode::depthInfoCallback, this);
    rgb_info_subscriber_ = node_.subscribe(
        rgb_info_topic_, 1, &BenchmarkNode::rgbInfoCallback, this);
    image_subscriber_ = node_.subscribe(
        input_image_, 1, &BenchmarkNode::imageCallback, this,
        ros::TransportHints().tcpNoDelay());
    ROS_INFO("Waiting for one native depth frame for equivalent CPU/CUDA benchmark");
  }

 private:
  void depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    depth_info_ = *message;
    have_depth_info_ = true;
  }

  void rgbInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    rgb_info_ = *message;
    have_rgb_info_ = true;
  }

  bool loadCalibration(const sensor_msgs::CameraInfo& depth_info,
                       const sensor_msgs::CameraInfo& rgb_info,
                       Calibration* calibration) {
    astra_camera::GetCameraParams service;
    if (!calibration_client_.call(service)) return false;
    calibration->depth_fx = static_cast<float>(depth_info.K[0]);
    calibration->depth_fy = static_cast<float>(depth_info.K[4]);
    calibration->depth_cx = static_cast<float>(depth_info.K[2]);
    calibration->depth_cy = static_cast<float>(depth_info.K[5]);
    calibration->rgb_fx = static_cast<float>(rgb_info.K[0]);
    calibration->rgb_fy = static_cast<float>(rgb_info.K[4]);
    calibration->rgb_cx = static_cast<float>(rgb_info.K[2]);
    calibration->rgb_cy = static_cast<float>(rgb_info.K[5]);
    for (std::size_t index = 0; index < 9; ++index) {
      calibration->right_to_left_rotation[index] = service.response.r2l_r[index];
    }
    for (std::size_t index = 0; index < 3; ++index) {
      calibration->right_to_left_translation[index] =
          service.response.r2l_t[index] * 0.001f;
    }
    std::fill(calibration->rgb_distortion, calibration->rgb_distortion + 5, 0.0f);
    for (std::size_t index = 0; index < std::min<std::size_t>(5, rgb_info.D.size());
         ++index) {
      calibration->rgb_distortion[index] = static_cast<float>(rgb_info.D[index]);
    }
    return calibration->depth_fx > 0.0f && calibration->depth_fy > 0.0f &&
           calibration->rgb_fx > 0.0f && calibration->rgb_fy > 0.0f;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    if (complete_) return;
    sensor_msgs::CameraInfo depth_info;
    sensor_msgs::CameraInfo rgb_info;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_depth_info_ || !have_rgb_info_) return;
      depth_info = depth_info_;
      rgb_info = rgb_info_;
      complete_ = true;
    }
    if ((message->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
         message->encoding != sensor_msgs::image_encodings::MONO16) ||
        message->is_bigendian || message->step < message->width * 2 ||
        depth_info.width != message->width || depth_info.height != message->height) {
      fail("incompatible native depth frame");
      return;
    }
    Calibration calibration{};
    if (!loadCalibration(depth_info, rgb_info, &calibration)) {
      fail("could not load valid Astra calibration");
      return;
    }
    const std::size_t input_count =
        static_cast<std::size_t>(message->width) * message->height;
    const std::size_t output_count =
        static_cast<std::size_t>(rgb_info.width) * rgb_info.height;
    std::vector<std::uint16_t> input(input_count);
    for (std::uint32_t row = 0; row < message->height; ++row) {
      std::memcpy(input.data() + static_cast<std::size_t>(row) * message->width,
                  message->data.data() + static_cast<std::size_t>(row) * message->step,
                  message->width * sizeof(std::uint16_t));
    }
    std::vector<std::uint16_t> cpu_output(output_count);
    std::vector<std::uint16_t> gpu_output(output_count);
    CpuRegistrar cpu;
    Registrar gpu;
    std::string error;
    float kernel_ms = 0.0f;
    for (int iteration = 0; iteration < warmup_iterations_; ++iteration) {
      if (!cpu.registerDepth(input.data(), message->width, message->height,
                             cpu_output.data(), rgb_info.width, rgb_info.height,
                             calibration, max_depth_metres_, hole_fill_max_delta_mm_,
                             &error) ||
          !gpu.registerDepth(input.data(), message->width, message->height,
                             gpu_output.data(), rgb_info.width, rgb_info.height,
                             calibration, max_depth_metres_, hole_fill_max_delta_mm_,
                             &kernel_ms, &error)) {
        fail(error);
        return;
      }
    }

    std::vector<double> cpu_times;
    std::vector<double> gpu_wall_times;
    std::vector<double> gpu_kernel_times;
    cpu_times.reserve(iterations_);
    gpu_wall_times.reserve(iterations_);
    gpu_kernel_times.reserve(iterations_);
    for (int iteration = 0; iteration < iterations_; ++iteration) {
      auto started = std::chrono::steady_clock::now();
      if (!cpu.registerDepth(input.data(), message->width, message->height,
                             cpu_output.data(), rgb_info.width, rgb_info.height,
                             calibration, max_depth_metres_, hole_fill_max_delta_mm_,
                             &error)) {
        fail(error);
        return;
      }
      auto finished = std::chrono::steady_clock::now();
      cpu_times.push_back(std::chrono::duration<double, std::milli>(
          finished - started).count());

      started = std::chrono::steady_clock::now();
      if (!gpu.registerDepth(input.data(), message->width, message->height,
                             gpu_output.data(), rgb_info.width, rgb_info.height,
                             calibration, max_depth_metres_, hole_fill_max_delta_mm_,
                             &kernel_ms, &error)) {
        fail(error);
        return;
      }
      finished = std::chrono::steady_clock::now();
      gpu_wall_times.push_back(std::chrono::duration<double, std::milli>(
          finished - started).count());
      gpu_kernel_times.push_back(kernel_ms);
    }

    std::size_t cpu_valid = 0;
    std::size_t gpu_valid = 0;
    std::size_t joint = 0;
    std::size_t union_count = 0;
    std::vector<double> errors;
    for (std::size_t index = 0; index < output_count; ++index) {
      const bool cpu_has_depth = cpu_output[index] != 0;
      const bool gpu_has_depth = gpu_output[index] != 0;
      cpu_valid += cpu_has_depth;
      gpu_valid += gpu_has_depth;
      joint += cpu_has_depth && gpu_has_depth;
      union_count += cpu_has_depth || gpu_has_depth;
      if (cpu_has_depth && gpu_has_depth) {
        errors.push_back(std::abs(static_cast<double>(cpu_output[index]) -
                                  static_cast<double>(gpu_output[index])));
      }
    }
    const double cpu_median = percentile(cpu_times, 0.5);
    const double gpu_wall_median = percentile(gpu_wall_times, 0.5);
    std::cout << std::fixed << std::setprecision(4)
              << "BENCHMARK_RESULT {\"iterations\":" << iterations_
              << ",\"width\":" << message->width
              << ",\"height\":" << message->height
              << ",\"cpu_median_ms\":" << cpu_median
              << ",\"cpu_p95_ms\":" << percentile(cpu_times, 0.95)
              << ",\"gpu_wall_median_ms\":" << gpu_wall_median
              << ",\"gpu_wall_p95_ms\":" << percentile(gpu_wall_times, 0.95)
              << ",\"gpu_kernel_median_ms\":" << percentile(gpu_kernel_times, 0.5)
              << ",\"gpu_kernel_p95_ms\":" << percentile(gpu_kernel_times, 0.95)
              << ",\"end_to_end_speedup\":" << cpu_median / gpu_wall_median
              << ",\"kernel_speedup\":" << cpu_median / percentile(gpu_kernel_times, 0.5)
              << ",\"cpu_valid_fraction\":"
              << static_cast<double>(cpu_valid) / output_count
              << ",\"gpu_valid_fraction\":"
              << static_cast<double>(gpu_valid) / output_count
              << ",\"valid_iou\":"
              << static_cast<double>(joint) / std::max<std::size_t>(1, union_count)
              << ",\"median_error_mm\":" << percentile(errors, 0.5)
              << ",\"p95_error_mm\":" << percentile(errors, 0.95)
              << "}" << std::endl;
    ros::shutdown();
  }

  void fail(const std::string& message) {
    ROS_FATAL("Registration benchmark failed: %s", message.c_str());
    ros::shutdown();
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber image_subscriber_;
  ros::Subscriber depth_info_subscriber_;
  ros::Subscriber rgb_info_subscriber_;
  ros::ServiceClient calibration_client_;
  std::string input_image_;
  std::string depth_info_topic_;
  std::string rgb_info_topic_;
  std::string calibration_service_;
  int iterations_ = 100;
  int warmup_iterations_ = 10;
  float max_depth_metres_ = 8.0f;
  unsigned int hole_fill_max_delta_mm_ = 50;
  std::mutex mutex_;
  sensor_msgs::CameraInfo depth_info_;
  sensor_msgs::CameraInfo rgb_info_;
  bool have_depth_info_ = false;
  bool have_rgb_info_ = false;
  bool complete_ = false;
};

}  // namespace cuda_depth_registration

int main(int argc, char** argv) {
  ros::init(argc, argv, "cuda_depth_registration_benchmark");
  try {
    cuda_depth_registration::BenchmarkNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Registration benchmark startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
