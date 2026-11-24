#include "cuda_depth_registration/registration.hpp"

#include <astra_camera/GetCameraParams.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace cuda_depth_registration {

class RegistrationNode {
 public:
  RegistrationNode() : private_node_("~") {
    private_node_.param<std::string>("input_image", input_image_,
                                     "/camera/depth/image_raw");
    private_node_.param<std::string>("depth_camera_info", depth_info_topic_,
                                     "/camera/depth/camera_info");
    private_node_.param<std::string>("rgb_camera_info", rgb_info_topic_,
                                     "/camera/rgb/camera_info");
    private_node_.param<std::string>("output_image", output_image_,
                                     "/camera/depth_registered_cuda/image_raw");
    private_node_.param<std::string>("output_camera_info", output_info_topic_,
                                     "/camera/depth_registered_cuda/camera_info");
    private_node_.param<std::string>("calibration_service", calibration_service_,
                                     "/camera/get_camera_params");
    private_node_.param("max_depth_metres", max_depth_metres_, 8.0f);
    private_node_.param("pixel_offset_x", pixel_offset_x_, -12.0f);
    private_node_.param("pixel_offset_y", pixel_offset_y_, 10.0f);
    int hole_fill_delta = 50;
    private_node_.param("hole_fill_max_delta_mm", hole_fill_delta, 50);
    if (max_depth_metres_ <= 0.0f || max_depth_metres_ > 20.0f) {
      throw std::runtime_error("max_depth_metres must be in (0, 20]");
    }
    if (hole_fill_delta < 0 || hole_fill_delta > 500) {
      throw std::runtime_error("hole_fill_max_delta_mm must be in [0, 500]");
    }
    if (std::abs(pixel_offset_x_) > 80.0f || std::abs(pixel_offset_y_) > 80.0f) {
      throw std::runtime_error("pixel offsets must be within 80 pixels");
    }
    hole_fill_max_delta_mm_ = static_cast<unsigned int>(hole_fill_delta);

    image_publisher_ = node_.advertise<sensor_msgs::Image>(output_image_, 1);
    info_publisher_ = node_.advertise<sensor_msgs::CameraInfo>(output_info_topic_, 1);
    depth_info_subscriber_ = node_.subscribe(
        depth_info_topic_, 1, &RegistrationNode::depthInfoCallback, this);
    rgb_info_subscriber_ = node_.subscribe(
        rgb_info_topic_, 1, &RegistrationNode::rgbInfoCallback, this);
    image_subscriber_ = node_.subscribe(
        input_image_, 1, &RegistrationNode::imageCallback, this,
        ros::TransportHints().tcpNoDelay());
    calibration_client_ = node_.serviceClient<astra_camera::GetCameraParams>(
        calibration_service_, true);
    ROS_INFO_STREAM("CUDA depth registration trial: " << input_image_ << " -> "
                    << output_image_ << " (pixel correction x="
                    << pixel_offset_x_ << ", y=" << pixel_offset_y_ << ")");
  }

 private:
  bool loadFactoryCalibration() {
    if (factory_calibration_loaded_) return true;
    astra_camera::GetCameraParams service;
    if (!calibration_client_.call(service)) {
      ROS_WARN_THROTTLE(5.0, "Waiting for Astra factory calibration service %s",
                        calibration_service_.c_str());
      return false;
    }
    for (std::size_t index = 0; index < 9; ++index) {
      calibration_.right_to_left_rotation[index] = service.response.r2l_r[index];
    }
    for (std::size_t index = 0; index < 3; ++index) {
      calibration_.right_to_left_translation[index] =
          service.response.r2l_t[index] * 0.001f;
    }
    factory_calibration_loaded_ = true;
    ROS_INFO("Loaded Astra factory RGB-to-depth extrinsics for CUDA registration");
    return true;
  }

  void depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    std::lock_guard<std::mutex> lock(info_mutex_);
    depth_info_ = *message;
    have_depth_info_ = true;
  }

  void rgbInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    std::lock_guard<std::mutex> lock(info_mutex_);
    rgb_info_ = *message;
    have_rgb_info_ = true;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    sensor_msgs::CameraInfo depth_info;
    sensor_msgs::CameraInfo rgb_info;
    {
      std::lock_guard<std::mutex> lock(info_mutex_);
      if (!have_depth_info_ || !have_rgb_info_) {
        ROS_WARN_THROTTLE(5.0, "Waiting for depth and RGB CameraInfo");
        return;
      }
      depth_info = depth_info_;
      rgb_info = rgb_info_;
    }
    if (!loadFactoryCalibration()) return;
    if ((message->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
         message->encoding != sensor_msgs::image_encodings::MONO16) ||
        message->is_bigendian || message->width < 2 || message->height < 2 ||
        depth_info.width != message->width || depth_info.height != message->height ||
        rgb_info.width < 2 || rgb_info.height < 2 ||
        message->step < message->width * sizeof(std::uint16_t)) {
      ROS_ERROR_THROTTLE(5.0, "CUDA registration rejected incompatible image/calibration");
      return;
    }

    const std::size_t input_count =
        static_cast<std::size_t>(message->width) * message->height;
    packed_input_.resize(input_count);
    for (std::uint32_t row = 0; row < message->height; ++row) {
      std::memcpy(packed_input_.data() + static_cast<std::size_t>(row) * message->width,
                  message->data.data() + static_cast<std::size_t>(row) * message->step,
                  message->width * sizeof(std::uint16_t));
    }
    const std::size_t output_count =
        static_cast<std::size_t>(rgb_info.width) * rgb_info.height;
    packed_output_.resize(output_count);

    calibration_.depth_fx = static_cast<float>(depth_info.K[0]);
    calibration_.depth_fy = static_cast<float>(depth_info.K[4]);
    calibration_.depth_cx = static_cast<float>(depth_info.K[2]);
    calibration_.depth_cy = static_cast<float>(depth_info.K[5]);
    calibration_.rgb_fx = static_cast<float>(rgb_info.K[0]);
    calibration_.rgb_fy = static_cast<float>(rgb_info.K[4]);
    // A timestamp-matched live overlay showed the projected depth consistently
    // above and right of raw RGB. Correct the shared registered stream here so
    // the dashboard, object-depth lookup, and planner rays use one geometry.
    calibration_.rgb_cx = static_cast<float>(rgb_info.K[2]) + pixel_offset_x_;
    calibration_.rgb_cy = static_cast<float>(rgb_info.K[5]) + pixel_offset_y_;
    std::fill(calibration_.rgb_distortion, calibration_.rgb_distortion + 5, 0.0f);
    for (std::size_t index = 0; index < std::min<std::size_t>(5, rgb_info.D.size()); ++index) {
      calibration_.rgb_distortion[index] = static_cast<float>(rgb_info.D[index]);
    }
    if (calibration_.depth_fx <= 0.0f || calibration_.depth_fy <= 0.0f ||
        calibration_.rgb_fx <= 0.0f || calibration_.rgb_fy <= 0.0f) {
      ROS_ERROR_THROTTLE(5.0, "CUDA registration rejected invalid camera intrinsics");
      return;
    }

    float elapsed_ms = 0.0f;
    std::string error;
    if (!registrar_.registerDepth(
            packed_input_.data(), message->width, message->height,
            packed_output_.data(), rgb_info.width, rgb_info.height,
            calibration_, max_depth_metres_, hole_fill_max_delta_mm_,
            &elapsed_ms, &error)) {
      ROS_ERROR_THROTTLE(5.0, "CUDA depth registration failed: %s", error.c_str());
      return;
    }

    sensor_msgs::Image output;
    output.header = message->header;
    output.header.frame_id = rgb_info.header.frame_id;
    output.height = rgb_info.height;
    output.width = rgb_info.width;
    output.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    output.is_bigendian = false;
    output.step = output.width * sizeof(std::uint16_t);
    output.data.resize(output_count * sizeof(std::uint16_t));
    std::memcpy(output.data.data(), packed_output_.data(), output.data.size());
    sensor_msgs::CameraInfo output_info = rgb_info;
    output_info.header = output.header;
    image_publisher_.publish(output);
    info_publisher_.publish(output_info);
    ++frames_;
    ROS_INFO_THROTTLE(5.0, "CUDA registered %llu frames; kernel %.3f ms",
                      static_cast<unsigned long long>(frames_), elapsed_ms);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber image_subscriber_;
  ros::Subscriber depth_info_subscriber_;
  ros::Subscriber rgb_info_subscriber_;
  ros::Publisher image_publisher_;
  ros::Publisher info_publisher_;
  ros::ServiceClient calibration_client_;
  std::string input_image_;
  std::string depth_info_topic_;
  std::string rgb_info_topic_;
  std::string output_image_;
  std::string output_info_topic_;
  std::string calibration_service_;
  float max_depth_metres_ = 8.0f;
  float pixel_offset_x_ = -12.0f;
  float pixel_offset_y_ = 10.0f;
  unsigned int hole_fill_max_delta_mm_ = 50;
  std::mutex info_mutex_;
  sensor_msgs::CameraInfo depth_info_;
  sensor_msgs::CameraInfo rgb_info_;
  bool have_depth_info_ = false;
  bool have_rgb_info_ = false;
  bool factory_calibration_loaded_ = false;
  Calibration calibration_{};
  Registrar registrar_;
  std::vector<std::uint16_t> packed_input_;
  std::vector<std::uint16_t> packed_output_;
  std::uint64_t frames_ = 0;
};

}  // namespace cuda_depth_registration

int main(int argc, char** argv) {
  ros::init(argc, argv, "cuda_depth_registration");
  try {
    cuda_depth_registration::RegistrationNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("CUDA depth registration startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
