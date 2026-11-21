#include "cuda_depth_registration/registration.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <climits>
#include <sstream>
#include <stdexcept>

namespace cuda_depth_registration {
namespace {

__constant__ Calibration device_calibration;

__global__ void projectDepthKernel(const std::uint16_t* input,
                                   int input_width,
                                   int input_height,
                                   unsigned int* z_buffer,
                                   int output_width,
                                   int output_height,
                                   float max_depth_metres) {
  const int u = blockIdx.x * blockDim.x + threadIdx.x;
  const int v = blockIdx.y * blockDim.y + threadIdx.y;
  if (u >= input_width || v >= input_height) return;

  const std::uint16_t depth_mm = input[v * input_width + u];
  if (depth_mm == 0) return;
  const float z_left = static_cast<float>(depth_mm) * 0.001f;
  if (z_left <= 0.0f || z_left > max_depth_metres) return;

  const float x_left = (static_cast<float>(u) - device_calibration.depth_cx) *
                       z_left / device_calibration.depth_fx;
  const float y_left = (static_cast<float>(v) - device_calibration.depth_cy) *
                       z_left / device_calibration.depth_fy;

  // Factory calibration is right/RGB -> left/depth. Invert the rigid
  // transform with R^T * (p_left - t) to project a depth point into RGB.
  const float px = x_left - device_calibration.right_to_left_translation[0];
  const float py = y_left - device_calibration.right_to_left_translation[1];
  const float pz = z_left - device_calibration.right_to_left_translation[2];
  const float* r = device_calibration.right_to_left_rotation;
  const float x_right = r[0] * px + r[3] * py + r[6] * pz;
  const float y_right = r[1] * px + r[4] * py + r[7] * pz;
  const float z_right = r[2] * px + r[5] * py + r[8] * pz;
  if (z_right <= 0.0f || z_right > max_depth_metres) return;

  float xn = x_right / z_right;
  float yn = y_right / z_right;
  const float radius2 = xn * xn + yn * yn;
  const float* d = device_calibration.rgb_distortion;
  const float radial = 1.0f + d[0] * radius2 + d[1] * radius2 * radius2 +
                       d[4] * radius2 * radius2 * radius2;
  const float distorted_x = xn * radial + 2.0f * d[2] * xn * yn +
                            d[3] * (radius2 + 2.0f * xn * xn);
  const float distorted_y = yn * radial + d[2] * (radius2 + 2.0f * yn * yn) +
                            2.0f * d[3] * xn * yn;
  const int output_u = __float2int_rn(device_calibration.rgb_fx * distorted_x +
                                      device_calibration.rgb_cx);
  const int output_v = __float2int_rn(device_calibration.rgb_fy * distorted_y +
                                      device_calibration.rgb_cy);
  if (output_u < 0 || output_u >= output_width ||
      output_v < 0 || output_v >= output_height) return;

  const unsigned int registered_mm =
      static_cast<unsigned int>(fminf(z_right * 1000.0f, 65535.0f));
  atomicMin(&z_buffer[output_v * output_width + output_u], registered_mm);
}

__global__ void packAndFillDepthKernel(const unsigned int* z_buffer,
                                       std::uint16_t* output,
                                       int width,
                                       int height,
                                       unsigned int max_delta_mm) {
  const std::size_t count = static_cast<std::size_t>(width) * height;
  const std::size_t index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) return;
  const unsigned int value = z_buffer[index];
  if (value != UINT_MAX) {
    output[index] = static_cast<std::uint16_t>(value);
    return;
  }

  // Fill only one-pixel holes inside a coherent surface. Two agreeing
  // four-neighbours are required, and the nearer value wins so filling cannot
  // replace a foreground edge with farther background depth.
  const int x = static_cast<int>(index % width);
  const int y = static_cast<int>(index / width);
  unsigned int minimum = UINT_MAX;
  unsigned int maximum = 0;
  int neighbours = 0;
  const int offsets[4] = {-1, 1, -width, width};
  const bool allowed[4] = {x > 0, x + 1 < width, y > 0, y + 1 < height};
#pragma unroll
  for (int offset_index = 0; offset_index < 4; ++offset_index) {
    if (!allowed[offset_index]) continue;
    const unsigned int neighbour = z_buffer[index + offsets[offset_index]];
    if (neighbour == UINT_MAX) continue;
    minimum = min(minimum, neighbour);
    maximum = max(maximum, neighbour);
    ++neighbours;
  }
  output[index] = (neighbours >= 2 && maximum - minimum <= max_delta_mm)
                      ? static_cast<std::uint16_t>(minimum)
                      : 0;
}

std::string cudaErrorMessage(const char* operation, cudaError_t result) {
  std::ostringstream stream;
  stream << operation << ": " << cudaGetErrorString(result);
  return stream.str();
}

}  // namespace

struct Registrar::Impl {
  std::uint16_t* input = nullptr;
  std::uint16_t* output = nullptr;
  unsigned int* z_buffer = nullptr;
  std::size_t input_capacity = 0;
  std::size_t output_capacity = 0;
  cudaStream_t stream = nullptr;
  cudaEvent_t started = nullptr;
  cudaEvent_t finished = nullptr;
};

Registrar::Registrar() : impl_(new Impl()) {
  cudaError_t result = cudaStreamCreateWithFlags(&impl_->stream, cudaStreamNonBlocking);
  if (result == cudaSuccess) result = cudaEventCreate(&impl_->started);
  if (result == cudaSuccess) result = cudaEventCreate(&impl_->finished);
  if (result != cudaSuccess) {
    const std::string message = cudaErrorMessage("initialize CUDA registrar", result);
    if (impl_->started) cudaEventDestroy(impl_->started);
    if (impl_->stream) cudaStreamDestroy(impl_->stream);
    delete impl_;
    impl_ = nullptr;
    throw std::runtime_error(message);
  }
}

Registrar::~Registrar() {
  if (impl_ == nullptr) return;
  cudaFree(impl_->input);
  cudaFree(impl_->output);
  cudaFree(impl_->z_buffer);
  if (impl_->started) cudaEventDestroy(impl_->started);
  if (impl_->finished) cudaEventDestroy(impl_->finished);
  if (impl_->stream) cudaStreamDestroy(impl_->stream);
  delete impl_;
}

bool Registrar::registerDepth(const std::uint16_t* input,
                              int input_width,
                              int input_height,
                              std::uint16_t* output,
                              int output_width,
                              int output_height,
                              const Calibration& calibration,
                              float max_depth_metres,
                              unsigned int hole_fill_max_delta_mm,
                              float* elapsed_ms,
                              std::string* error) {
  if (input == nullptr || output == nullptr || input_width <= 0 || input_height <= 0 ||
      output_width <= 0 || output_height <= 0 || max_depth_metres <= 0.0f) {
    if (error) *error = "invalid CUDA registration input";
    return false;
  }
  const std::size_t input_count = static_cast<std::size_t>(input_width) * input_height;
  const std::size_t output_count = static_cast<std::size_t>(output_width) * output_height;

  auto check = [&](cudaError_t result, const char* operation) {
    if (result == cudaSuccess) return true;
    if (error) *error = cudaErrorMessage(operation, result);
    return false;
  };
  if (input_count > impl_->input_capacity) {
    cudaFree(impl_->input);
    if (!check(cudaMalloc(&impl_->input, input_count * sizeof(std::uint16_t)),
               "cudaMalloc input")) return false;
    impl_->input_capacity = input_count;
  }
  if (output_count > impl_->output_capacity) {
    cudaFree(impl_->output);
    cudaFree(impl_->z_buffer);
    if (!check(cudaMalloc(&impl_->output, output_count * sizeof(std::uint16_t)),
               "cudaMalloc output") ||
        !check(cudaMalloc(&impl_->z_buffer, output_count * sizeof(unsigned int)),
               "cudaMalloc z-buffer")) return false;
    impl_->output_capacity = output_count;
  }

  if (!check(cudaMemcpyToSymbolAsync(device_calibration, &calibration,
                                     sizeof(calibration), 0, cudaMemcpyHostToDevice,
                                     impl_->stream), "copy calibration") ||
      !check(cudaMemcpyAsync(impl_->input, input,
                            input_count * sizeof(std::uint16_t),
                            cudaMemcpyHostToDevice, impl_->stream), "copy depth input") ||
      !check(cudaMemsetAsync(impl_->z_buffer, 0xff,
                            output_count * sizeof(unsigned int), impl_->stream),
             "clear z-buffer")) return false;

  cudaEventRecord(impl_->started, impl_->stream);
  const dim3 block(16, 16);
  const dim3 grid((input_width + block.x - 1) / block.x,
                  (input_height + block.y - 1) / block.y);
  projectDepthKernel<<<grid, block, 0, impl_->stream>>>(
      impl_->input, input_width, input_height, impl_->z_buffer,
      output_width, output_height, max_depth_metres);
  const int threads = 256;
  const int blocks = static_cast<int>((output_count + threads - 1) / threads);
  packAndFillDepthKernel<<<blocks, threads, 0, impl_->stream>>>(
      impl_->z_buffer, impl_->output, output_width, output_height,
      hole_fill_max_delta_mm);
  if (!check(cudaGetLastError(), "launch registration kernels")) return false;
  cudaEventRecord(impl_->finished, impl_->stream);
  if (!check(cudaMemcpyAsync(output, impl_->output,
                            output_count * sizeof(std::uint16_t),
                            cudaMemcpyDeviceToHost, impl_->stream), "copy registered depth") ||
      !check(cudaStreamSynchronize(impl_->stream), "synchronize registration")) return false;
  if (elapsed_ms) cudaEventElapsedTime(elapsed_ms, impl_->started, impl_->finished);
  return true;
}

}  // namespace cuda_depth_registration
