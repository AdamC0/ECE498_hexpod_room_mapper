cmake_minimum_required(VERSION 3.5)
project(cuda_depth_registration)

find_package(catkin REQUIRED COMPONENTS
  astra_camera
  roscpp
  sensor_msgs
)
find_package(CUDA 10.2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS astra_camera roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

list(APPEND CUDA_NVCC_FLAGS
  -gencode=arch=compute_53,code=sm_53
  --use_fast_math
  -lineinfo
)

cuda_add_library(cuda_registration_kernel
  src/cuda_registration_kernel.cu
)
target_link_libraries(cuda_registration_kernel ${CUDA_LIBRARIES})

add_library(cpu_registration
  src/cpu_registration.cpp
)
target_compile_options(cpu_registration PRIVATE -std=c++11 -O3 -Wall -Wextra)

add_executable(cuda_depth_registration_node
  src/cuda_depth_registration_node.cpp
)
add_dependencies(cuda_depth_registration_node ${catkin_EXPORTED_TARGETS})
target_compile_options(cuda_depth_registration_node PRIVATE -std=c++11 -Wall -Wextra)
target_link_libraries(cuda_depth_registration_node
  cuda_registration_kernel
  ${catkin_LIBRARIES}
  ${CUDA_LIBRARIES}
)

add_executable(cuda_depth_registration_benchmark
  src/registration_benchmark_node.cpp
)
add_dependencies(cuda_depth_registration_benchmark ${catkin_EXPORTED_TARGETS})
target_compile_options(cuda_depth_registration_benchmark PRIVATE -std=c++11 -O3 -Wall -Wextra)
target_link_libraries(cuda_depth_registration_benchmark
  cpu_registration
  cuda_registration_kernel
  ${catkin_LIBRARIES}
  ${CUDA_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/compare_registration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS cuda_depth_registration_node cuda_depth_registration_benchmark
  cuda_registration_kernel cpu_registration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
