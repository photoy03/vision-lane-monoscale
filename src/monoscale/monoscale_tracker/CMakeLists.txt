cmake_minimum_required(VERSION 3.8)
project(monoscale_tracker)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
# MONOSCALE_TRACKER_OPENCV_DIR: build this node against another OpenCV (e.g. one with the
# CUDA modules, for the GPU optical flow) instead of the system one. Only this node links
# it, so no process holds two OpenCVs. The libraries are found at run time through the
# install RPATH below.
set(MONOSCALE_TRACKER_OPENCV_DIR "$ENV{MONOSCALE_TRACKER_OPENCV_DIR}" CACHE PATH
  "OpenCVConfig.cmake directory for feature_tracker (empty: the default OpenCV)")
if(MONOSCALE_TRACKER_OPENCV_DIR)
  # The workspace build passes -DOpenCV_DIR (the system one) to every package; this
  # package's own cache entry is pointed elsewhere.
  set(OpenCV_DIR "${MONOSCALE_TRACKER_OPENCV_DIR}" CACHE PATH "" FORCE)
  find_package(OpenCV REQUIRED PATHS "${MONOSCALE_TRACKER_OPENCV_DIR}" NO_DEFAULT_PATH)
  message(STATUS "monoscale_tracker: OpenCV ${OpenCV_VERSION} from ${MONOSCALE_TRACKER_OPENCV_DIR}")
else()
  find_package(OpenCV REQUIRED)
endif()
# Header-only helper (compressed_input.hpp); OpenCV-only, no message libraries needed.
find_package(vision_lane_interfaces REQUIRED)

# cv_bridge is deliberately not used. It links against whichever OpenCV the ROS
# distribution was built with, and this node is meant to be pointed at a second
# OpenCV that has the CUDA modules -- two of them in one process is an ABI
# gamble that shows up as a crash a long way from its cause. Converting an
# Image to a Mat by hand is fifteen lines and removes the question.

# The optical flow has a GPU twin where OpenCV was built with cudaoptflow.
# Nothing is required: without it the node builds and runs exactly as before,
# and `use_cuda` reports that it cannot be honoured rather than failing.
set(HAVE_CUDA_OPTFLOW FALSE)
if(TARGET opencv_cudaoptflow OR "cudaoptflow" IN_LIST OpenCV_LIB_COMPONENTS)
  set(HAVE_CUDA_OPTFLOW TRUE)
endif()
if(NOT HAVE_CUDA_OPTFLOW)
  foreach(component ${OpenCV_LIBS})
    if(component MATCHES "cudaoptflow")
      set(HAVE_CUDA_OPTFLOW TRUE)
    endif()
  endforeach()
endif()

add_executable(feature_tracker src/feature_tracker.cpp)
target_include_directories(feature_tracker PRIVATE include ${OpenCV_INCLUDE_DIRS} ${vision_lane_interfaces_INCLUDE_DIRS})
target_link_libraries(feature_tracker ${OpenCV_LIBS})
ament_target_dependencies(feature_tracker rclcpp sensor_msgs std_msgs geometry_msgs rosbag2_cpp)
if(HAVE_CUDA_OPTFLOW)
  message(STATUS "monoscale_tracker: OpenCV has cudaoptflow; the GPU path is built")
  target_compile_definitions(feature_tracker PRIVATE MONOSCALE_TRACKER_HAS_CUDA)
  # The road-step warps on the GPU (road_step_gpu): a CUDA kernel of this package.
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    # The deployment GPU (RTX 3050 Laptop, sm_86), as monoscale_occupancy_grid_map; the
    # Gram kernel needs >= 6.0 for double atomicAdd.
    set(MONOSCALE_TRACKER_CUDA_ARCH "${MONOSCALE_CUDA_ARCHITECTURES}")
    if(NOT MONOSCALE_TRACKER_CUDA_ARCH)
      set(MONOSCALE_TRACKER_CUDA_ARCH 86)
    endif()
    target_sources(feature_tracker PRIVATE src/road_warp.cu)
    set_target_properties(feature_tracker PROPERTIES CUDA_ARCHITECTURES "${MONOSCALE_TRACKER_CUDA_ARCH}")
    target_compile_definitions(feature_tracker PRIVATE MONOSCALE_TRACKER_ROAD_WARP_CUDA)
    find_package(CUDAToolkit REQUIRED)
    target_link_libraries(feature_tracker CUDA::cudart)
    message(STATUS "monoscale_tracker: road-step GPU kernel built (arch ${MONOSCALE_TRACKER_CUDA_ARCH})")
  endif()
else()
  message(STATUS "monoscale_tracker: no cudaoptflow in OpenCV; CPU only")
endif()
target_compile_options(feature_tracker PRIVATE -O3)
set_target_properties(feature_tracker PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS feature_tracker DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # Assert-based; keep asserts active in Release.
  add_executable(test_gyro_rotation test/test_gyro_rotation.cpp)
  target_include_directories(test_gyro_rotation PRIVATE include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_gyro_rotation ${OpenCV_LIBS})
  target_compile_options(test_gyro_rotation PRIVATE -UNDEBUG)
  add_test(NAME test_gyro_rotation COMMAND test_gyro_rotation)
endif()
ament_package()
