cmake_minimum_required(VERSION 3.16)
project(monoscale_occupancy_grid_map)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The CUDA path builds the kernel in src/sweep_kernels.cu, which is the same
# source the python reference was checked against, so the two share the
# arithmetic. Without a CUDA compiler the Sweep falls back to sweep.cpp.
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  # This workspace is deployed on an RTX 3050 Laptop GPU (Ampere, sm_86).
  # Keep it cache-configurable so another machine can override it with
  # -DMONOSCALE_CUDA_ARCHITECTURES=<compute capability>.
  set(MONOSCALE_CUDA_ARCHITECTURES "86" CACHE STRING
    "CUDA architectures used by the MonoScale occupancy kernel")
  set(CMAKE_CUDA_ARCHITECTURES "${MONOSCALE_CUDA_ARCHITECTURES}")
  message(STATUS
    "MonoScale occupancy CUDA enabled: compiler=${CMAKE_CUDA_COMPILER}, architectures=${CMAKE_CUDA_ARCHITECTURES}")
else()
  message(STATUS "MonoScale occupancy CUDA disabled: no CUDA compiler was detected")
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(monoscale_pose REQUIRED)
find_package(vision_lane_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(SWEEP_SOURCES src/sweep.cpp src/publish.cpp)
# The kernel lives here. It used to be pulled from a sibling workspace by an
# absolute path, which meant a fresh clone silently built the CPU path -- 28
# minutes for 674 keyframes against the CUDA path's 0.2 s each -- while the
# README advertised the GPU one. Vendored 2026-09-17 so the repository builds
# what it documents. Still overridable, for a tree that keeps its own copy.
set(MONOSCALE_SWEEP_KERNELS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sweep_kernels.cu
  CACHE FILEPATH "CUDA kernels for the plane sweep")
if(CMAKE_CUDA_COMPILER AND EXISTS ${MONOSCALE_SWEEP_KERNELS})
  list(APPEND SWEEP_SOURCES src/cuda_backend.cpp ${MONOSCALE_SWEEP_KERNELS})
  add_compile_definitions(MONOSCALE_OCCUPANCY_HAS_CUDA)
endif()

add_library(monoscale_occupancy_core ${SWEEP_SOURCES})
target_include_directories(monoscale_occupancy_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(monoscale_occupancy_core ${OpenCV_LIBS}
  $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>)
if(CMAKE_CUDA_COMPILER AND EXISTS ${MONOSCALE_SWEEP_KERNELS})
  set_target_properties(monoscale_occupancy_core PROPERTIES CUDA_SEPARABLE_COMPILATION OFF)
  # E-9: every host thread launches on its own default stream, so two camera
  # workers do not serialise on (or synchronise) each other's work.
  target_compile_options(monoscale_occupancy_core PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--default-stream per-thread>)
  find_package(CUDAToolkit REQUIRED)
  target_link_libraries(monoscale_occupancy_core CUDA::cudart)
endif()

# Static core: the CUDA runtime it pulls in must reach the executable too.
if(CMAKE_CUDA_COMPILER AND EXISTS ${MONOSCALE_SWEEP_KERNELS})
  set(SWEEP_CUDART CUDA::cudart)
endif()

# Offline runner: cache -> grid -> npy, for parity against plane_sweep.py.
add_executable(sweep_offline src/offline.cpp)
target_link_libraries(sweep_offline monoscale_occupancy_core ${OpenCV_LIBS} ${SWEEP_CUDART})

# The online node.
add_executable(sweep_node src/sweep_node.cpp)
target_link_libraries(sweep_node monoscale_occupancy_core ${OpenCV_LIBS} ${SWEEP_CUDART})
ament_target_dependencies(sweep_node
  rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge monoscale_pose vision_lane_interfaces)

# E-9 job harness: recorded keyframe jobs through the node's own workers (a tool, not a
# test: the jobs come from a recorded run, VISION_SWEEP_JOB_DUMP).
add_executable(sweep_jobs_golden test/sweep_jobs_golden.cpp)
target_include_directories(sweep_jobs_golden PRIVATE include)
target_link_libraries(sweep_jobs_golden monoscale_occupancy_core ${OpenCV_LIBS} ${SWEEP_CUDART})
ament_target_dependencies(sweep_jobs_golden
  rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge monoscale_pose vision_lane_interfaces)

# E-9 JobBuilder golden: a recorded input event sequence through the node's callbacks or a
# standalone JobBuilder (observer_eval_E9j 2). Reads the measurement bags read-only.
find_package(SQLite3 REQUIRED)
add_executable(job_builder_golden test/job_builder_golden.cpp)
target_include_directories(job_builder_golden PRIVATE include)
target_link_libraries(job_builder_golden monoscale_occupancy_core ${OpenCV_LIBS} ${SWEEP_CUDART} SQLite::SQLite3)
ament_target_dependencies(job_builder_golden
  rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge monoscale_pose vision_lane_interfaces)

# E-9 deterministic occupancy replay (observer_eval_E9j): the node's code serially over a
# recorded input event sequence, with a fixed carve rule. Driven by tools/run/occ_offline.py.
add_executable(occ_offline test/occ_offline.cpp)
target_include_directories(occ_offline PRIVATE include)
target_link_libraries(occ_offline monoscale_occupancy_core ${OpenCV_LIBS} ${SWEEP_CUDART} SQLite::SQLite3)
ament_target_dependencies(occ_offline
  rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge monoscale_pose vision_lane_interfaces)

install(TARGETS monoscale_occupancy_core sweep_offline sweep_node sweep_jobs_golden job_builder_golden occ_offline
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The dense sweep chain is unchanged and is covered by the offline parity
  # run; what is new is the tile storage the chain now reads its window from.
  ament_add_gtest(test_tiles test/test_tiles.cpp)
  target_link_libraries(test_tiles monoscale_occupancy_core ${OpenCV_LIBS} ${SWEEP_CUDART})
  ament_add_gtest(test_fisheye test/test_fisheye.cpp)
  target_link_libraries(test_fisheye monoscale_occupancy_core ${OpenCV_LIBS} ${SWEEP_CUDART})
  ament_add_gtest(test_pose_interpolation test/test_pose_interpolation.cpp)
  target_include_directories(test_pose_interpolation PRIVATE include)
  target_link_libraries(test_pose_interpolation monoscale_occupancy_core ${OpenCV_LIBS} ${SWEEP_CUDART})
  ament_add_gtest(test_camera_liveness test/test_camera_liveness.cpp)
  target_include_directories(test_camera_liveness PRIVATE include)
  ament_target_dependencies(test_camera_liveness vision_lane_interfaces)
  ament_add_gtest(test_image_quality test/test_image_quality.cpp)
  target_include_directories(test_image_quality PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_image_quality ${OpenCV_LIBS})
  ament_target_dependencies(test_image_quality vision_lane_interfaces)
endif()

ament_package()
