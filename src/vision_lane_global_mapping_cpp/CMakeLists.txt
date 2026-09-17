cmake_minimum_required(VERSION 3.16)
project(vision_lane_global_mapping_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_lane_interfaces REQUIRED)
find_package(OpenCV REQUIRED
  COMPONENTS core imgproc calib3d videoio
  OPTIONAL_COMPONENTS cudaarithm cudaimgproc cudawarping cudafilters)

add_executable(global_mapping_node src/global_mapping_node.cpp)
ament_target_dependencies(global_mapping_node
  rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs cv_bridge
  vision_lane_interfaces)
target_link_libraries(global_mapping_node ${OpenCV_LIBS})
target_include_directories(global_mapping_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_options(global_mapping_node PRIVATE -O3 -DNDEBUG)

set(VISION_LANE_OPENCV_CUDA_FOUND TRUE)
foreach(cuda_component cudaarithm cudaimgproc cudawarping cudafilters)
  if(NOT OpenCV_${cuda_component}_FOUND AND NOT TARGET opencv_${cuda_component})
    set(VISION_LANE_OPENCV_CUDA_FOUND FALSE)
  endif()
endforeach()

if(VISION_LANE_OPENCV_CUDA_FOUND)
  target_compile_definitions(global_mapping_node PRIVATE VISION_LANE_HAS_OPENCV_CUDA=1)
  foreach(cuda_component cudaarithm cudaimgproc cudawarping cudafilters)
    if(TARGET opencv_${cuda_component})
      target_link_libraries(global_mapping_node opencv_${cuda_component})
    endif()
  endforeach()
  message(STATUS "vision_lane: OpenCV CUDA preprocessing enabled")
else()
  target_compile_definitions(global_mapping_node PRIVATE VISION_LANE_HAS_OPENCV_CUDA=0)
  message(WARNING "vision_lane: OpenCV CUDA modules not found; CPU backend only")
endif()

install(TARGETS global_mapping_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/convert_fisheye_npz.py scripts/compare_profiles.py
  scripts/compare_preprocess_backends.py scripts/python_v52_benchmark_node.py
  scripts/plot_profiles.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
