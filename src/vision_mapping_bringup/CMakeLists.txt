cmake_minimum_required(VERSION 3.16)
project(vision_mapping_bringup)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/measurement_image_compressor.py
  scripts/clock_guard.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
