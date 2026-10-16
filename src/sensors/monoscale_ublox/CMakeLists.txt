cmake_minimum_required(VERSION 3.8)
project(monoscale_ublox)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(f9r_imu src/f9r_imu.cpp)
target_compile_features(f9r_imu PUBLIC cxx_std_17)
ament_target_dependencies(f9r_imu rclcpp sensor_msgs)

install(TARGETS f9r_imu DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
