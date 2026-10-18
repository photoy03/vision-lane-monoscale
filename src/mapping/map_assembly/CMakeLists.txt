cmake_minimum_required(VERSION 3.8)
project(map_assembly)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vision_lane_interfaces REQUIRED)
find_package(monoscale_pose REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_executable(map_assembler_node src/map_assembler_node.cpp)
target_include_directories(map_assembler_node PRIVATE include)
ament_target_dependencies(map_assembler_node rclcpp std_msgs nav_msgs vision_lane_interfaces monoscale_pose)

add_executable(map_visualizer_node src/map_visualizer_node.cpp)
target_include_directories(map_visualizer_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(map_visualizer_node rclcpp sensor_msgs std_msgs vision_lane_interfaces)
target_link_libraries(map_visualizer_node ${OpenCV_LIBS})

install(TARGETS map_assembler_node map_visualizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_assembler test/test_assembler.cpp)
  target_include_directories(test_assembler PRIVATE include)
  target_compile_options(test_assembler PRIVATE -UNDEBUG)
  add_test(NAME test_assembler COMMAND test_assembler)
endif()

ament_package()
