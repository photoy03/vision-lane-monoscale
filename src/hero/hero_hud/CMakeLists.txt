cmake_minimum_required(VERSION 3.16)
project(hero_hud)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_lane_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Quick)
# 표지판 SVG 는 선택이다. libqt5svg5-dev 가 없으면 표지판 자리만 비우고 빌드한다.
find_package(Qt5 QUIET COMPONENTS Svg)

# ---------------------------------------------------------------- 표시 계약
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/VehicleState.msg
  msg/TrajectoryRibbon.msg
  msg/ObstacleMarker.msg
  msg/SignMarker.msg
  msg/TrafficLightMarker.msg
  msg/SignalIndicator.msg
  msg/AmbientLight.msg
  msg/CoachingMessage.msg
  msg/SourceHealth.msg
  msg/SceneOverlay.msg
  DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(msgs_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ------------------------------------------------- 순수 기하 (Qt·ROS 무관)
# 이 라이브러리는 의존성이 없다. 그것이 요점이다 — 투영과 차량 모델은
# 시뮬레이터도 렌더러도 없이 단독으로 검증된다.
add_library(hero_hud_core STATIC
  src/projection.cpp
  src/vehicle_model.cpp
  src/angular.cpp
  # Qt·ROS 의존이 없다. 표시 어법 판단(주차/주행)을 테스트에서 걸기 위해
  # 여기 둔다 — 렌더러에만 있으면 검증할 수단이 없다.
  src/hud_state.cpp
)
target_include_directories(hero_hud_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(hero_hud_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --------------------------------------------------------------- HUD 렌더러
# Q_OBJECT를 가진 헤더는 반드시 소스 목록에 넣어야 AUTOMOC이 찾는다
# (헤더가 소스와 다른 디렉터리에 있으므로 basename 매칭이 안 걸린다).
add_executable(hud_node
  src/main.cpp
  src/ros_bridge.cpp
  src/world_item.cpp
  src/chrome_item.cpp
  include/hero_hud/ros_bridge.hpp
  include/hero_hud/world_item.hpp
  include/hero_hud/chrome_item.hpp
)
set_target_properties(hud_node PROPERTIES AUTOMOC ON)
target_include_directories(hud_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hud_node
  hero_hud_core
  Qt5::Core Qt5::Gui Qt5::Quick
  "${msgs_target}")
if(Qt5Svg_FOUND)
  target_link_libraries(hud_node Qt5::Svg)
  target_compile_definitions(hud_node PRIVATE HERO_HUD_HAVE_SVG)
else()
  message(WARNING "Qt5Svg development files not found: sign icons are not drawn (apt install libqt5svg5-dev)")
endif()
ament_target_dependencies(hud_node rclcpp std_msgs geometry_msgs visualization_msgs vision_lane_interfaces ament_index_cpp)

# ------------------------------------------------- 테스트 시나리오 발행 노드
# hero_coach가 아직 없으므로 그 자리를 대신한다. 인지 스택도 CARLA도 bag도
# 없이 HUD 전체를 구동할 수 있어야 한다는 것이 설계 요구사항이다.
add_executable(scenario_publisher src/scenario_publisher.cpp)
target_include_directories(scenario_publisher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(scenario_publisher hero_hud_core "${msgs_target}")
ament_target_dependencies(scenario_publisher rclcpp std_msgs geometry_msgs)

install(TARGETS hud_node scenario_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch signs
  DESTINATION share/${PROJECT_NAME})

# --------------------------------------------------------------------- 테스트
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  if(TARGET test_geometry)
    target_link_libraries(test_geometry hero_hud_core)
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
