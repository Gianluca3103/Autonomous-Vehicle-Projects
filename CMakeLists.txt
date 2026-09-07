cmake_minimum_required(VERSION 3.8)
project(turtlebot3_waypoint_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(controller_core
  src/controller_math.cpp
  src/odom_listener.cpp
  src/telemetry_logger.cpp)
target_compile_features(controller_core PUBLIC cxx_std_17)
target_include_directories(controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(controller_core geometry_msgs nav_msgs rclcpp)

add_executable(project2_controller src/main.cpp)
target_compile_features(project2_controller PUBLIC cxx_std_17)
target_link_libraries(project2_controller controller_core)
ament_target_dependencies(project2_controller rclcpp)

install(TARGETS project2_controller
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS controller_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
