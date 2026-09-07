cmake_minimum_required(VERSION 3.8)
project(turtlebot3_waypoint_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(project2_controller src/project2_controller.cpp)
target_compile_features(project2_controller PUBLIC cxx_std_17)
ament_target_dependencies(project2_controller geometry_msgs nav_msgs rclcpp)

install(TARGETS project2_controller
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
