cmake_minimum_required(VERSION 3.8)
project(rhphumanoid_head_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include)

add_executable(head_controller src/head_controller.cpp)
ament_target_dependencies(head_controller
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)

install(TARGETS
  head_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
