cmake_minimum_required(VERSION 3.8)
project(rhphumanoid_manual_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(humanoid_manual_node src/humanoid_manual_node.cpp)
ament_target_dependencies(humanoid_manual_node
  rclcpp
  trajectory_msgs
)

install(TARGETS
  humanoid_manual_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/wave_arm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
