cmake_minimum_required(VERSION 3.8)
project(rhphumanoid_walking_pattern)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# 헤더 파일 경로 포함
include_directories(include)

add_executable(walking_controller src/walking_controller.cpp)
ament_target_dependencies(walking_controller
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)

install(TARGETS
  walking_controller
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치 (다른 패키지에서도 쓸 수 있게 하려면)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
