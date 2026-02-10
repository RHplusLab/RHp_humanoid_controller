cmake_minimum_required(VERSION 3.8)
project(rhphumanoid_arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# 헤더 파일 경로 설정 (include 폴더)
include_directories(include)

# 실행 파일 생성
add_executable(arm_controller src/arm_controller.cpp)
ament_target_dependencies(arm_controller
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)

# 설치 설정
install(TARGETS
  arm_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()