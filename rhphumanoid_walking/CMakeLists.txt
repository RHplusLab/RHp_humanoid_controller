cmake_minimum_required(VERSION 3.8)
project(rhphumanoid_walking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Library: kinematics + walking logic
add_library(${PROJECT_NAME}_lib
  src/link_data.cpp
  src/rhphumanoid_kinematics_dynamics.cpp
  src/rhphumanoid_walking_module.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp rclcpp_action std_msgs sensor_msgs trajectory_msgs control_msgs
)

target_link_libraries(${PROJECT_NAME}_lib yaml-cpp)

# Executable: walking node
add_executable(rhphumanoid_walking_node src/rhphumanoid_walking_node.cpp)
target_link_libraries(rhphumanoid_walking_node ${PROJECT_NAME}_lib)
ament_target_dependencies(rhphumanoid_walking_node rclcpp)

# Install
install(TARGETS rhphumanoid_walking_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
