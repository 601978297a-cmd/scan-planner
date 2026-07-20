cmake_minimum_required(VERSION 3.8)
project(m20_sensor_pose_adapter_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(sensor_pose_adapter_cpp src/sensor_pose_adapter.cpp)
target_compile_features(sensor_pose_adapter_cpp PUBLIC cxx_std_17)
ament_target_dependencies(
  sensor_pose_adapter_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

install(
  TARGETS sensor_pose_adapter_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
