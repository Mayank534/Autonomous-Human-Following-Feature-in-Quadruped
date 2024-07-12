cmake_minimum_required(VERSION 3.5)
project(human_follower)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executables
add_executable(human_follower human_follower.cpp)
add_executable(human_position_publisher human_position_publisher.cpp)

# Link libraries and dependencies
ament_target_dependencies(human_follower
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(human_position_publisher
  rclcpp
  geometry_msgs
)

# Install executables
install(TARGETS
  human_follower
  human_position_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
