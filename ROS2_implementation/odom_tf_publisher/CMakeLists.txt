cmake_minimum_required(VERSION 3.5)
project(odom_tf_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_target_dependencies(odom_to_tf rclcpp nav_msgs tf2_ros geometry_msgs)

install(TARGETS
  odom_to_tf
  DESTINATION lib/${PROJECT_NAME})

ament_package()
