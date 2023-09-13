cmake_minimum_required(VERSION 3.5)
project(camera_sensor_plugin)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Find ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Build our plugin
add_library(camera_sensor_plugin SHARED camera_sensor_plugin.cc)
ament_target_dependencies(camera_sensor_plugin gazebo_ros ros1_bridge) # Use ros1_bridge for ROS 1 dependencies
target_link_libraries(camera_sensor_plugin ${GAZEBO_LIBRARIES})
