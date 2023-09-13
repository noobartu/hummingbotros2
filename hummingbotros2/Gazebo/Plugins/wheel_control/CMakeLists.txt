cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Find ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build our plugin
add_library(hummingbot_plugin SHARED hummingbot_plugin.cc)
target_link_libraries(hummingbot_plugin ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES})

# Install the plugin
install(
  TARGETS hummingbot_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
