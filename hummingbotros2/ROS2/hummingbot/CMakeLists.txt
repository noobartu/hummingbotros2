cmake_minimum_required(VERSION 3.5)
project(hummingbot)

# Find dependencies in ROS 2
find_package(ament_cmake REQUIRED)

# Uncomment this if the package has Python scripts
# find_package(ament_cmake_python REQUIRED)

################################################
## Declare ROS 2 messages, services, and actions ##
################################################

# To declare and build messages, services, or actions from within this
# package, follow these steps:
# * In the file package.xml:
#   * add a build_depend tag for "message_generation"
#   * add a build_depend and an exec_depend tag for each package in MSG_DEP_SET
#   * If MSG_DEP_SET isn't empty, the following dependency has been pulled in
#     but can be declared for certainty nonetheless:
#     * add an exec_depend tag for "message_runtime"
# * In this file (CMakeLists.txt):
#   * add "message_generation" and every package in MSG_DEP_SET to
#     find_package(ament_cmake REQUIRED COMPONENTS ...)
#   * add "message_runtime" and every package in MSG_DEP_SET to
#     ament_package(DEPENDS ...)

# Generate messages in the 'msg' folder (if any)
# add_message_files(
#   DIRECTORY msg
#   FILES
#   Message1.msg
#   Message2.msg
# )

# Generate services in the 'srv' folder (if any)
# add_service_files(
#   DIRECTORY srv
#   FILES
#   Service1.srv
#   Service2.srv
# )

# Generate actions in the 'action' folder (if any)
# add_action_files(
#   DIRECTORY action
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS 2 dynamic reconfigure parameters ##
################################################

# To declare and build dynamic reconfigure parameters within this
# package, follow these steps:
# * In the file package.xml:
#   * add a build_depend and an exec_depend tag for "dynamic_reconfigure"
# * In this file (CMakeLists.txt):
#   * add "dynamic_reconfigure" to find_package(ament_cmake REQUIRED COMPONENTS ...)
#   * uncomment the "ros2_generate_dynamic_reconfigure_options" section below
#     and list every .cfg file to be processed

# Generate dynamic reconfigure parameters in the 'cfg' folder (if any)
# ros2_generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
# The ament_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if your package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# AMENT_DEPENDS: ament packages dependent projects also need
ament_package(
  # INCLUDE_DIRS include
  # LIBRARIES hummingbot
  # AMENT_DEPENDS other_ament_pkg
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  # include
  # ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library (if you have any)
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hummingbot.cpp
# )

# Add cmake target dependencies of the library (if any)
# as an example, code may need to be generated before libraries
# either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable (if you have any)
# With ament_cmake, all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/hummingbot_node.cpp)

# Rename C++ executable without prefix (if you have any)
# The above recommended prefix causes long target names; the following renames the
# target back to the shorter version for ease of user use
# e.g., "ros2 run someones_pkg node" instead of "ros2 run someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# Add cmake target dependencies of the executable (if any)
# Same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against (if any)
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# All install targets should use ament DESTINATION variables
# See https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html#use-the-ament-macro

## Mark executable scripts (Python, etc.) for installation
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION lib/${PROJECT_NAME}
# )

## Mark executables for installation
# See https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html#use-the-ament-macro
# install(TARGETS ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME}
# )

## Mark libraries for installation
# See https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html#use-the-ament-macro
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g., launch and bag files, etc.)
# See https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html#use-the-ament-macro
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION share/${PROJECT_NAME}
# )

#############
## Testing ##
#############

## Add gtest-based cpp test target and link libraries
# ament_add_gtest(${PROJECT_NAME}-test test/test_hummingbot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# See https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html#use-the-ament-macro
# ament_add_nosetests(test)
