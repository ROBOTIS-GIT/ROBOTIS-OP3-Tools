################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(op3_offset_tuner_server)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  cmake_modules
  dynamixel_sdk
  robotis_device
  robotis_framework_common
  robotis_math
  op3_base_module
  op3_kinematics_dynamics
  robotis_controller_msgs
  robotis_controller
  op3_offset_tuner_msgs
)

find_package(Eigen REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(yaml_cpp REQUIRED yaml-cpp)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(offset_tuner_server_node 
   src/op3_offset_tuner_server.cpp
   src/main.cpp
)

target_link_libraries(offset_tuner_server_node
  ${yaml_cpp_LIBRARIES}
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
