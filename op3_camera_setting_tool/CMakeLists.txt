################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(op3_camera_setting_tool)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  dynamic_reconfigure
  message_generation
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

add_message_files(
  FILES
  V4lParameter.msg
  V4lParameters.msg
  CameraParams.msg
)

add_service_files(
  FILES
  GetParameters.srv
  SetParameters.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################
generate_dynamic_reconfigure_options(cfg/camera_params.cfg)

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy dynamic_reconfigure
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(op3_camera_setting_tool src/main.cpp)

add_dependencies(op3_camera_setting_tool ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(op3_camera_setting_tool
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
