cmake_minimum_required(VERSION 3.5)
project(op3_offset_tuner_server)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(op3_offset_tuner_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robotis_controller REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_base_module REQUIRED)

# Resolve system dependency on yaml-cpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

# Include directories
include_directories(
  include
  ${robotis_math_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Add executable
add_executable(offset_tuner_server_node src/op3_offset_tuner_server.cpp src/main.cpp)
ament_target_dependencies(offset_tuner_server_node ament_index_cpp rclcpp std_msgs op3_offset_tuner_msgs dynamixel_sdk robotis_controller robotis_math op3_base_module)

# Link libraries
target_link_libraries(offset_tuner_server_node ${YAML_CPP_LIBRARIES})

# Install
install(TARGETS offset_tuner_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}/launch
)

# Ament package
ament_package()
