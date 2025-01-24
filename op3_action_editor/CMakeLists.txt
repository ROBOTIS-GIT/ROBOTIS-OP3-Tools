cmake_minimum_required(VERSION 3.8)
project(op3_action_editor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robotis_controller REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_action_module REQUIRED)
find_package(op3_base_module REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

## N curses
find_package(Curses REQUIRED)

## YAML related
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

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIRS}
  ${op3_base_module_INCLUDE_DIRS}
  ${op3_action_module_INCLUDE_DIRS}
  ${robotis_math_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp src/action_editor.cpp)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES} ${CURSES_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  std_msgs
  dynamixel_sdk
  robotis_controller
  robotis_device
  robotis_math
  op3_action_module
  op3_base_module
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/executor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

