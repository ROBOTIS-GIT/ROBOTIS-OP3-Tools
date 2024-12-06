cmake_minimum_required(VERSION 3.5)
project(op3_camera_setting_tool)

set(ROS_PACKAGE_NAME ${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(op3_camera_setting_tool_msgs REQUIRED)
# find_package(dynamic_reconfigure REQUIRED)
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
endif()

add_executable(${PROJECT_NAME} src/main.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs op3_camera_setting_tool_msgs ament_index_cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

target_compile_definitions(${PROJECT_NAME} PUBLIC "ROS_PACKAGE_NAME=\"${ROS_PACKAGE_NAME}\"")

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
