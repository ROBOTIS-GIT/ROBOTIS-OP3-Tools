cmake_minimum_required(VERSION 3.5)
project(op3_gui_demo)

set(ROS_PACKAGE_NAME ${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(op3_walking_module_msgs REQUIRED)
find_package(op3_online_walking_module_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets REQUIRED)

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

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(op3_gui_demo_SRCS
  src/main.cpp
  src/main_window.cpp
  src/preview_walking_form.cpp
  src/qnode.cpp
  src/qnode_default_demo.cpp
  src/qnode_preview_walking.cpp
)

set(op3_gui_demo_HDRS
  include/op3_gui_demo/main_window.hpp
  include/op3_gui_demo/preview_walking_form.h
  include/op3_gui_demo/qnode.hpp
)

set(op3_gui_demo_UIS
  ui/main_window.ui
  ui/preview_walking_form.ui
)

set(op3_gui_demo_RESOURCES
  resources/images.qrc
)

qt5_wrap_cpp(op3_gui_demo_MOCS ${op3_gui_demo_HDRS})
qt5_wrap_ui(op3_gui_demo_UIS_H ${op3_gui_demo_UIS})
qt5_add_resources(op3_gui_demo_RCC ${op3_gui_demo_RESOURCES})

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${interactive_markers_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${op3_gui_demo_SRCS} ${op3_gui_demo_MOCS} ${op3_gui_demo_UIS_H} ${op3_gui_demo_RCC})
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  robotis_controller_msgs
  op3_walking_module_msgs
  op3_online_walking_module_msgs
  interactive_markers
  tf2_geometry_msgs
  tf2_ros
  Eigen3
  Qt5Widgets
)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PUBLIC "ROS_PACKAGE_NAME=\"${ROS_PACKAGE_NAME}\"")

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resources launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
