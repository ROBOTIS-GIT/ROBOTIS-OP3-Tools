cmake_minimum_required(VERSION 3.5)
project(op3_tuner_client)

set(ROS_PACKAGE_NAME ${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(op3_tuning_module_msgs REQUIRED)
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

set(op3_tuner_client_SRCS
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

set(op3_tuner_client_HDRS
  include/op3_tuner_client/main_window.hpp
  include/op3_tuner_client/qnode.hpp
)

set(op3_tuner_client_UIS
  ui/main_window.ui
)

set(op3_tuner_client_RESOURCES
  resources/images.qrc
)

qt5_wrap_cpp(op3_tuner_client_MOCS ${op3_tuner_client_HDRS})
qt5_wrap_ui(op3_tuner_client_UIS_H ${op3_tuner_client_UIS})
qt5_add_resources(op3_tuner_client_RCC ${op3_tuner_client_RESOURCES})

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${op3_tuning_module_msgs_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${op3_tuner_client_SRCS} ${op3_tuner_client_MOCS} ${op3_tuner_client_UIS_H} ${op3_tuner_client_RCC})

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  std_msgs
  op3_tuning_module_msgs
  Qt5Widgets
)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PUBLIC "ROS_PACKAGE_NAME=\"${ROS_PACKAGE_NAME}\"")

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resources launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
