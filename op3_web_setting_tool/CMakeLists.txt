cmake_minimum_required(VERSION 2.8.3)
project(op3_web_setting_tool)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
  web_video_server
)

###################################
## catkin specific configuration ##
###################################
catkin_package()
