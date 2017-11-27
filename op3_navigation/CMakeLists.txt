cmake_minimum_required(VERSION 2.8.3)
project(op3_navigation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  footstep_planner
  map_server
  octomap_server
)

###################################
## catkin specific configuration ##
###################################
catkin_package()
