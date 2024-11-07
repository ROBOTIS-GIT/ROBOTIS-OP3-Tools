cmake_minimum_required(VERSION 3.5)
project(op3_web_setting_tool)

find_package(ament_cmake REQUIRED)

install(DIRECTORY html launch 
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
