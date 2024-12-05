cmake_minimum_required(VERSION 3.5)
project(op3_navigation)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config launch maps
	DESTINATION share/${PROJECT_NAME}
)
