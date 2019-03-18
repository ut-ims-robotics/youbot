cmake_minimum_required(VERSION 2.8.3)
project(youbot_navigation_common)

find_package(catkin REQUIRED roscpp geometry_msgs)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(lowpass_filter
	src/lowpass_filter.cpp
)


target_link_libraries(lowpass_filter ${catkin_LIBRARIES})

