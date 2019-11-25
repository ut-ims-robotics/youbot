cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
project(youbot_manual_operation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  #keyboard_reader
  actionlib
  message_generation
  joy
)

add_service_files(
  FILES
  executeTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#   INCLUDE_DIRS include
#  INCLUDE_DIRS include
#  LIBRARIES youbot_joint_recorder
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
#   include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${EIGEN_INCLUDE_DIRS}	
)


## Declare a C++ executable
#add_executable(youbot_manual_operation src/youbotManualOperation.cpp)
#add_executable(youbot_trajectory_recorder src/trajectoryRecorder.cpp)
add_executable(youbot_joy src/youbot_joy.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(youbot_manual_operation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(youbot_trajectory_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(youbot_joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(youbot_manual_operation ${catkin_LIBRARIES})
#target_link_libraries(youbot_trajectory_recorder ${catkin_LIBRARIES})
target_link_libraries(youbot_joy ${catkin_LIBRARIES})
