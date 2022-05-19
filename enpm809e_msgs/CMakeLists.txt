cmake_minimum_required(VERSION 2.8.3)
project(enpm809e_msgs)

# This package was developed with gazebo7, but now uses gazebo9.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  std_srvs
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  message_generation
)


########################
## Message generation ##
########################

add_message_files(DIRECTORY msg
  FILES
  LogicalCameraImage.msg
  Model.msg
  PartInfo.msg
  PartInfos.msg
  VacuumGripperState.msg
)

add_service_files(DIRECTORY srv
  FILES
  VacuumGripperControl.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_srvs
)


