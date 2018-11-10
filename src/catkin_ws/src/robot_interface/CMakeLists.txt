cmake_minimum_required(VERSION 2.8.3)
project(robot_interface)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib_msgs
  message_generation
)

add_action_files(DIRECTORY action
                 FILES
                 execute_skill.action)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    rospy
)

# add_dependencies(your_program ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Installation
#install(DIRECTORY launch
	#DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
