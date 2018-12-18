cmake_minimum_required (VERSION 3.4)
project (iam_robolib
  VERSION 0.0.1 
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra)

## Dependencies
# find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(Eigen3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost COMPONENTS REQUIRED filesystem system thread)

include_directories(${Boost_INCLUDE_DIRS})

# find_package(nlohmann_json 3.4.0 REQUIRED)

# find_library(iam_robolib_common PATHS ../)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../iam_robolib_common")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../libfranka")

## Library

add_library(iam_robolib SHARED
  src/run_loop.cpp
  src/duration.cpp
  src/contact_termination_handler.cpp
  src/contact_termination_handler.h
  src/skill_info.cpp
  src/skill_info.h
  src/skill_info_manager.cpp
  src/skill_info_manager.h
  src/trajectory_generator.cpp
  src/trajectory_generator.h
  src/goal_linear_trajectory_generator.cpp
  src/goal_linear_trajectory_generator.h
  src/linear_trajectory_generator.cpp
  src/linear_trajectory_generator.h
  src/linear_trajectory_generator_with_time_and_goal.cpp
  src/linear_trajectory_generator_with_time_and_goal.h
  src/linear_trajectory_generator_with_time_and_goal_termination_handler.cpp
  src/linear_trajectory_generator_with_time_and_goal_termination_handler.h
  src/torque_feedback_controller.cpp
  src/torque_feedback_controller.h
  src/counter_trajectory_generator.cpp
  src/counter_trajectory_generator.h
  src/FeedbackController.cpp
  src/FeedbackController.h
  src/NoopFeedbackController.cpp
  src/NoopFeedbackController.h
  src/TerminationHandler.cpp
  src/TerminationHandler.h
  src/CounterTerminationHandler.cpp
  src/CounterTerminationHandler.h
  src/time_termination_handler.cpp
  src/time_termination_handler.h
  src/stay_in_initial_position_trajectory_generator.cpp
  src/stay_in_initial_position_trajectory_generator.h
  src/SensorData.cpp
  src/SensorData.h
  src/NoopTerminationHandler.cpp
  src/NoopTerminationHandler.h
  src/RunLoopLogger.cpp
  src/RunLoopLogger.h
  src/FinalPoseTerminationHandler.cpp
  src/FinalPoseTerminationHandler.h
  src/ControlLoopData.cpp
  src/ControlLoopData.h
  src/FinalJointTerminationHandler.cpp
  src/FinalJointTerminationHandler.h
  src/LinearJointTrajectoryController.cpp
  src/LinearJointTrajectoryController.h
  src/GripperOpenSkill.cpp
  src/GripperOpenSkill.h
  src/GripperOpenTrajectoryGenerator.cpp
  src/GripperOpenTrajectoryGenerator.h
  src/BaseSkill.cpp
  src/BaseSkill.h
  #src/JsonFileLogger.cpp
  #src/JsonFileLogger.h
  src/FileStreamLogger.cpp
  src/FileStreamLogger.h
  src/DMPTrajectoryGenerator.cpp
  src/DMPTrajectoryGenerator.h
  src/JointPoseSkill.cpp
  src/JointPoseSkill.h
  src/JointPoseWithTorqueControlSkill.cpp
  src/JointPoseWithTorqueControlSkill.h
  src/relative_linear_trajectory_generator.cpp
  src/relative_linear_trajectory_generator.h
  src/CustomGainTorqueController.cpp
  src/CustomGainTorqueController.h
  src/SaveTrajectorySkill.cpp
  src/SaveTrajectorySkill.h
  src/JointPoseContinuousSkill.cpp
  src/JointPoseContinuousSkill.h
  src/BaseMetaSkill.cpp
  src/BaseMetaSkill.h
  src/SharedMemoryHandler.cpp
  src/SharedMemoryHandler.h
  src/TrajectoryGeneratorFactory.cpp
  src/TrajectoryGeneratorFactory.h
)

# Add namespace
# add_library(IAM_RoboLib::IAM_RoboLib ALIAS iam_robolib)

target_link_libraries(iam_robolib PRIVATE
  Eigen3::Eigen3
  Threads::Threads
  ${Boost_LIBRARIES}
  rt
  iam_robolib_common
  Franka::Franka
  # nlohmann_json::nlohmann_json
)


target_include_directories(iam_robolib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set_target_properties(iam_robolib PROPERTIES PUBLIC_HEADER include/run_loop.h)

target_include_directories(iam_robolib PRIVATE src)
target_include_directories(iam_robolib PRIVATE include)

## Installation
include(GNUInstallDirs)
install(TARGETS iam_robolib
  # EXPORT FrankaTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  # RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
