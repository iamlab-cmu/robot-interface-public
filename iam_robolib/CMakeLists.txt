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
  src/TerminationHandler/contact_termination_handler.cpp
  src/TerminationHandler/contact_termination_handler.h
  src/Skills/skill_info.cpp
  src/Skills/skill_info.h
  src/skill_info_manager.cpp
  src/skill_info_manager.h
  src/TrajectoryGenerator/trajectory_generator.cpp
  src/TrajectoryGenerator/trajectory_generator.h
  src/TrajectoryGenerator/goal_linear_trajectory_generator.cpp
  src/TrajectoryGenerator/goal_linear_trajectory_generator.h
  src/TrajectoryGenerator/linear_trajectory_generator.cpp
  src/TrajectoryGenerator/linear_trajectory_generator.h
  src/TrajectoryGenerator/linear_trajectory_generator_with_time_and_goal.cpp
  src/TrajectoryGenerator/linear_trajectory_generator_with_time_and_goal.h
  src/TrajectoryGenerator/linear_trajectory_generator_with_time_and_goal_termination_handler.cpp
  src/TrajectoryGenerator/linear_trajectory_generator_with_time_and_goal_termination_handler.h
  src/FeedbackController/torque_feedback_controller.cpp
  src/FeedbackController/torque_feedback_controller.h
  src/TrajectoryGenerator/counter_trajectory_generator.cpp
  src/TrajectoryGenerator/counter_trajectory_generator.h
  src/FeedbackController/FeedbackController.cpp
  src/FeedbackController/FeedbackController.h
  src/FeedbackController/NoopFeedbackController.cpp
  src/FeedbackController/NoopFeedbackController.h
  src/TerminationHandler/TerminationHandler.cpp
  src/TerminationHandler/TerminationHandler.h
  src/TerminationHandler/CounterTerminationHandler.cpp
  src/TerminationHandler/CounterTerminationHandler.h
  src/TerminationHandler/time_termination_handler.cpp
  src/TerminationHandler/time_termination_handler.h
  src/TrajectoryGenerator/stay_in_initial_position_trajectory_generator.cpp
  src/TrajectoryGenerator/stay_in_initial_position_trajectory_generator.h
  src/SensorData.cpp
  src/SensorData.h
  src/TerminationHandler/NoopTerminationHandler.cpp
  src/TerminationHandler/NoopTerminationHandler.h
  src/RunLoopLogger.cpp
  src/RunLoopLogger.h
  src/TerminationHandler/FinalPoseTerminationHandler.cpp
  src/TerminationHandler/FinalPoseTerminationHandler.h
  src/ControlLoopData.cpp
  src/ControlLoopData.h
  src/TerminationHandler/FinalJointTerminationHandler.cpp
  src/TerminationHandler/FinalJointTerminationHandler.h
  src/TrajectoryGenerator/LinearJointTrajectoryController.cpp
  src/TrajectoryGenerator/LinearJointTrajectoryController.h
  src/Skills/GripperOpenSkill.cpp
  src/Skills/GripperOpenSkill.h
  src/TrajectoryGenerator/GripperOpenTrajectoryGenerator.cpp
  src/TrajectoryGenerator/GripperOpenTrajectoryGenerator.h
  src/Skills/BaseSkill.cpp
  src/Skills/BaseSkill.h
  #src/JsonFileLogger.cpp
  #src/JsonFileLogger.h
  src/FileStreamLogger.cpp
  src/FileStreamLogger.h
  src/TrajectoryGenerator/DMPTrajectoryGenerator.cpp
  src/TrajectoryGenerator/DMPTrajectoryGenerator.h
  src/Skills/JointPoseSkill.cpp
  src/Skills/JointPoseSkill.h
  src/Skills/JointPoseWithTorqueControlSkill.cpp
  src/Skills/JointPoseWithTorqueControlSkill.h
  src/TrajectoryGenerator/relative_linear_trajectory_generator.cpp
  src/TrajectoryGenerator/relative_linear_trajectory_generator.h
  src/FeedbackController/CustomGainTorqueController.cpp
  src/FeedbackController/CustomGainTorqueController.h
  src/Skills/SaveTrajectorySkill.cpp
  src/Skills/SaveTrajectorySkill.h
  src/Skills/JointPoseContinuousSkill.cpp
  src/Skills/JointPoseContinuousSkill.h
  src/Skills/BaseMetaSkill.cpp
  src/Skills/BaseMetaSkill.h
  src/SharedMemoryHandler.cpp
  src/SharedMemoryHandler.h
  src/TrajectoryGeneratorFactory.cpp
  src/TrajectoryGeneratorFactory.h
  src/FeedbackControllerFactory.cpp
  src/FeedbackControllerFactory.h
  src/TerminationHandlerFactory.cpp
  src/TerminationHandlerFactory.h
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
