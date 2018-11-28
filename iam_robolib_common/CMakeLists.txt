cmake_minimum_required (VERSION 3.4)
project (iam_robolib_common
  VERSION 0.0.1 
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Dependencies
# find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(Eigen3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost COMPONENTS REQUIRED filesystem system thread)

include_directories(${Boost_INCLUDE_DIRS})

## Library
add_library(iam_robolib_common SHARED
  src/run_loop_process_info.cpp
  src/SharedMemoryInfo.cpp
)

# Add namespace
# add_library(IAM_RoboLib::IAM_RoboLib ALIAS iam_robolib)

target_link_libraries(iam_robolib_common PRIVATE
  Threads::Threads
  ${Boost_LIBRARIES}
  rt
)


target_include_directories(iam_robolib_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set_target_properties(iam_robolib_common PROPERTIES PUBLIC_HEADER include/run_loop_info.h)

target_include_directories(iam_robolib_common PRIVATE src)


## Installation
include(GNUInstallDirs)
install(TARGETS iam_robolib_common
  # EXPORT FrankaTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
