cmake_minimum_required (VERSION 3.4)
project (test_iam_robolib
  VERSION 0.0.1 
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra)

add_subdirectory(iam_robolib)
add_subdirectory(iam_robolib_common)
add_subdirectory(libfranka)

add_executable(test_iam_robolib test_robolib.cpp)

target_link_libraries(test_iam_robolib 
  iam_robolib 
  iam_robolib_common
  Franka::Franka)

install(TARGETS test_iam_robolib DESTINATION bin)
