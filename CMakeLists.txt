cmake_minimum_required(VERSION 3.10)
project(router LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(router
  src/main.cpp
  src/include/impl/pathfinder.cpp
  src/include/impl/parser.cpp
  src/include/impl/output.cpp
)
target_include_directories(router PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/external
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
)
