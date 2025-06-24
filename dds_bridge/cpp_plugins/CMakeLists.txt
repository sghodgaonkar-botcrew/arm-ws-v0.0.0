cmake_minimum_required(VERSION 3.14)
project(cpp_plugins LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

# Allow user to specify install location via env var or default path
if(NOT DEFINED ENV{CYCLONE_INSTALL})
    message(FATAL_ERROR "Please set environment variable CYCLONE_INSTALL to point to CycloneDDS installation")
endif()

list(APPEND CMAKE_PREFIX_PATH "$ENV{CYCLONE_INSTALL}")
message(STATUS "CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")

# Find CycloneDDS-CXX package
find_package(CycloneDDS-CXX REQUIRED)

# Fetch Moteus from GitHub
include(FetchContent)

FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        2a44cf4e27902cb786d154b85b56bf5d9b567c22
)

# Make moteus available
FetchContent_MakeAvailable(moteus)

# Location where IDL-generated code exists
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Your sources
add_executable(ik_solver_sub
  src/ik_solver_sub.cpp
  ${GEN_DIR}/arm_msgs.cpp
)

# Include generated headers + your own include directory
target_include_directories(ik_solver_sub PRIVATE
  ${GEN_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${moteus_SOURCE_DIR}/lib
)

# Link against CycloneDDS-CXX
target_link_libraries(ik_solver_sub
  CycloneDDS-CXX::ddscxx
  moteus::cpp
)
