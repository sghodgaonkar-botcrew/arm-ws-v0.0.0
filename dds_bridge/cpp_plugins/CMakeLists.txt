cmake_minimum_required(VERSION 3.5)
project(pcl_cpp_pluggins LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# 1) Let CMake look under your custom DDS prefixes
list(APPEND CMAKE_PREFIX_PATH
  "$ENV{CYCLONE_INSTALL}"
  "$ENV{CXX_INSTALL}"
)
message(STATUS "CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")

# 2) Pull in Iceoryx C binding (installed via apt)
find_package(iceoryx_binding_c CONFIG REQUIRED)

# 3) Pull in CycloneDDS C++ wrapper
find_package(CycloneDDS-CXX CONFIG REQUIRED)


# Fetch Moteus from GitHub
include(FetchContent)

FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        2a44cf4e27902cb786d154b85b56bf5d9b567c22
)

# Make moteus available
FetchContent_MakeAvailable(moteus)


# 4) Where your IDL‐generated code lives
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

# 5) Image_odom_db
add_executable(ik_solver_sub
  src/ik_solver_sub.cpp
  src/motor_controller.cpp 
  ${GEN_DIR}/arm_msgs.cpp
)

target_include_directories(ik_solver_sub PRIVATE
  ${GEN_DIR}
  $ENV{CXX_INSTALL}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ik_solver_sub
  CycloneDDS-CXX::ddscxx
  iceoryx_binding_c::iceoryx_binding_c
)