cmake_minimum_required(VERSION 3.16)
project(cpp_plugins CXX)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Check for required environment variables
# if(NOT DEFINED ENV{CXX_INSTALL})
#     message(FATAL_ERROR "CXX_INSTALL environment variable is not set")
# endif()

# 1) Let CMake look under your custom DDS prefixes
# list(APPEND CMAKE_PREFIX_PATH
#   "$ENV{CYCLONE_INSTALL}"
#   "$ENV{CXX_INSTALL}"
#   # "/opt/openrobots"
# )
message(STATUS "CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")

# message(STATUS "CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")

# 2) Pull in Iceoryx C binding (installed via apt)
# find_package(iceoryx_binding_c CONFIG REQUIRED)

# 3) Pull in CycloneDDS C++ wrapper
find_package(CycloneDDS-CXX CONFIG REQUIRED)

# … your other find_package() calls …
find_package(pinocchio REQUIRED)
# message(STATUS "Pinocchio include dirs: ${pinocchio_INCLUDE_DIRS}")
# message(STATUS "Pinocchio CMake dir:   ${pinocchio_DIR}")

# Debug pinocchio targets and variables
get_property(pinocchio_targets DIRECTORY PROPERTY TARGETS)
message(STATUS "Available targets: ${pinocchio_targets}")

# Check if pinocchio::pinocchio target exists
if(TARGET pinocchio::pinocchio)
    message(STATUS "pinocchio::pinocchio target found")
else()
    message(STATUS "pinocchio::pinocchio target NOT found")
    # List all targets that contain "pinocchio"
    get_property(all_targets DIRECTORY PROPERTY TARGETS)
    foreach(target ${all_targets})
        if(target MATCHES ".*pinocchio.*")
            message(STATUS "Found pinocchio-related target: ${target}")
        endif()
    endforeach()
endif()

# Fetch Moteus from GitHub
include(FetchContent)

FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        2a44cf4e27902cb786d154b85b56bf5d9b567c22
)

# Make Moteus available
FetchContent_MakeAvailable(moteus)

# 4) Where your IDL‐generated code lives
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

# 5) Image_odom_db
add_executable(ik_solver_sub
  src/ik_solver_sub.cpp
  src/motor_controller.cpp 
  src/ik_model.cpp
  ${GEN_DIR}/arm_msgs.cpp
)

target_include_directories(ik_solver_sub PRIVATE
  ${GEN_DIR}
  $ENV{CXX_INSTALL}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  # /opt/openrobots/include/pinocchio/
)

target_link_libraries(ik_solver_sub PUBLIC
pinocchio::pinocchio  
CycloneDDS-CXX::ddscxx
)