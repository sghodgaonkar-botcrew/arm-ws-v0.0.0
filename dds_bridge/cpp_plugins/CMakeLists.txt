cmake_minimum_required(VERSION 3.17)
project(cpp_plugins CXX)

# Enable debug symbols
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# Enable ccache for faster rebuilds
# find_program(CCACHE_PROGRAM ccache)
# if(CCACHE_PROGRAM)
#     set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
#     message(STATUS "Using ccache for faster builds")
# endif()

# # Disable Moteus updates to speed up builds
# set(FETCHCONTENT_UPDATES_DISCONNECTED ON)

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

# Find IPOPT
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# … your other find_package() calls …
find_package(pinocchio REQUIRED)
# message(STATUS "Pinocchio include dirs: ${pinocchio_INCLUDE_DIRS}")
# message(STATUS "Pinocchio CMake dir:   ${pinocchio_DIR}")

# Debug pinocchio targets and variables (commented out for performance)
# get_property(pinocchio_targets DIRECTORY PROPERTY TARGETS)
# message(STATUS "Available targets: ${pinocchio_targets}")

# Check if pinocchio::pinocchio target exists
# if(TARGET pinocchio::pinocchio)
#     message(STATUS "pinocchio::pinocchio target found")
# else()
#     message(STATUS "pinocchio::pinocchio target NOT found")
#     # List all targets that contain "pinocchio"
#     get_property(all_targets DIRECTORY PROPERTY TARGETS)
#     foreach(target ${all_targets})
#         if(target MATCHES ".*pinocchio.*")
#             message(STATUS "Found pinocchio-related target: ${target}")
#         endif()
#     endforeach()
# endif()

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
add_executable(ik_sub
  src/ik_sub.cpp
  src/motor_controller.cpp 
  src/ik_model.cpp
  ${GEN_DIR}/arm_msgs.cpp
)

target_include_directories(ik_sub PRIVATE
  ${GEN_DIR}
  $ENV{CXX_INSTALL}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  # /opt/openrobots/include/pinocchio/
)

target_link_libraries(ik_sub PUBLIC
pinocchio::pinocchio  
CycloneDDS-CXX::ddscxx
)

# 6) IPOPT IK Solver test executable (commented out - files deleted)
# add_executable(test_ipopt_solver
#   src/test_ipopt_solver.cpp
#   src/ipopt_ik_solver.cpp
#   src/ik_model.cpp
# )
# 
# target_include_directories(test_ipopt_solver PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${IPOPT_INCLUDE_DIRS}
# )
# 
# target_link_libraries(test_ipopt_solver PUBLIC
#   pinocchio::pinocchio
#   ${IPOPT_LIBRARIES}
# )
# 
# target_compile_options(test_ipopt_solver PRIVATE ${IPOPT_CFLAGS_OTHER})

# 7) IK Model test executable
add_executable(test_ik_model
  src/test_ik_model.cpp
  src/ik_model.cpp
)

target_include_directories(test_ik_model PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_ik_model PUBLIC
  pinocchio::pinocchio
)