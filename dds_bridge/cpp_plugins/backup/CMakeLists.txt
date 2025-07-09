cmake_minimum_required(VERSION 3.17)
project(cpp_plugins CXX)

# Enable debug symbols
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# # Function to find and install package if not found
# function(find_and_install_package package_name)
#     find_package(${package_name} QUIET)
#     if(NOT ${package_name}_FOUND)
#         message(STATUS "${package_name} not found, attempting to install...")
        
#         # Try to install using apt (Ubuntu/Debian)
#         execute_process(
#             COMMAND sudo apt update
#             RESULT_VARIABLE apt_update_result
#             OUTPUT_QUIET ERROR_QUIET
#         )
        
#         if(apt_update_result EQUAL 0)
#             execute_process(
#                 COMMAND sudo apt install -y ${package_name}
#                 RESULT_VARIABLE install_result
#                 OUTPUT_QUIET ERROR_QUIET
#             )
            
#             if(install_result EQUAL 0)
#                 message(STATUS "Successfully installed ${package_name}")
#                 find_package(${package_name} REQUIRED)
#             else()
#                 message(WARNING "Failed to install ${package_name} via apt")
#             endif()
#         else()
#             message(WARNING "Failed to update package list")
#         endif()
#     else()
#         message(STATUS "${package_name} found")
#     endif()
# endfunction()

# # Function to find and install development package if not found
# function(find_and_install_dev_package package_name dev_package_name)
#     find_package(${package_name} QUIET)
#     if(NOT ${package_name}_FOUND)
#         message(STATUS "${package_name} not found, attempting to install development package...")
        
#         # Try to install using apt (Ubuntu/Debian)
#         execute_process(
#             COMMAND sudo apt update
#             RESULT_VARIABLE apt_update_result
#             OUTPUT_QUIET ERROR_QUIET
#         )
        
#         if(apt_update_result EQUAL 0)
#             execute_process(
#                 COMMAND sudo apt install -y ${dev_package_name}
#                 RESULT_VARIABLE install_result
#                 OUTPUT_QUIET ERROR_QUIET
#             )
            
#             if(install_result EQUAL 0)
#                 message(STATUS "Successfully installed ${dev_package_name}")
#                 find_package(${package_name} REQUIRED)
#             else()
#                 message(WARNING "Failed to install ${dev_package_name} via apt")
#             endif()
#         else()
#             message(WARNING "Failed to update package list")
#         endif()
#     else()
#         message(STATUS "${package_name} found")
#     endif()
# endfunction()

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
find_package(CycloneDDS-CXX)

# Find PkgConfig for other dependencies
find_package(PkgConfig)

# Find Ceres
find_package(Ceres REQUIRED)

# Find Pinocchio
find_package(pinocchio REQUIRED)

# Find Eigen3 (required for Pinocchio and other components)
find_package(Eigen3 REQUIRED)

# Find IPOPT - try system installation first, then custom installation
find_package(IPOPT QUIET)
if(NOT IPOPT_FOUND)
    message(WARNING "IPOPT not found via find_package. Trying manual search...")
    
    # Try to find IPOPT in system locations
    find_path(IPOPT_INCLUDE_DIRS
        NAMES IpIpoptApplication.hpp
        PATHS /usr/include/coin /usr/local/include/coin-or
        NO_DEFAULT_PATH
    )
    
    find_library(IPOPT_LIBRARIES
        NAMES ipopt
        PATHS /usr/lib /usr/local/lib
        NO_DEFAULT_PATH
    )
    
    if(IPOPT_INCLUDE_DIRS AND IPOPT_LIBRARIES)
        message(STATUS "IPOPT found in system installation")
        set(IPOPT_FOUND TRUE)
    else()
        # Try custom installation as fallback
        set(IPOPT_ROOT "/home/shanto/ipopt-install")
        find_path(IPOPT_INCLUDE_DIRS
            NAMES IpIpoptApplication.hpp
            PATHS ${IPOPT_ROOT}/include ${IPOPT_ROOT}/src/Interfaces
            NO_DEFAULT_PATH
        )
        
        find_library(IPOPT_LIBRARIES
            NAMES ipopt
            PATHS ${IPOPT_ROOT}/lib ${IPOPT_ROOT}/src/.libs
            NO_DEFAULT_PATH
        )
        
        if(IPOPT_INCLUDE_DIRS AND IPOPT_LIBRARIES)
            message(STATUS "IPOPT found in custom installation: ${IPOPT_ROOT}")
            set(IPOPT_FOUND TRUE)
        else()
            message(FATAL_ERROR "IPOPT not found in system or custom installation")
            message(STATUS "IPOPT include dir: ${IPOPT_INCLUDE_DIRS}")
            message(STATUS "IPOPT library: ${IPOPT_LIBRARIES}")
        endif()
    endif()
else()
    message(STATUS "IPOPT found via find_package")
endif()

# Find RobOptim
# find_and_install_dev_package(RobOptim roboptim-core-dev)

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

# # 5) Image_odom_db
# add_executable(ik_sub
#   src/ik_sub.cpp
#   src/motor_controller.cpp 
#   src/ik_model.cpp
#   ${GEN_DIR}/arm_msgs.cpp
# )

# target_include_directories(ik_sub PRIVATE
#   ${GEN_DIR}
#   $ENV{CXX_INSTALL}/include
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   # /opt/openrobots/include/pinocchio/
# )

# target_link_libraries(ik_sub PUBLIC
# pinocchio::pinocchio  
# CycloneDDS-CXX::ddscxx
# )

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
# add_executable(test_ik_model
#   src/test_ik_model.cpp
#   src/ik_model.cpp
# )

# target_include_directories(test_ik_model PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# target_link_libraries(test_ik_model PUBLIC
#   pinocchio::pinocchio
# )

# # 8) IK Solver test executable
# add_executable(test_ik_solver
#   src/test_ik_solver.cpp
#   src/ik_solver.cpp
#   src/ik_model.cpp
# )

# target_include_directories(test_ik_solver PRIVATEe
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${IPOPT_INCLUDE_DIRS}
# )

# target_link_libraries(test_ik_solver PUBLIC
#   pinocchio::pinocchio
#   ${IPOPT_LIBRARIES}
# )

# target_compile_options(test_ik_solver PRIVATE ${IPOPT_CFLAGS_OTHER})

# # 9) Ceres IK Solver test executable
# add_executable(test_ik_solver
#   src/test_ik_solver.cpp
#   src/ik_solver.cpp
#   src/ik_model.cpp
# )

# target_include_directories(test_ik_solver PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# target_link_libraries(test_ik_solver PUBLIC
#   pinocchio::pinocchio
#   Ceres::ceres
# )

# 10) IPOPT IK Solver test executable
add_executable(test_ik_solver_v2
  src/test_ik_solver_v2.cpp
  src/ik_solver_v2.cpp
  src/ik_model.cpp
)

target_include_directories(test_ik_solver_v2 PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${IPOPT_INCLUDE_DIRS}
)

target_link_libraries(test_ik_solver_v2 PUBLIC
  pinocchio::pinocchio
  ${IPOPT_LIBRARIES}
)

target_compile_options(test_ik_solver_v2 PRIVATE ${IPOPT_CFLAGS_OTHER})

# 11) RobOptim IK Solver v3 test executable
# add_executable(test_ik_solver_v3
#   src/test_ik_solver_v3.cpp
#   src/ik_solver_v3.cpp
# )
# 
# target_include_directories(test_ik_solver_v3 PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )