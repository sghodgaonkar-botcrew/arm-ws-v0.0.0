cmake_minimum_required(VERSION 3.16)
project(ik_solver_v2)

# Set CMake policies to handle warnings and conflicts
cmake_policy(SET CMP0077 NEW)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set debug flags for development
if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Debug)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
    message(STATUS "Building in Debug mode with symbols")
endif()

# Include FetchContent for downloading dependencies
include(FetchContent)

# Disable test builds to prevent target name conflicts
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
set(BUILD_TESTING OFF)
set(NANOFLANN_BUILD_TESTS OFF)
set(JSONCONS_BUILD_TESTS OFF)
set(CPPZMQ_BUILD_TESTS OFF)

# Fetch Nanoflann for efficient k-d tree operations
FetchContent_Declare(
    nanoflann
    GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
    GIT_TAG master
)
FetchContent_MakeAvailable(nanoflann)

# Fetch Moteus from GitHub
FetchContent_Declare(
    moteus
    GIT_REPOSITORY https://github.com/mjbots/moteus.git
    GIT_TAG 2a44cf4e27902cb786d154b85b56bf5d9b567c22
)

# Make Moteus available
FetchContent_MakeAvailable(moteus)

# Fetch jsoncons for JSON handling
FetchContent_Declare(
    jsoncons
    GIT_REPOSITORY https://github.com/danielaparker/jsoncons.git
    GIT_TAG v0.171.0
)
FetchContent_MakeAvailable(jsoncons)

# Fetch cppzmq for ZeroMQ communication
FetchContent_Declare(
    cppzmq
    GIT_REPOSITORY https://github.com/zeromq/cppzmq.git
    GIT_TAG v4.10.0
)
FetchContent_MakeAvailable(cppzmq)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

# Find Pinocchio from system installation
find_package(pinocchio REQUIRED)

# Find ProxSuite from system installation
list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/cmake/proxsuite")
find_package(proxsuite REQUIRED)

# Find CycloneDDS-CXX for DDS communication
find_package(CycloneDDS-CXX CONFIG REQUIRED)

# Find Boost for reachable_workspace and OMPL
find_package(Boost REQUIRED COMPONENTS random filesystem)

# Find OpenMP for reachable_workspace
find_package(OpenMP)

# Find ZeroMQ (required for cppzmq)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# Find OMPL (Open Motion Planning Library)
# Add OMPL CMake config path
list(APPEND CMAKE_PREFIX_PATH "/usr/local/share/ompl/cmake")
find_package(ompl CONFIG REQUIRED)
if(ompl_FOUND)
    message(STATUS "OMPL found via CMake config")
else()
    message(FATAL_ERROR "OMPL not found. Please install libompl-dev")
endif()

# Create test_cps_link executable
# Note: This requires CoppeliaSim remote API to be installed
# The RemoteAPIClient.h header should be available in CoppeliaSim installation
add_executable(test_cps_link
    src/test_cps_link.cpp
    src/cps_link.cpp
    src/planner_v0.cpp
    src/robot_model.cpp
    src/ik_solver_w_rws.cpp
    /home/shanto/CoppeliaSim_Pro_V4_10_0_rev0_Ubuntu22_04/programming/zmqRemoteApi/clients/cpp/RemoteAPIClient.cpp
)

# Create ik_solver_w_rws_dev executable
add_executable(ik_solver_w_rws_dev
    src/ik_solver_w_rws_dev.cpp
    src/robot_model.cpp
)

# Create test_ik_solver_w_rws executable
add_executable(test_ik_solver_w_rws
    src/test_ik_solver_w_rws.cpp
    src/ik_solver_w_rws.cpp
    src/robot_model.cpp
)

# Create ik_sub executable
add_executable(ik_sub
    src/ik_sub.cpp
    src/ik_solver_w_rws.cpp
    src/robot_model.cpp
    src/motor_controller.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/generated/arm_msgs.cpp
)

# Create reachable workspace generator library
add_library(reachable_workspace_generator
    src/reachable_workspace_generator.cpp
    src/robot_model.cpp
)

# Create example workspace generator executable
add_executable(example_workspace_generator
    src/example_workspace_generator.cpp
)

# Create test_planner executable
add_executable(test_planner
    src/test_planner.cpp
    src/planner_v0.cpp
    src/robot_model.cpp
    src/ik_solver_w_rws.cpp
)

# Set include directories
target_include_directories(test_cps_link PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${jsoncons_SOURCE_DIR}/include
    ${cppzmq_SOURCE_DIR}
    ${ZMQ_INCLUDE_DIRS}
    /home/shanto/CoppeliaSim_Pro_V4_10_0_rev0_Ubuntu22_04/programming/zmqRemoteApi/clients/cpp # CoppeliaSim remote API headers
    /usr/local/include/ompl-1.7 # OMPL headers
    /usr/include/eigen3 # Eigen3 headers
    /opt/openrobots/include # System Pinocchio
    ${nanoflann_SOURCE_DIR}/include # Nanoflann headers
)

target_include_directories(ik_solver_w_rws_dev PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    /opt/openrobots/include # System Pinocchio
    /home/shanto/proxsuite/include # ProxSuite headers
    ${nanoflann_SOURCE_DIR}/include # Nanoflann headers
)

target_include_directories(test_ik_solver_w_rws PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    /opt/openrobots/include # System Pinocchio
    /home/shanto/proxsuite/include # ProxSuite headers
    ${nanoflann_SOURCE_DIR}/include # Nanoflann headers
)

target_include_directories(ik_sub PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/generated # Generated DDS message headers
    ${moteus_SOURCE_DIR}/lib/cpp # Moteus library headers
    ${EIGEN3_INCLUDE_DIRS}
    /opt/openrobots/include # System Pinocchio
    /home/shanto/proxsuite/include # ProxSuite headers
    ${nanoflann_SOURCE_DIR}/include # Nanoflann headers
)

target_include_directories(reachable_workspace_generator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    /opt/openrobots/include # System Pinocchio
)

target_include_directories(example_workspace_generator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    /opt/openrobots/include # System Pinocchio
)

target_include_directories(test_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/local/include/ompl-1.7
    /usr/include/eigen3
    /opt/openrobots/include # System Pinocchio
    ${nanoflann_SOURCE_DIR}/include # Nanoflann headers
)

# Link libraries
target_link_libraries(test_cps_link
    ${ZMQ_LIBRARIES}
    cppzmq
    ${Boost_LIBRARIES}
    ompl::ompl
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
)

target_link_libraries(ik_solver_w_rws_dev
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
    proxsuite::proxsuite
)

target_link_libraries(test_ik_solver_w_rws
    reachable_workspace_generator
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
    proxsuite::proxsuite
)

target_link_libraries(ik_sub
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
    proxsuite::proxsuite
    CycloneDDS-CXX::ddscxx
    moteus::cpp
)

target_link_libraries(reachable_workspace_generator
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
    ${Boost_LIBRARIES}
)

target_link_libraries(example_workspace_generator
    reachable_workspace_generator
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
    ${Boost_LIBRARIES}
)

target_link_libraries(test_planner
    ompl::ompl
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
)

# Print status
message(STATUS "=== Build Configuration ===")
message(STATUS "  Eigen3: ${EIGEN3_FOUND}")
message(STATUS "  Pinocchio: ${PINOCCHIO_FOUND} (from /opt/openrobots)")
message(STATUS "  ProxSuite: ${proxsuite_FOUND} (from /usr/local)")
message(STATUS "  CycloneDDS-CXX: ${CycloneDDS-CXX_FOUND}")
message(STATUS "  Moteus: ${moteus_POPULATED}")
message(STATUS "  Boost: ${Boost_FOUND}")
message(STATUS "  OpenMP: ${OpenMP_CXX_FOUND}")
message(STATUS "  jsoncons: ${jsoncons_POPULATED}")
message(STATUS "  cppzmq: ${cppzmq_POPULATED}")
message(STATUS "  ZeroMQ: ${ZMQ_FOUND}")
message(STATUS "  OMPL: ${OMPL_FOUND}")
message(STATUS "  CoppeliaSim Remote API: Installed at /home/shanto/CoppeliaSim_Pro_V4_10_0_rev0_Ubuntu22_04")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
