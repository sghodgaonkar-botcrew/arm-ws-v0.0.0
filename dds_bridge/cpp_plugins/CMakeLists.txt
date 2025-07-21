cmake_minimum_required(VERSION 3.16)
project(ik_solver_v2)

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

# Find Boost for reachable_workspace
find_package(Boost REQUIRED COMPONENTS random)

# Find OpenMP for reachable_workspace
find_package(OpenMP)

# Create ik_solver_v2 executable
add_executable(ik_solver_v2
    src/ik_solver_v2.cpp
    src/robot_model.cpp
)

# Create profiled version of ik_solver_v2
add_executable(ik_solver_v2_profiled
    src/ik_solver_v2_profiled.cpp
    src/robot_model.cpp
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

# Create reachable_workspace executable
add_executable(reachable_workspace
    src/reachable_workspace.cpp
    src/robot_model.cpp
)

# Create optimized reachable_workspace executable
add_executable(reachable_workspace_optimized
    src/reachable_workspace_optimized.cpp
    src/robot_model.cpp
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

# Set include directories
target_include_directories(ik_solver_v2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    /opt/openrobots/include # System Pinocchio
    /home/shanto/proxsuite/include # ProxSuite headers
)

target_include_directories(ik_solver_v2_profiled PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    /opt/openrobots/include # System Pinocchio
    /home/shanto/proxsuite/include # ProxSuite headers
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

target_include_directories(reachable_workspace PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    /opt/openrobots/include # System Pinocchio
)

target_include_directories(reachable_workspace_optimized PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    /opt/openrobots/include # System Pinocchio
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

# Link libraries
target_link_libraries(ik_solver_v2
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
    proxsuite::proxsuite
)

target_link_libraries(ik_solver_v2_profiled
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
    proxsuite::proxsuite
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

target_link_libraries(reachable_workspace
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
    ${Boost_LIBRARIES}
)

target_link_libraries(reachable_workspace_optimized
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
    ${Boost_LIBRARIES}
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

# Add OpenMP support if available
if(OpenMP_CXX_FOUND)
    target_link_libraries(reachable_workspace OpenMP::OpenMP_CXX)
    target_compile_options(reachable_workspace PRIVATE ${OpenMP_CXX_FLAGS})
endif()

# Print status
message(STATUS "=== Build Configuration ===")
message(STATUS "  Eigen3: ${EIGEN3_FOUND}")
message(STATUS "  Pinocchio: ${PINOCCHIO_FOUND} (from /opt/openrobots)")
message(STATUS "  ProxSuite: ${proxsuite_FOUND} (from /usr/local)")
message(STATUS "  CycloneDDS-CXX: ${CycloneDDS-CXX_FOUND}")
message(STATUS "  Moteus: ${moteus_POPULATED}")
message(STATUS "  Boost: ${Boost_FOUND}")
message(STATUS "  OpenMP: ${OpenMP_CXX_FOUND}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
