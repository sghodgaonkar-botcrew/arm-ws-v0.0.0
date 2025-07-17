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

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

# Find Pinocchio from system installation
find_package(pinocchio REQUIRED)

# Find ProxSuite from system installation
list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/cmake/proxsuite")
find_package(proxsuite REQUIRED)

# Find Boost for reachable_workspace
find_package(Boost REQUIRED COMPONENTS random)

# Find OpenMP for reachable_workspace
find_package(OpenMP)

# Create ik_solver_v2 executable
add_executable(ik_solver_v2
    src/ik_solver_v2.cpp
    src/ik_model.cpp
)

# Create profiled version of ik_solver_v2
add_executable(ik_solver_v2_profiled
    src/ik_solver_v2_profiled.cpp
    src/ik_model.cpp
)

# Create ik_solver_w_rws executable
add_executable(ik_solver_w_rws
    src/ik_solver_w_rws.cpp
    src/ik_model.cpp
)

# Create reachable_workspace executable
add_executable(reachable_workspace
    src/reachable_workspace.cpp
    src/ik_model.cpp
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

target_include_directories(ik_solver_w_rws PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
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

target_link_libraries(ik_solver_w_rws
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
    proxsuite::proxsuite
)

target_link_libraries(reachable_workspace
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
message(STATUS "  Boost: ${Boost_FOUND}")
message(STATUS "  OpenMP: ${OpenMP_CXX_FOUND}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
