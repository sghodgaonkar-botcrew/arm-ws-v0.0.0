cmake_minimum_required(VERSION 3.16)
project(ik_solver_v2)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

# Find Pinocchio from system installation
find_package(pinocchio REQUIRED)

# Find ProxSuite from system installation
list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/cmake/proxsuite")
find_package(proxsuite REQUIRED)

# Create ik_solver_v2 executable
add_executable(ik_solver_v2
    src/ik_solver_v2.cpp
    src/ik_model.cpp
)

# Set include directories
target_include_directories(ik_solver_v2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    /opt/openrobots/include # System Pinocchio
    /home/shanto/proxsuite/include # ProxSuite headers
)

# Link libraries
target_link_libraries(ik_solver_v2
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
    ${EIGEN3_LIBRARIES}
    proxsuite::proxsuite
)

# Print status
message(STATUS "=== Build Configuration ===")
message(STATUS "  Eigen3: ${EIGEN3_FOUND}")
message(STATUS "  Pinocchio: ${PINOCCHIO_FOUND} (from /opt/openrobots)")
message(STATUS "  ProxSuite: ${proxsuite_FOUND} (from /usr/local)")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
