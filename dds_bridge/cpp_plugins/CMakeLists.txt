cmake_minimum_required(VERSION 3.16)
project(ik_profiling_example)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

# Find Pinocchio (default installation)
find_package(pinocchio REQUIRED)

# Find OpenMP (optional, for reachable_workspace parallelization)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found - reachable_workspace will use parallel processing")
else()
    message(STATUS "OpenMP not found - reachable_workspace will use single-threaded processing")
endif()

# Try to find IPOPT (optional)
find_package(PkgConfig QUIET)
pkg_check_modules(IPOPT QUIET ipopt)
if(IPOPT_FOUND)
    add_definitions(-DHAVE_IPOPT)
    message(STATUS "IPOPT found - IK solver will be available")
else()
    message(STATUS "IPOPT not found - IK solver will be disabled")
endif()

# Try to find Acados (optional)
# First try pkg-config, then try custom installation path
find_package(PkgConfig QUIET)
pkg_check_modules(ACADOS QUIET acados)

if(NOT ACADOS_FOUND)
    # Try to find Acados in custom installation path
    set(ACADOS_CUSTOM_PATH "/home/shanto/acados_install")
    if(EXISTS "${ACADOS_CUSTOM_PATH}")
        set(ACADOS_FOUND TRUE)
        set(ACADOS_INCLUDE_DIRS "${ACADOS_CUSTOM_PATH}/include")
        set(ACADOS_LIBRARY_DIRS "${ACADOS_CUSTOM_PATH}/lib")
        set(ACADOS_LIBRARIES "acados")

        # Add additional include paths for blasfeo and hpipm
        list(APPEND ACADOS_INCLUDE_DIRS "${ACADOS_CUSTOM_PATH}/include/blasfeo/include")
        list(APPEND ACADOS_INCLUDE_DIRS "${ACADOS_CUSTOM_PATH}/include/hpipm/include")

        # Also check for blasfeo and hpipm dependencies
        if(EXISTS "${ACADOS_CUSTOM_PATH}/lib/libblasfeo.so")
            list(APPEND ACADOS_LIBRARIES "blasfeo")
        endif()
        if(EXISTS "${ACADOS_CUSTOM_PATH}/lib/libhpipm.so")
            list(APPEND ACADOS_LIBRARIES "hpipm")
        endif()

        # Add math library (always available)
        list(APPEND ACADOS_LIBRARIES "m")

        message(STATUS "Acados found in custom path: ${ACADOS_CUSTOM_PATH}")
    endif()
endif()

if(ACADOS_FOUND)
    add_definitions(-DHAVE_ACADOS)
    message(STATUS "Acados found - IK solver v3 will be available")
    message(STATUS "  Acados include dirs: ${ACADOS_INCLUDE_DIRS}")
    message(STATUS "  Acados library dirs: ${ACADOS_LIBRARY_DIRS}")
else()
    message(STATUS "Acados not found - IK solver v3 will be disabled")
endif()

# Add profiling definitions
add_definitions(-DENABLE_PROFILING)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PINOCCHIO_INCLUDE_DIRS})

# Add Acados include directories if found
if(ACADOS_FOUND)
    include_directories(${ACADOS_INCLUDE_DIRS})
endif()

# Create library
add_library(ik_model_lib
    src/ik_model.cpp
)

# Link libraries for ik_model_lib
target_link_libraries(ik_model_lib
    ${PINOCCHIO_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)

# Create IK solver library (only if IPOPT is available)
if(IPOPT_FOUND)
    add_library(ik_solver_lib
        src/ik_solver_v2.cpp
    )

    target_link_libraries(ik_solver_lib
        ik_model_lib
        ${IPOPT_LIBRARIES}
    )

    target_include_directories(ik_solver_lib PRIVATE
        ${IPOPT_INCLUDE_DIRS}
    )
endif()

# Create Acados-based IK solver library (only if Acados is available)
if(ACADOS_FOUND)
    add_library(ik_solver_v3_lib
        src/ik_solver_v3.cpp
    )

    target_link_libraries(ik_solver_v3_lib
        ${ACADOS_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/../c_code/ik_solver_acados/libacados_ocp_solver_ik_model.so
    )

    target_include_directories(ik_solver_v3_lib PRIVATE
        ${ACADOS_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../c_code/ik_solver_acados
    )

    target_link_directories(ik_solver_v3_lib PRIVATE
        ${ACADOS_LIBRARY_DIRS}
    )
endif()

# # Create profiling example executable
# add_executable(profile_example
#     src/profile_example.cpp
# )

# # Link libraries for profiling example
# target_link_libraries(profile_example
#     ik_model_lib
# )

# if(IPOPT_FOUND)
#     target_include_directories(profile_example PRIVATE
#         ${IPOPT_INCLUDE_DIRS}
#     )
#     target_link_libraries(profile_example
#         ik_solver_lib
#         ${IPOPT_LIBRARIES}
#     )
# endif()

# Create IPOPT profiling test executable (only if IPOPT is available)
if(IPOPT_FOUND)
    #     add_executable(ipopt_profile_test
    #         src/ipopt_profile_test.cpp
    #     )

    #     target_link_libraries(ipopt_profile_test
    #         ik_model_lib
    #         ik_solver_lib
    #         ${IPOPT_LIBRARIES}
    #     )

    #     target_include_directories(ipopt_profile_test PRIVATE
    #         ${IPOPT_INCLUDE_DIRS}
    #     )

    #     # Create detailed IPOPT profiling test
    #     add_executable(ipopt_detailed_profile
    #         src/ipopt_detailed_profile.cpp
    #     )

    #     target_link_libraries(ipopt_detailed_profile
    #         ik_model_lib
    #         ik_solver_lib
    #         ${IPOPT_LIBRARIES}
    #     )

    #     target_include_directories(ipopt_detailed_profile PRIVATE
    #         ${IPOPT_INCLUDE_DIRS}
    #     )

    #     # Create simple IPOPT profiling test
    #     add_executable(ipopt_simple_profile
    #         src/ipopt_simple_profile.cpp
    #     )

    #     target_link_libraries(ipopt_simple_profile
    #         ik_model_lib
    #         ik_solver_lib
    #         ${IPOPT_LIBRARIES}
    #     )

    #     target_include_directories(ipopt_simple_profile PRIVATE
    #         ${IPOPT_INCLUDE_DIRS}
    #     )


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

    option(ENABLE_ASAN "Build with AddressSanitizer for test_ik_solver_v2" OFF)
    option(VALGRIND_SETUP "Build with Valgrind debugging flags for test_ik_solver_v2" OFF)

    if(ENABLE_ASAN)
        message(STATUS "AddressSanitizer for test_ik_solver_v2: ENABLED")
        target_compile_options(test_ik_solver_v2 PRIVATE
            -g -O1 -fsanitize=address
        )
        target_link_options(test_ik_solver_v2 PRIVATE
            -fsanitize=address
        )
    else()
        message(STATUS "AddressSanitizer for test_ik_solver_v2: DISABLED")
    endif()

    if(VALGRIND_SETUP)
        message(STATUS "Valgrind setup for test_ik_solver_v2: ENABLED")
        target_compile_options(test_ik_solver_v2 PRIVATE -g -O0)
    else()
        message(STATUS "Valgrind setup for test_ik_solver_v2: DISABLED")
    endif()


    message(STATUS "IPOPT profiling test targets created")
else()
    message(STATUS "IPOPT profiling test targets skipped (IPOPT not found)")
endif()

# Create Acados IK Solver test executable (only if Acados is available)
if(ACADOS_FOUND)
    add_executable(test_ik_solver_v3
        src/ik_solver_v3.cpp
    )

    target_include_directories(test_ik_solver_v3 PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ACADOS_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../c_code/ik_solver_acados
    )

    target_link_libraries(test_ik_solver_v3 PUBLIC
        ${ACADOS_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/../c_code/ik_solver_acados/libacados_ocp_solver_ik_model.so
    )

    target_link_directories(test_ik_solver_v3 PRIVATE
        ${ACADOS_LIBRARY_DIRS}
    )

    target_compile_options(test_ik_solver_v3 PRIVATE ${ACADOS_CFLAGS_OTHER})

    option(ENABLE_ASAN_V3 "Build with AddressSanitizer for test_ik_solver_v3" OFF)
    option(VALGRIND_SETUP_V3 "Build with Valgrind debugging flags for test_ik_solver_v3" OFF)

    if(ENABLE_ASAN_V3)
        message(STATUS "AddressSanitizer for test_ik_solver_v3: ENABLED")
        target_compile_options(test_ik_solver_v3 PRIVATE
            -g -O1 -fsanitize=address
        )
        target_link_options(test_ik_solver_v3 PRIVATE
            -fsanitize=address
        )
    else()
        message(STATUS "AddressSanitizer for test_ik_solver_v3: DISABLED")
    endif()

    if(VALGRIND_SETUP_V3)
        message(STATUS "Valgrind setup for test_ik_solver_v3: ENABLED")
        target_compile_options(test_ik_solver_v3 PRIVATE -g -O0)
    else()
        message(STATUS "Valgrind setup for test_ik_solver_v3: DISABLED")
    endif()

    message(STATUS "Acados IK solver test targets created")
else()
    message(STATUS "Acados IK solver test targets skipped (Acados not found)")
endif()

# Add reachable_workspace executable
add_executable(reachable_workspace src/reachable_workspace.cpp)

# Link libraries for reachable_workspace
target_link_libraries(reachable_workspace
    ik_model_lib
    ${PINOCCHIO_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
)

# Add OpenMP support for reachable_workspace if available
if(OpenMP_CXX_FOUND)
    target_link_libraries(reachable_workspace OpenMP::OpenMP_CXX)
    target_compile_options(reachable_workspace PRIVATE ${OpenMP_CXX_FLAGS})
    message(STATUS "OpenMP linked to reachable_workspace")
endif()

# Enable AddressSanitizer for reachable_workspace only
# target_compile_options(reachable_workspace PRIVATE
#     -g -O1 -fsanitize=address -fno-omit-frame-pointer
# )
# target_link_options(reachable_workspace PRIVATE
#     -fsanitize=address
# )



# Set compiler flags for optimization and debugging
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -g")

# Print configuration summary
message(STATUS "Configuration Summary:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Profiling: ENABLED")
message(STATUS "  IPOPT: ${IPOPT_FOUND}")
message(STATUS "  Acados: ${ACADOS_FOUND}")
message(STATUS "  Eigen3: ${EIGEN3_FOUND}")
message(STATUS "  Pinocchio: ${PINOCCHIO_FOUND}")
message(STATUS "  OpenMP: ${OpenMP_CXX_FOUND}")
message(STATUS "  AddressSanitizer: ${ENABLE_ASAN}")
message(STATUS "  Valgrind Setup: ${VALGRIND_SETUP}")
