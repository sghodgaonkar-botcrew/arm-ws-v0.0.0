cmake_minimum_required(VERSION 3.8)
project(ros_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH
  "$ENV{CYCLONE_INSTALL}"
  "$ENV{CXX_INSTALL}"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# --- Cyclone DDS C++ bindings ---
find_package(CycloneDDS-CXX CONFIG REQUIRED)
find_package(iceoryx_binding_c CONFIG REQUIRED)

set(GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../cpp_plugins/build/generated")

include_directories(${CMAKE_SOURCE_DIR}/generated)

add_executable(ros_dds_bridge
  src/ros_dds_bridge.cpp
  ${GEN_DIR}/arm_msgs.cpp)

ament_target_dependencies(ros_dds_bridge
  rclcpp
  std_msgs
)


target_include_directories(ros_dds_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${GEN_DIR}
  $ENV{CXX_INSTALL}/include)

target_link_libraries(ros_dds_bridge
  CycloneDDS-CXX::ddscxx
  iceoryx_binding_c::iceoryx_binding_c
)

target_compile_features(ros_dds_bridge PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

install(TARGETS ros_dds_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
