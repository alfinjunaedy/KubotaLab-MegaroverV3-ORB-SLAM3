cmake_minimum_required(VERSION 3.8)
project(rover_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# Dependencies
# ============================================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ============================================================================
# Rover Controller
# ============================================================================

add_executable(
  rover_controller
  src/rover_controller.cpp
)

ament_target_dependencies(
  rover_controller
  rclcpp
  geometry_msgs
  std_msgs
)

# ============================================================================
# ORB Localization Node
# ============================================================================

add_executable(
  orb_localization_node
  src/orb_localization_node.cpp
)

ament_target_dependencies(
  orb_localization_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ============================================================================
# Install
# ============================================================================

# Install executables
install(
  TARGETS
    rover_controller
    orb_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# Testing
# ============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()