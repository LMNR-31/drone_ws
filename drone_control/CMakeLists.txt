cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# Adicionar no final do arquivo drone_control/CMakeLists.txt:

add_executable(drone_activate_and_go_forward src/drone_activate_and_go_forward.cpp)

ament_target_dependencies(drone_activate_and_go_forward
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
)

install(TARGETS
  drone_activate_and_go_forward
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(drone_soft_land src/drone_soft_land.cpp)

ament_target_dependencies(drone_soft_land
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
)

install(TARGETS drone_soft_land
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(mission_manager src/mission_manager.cpp)

ament_target_dependencies(mission_manager rclcpp)

install(TARGETS
  mission_manager
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(drone_publish_landing_waypoints src/drone_publish_landing_waypoints.cpp)

ament_target_dependencies(drone_publish_landing_waypoints
  rclcpp
  geometry_msgs
)

install(TARGETS drone_publish_landing_waypoints
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/camera_viewer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_viewer
)

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
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)