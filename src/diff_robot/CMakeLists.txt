cmake_minimum_required(VERSION 3.8)
project(diff_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(controller_manager REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch urdf world control map planning config rviz
  DESTINATION share/${PROJECT_NAME}/
)




add_executable(pose_publisher src/odom_pose.cpp)
ament_target_dependencies(pose_publisher rclcpp nav_msgs geometry_msgs)

add_executable(vel_publisher src/vel_publisher.cpp)
ament_target_dependencies(vel_publisher rclcpp geometry_msgs)

# Install Python scripts
install(PROGRAMS
  scripts/wheel_sync_node.py
  scripts/diagnose_gps.py
  scripts/trailer_joint_publisher.py
  scripts/cmd_vel_safety_node.py
  scripts/trailer_aware_controller_node.py
  scripts/trailer_dashboard_node.py
  scripts/trailer_control_node.py
  scripts/dynamic_footprint_node.py
  control/keyboard_control.py
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

install(TARGETS
  pose_publisher
  vel_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
