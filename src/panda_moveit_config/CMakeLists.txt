cmake_minimum_required(VERSION 3.5)
project(panda_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(panda_description REQUIRED)

# Install directories
install(DIRECTORY config launch rviz srdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS 
  scripts/pick_and_place_demo.py 
  scripts/object_detector.py 
  scripts/planning_scene_setup.py
  scripts/trajectory_recorder.py
  scripts/pose_sweeper.py
  DESTINATION lib/${PROJECT_NAME})

# Setup the project
ament_package()
