cmake_minimum_required(VERSION 3.8)
project(archivebot_description)

find_package(ament_cmake REQUIRED)

# Install the Gazebo world, Xacro model, ROS-Gazebo bridge, and launch file.
install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# These existing Python nodes are deliberately left unchanged.
install(PROGRAMS
  scripts/archivebot_vision.py
  scripts/archivebot_navigation_v11.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
