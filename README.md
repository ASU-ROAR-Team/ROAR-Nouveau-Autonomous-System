# ROAR Autonomous System Workspace

This workspace contains the main modules for our autonomous robot:

- Localization/
- Perception/
- Path-Planning/
- Control/

Each module contains ROS2 packages created by different team members.

## Build
colcon build --symlink-install

## Source
source install/setup.bash

## To Make A New Package 
## Inside your module folder
ros2 pkg create pkg_name --build-type ament_cmake
