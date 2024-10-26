cmake_minimum_required(VERSION 3.0.2)
project(chassis)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  dynamic_reconfigure
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python nodes) for installation
catkin_install_python(PROGRAMS
  scripts/donkey_chassis.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
