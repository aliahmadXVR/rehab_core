cmake_minimum_required(VERSION 3.0.2)
project(rehab_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
)
include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
add_executable(rehab_teleop src/rehab_teleop.cpp)
target_link_libraries(rehab_teleop ${catkin_LIBRARIES})
install(TARGETS rehab_teleop DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_executable(rehab_virtual_teleop src/rehab_virtual_teleop.cpp)
target_link_libraries(rehab_virtual_teleop ${catkin_LIBRARIES})