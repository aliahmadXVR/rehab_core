cmake_minimum_required(VERSION 3.0.2)
project(rehab_msgs)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)
add_message_files(
  FILES
  Encoders.msg
  EncodersStamped.msg
  AngularVelocities.msg
  AngularVelocitiesStamped.msg
  WheelsCmd.msg
  WheelsCmdStamped.msg
  bumper.msg
  SensorState.msg
  cliff.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
