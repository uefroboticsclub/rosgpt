cmake_minimum_required(VERSION 3.0.2)
project(turtle_agent)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)