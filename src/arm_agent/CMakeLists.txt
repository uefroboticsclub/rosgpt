cmake_minimum_required(VERSION 3.0.2)
project(arm_agent)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/arm_agent.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)