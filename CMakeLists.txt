cmake_minimum_required(VERSION 3.0.2)
project(rosgpt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib
)

catkin_package()

catkin_install_python(PROGRAMS
  rosgpt.py
  rosgpt_client.py
  rosgptparser_turtlesim.py
  rosgptparser_tb3_nav.py
  rosgptparser_amr.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

