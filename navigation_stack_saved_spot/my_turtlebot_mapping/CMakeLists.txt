cmake_minimum_required(VERSION 3.0.2)
project(my_turtlebot_mapping)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_turtlebot_mapping
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
