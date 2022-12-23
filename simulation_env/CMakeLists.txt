cmake_minimum_required(VERSION 3.0.2)
project(simulation_env)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulation_env
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
