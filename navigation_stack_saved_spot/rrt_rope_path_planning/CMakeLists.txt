cmake_minimum_required(VERSION 3.0.2)
project(rrt_rope_path_planning)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  std_msgs
  message_generation
  rospy
  visualization_msgs
)

 add_service_files(
   FILES
   PathPlanningPlugin.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS nav_msgs rospy visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
