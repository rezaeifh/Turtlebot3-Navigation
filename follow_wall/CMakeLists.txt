cmake_minimum_required(VERSION 3.0.2)
project(follow_wall)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  nav_msgs

)

 add_service_files(
   FILES
   FindWall.srv
)

 add_action_files(
   FILES
   OdomRecord.action
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   nav_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES course_project
  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
