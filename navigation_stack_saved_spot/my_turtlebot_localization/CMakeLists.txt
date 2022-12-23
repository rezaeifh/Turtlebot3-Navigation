cmake_minimum_required(VERSION 3.0.2)
project(my_turtlebot_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  MyServiceMessage.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
      CATKIN_DEPENDS
      rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
