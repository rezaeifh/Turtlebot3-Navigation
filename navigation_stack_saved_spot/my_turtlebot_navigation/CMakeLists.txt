cmake_minimum_required(VERSION 3.0.2)
project(my_turtlebot_navigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  MyCustomServiceMessage.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
      CATKIN_DEPENDS
      rospy
      roscpp
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(get_coordinates_service_server src/get_coordinates_service_server.cpp)
add_dependencies(get_coordinates_service_server ${get_coordinates_service_server_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_coordinates_service_server
   ${catkin_LIBRARIES}
 )
 
add_executable(send_coordinates_action_client src/send_coordinates_action_client.cpp)
add_dependencies(send_coordinates_action_client ${send_coordinates_action_client_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(send_coordinates_action_client
   ${catkin_LIBRARIES}
 )
