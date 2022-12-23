cmake_minimum_required(VERSION 3.0.2)
project(follow_wall_cpp)

find_package(catkin REQUIRED COMPONENTS

  roscpp
  actionlib
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
  CATKIN_DEPENDS roscpp std_msgs actionlib_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
 
add_executable(odom_record_action src/odom_record_action.cpp)
add_dependencies(odom_record_action ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_record_action
   ${catkin_LIBRARIES}
)
 
add_executable(find_nearest_wall src/find_nearest_wall.cpp)
add_dependencies(find_nearest_wall ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_nearest_wall
   ${catkin_LIBRARIES}
)
 
add_executable(follow_wall_run src/follow_wall_run.cpp)
add_dependencies(follow_wall_run ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(follow_wall_run
   ${catkin_LIBRARIES}
)
