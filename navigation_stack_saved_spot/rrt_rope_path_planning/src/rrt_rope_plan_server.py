#!/usr/bin/env python3

import rospy
from rrt_rope_path_planning.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from rrt_rope import Planning

def make_plan(req):

  # Convert costmap from a 1-D tuple flat map representation
  map = list(req.costmap_ros)
  # Change values on the map from unknown to free space
  map[map==255] = 1
  width = req.width
  height = req.height
  start_index = req.start
  goal_index = req.goal

  initial_position = indexToGridCell(start_index, width)
  target_position = indexToGridCell(goal_index, width)

  # time statistics
  start_time = rospy.Time.now()

  
  # Calculate a path using RRT-Rope
  
  rrt_con= Planning(initial_position, target_position, width, height, map)
  
  path = rrt_con.rrt_rope()

  if not path:
    rospy.logwarn("No path returned by RRT-Rope")
    return_path = []
  else:
    # print time statistics
    execution_time = rospy.Time.now() - start_time
    print("\n")
    rospy.loginfo('+++++++++++ RRT-Rope execution metrics ++++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")
    rospy.loginfo('RRT-Rope: Path sent to navigation stack')

    return_path = []
    for cell in path:
        return_path.append(cell[0]+width*cell[1])

  resp = PathPlanningPluginResponse()
  resp.plan = return_path
  
  return resp

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

def indexToGridCell(flat_map_index, map_width):

  grid_cell_map_x = flat_map_index % map_width
  grid_cell_map_y = flat_map_index // map_width
  return [grid_cell_map_x, grid_cell_map_y]

if __name__ == '__main__':
  rospy.init_node('rrt_rope_path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  rospy.on_shutdown(clean_shutdown)
  
  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
