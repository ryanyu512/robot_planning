#!/usr/bin/env python

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from gridviz import GridViz
from algorithms.a_star import *

def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  ''' 

  # costmap as 1-D array representation
  costmap = req.costmap_ros

  # number of columns in the occupancy grid
  map_w = req.width

  # number of rows in the occupancy grid
  map_h = req.height

  #initialise start and goal position
  s_ind = req.start
  g_ind = req.goal

  # side of each grid map square in meters
  resolution = 0.05
  # origin of grid map
  origin = [-10, -10, 0]

  viz = GridViz(costmap, resolution, origin, s_ind, g_ind, map_w)

  # time statistics
  start_time = rospy.Time.now()

  #convert 1d index to 2d index
  s_pos = (s_ind % map_h, int(s_ind / map_w))
  g_pos = (g_ind % map_h, int(g_ind / map_w))
  
 # calculate the shortest path
  astar = Astar(costmap, map_h, map_w)
  path  = astar.search_path(s_pos, g_pos)

  if not path:
    rospy.logwarn("No path returned by the path algorithm")
    path = []
  else:
    execution_time = rospy.Time.now() - start_time
    print("\n")
    rospy.loginfo('++++++++ Path Planning execution metrics ++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")
    rospy.loginfo('Path sent to navigation stack')

  resp = PathPlanningPluginResponse()
  resp.plan = path
  return resp

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

if __name__ == '__main__':
  rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  rospy.on_shutdown(clean_shutdown)

  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
