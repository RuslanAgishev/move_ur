#!/usr/bin/env python

## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import tf2_ros
import tf

def move_ur(x,y,z, group):
  ## Planning to a Pose goal
  ##
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plan:1"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.w = 0.27227499757
  pose_target.orientation.x = -0.64832087713
  pose_target.orientation.y = 0.247384081098
  pose_target.orientation.z = 0.666593941163
  pose_target.position.x = x
  pose_target.position.y = y
  pose_target.position.z = 0.15
  group.set_pose_target(pose_target)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  plan_target = group.plan()
  #print "============ Waiting while RVIZ displays plan1..."
  #rospy.sleep(5)
  group.go(wait=True)

def find_object(obj):
  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)
  rate = rospy.Rate(10.0)
  isfound = False
  while not isfound:
      try:
          trans = tfBuffer.lookup_transform('base_link', obj, rospy.Time())
          isfound = True
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          rate.sleep()
          isfound = False
          continue
      rate.sleep()
  return trans

if __name__=='__main__':
  rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
  moveit_commander.roscpp_initialize(sys.argv)
  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()
  group = moveit_commander.MoveGroupCommander("manipulator")
  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()
  initial_position = [-1.534351650868551, -2.02321702638735, 0.17670011520385742,
  -0.5149157683001917, -1.5798452536212366, 2.372737169265747]
  group.set_joint_value_target(initial_position)
  plan_init = group.plan()
  rospy.sleep(5)
  group.go(wait=True)

  trans = find_object(str(sys.argv[1]))
  x = trans.transform.translation.x
  y = trans.transform.translation.y
  z = trans.transform.translation.z
  try: 
    #move_ur(-0.05, -0.30, 0.03 + 0.4, group)
    move_ur(x+0.06,y-0.08,z, group)
  except rospy.ROSInterruptException:
    pass


  group.set_joint_value_target(initial_position)
  plan_init = group.plan()
  group.go(wait=True)
