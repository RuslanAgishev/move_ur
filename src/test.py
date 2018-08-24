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

def move_ur(x,y,z, xr,yr,zr,w):
  ## First initialize moveit_commander and rospy.
  print "============ Starting setup"
  moveit_commander.roscpp_initialize(sys.argv)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("manipulator")


  ## Planning to a Pose goal
  ##
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plan:1"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.w = w
  pose_target.orientation.x = xr
  pose_target.orientation.y = yr
  pose_target.orientation.z = zr
  pose_target.position.x = x
  pose_target.position.y = y
  pose_target.position.z = z
  group.set_pose_target(pose_target)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  plan1 = group.plan()
  

  #print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(5)

  #group.execute(plan1)

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
  group = moveit_commander.MoveGroupCommander("manipulator")
  print group.get_current_pose()

  # group.clear_pose_targets()
  # rospy.sleep(5)
  # print(plan_init)
  # group.execute(plan_init)
  # trans = find_object('object_45')
  # x = trans.transform.translation.x
  # y = trans.transform.translation.y
  # z = trans.transform.translation.z
  # xr = trans.transform.rotation.x
  # yr = trans.transform.rotation.y
  # zr = trans.transform.rotation.z
  # w = trans.transform.rotation.w
  # roll, pitch, yaw = tf.transformations.euler_from_quaternion([xr,yr,zr,w])
  # print (roll)
  # print(pitch)
  # print(yaw)
  # try: 
  #   move_ur(-0.05, -0.30, 0.03 + 0.4, 0,0,1,1)
  #   #move_ur(x,y,z, xr,yr+3.14,zr,w)
  # except rospy.ROSInterruptException:
  #   pass
  # rospy.sleep(3)
  # group.execute(plan_init)