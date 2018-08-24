import rospy
import tf2_ros
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

def move_ur(x,y,z):
  ## First initialize moveit_commander and rospy.
  print "============ Starting setup"
  moveit_commander.roscpp_initialize(sys.argv)
  #rospy.init_node('move_group_python_interface_tutorial',
  #                anonymous=True)
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
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.w = 1.0
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


if __name__ == '__main__':
    rospy.init_node('supernode', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    isfound = False
    while not isfound:
        try:
            trans = tfBuffer.lookup_transform('base_link', 'object_30', rospy.Time())
            isfound = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            isfound = False
            continue

        print(trans.transform.translation.x)
        print(trans.transform.translation.y)
        print(trans.transform.translation.z)
        print('______')
        rate.sleep()

    x = trans.transform.translation.x
    y = trans.transform.translation.y
    z = trans.transform.translation.z
    move_ur(x,y,z + 0.1)