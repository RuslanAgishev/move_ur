#!/usr/bin/env python

import rospy
import moveit_commander

if __name__=='__main__':
  rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
  group = moveit_commander.MoveGroupCommander("manipulator")
  initial_position = [-1.534351650868551, -2.02321702638735, 0.17670011520385742,
  -0.5149157683001917, -1.5798452536212366, 2.372737169265747]
  group.set_joint_value_target(initial_position)
  plan_init = group.plan()
  rospy.sleep(5)
  group.execute(plan_init)