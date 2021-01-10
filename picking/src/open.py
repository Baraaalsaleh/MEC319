#!/usr/bin/env python2
import sys
import rospy
import moveit_commander
import geometry_msgs



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("arm")

hand_group = moveit_commander.MoveGroupCommander("hand")

hand_group.set_named_target("open")
plan2 = hand_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()