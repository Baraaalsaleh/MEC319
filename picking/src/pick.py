#!/usr/bin/env python2
import sys
import rospy
import moveit_commander
import geometry_msgs


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("arm")

arm_group.set_named_target("One")
plan1 = arm_group.go()

arm_group.set_named_target("Two")
plan1 = arm_group.go()

arm_group.set_named_target("Three")
plan1 = arm_group.go()

arm_group.set_named_target("Zero")
plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()