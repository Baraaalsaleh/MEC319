#!/usr/bin/env python2
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("arm")

hand_group = moveit_commander.MoveGroupCommander("hand")


# We can get the name of the reference frame for this robot:
#planning_frame = arm_group.get_planning_frame()
#print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
#eef_link = arm_group.get_end_effector_link()
#print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
#group_names = robot.get_group_names()
#print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:

#print "============ Printing robot state"
#print arm_group.get_current_state()
#print ""

"""
arm_group.set_named_target("start")
plan1 = arm_group.go()

hand_group.set_named_target("start")
plan2 = hand_group.go()

hand_group.set_named_target("open")
plan2 = hand_group.go()

arm_group.set_named_target("up")
plan1 = arm_group.go()

hand_group.set_named_target("close")
plan2 = hand_group.go()

"""

up_joints = [0, -pi/2, 0, 0, 0, 0]

pick_joints_1 = [0, -0.95, 1.9, (-pi/2)-0.95, -pi/2, 0]
pick_joints_2 = [0, -0.8, 1.8, (-pi/2)-0.9, -pi/2, 0]
# We can get the joint values from the group and adjust some of the values:


# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
arm_group.clear_pose_targets()
arm_group.go(up_joints, wait=True)

hand_group.set_named_target("open")
plan2 = hand_group.go()

arm_group.clear_pose_targets()
arm_group.go(pick_joints_1, wait=True)

arm_group.clear_pose_targets()
arm_group.go(pick_joints_2, wait=True)

hand_group.set_named_target("close")
plan2 = hand_group.go()

arm_group.clear_pose_targets()
arm_group.go(up_joints, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
#arm_group.stop()
#
#arm_group.clear_pose_targets()
#pose_goal = geometry_msgs.msg.Pose()
#pose_goal.position.x = 1
#pose_goal.position.y = 1.5
#pose_goal.position.z = 0.8
#arm_group.set_pose_target(pose_goal)

#plan1 = arm_group.go()
# Calling `stop()` ensures that there is no residual movement
#arm_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
arm_group.clear_pose_targets()

#arm_group.set_named_target("Zero")
#plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()