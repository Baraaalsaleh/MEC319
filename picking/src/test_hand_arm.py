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

def prepare_pose(goal_pose):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = goal_pose.position.x + 0.1
    pose.position.y = goal_pose.position.y - 1.25
    pose.position.z = goal_pose.position.z - 0.65
    pose.orientation.w = goal_pose.orientation.w
    pose.orientation.x = goal_pose.orientation.x
    pose.orientation.y = goal_pose.orientation.y
    pose.orientation.z = goal_pose.orientation.z


    return pose

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("hand_arm")

##Joints: 
# 0 = shoulder_pan_joint
# 1 = shoulder_lift_joint
# 2 = elbow_joint
# 3 = wrist_1_joint
# 4 = wrist_2_joint
# 5 = wrist_3_joint
# 6 = gripper_joint


joint_goal = arm_group.get_current_joint_values()

joint_goal[0] = pi/2
joint_goal[1] = -pi/2
joint_goal[2] = 0.0
joint_goal[3] = pi/2
joint_goal[4] = pi/2
joint_goal[5] = pi/2
joint_goal[6] = 0.5

#arm_group.clear_pose_targets()
arm_group.go(joint_goal, wait=True)

#arm_group.stop()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()