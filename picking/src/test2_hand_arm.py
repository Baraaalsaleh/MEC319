#!/usr/bin/env python2
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
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

kasten_angles = [(-pi/2)-(pi/12), (-pi/2)-(3*pi/12), (-pi/2)-(5*pi/12), (pi/2)+(5*pi/12), (pi/2)+(3*pi/12), (pi/2)+(pi/12)]
ready_to_pick = [0, -0.7, 1.1, -(pi/2)-0.45, 3*(pi/2), 0, 0.7]
ready_to_pick_1 = [0, -1.1, 1.1, -(pi/2), 3*(pi/2), 0, 0]
ready_to_pick_2 = [0.0, 0.18, 0.0, -0.18, pi, 0.0, 0.0]


def ready(moveit_group):
    joint_goal = moveit_group.get_current_joint_values()
    #joint_goal[0] = +0.35
    joint_goal[1] = -pi/2
    moveit_group.go(joint_goal, wait=True)
    moveit_group.go(ready_to_pick, wait=True)
    #moveit_group.go(ready_to_pick_1, wait=True)
    #moveit_group.go(ready_to_pick_2, wait=True)

def pick(moveit_group ,size_in_cm):
    joint_goal = moveit_group.get_current_joint_values()
    #joint_goal[0] = -0.25
    joint_goal[6] = 0.8-(0.0655*size_in_cm) #0.4
    #eef_link = moveit_group.get_end_effector_link()
    #print(eef_link)
    #scene.attach_box(eef_link, "redBox1", touch_links=touch_links)

    moveit_group.go(joint_goal, wait=True)

def hold_up(moveit_group):
    joint_goal = moveit_group.get_current_joint_values()
    joint_goal[3] = -pi/2
    joint_goal[4] = pi/2
    moveit_group.go(joint_goal, wait=True)

def place(moveit_group, kasten_index):
    joint_goal = moveit_group.get_current_joint_values()
    joint_goal[1] = -pi/2
    joint_goal[3] = -pi/2
    moveit_group.go(joint_goal, wait=True)
    joint_goal = moveit_group.get_current_joint_values()
    joint_goal[0] = kasten_angles[kasten_index]
    moveit_group.go(joint_goal, wait=True)
    joint_goal = moveit_group.get_current_joint_values()
    joint_goal[1] = -pi/4
    moveit_group.go(joint_goal, wait=True)
    joint_goal = moveit_group.get_current_joint_values()
    joint_goal[3] = ready_to_pick[3]
    joint_goal[4] = ready_to_pick[4]
    joint_goal[6] = 0
    moveit_group.go(joint_goal, wait=True)



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("hand_arm")

#scene = moveit_commander.PlanningSceneInterface()

#touch_links = robot.get_link_names(group=arm_group)

#print(touch_links)
#i = 0
#while i < 20:
#    ready(arm_group)
#    i += 1


ready(arm_group)
#joint_goal = arm_group.get_current_joint_values()
#print(joint_goal)
#pick(arm_group, 5.0)
#time.sleep(0.5)
#hold_up(arm_group)
#place(arm_group, 3)

##Joints: 
# 0 = shoulder_pan_joint
# 1 = shoulder_lift_joint
# 2 = elbow_joint
# 3 = wrist_1_joint
# 4 = wrist_2_joint
# 5 = wrist_3_joint
# 6 = gripper_joint


#joint_goal = arm_group.get_current_joint_values()

#joint_goal[0] = pi/2#
#joint_goal[1] = -pi/2
#joint_goal[2] = 0.0
#joint_goal[3] = pi/2
#joint_goal[4] = pi/2
#joint_goal[5] = pi/2
#joint_goal[6] = 0.35

#arm_group.clear_pose_targets()
#arm_group.go(joint_goal, wait=True)

#arm_group.set_named_target("ready_to_pick")
#arm_group.go(wait=True)

#arm_group.stop()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()