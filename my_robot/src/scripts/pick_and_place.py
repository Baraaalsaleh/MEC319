#!/usr/bin/env python
import rospy
from std_msgs.msg import *
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

kasten_angles = [(-pi/2)-(pi/12), (-pi/2)-(3*pi/12), (-pi/2)-(5*pi/12), (pi/2)+(5*pi/12), (pi/2)+(3*pi/12), (pi/2)+(pi/12)]
ready_to_pick = [0, -0.68, 1.13, -(pi/2)-0.45, 3*(pi/2), pi/2, 0]
ready_to_pick_1 = [0, -1.1, 1.1, -(pi/2), 3*(pi/2), 0, 0]
ready_to_pick_2 = [0.0, 0.18, 0.0, -0.18, pi, 0.0, 0.0]

def ready(moveit_group):
    joint_goal = moveit_group.get_current_joint_values()
    joint_goal[0] = 0.25
    joint_goal[1] = -pi/2
    moveit_group.go(joint_goal, wait=True)
    #moveit_group.go(ready_to_pick, wait=True)
    moveit_group.go(ready_to_pick_2, wait=True)

def pick(moveit_group ,size_in_cm):
    joint_goal = moveit_group.get_current_joint_values()
    joint_goal[6] = 0.8-(0.066*size_in_cm) #0.4
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

class controller:

    def __init__(self, object_class, x_coordinates, y_coordinates, z_coordinates, belt_velocity):

        self.set_object_class(object_class)
        self.set_object_coordinates([x_coordinates, y_coordinates, z_coordinates])
        self.set_belt_velocity(belt_velocity)
        self._t0 = time.time()
        self.time_difference = 0
        self.set_classes(["Rote Zylinder", "Gruene Zylinder", "Blaue Zylinder", "Rote Wuerfel", "Gruene Wuerfel", "Blaue Wuerfel"])

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_and_place', anonymous=True)
        robot = moveit_commander.RobotCommander()

        arm_group = moveit_commander.MoveGroupCommander("hand_arm")

        self.set_moveit_group(arm_group)


        self.infos_sub = rospy.Subscriber("class", String, self.callback)
        print ("<< Subscribed to topic class")
        ready(self._moveit_group)


    def set_object_class(self, object_class):
        self._object_class = object_class
    
    def set_object_coordinates(self, object_coordinates):
        self._object_coordinates = object_coordinates

    def set_belt_velocity(self, belt_velocity):
        self._belt_velocity = belt_velocity

    def set_classes(self, classes):
        self._classes = classes

    def set_moveit_group(self, moveit_group):
        self._moveit_group = moveit_group
    
    def callback(self, data):

        infos = data.data.split('+')
        self.set_object_class(int(infos[0]))
        self.set_object_coordinates([float(infos[1]), float(infos[2]), float(infos[3])])
        self.set_belt_velocity(float(infos[4]))
        self._t0 = float(infos[5])

        print("Klasse des Objekts: >> " + str(self._object_class))
        print("Koordinaten des Objekts: >> " + str(self._object_coordinates))
        print("Geschwindigkeit des Fliessbandes: >> " + str(self._belt_velocity))
        self.time_difference = time.time() - self._t0
        print("Das Objekt wird vor " + str(self.time_difference) + " Sekunden gefunden")
        
        tic = time.time()
        sleep_time = ((3.0/self._belt_velocity) - self.time_difference) 
        time.sleep(sleep_time)
        pick(self._moveit_group, 5.0)
        
        place(self._moveit_group, self._object_class)
        ready(self._moveit_group)
        toc = time.time()

        print("it took " + str(toc - tic))


def main(args):

    ic = controller(0, -3.0, 1.5, 0.65, 0.5)
    #rospy.init_node('pick_and_place', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)