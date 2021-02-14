# Turkish - German University
# MEC319: Mechatronic Project
Gruppe 6 - Pick &amp; Place with UR3 in ROS


To be able to use this package you can clone using the following commands:

    $ mkdir -p catkin_ws
    $ cd catkin_ws
    $ mkdir -p src
    $ cd src
    $ git clone https://github.com/Baraaalsaleh/MEC319
    $ cd ..
    $ rosdep update
    $ rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
    
Making the package using catkin build:

    $ catkin build

Source the current directory:

    $ source devel/setup.bash

To test the repository:
    
    $ roslaunch my_robot mybot.launch

You should get something like this:



Optional after launching the previous file successfully, you can change start move the conveyor belt by changing the velocity of the joint "movable" of the conveyor belt (wait a little bit until the belt gain speed)

you can now launch objects using the following command:
  
    $ roslaunch my_robot objects.launch

To be able to use the classifier node "locate_classify_objects.py" you need now to change the path for the "model10.h5" file to the right path on your local machine. You need then to make it an executable using the following command:

    $ chmod +x src/MEC319/my_robot/src/scripts/classifier.py

Now run the following commands in a different terminal to start the object classifier:
  
    $ rosrun my_robot classifier.py

Note: You need to have the following python libraries installed in order for the classifiert to work; numpy, tensorflow 1.14.0 or tensorflow-gpu 1.14.0, keras 2.3.1, openCV. This can be done using the following pip commands:
    
    $ pip install numpy
    $ pip install opencv-contrib-python
    $ pip install tensorflow==1.14.0
    $ pip install keras==2.3.1
    $ pip install tensorflow-gpu=1.14.0

You can start the robot now using:
 
    $ roslaunch fh_desc ur10_with_gripper.launch

You can start the ur10_moveit_planning_execution.launch now:

    $ roslaunch test_moveit_config ur10_moveit_planning_execution.launch

You can now run the python script to move the robot after making the file ur10_pick.py executable:

    $ chmod +x src/MEC319/picking/src/ur10_pick.py

To start moving the robot you can test the script "ur10_pick.py":

    $ rosrun picking ur10_pick.py

You can also try UR3 with different gripper (note: stop ur10 from the terminal or just restart the simulation using the command in line 26. If you want to use both robots at the same time, you need to change the names of the controllers):

    $ roslaunch ur_gazebo ur3_with_gripper.launch
    
    $ roslaunch test_with_gripper_moveit_config ur3_with_gripper_moveit_planning_execution.launch

Then to be able to move the robot you need to make the node "test_hand_arm.py" executable:
    
    $ chmod +x src/MEC319/my_robot/src/scripts/pick_and_place.py

Now you can run the controller node:

    $ rosrun my_robot pick_and_place.py
    
Have fun

The used packages in this projects:

fmauch_universal_robots:	https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
smart_grasping_sandbox:		https://github.com/shadow-robot/smart_grasping_sandbox
robotiq:		        	https://github.com/cambel/ur3
