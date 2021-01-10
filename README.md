# MEC319
Gruppe 11 - Pick &amp; Place UR3 Mit ROS


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
    
Optional after launching the previous file successfully, you can change start move the conveyor belt by changing the velocity of the joint "movable" of the conveyor belt (wait a little bit until the belt gain speed)

you can now launch objects using the following command:
  
    $ roslaunch my_robot objects.launch

To be able to use the classifier node "locate_classify_objects.py" you need now to change the path for the "model10.h5" file to the right path on your local machine. You need then to make it an executable using the following command:

    $ chmod +x src/MEC319/my_robot/src/scripts/locate_classify_objects.py

Now run the following commands in a different terminal to start the object classifier:
  
    $ rosrun my_robot locate_classify_objects.py

You can start the robot now using:
 
    $ roslaunch fh_desc ur10_with_gripper.launch

You can start the ur10_moveit_planning_execution.launch now:

    $ roslaunch test_moveit_config ur3_moveit_planning_execution.launch

To start moving the robot you can test the script "ur10_pick.py":

    $rosrun picking ur10_pick.py

Have fun

