BAXTER ARMS


Useful links:

SDK Install - https://sdk.rethinkrobotics.com/wiki/Workstation_Setup
Simulator Install - https://sdk.rethinkrobotics.com/wiki/Simulator_Installation 
Enable Robot - https://sdk.rethinkrobotics.com/wiki/Enable_Robot_Tool 
API reference - https://sdk.rethinkrobotics.com/wiki/API_Reference#Arm_Joints_Control_2 


Notes -

1. Use catkin_make and catkin_make install not catkin build
2. source /opt/ros/kinetic/setup.bash


Running the exmaple.py file -

./baxter.sh sim
roslaunch baxter_gazebo baxter_world.launch 

rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py

roslaunch baxter_moveit_config baxter_grippers.launch

rosrun baxter_arms arms.py


for simulator route

for all windows
>cd Documents/EE4-blah blah
>./baxter.sh sim

then in individual windows

window 1:
>roscore

window 2:
>roslaunch baxter_gazebo baxter_world.launch
*launches simulator 

window 3:
>rosrun baxter_tools tuck_arms.py -u
*arms should tuck to neutral position

>rosrun baxter_tools enable_robot.py -e
*unlocks the arms so that we can move them

>roslaunch baxter_moveit_tutorial moveit_init.launch
*launches the planning_context and move_group systems
 and starts the joint_trajectory_server


window 4: (optional)
> rviz
then change the rviz settings: fixed frame map->world; add motionPlanning object
you should then be able to move the arm under rviz.

window 5:
> roslaunch baxter_arms arm_test.launch
* runs our demo file (as an alternative you can rosrun the arms.py file) 


***for real robot skip windows 1 and 2 as these are already on baxter.
