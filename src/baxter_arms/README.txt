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