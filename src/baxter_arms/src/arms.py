#!/usr/bin/env python

# Steps to run this code
# 1) roslaunch baxter_moveit_tutorial moveit_init.launch
# 2) rosrun baxter_moveit_tutorial example.py
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String
import tf
import baxter_interface
from vision.msg import DetectedClass

import pandas as pd
import numpy as np 


class RobotControl:
    #robot = None
    #group = None

    def __init__(self):
        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('moveit_baxter_example', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        ## Left Arm
        self.left_group = moveit_commander.MoveGroupCommander("left_arm")
        self.left_group.set_goal_orientation_tolerance(0.1)
        self.left_group.set_goal_position_tolerance(0.02)
        self.left_group.set_max_velocity_scaling_factor(0.5)
        self.left_current_pose = self.left_group.get_current_pose(end_effector_link='left_gripper').pose
        # Right Arm
        self.right_group = moveit_commander.MoveGroupCommander("right_arm")
        self.right_group.set_goal_orientation_tolerance(0.1)
        self.right_group.set_goal_position_tolerance(0.02)
        self.right_group.set_max_velocity_scaling_factor(0.5)
        self.right_current_pose = self.right_group.get_current_pose(end_effector_link='right_gripper').pose
        print 'our stuff starts below'

        # Right Gripper
        self.right_gripper = baxter_interface.Gripper('right')
        print(self.right_gripper)
        calibrated = self.right_gripper.calibrate(block=True, timeout=10)
        self.right_gripper.set_moving_force(10.0)

        self.current_coord = DetectedClass()
        self.current_coord.x = 887
        self.current_coord.y = 395
        self.current_coord.z = 0.55


        self.transformation_matrix = np.asarray(
            [[8.21E-05,	-0.0006729464,	0.5699639555, 0.3332467225],
            [-0.0005697659,	4.31E-05,	0.0971212127,	0.4828013787],
            [-3.24E-06,	-0.0002107239,	-0.965566911,	0.8521097371],
            [0,	0,	0,	1]])


    def pick_up(self, item):
        print 'opening'
        self.right_gripper.open(block=True)
        rospy.sleep(3)

        right_target_pose = geometry_msgs.msg.Pose()

        if item == 'bear':
            right_target_pose.position.x = 0.627531992154
            right_target_pose.position.y = -0.301985568095
            right_target_pose.position.z = -0.118618669885


            right_target_pose.orientation.x = -0.562112597039
            right_target_pose.orientation.y = 0.817106960376
            right_target_pose.orientation.z = -0.101709746211
            right_target_pose.orientation.w = 0.0775936278379
        else:

            right_target_pose.position.x =  0.700833406129
            right_target_pose.position.y = 0.0556458176873
            right_target_pose.position.z = -0.0404490780563

            right_target_pose.orientation.x = -0.00136236940212
            right_target_pose.orientation.y = 0.994179833903
            right_target_pose.orientation.z = 0.0818669626783
            right_target_pose.orientation.w = 0.0700171567036
        self.right_group.set_pose_target(right_target_pose)
        traj = self.right_group.plan()
        if not traj.joint_trajectory.points:
            print "[ERROR] No trajectory found"
        else:
            self.right_group.go(wait=True)
        rospy.sleep(3)
        print 'closing'
        self.right_gripper.close(block=True)
        

    def rest(self):
        print "moving to resting/rolling position"
        right_target_pose = geometry_msgs.msg.Pose()
        right_target_pose.position.x = 0.646565877128
        right_target_pose.position.y = -0.324687045393
        right_target_pose.position.z = 0.110748363608

        right_target_pose.orientation.x = -0.277069754103
        right_target_pose.orientation.y = 0.955001504041
        right_target_pose.orientation.z = -0.0595709449742
        right_target_pose.orientation.w = 0.0874973208454
        self.right_group.set_pose_target(right_target_pose)
        traj = self.right_group.plan()
        if not traj.joint_trajectory.points:
            print "[ERROR] No trajectory found"
        else:
            self.right_group.go(wait=True)
            print('Moved Right')

        left_target_pose = geometry_msgs.msg.Pose()
        left_target_pose.position.x = 0.639104101188
        left_target_pose.position.y = 0.322326413121
        left_target_pose.position.z = 0.118001132307
        left_target_pose.orientation.x = -0.00602645351038
        left_target_pose.orientation.y = 0.999365482199
        left_target_pose.orientation.z = 0.0343782452756
        left_target_pose.orientation.w = 0.00710289372949
        self.left_group.set_pose_target(left_target_pose)
        traj = self.left_group.plan()
        if not traj.joint_trajectory.points:
            print "[ERROR] No trajectory found"
        else:
            self.left_group.go(wait=True)
            print('Moved Left')

    def drop_off(self):
        print "moving to drop off position"

        right_target_pose = geometry_msgs.msg.Pose()
        right_target_pose.position.x = 0.550843128065
        right_target_pose.position.y = -0.0246437525824
        right_target_pose.position.z = -0.05538287074

        right_target_pose.orientation.x = -0.515323953228
        right_target_pose.orientation.y =  0.773865032271
        right_target_pose.orientation.z = 0.356120887928
        right_target_pose.orientation.w = 0.093552382323
        self.right_group.set_pose_target(right_target_pose)
        traj = self.right_group.plan()
        if not traj.joint_trajectory.points:
            print "[ERROR] No trajectory found"
        else:
            self.right_group.go(wait=True)
            print('Moved Right')
    
    def exit(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def coordinate_transform(self):
        # #matrix goes here
        print(1280*self.current_coord.x, 720*(1-self.current_coord.y))
        calc = np.asarray([1280*self.current_coord.x, 720*(1-self.current_coord.y), self.current_coord.z, 1])
        print('calc: ', calc)
        output = list(np.dot(self.transformation_matrix, calc))

        print('output', output)

        output[0] = output[0]/output[3]
        output[1] = output[1]/output[3]
        output[2] = output[2]/output[3]

        #TODO compute new x,y,z using generated transformation matrix
        new_x =  output[0]
        new_y =  output[1]
        new_z =  output[2]
        return (new_x,new_y,new_z)

    def print_state(self):
        print "Right CURRENT POSE "
        print(self.right_group.get_current_pose(end_effector_link='right_gripper').pose)
        
        # print "Left CURRENT POSE"
        # print(self.left_group.get_current_pose(end_effector_link='left_gripper').pose)


    def cmd_callback(self,cmd):
        if cmd.data =='pick_bear':
            print('pick_bear')
            self.rest()
            rospy.sleep(4)
            self.pick_up('bear')
            rospy.sleep(3)
            self.rest()
            
        if cmd.data =='pick_r':
            print('pick_r')
            self.rest()
            rospy.sleep(4)
            self.pick_up('book')
            rospy.sleep(3)
            self.rest()

        elif cmd.data =='give':
            print('give')
            self.drop_off()
            rospy.sleep(2)
            self.right_gripper.open(block=True)
            self.rest()

        elif cmd.data =='rest':
            self.rest()
        elif cmd.data == 'exit':
            self.exit()
        elif cmd.data == 'state':
            self.print_state()
        elif cmd.data == 'close':
            self.right_gripper.close(block=True)
            rospy.sleep(2)

        else:
            # Didn't understand
            # Say something
            pass


    def update_object_callback(self,coordinate):
        self.current_coord = coordinate
        ## Array processing here to

    def run(self):
        # rospy.Subscriber("object_params",DetectedClass,self.update_object_callback)
        # self.rest()
        # rospy.sleep(5)
        # self.rest()
        # rospy.sleep(5)
        # self.drop_off()
        # rospy.sleep(2)
        # self.right_gripper.open(block=True)
        # self.rest()
    
        #self.drop_off()
        print('Listening')
        rospy.Subscriber("arm_commander", String, self.cmd_callback)
        rospy.spin()
        # rospy.sleep(5)
        # mapped_coordinate = self.coordinate_transform()
        # print("MOVING TO THIS VALUE")
        # print(mapped_coordinate)
        # self.pick_up(mapped_coordinate)
        # rospy.sleep(5)

        


if __name__ == '__main__':
    try:
        arm_move = RobotControl()
        arm_move.run()


    except rospy.ROSInterruptException:
        pass