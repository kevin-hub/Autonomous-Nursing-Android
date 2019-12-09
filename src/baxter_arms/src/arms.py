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
        self.left_group.set_goal_position_tolerance(0.05)
        self.left_group.set_max_velocity_scaling_factor(0.5)
        self.left_current_pose = self.left_group.get_current_pose(end_effector_link='left_gripper').pose
        # Right Arm
        self.right_group = moveit_commander.MoveGroupCommander("right_arm")
        self.right_group.set_goal_orientation_tolerance(0.1)
        self.right_group.set_goal_position_tolerance(0.05)
        self.right_group.set_max_velocity_scaling_factor(0.5)
        self.right_current_pose = self.right_group.get_current_pose(end_effector_link='right_gripper').pose
        print 'our stuff starts below'

        # Right Gripper
        self.right_gripper = baxter_interface.Gripper('right')
        print(self.right_gripper)
        calibrated = self.right_gripper.calibrate(block=True, timeout=10)
        self.right_gripper.set_moving_force(10.0)

        self.current_coord = DetectedClass()


    def pick_up(self,position):
        print 'opening'
        self.right_gripper.open(block=True)
        rospy.sleep(3)
        print "moving to pick up position"
        right_target_pose = geometry_msgs.msg.Pose()

        right_target_pose.position.x = position[0]
        right_target_pose.position.y = position[1]
        right_target_pose.position.z = position[2]

        right_target_pose.orientation.x = -0.10829815208
        right_target_pose.orientation.y = 0.993432052162
        right_target_pose.orientation.z = 0.0337987305702
        right_target_pose.orientation.w = 0.0148967716675
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

    def coordinate_transform(self,coordinate):
        #matrix goes here
        #TODO compute new x,y,z using generated transformation matrix
        new_x =  0.680813877739
        new_y = -0.0274510576237
        new_z = -0.0568065638977
        return (new_x,new_y,new_z)

    def print_state(self):
        print "Right CURRENT POSE "
        print(self.right_group.get_current_pose(end_effector_link='right_gripper').pose)
        

        # print "Left CURRENT POSE"
        # print(self.left_group.get_current_pose(end_effector_link='left_gripper').pose)


    def cmd_callback(self,cmd):
        if cmd.data =='pick':
            print('pick')
            self.rest()
            rospy.sleep(4)
            mapped_coordinate = self.coordinate_transform((1,2,3))
            self.pick_up(mapped_coordinate)
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

    def run(self):
        # self.rest()
        # rospy.sleep(5)
        # mapped_coordinate = self.coordinate_transform((1,2,3))
        # self.pick_up(mapped_coordinate)
        # rospy.sleep(5)
        # self.rest()
        # rospy.sleep(5)
        # self.drop_off()
        # rospy.sleep(2)
        # self.right_gripper.open(block=True)
        # self.rest()
        # rospy.sleep(5)
        # rospy.Subscriber("object_params",DetectedClass,self.update_object_callback)
        #self.drop_off()
        print('Listening')
        rospy.Subscriber("arm_commander", String, self.cmd_callback)
        rospy.spin()

        


if __name__ == '__main__':
    try:
        arm_move = RobotControl()
        arm_move.run()


    except rospy.ROSInterruptException:
        pass