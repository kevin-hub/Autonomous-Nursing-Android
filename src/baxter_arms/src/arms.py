#!/usr/bin/env python

# Steps to run this code
# 1) roslaunch baxter_moveit_tutorial moveit_init.launch
# 2) rosrun baxter_moveit_tutorial example.py
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
import tf
import baxter_interface


class robotControl:
    #robot = None
    #group = None

    def __init__(self):
        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('moveit_baxter_example', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("left_arm")
        self.group.set_goal_orientation_tolerance(3.14/40)
        self.group.set_goal_position_tolerance(0.1)
        #self.group.set_max_velocity_scaling_factor(0.4)
        self.left_current_pose = self.group.get_current_pose(end_effector_link='left_gripper').pose
        print 'SSTSGASGJASGKJASGJKSA'
        self.left_gripper = baxter_interface.Gripper('left')
        print 'setting force'
        self.left_gripper.set_moving_force(10.0)
        #print 'opening'
        #self.left_gripper.open(block=True)
        #print 'closing'
        #self.left_gripper.close(block=True)
        print "CURRENT POSE ^^^^^^^^^^^^^^^^^^^^ "
        print(self.left_current_pose)


    def pick_up(self):
        print 'opening'
        self.left_gripper.open(block=True)
        print "moving to pick up position"
        left_target_pose = geometry_msgs.msg.Pose()
        left_target_pose.position.x = 0.440295202756
        left_target_pose.position.y = 0.0405952386528
        left_target_pose.position.z = 0.203092485056
        left_target_pose.orientation.x = 0.574412131587
        left_target_pose.orientation.y = 0.551248918125
        left_target_pose.orientation.z = -0.440796719967
        left_target_pose.orientation.w = 0.414576392259
        self.group.set_pose_target(left_target_pose)
        traj = self.group.plan()
        if not traj.joint_trajectory.points:
            print "[ERROR] No trajectory found"
        else:
            self.group.go(wait=True)
            print('hello, ive finishd')
        print 'closing'
        self.left_gripper.close(block=True)
        

    def rest(self):
        print "moving to resting/rolling position"
        left_target_pose = geometry_msgs.msg.Pose()
        left_target_pose.position.x = 0.579687639826
        left_target_pose.position.y = 0.183309444672
        left_target_pose.position.z = 0.113681797851
        left_target_pose.orientation.x = 0.140765804698
        left_target_pose.orientation.y = 0.989646583451
        left_target_pose.orientation.z = 0.0116585935438
        left_target_pose.orientation.w = 0.0254696935054
        self.group.set_pose_target(left_target_pose)
        traj = self.group.plan()
        if not traj.joint_trajectory.points:
            print "[ERROR] No trajectory found"
        else:
            self.group.go(wait=True)
            print('hello, ive finishd')

    def drop_off(self):
        print "moving to drop off position"
        left_target_pose = geometry_msgs.msg.Pose()
        left_target_pose.position.x = 0.844642646793 
        left_target_pose.position.y = 0.372739141939
        left_target_pose.position.z = 0.1630244347
        left_target_pose.orientation.x = -0.198319109883
        left_target_pose.orientation.y = 0.670020127276
        left_target_pose.orientation.z = 0.173505805549
        left_target_pose.orientation.w = 0.694001653558
        self.group.set_pose_target(left_target_pose)
        traj = self.group.plan()
        if not traj.joint_trajectory.points:
            print "[ERROR] No trajectory found"
        else:
            self.group.go(wait=True)
            print('hello, ive finishd')
    
    def exit(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def run(self):
        self.rest()
        rospy.sleep(5)
        self.pick_up()
        rospy.sleep(5)
        self.rest()
        rospy.sleep(5)
        self.pick_up()
        rospy.sleep(5)
        self.rest()
        rospy.sleep(5)
        self.drop_off()



        self.exit()
        

    def moveit_baxter_example():
        # initialize moveit_commander and rospy.
        # joint_state_topic = ['joint_states:=/robot/joint_states']
        # moveit_commander.roscpp_initialize(joint_state_topic)
        # rospy.init_node('moveit_baxter_example', anonymous=True)

        # Instantiate a RobotCommander object.  This object is
        # an interface to the robot as a whole.
        #robot = moveit_commander.RobotCommander()
       # group = moveit_commander.MoveGroupCommander("left_arm")
      # group.set_goal_orientation_tolerance(3.14/40)
        #group.set_goal_position_tolerance(0.1)

        # Planning to a Pose goal
        #left_current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
        #right_current_pose = group.get_current_pose(end_effector_link='right_gripper').pose
        #print "CURRENT POSE ^^^^^^^^^^^^^^^^^^^^ "
        #print(left_current_pose)

        left_target_pose = left_current_pose
        left_target_pose.position.x -= 0.1 #left_current_pose.position.x + 0.15 # 0.1m = 10 cm
        left_target_pose.position.y = 0.4 #left_current_pose.position.z + 0.15
        left_target_pose.position.z = 0.2
        left_target_pose = geometry_msgs.msg.Pose()
        left_target_pose.position.x = 0.2
        left_target_pose.position.y = 0.3
        quarternion =  tf.transformations.quaternion_from_euler(0,0,1)
        left_target_pose.orientation.x = 0.580043893716
        left_target_pose.orientation.y = 0.578305362022
        left_target_pose.orientation.z = -0.431779303959
        left_target_pose.orientation.w = 0.377728238673
        print(left_target_pose)

        # right_target_pose = right_current_pose
        # right_target_pose.position.x = right_current_pose.position.x - 0.15
        # right_target_pose.position.z = right_current_pose.position.z - 0.15

        group.set_pose_target(left_target_pose, end_effector_link='left_gripper')
        # group.set_pose_target(right_target_pose, end_effector_link='right_gripper')

        plan = group.plan()
        if not plan.joint_trajectory.points:
            print "[ERROR] No trajectory found"
        else:
            group.go(wait=True)

        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        arm_move = robotControl()
        arm_move.run()


    except rospy.ROSInterruptException:
        pass