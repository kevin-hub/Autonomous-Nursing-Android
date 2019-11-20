#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf 




def make_waypoint_pose(x,y,theta):
	
	return pose()

def publish_waypoint():
    rospy.init_node('wp_publisher')

    x = 0
    y = 0
    theta = 3.14
    pose = PoseStamped()
    pose.header.seq = 1
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "odom"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)

    #while not rospy.is_shutdown():
    rospy.loginfo(pose)
    rospy.Rate(5).sleep()
    pub.publish(pose)


if __name__=='__main__':
    try:
        publish_waypoint()
        
    except rospy.ROSInterruptException:
        pass





