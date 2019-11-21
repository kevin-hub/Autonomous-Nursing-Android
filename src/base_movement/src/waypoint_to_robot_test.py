#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf 

def publish_waypoint(x,y,theta):
    rospy.init_node('wp_publisher')


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

    pub = rospy.Publisher('/waypoint',PoseStamped,queue_size=10)

    #while not rospy.is_shutdown():
    rospy.loginfo(pose)
    rospy.Rate(5).sleep()
    pub.publish(pose)


if __name__=='__main__':
    try:
        x = -5
        y = 2
        theta = 1.5
        publish_waypoint(x,y,theta)
        
    except rospy.ROSInterruptException:
        pass





