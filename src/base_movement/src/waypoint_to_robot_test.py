#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf 

def publish_waypoint(x,y,theta):
    rospy.init_node('wp_publisher')


    pose = PoseStamped()
    pose.header.seq = 1
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = 3.52314562365
    pose.pose.position.y = 3.72042830714
    pose.pose.position.z = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
    pose.pose.orientation.x = 0#quaternion[0]
    pose.pose.orientation.y = 0#quaternion[1]
    pose.pose.orientation.z = 0.780326618642#quaternion[2]
    pose.pose.orientation.w = 0.625372183775 #quaternion[3]

    pub = rospy.Publisher('/waypoint',PoseStamped,queue_size=10)

    #while not rospy.is_shutdown():
    rospy.loginfo(pose)
    rospy.Rate(5).sleep()
    pub.publish(pose)


if __name__=='__main__':
    try:
        x = 6.29352000778#book
        y = 3.5092050533#book
        theta = 0
        publish_waypoint(x,y,theta)
        
    except rospy.ROSInterruptException:
        pass





