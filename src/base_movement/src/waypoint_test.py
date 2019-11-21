#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf 

def publish_waypoint(pose):
    pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)

    #while not rospy.is_shutdown():
    rospy.loginfo(pose)
    rospy.Rate(5).sleep()
    pub.publish(pose)

def start_waypoint_subscriber():
    rospy.init_node('waypoint_subscriber')
    rospy.Subscriber('/waypoint',PoseStamped,publish_waypoint)
    rospy.spin()


if __name__=='__main__':
    try:
        start_waypoint_subscriber()
        
    except rospy.ROSInterruptException:
        pass





