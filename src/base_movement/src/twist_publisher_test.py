#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def vel_publisher():
    print(5)
    msg = Twist()
    rospy.init_node('vel_publisher')
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg.linear.x = 1
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.z = 0
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__=='__main__':
    try:
        vel_publisher()
    except rospy.ROSInterruptException:
        pass



