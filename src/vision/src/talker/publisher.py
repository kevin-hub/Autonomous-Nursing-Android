#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from vision.msg import DetectedClass

def talker():    

    pub = rospy.Publisher('custom_chatter', DetectedClass)
    rospy.init_node('talker', anonymous=True)

    msg = DetectedClass()
    
    msg.object_index = 0
    
    msg.x = 5
    msg.y = 5
    msg.z = 5

    msg.height = 3
    msg.width = 3

    # Paired with rate.sleep() for looping at desired rate
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "Hello world %s" % rospy.get_time()
        
        # Messages get printed to screen
        # Messages written to NOde's log file
        # Messages get written to rosout
        rospy.loginfo(msg)
        
        # Publishes string to chatter topic
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass