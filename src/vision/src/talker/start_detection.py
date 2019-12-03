#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from vision.msg import DetectedClass

def talker():    

    pub = rospy.Publisher('start_object_detection', Bool)
    rospy.init_node('Detection_Flag', anonymous=True)

    # Paired with rate.sleep() for looping at desired rate
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        startDetection = True
        #rospy.loginfo(startDetection)
        pub.publish(startDetection)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass