#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from vision.msg import DetectedClass

class listener():
    def __init__(self):
        rospy.Subscriber("object_params", DetectedClass, self.callback)

        rospy.set_param('/start_detecting', True)

    def callback(self, data):
        classNames = ["teddy", "remote", "book"]
        rospy.loginfo("\n Detected Object: %s \n with x, y, z: (%s, %s, %s) \n and width, height: (%s, %s)", classNames[data.object_index], data.x, data.y, data.z, data.width, data.height)

if __name__ == '__main__':
    rospy.init_node('Controller', anonymous=True)
    listen = listener()
    # spin() simply keeps python from 
    rospy.spin()
