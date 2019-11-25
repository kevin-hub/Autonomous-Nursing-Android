#!/usr/bin/env python

import time
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg

rospack = rospkg.RosPack()
path = rospack.get_path('visuals') + '/images/'

class image_conv:

    def __init__(self):
        self.bridge = CvBridge()
        self.image = cv2.imread(path + 'intro.jpg')
        #cv2.imshow("Image window", self.image)

        self.image_pub = rospy.Publisher("visuals/Face", Image, queue_size=10)
        rospy.Subscriber("speech_out", String, self.callback)
        rospy.init_node('face',anonymous=True)
        time.sleep(1)

    def callback(data):
        self.image = cv2.imread(path + data.data + '.jpg')
        try:
            ros_image = self.bridge.cv2_to_imgmsg(self.image, encoding = "bgr8")
        except CvBridgeError as e:
            print(e)
        self.image_pub.publish(ros_image)

if __name__ == '__main__':
    conv = image_conv()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
