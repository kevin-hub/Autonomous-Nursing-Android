#!/usr/bin/env python

import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import message_filters
import rospy
from std_msgs.msg import String

import numpy as np

#from vision.msg import DetectedClass

# defining keys
key_esc = 27
key_q = ord('q')

def object_detction(color_frame, depth_frame):
    
    #images = np.hstack((color_frame, depth_frame))
    color_image_pixels = color_frame
    depth_image_pixels = depth_frame

    height, width = color_image_pixels.shape[:2]
    expected = 300
    aspect = width/height
    resized_color_image = cv2.resize(color_image_pixels, (int(round(expected * aspect)), expected))
    resized_depth_image = cv2.resize(depth_image_pixels, (int(round(expected * aspect)), expected))
    crop_start = round(expected*(aspect-1)/2)
    crop_color_img = resized_color_image[0:expected, int(crop_start):int(crop_start+expected)]
    crop_depth_img = resized_depth_image[0:expected, int(crop_start):int(crop_start+expected)]

    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(crop_depth_img, alpha = 0.03), cv2.COLORMAP_JET)

    images = np.hstack((crop_color_img, depth_colormap))
    cv2.imshow('Camera streams', images)              

    key = cv2.waitKey(1)


def callback(color_msg, depth_msg):
    bridge = CvBridge()
    color_frame = bridge.imgmsg_to_cv2(color_msg, desired_encoding="passthrough")
    depth_frame = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

    object_detction(color_frame, depth_frame)

def listener():
    rospy.init_node('listener', anonymous=True)
    
    # Subscribes to chatter topic
    # Type: std_msg.msgs.String
    # When new messages are recieved callback is invoked with message as first argument
    color_msg = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth_msg = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
    ts = message_filters.ApproximateTimeSynchronizer([color_msg, depth_msg], 10, 0.1, allow_headerless=False)
    ts.registerCallback(callback)

    print('Init done')

    # spin() simply keeps python from 
    rospy.spin()

if __name__ == '__main__':
    listener()
