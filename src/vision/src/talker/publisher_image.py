#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from vision.msg import DetectedClass


def image_converter(frame):
    image_pub = rospy.Publisher("image_topic", Image)
    rospy.init_node('talker')
    bridge = CvBridge()

    try:
        if not rospy.is_shutdown():
            rospy.loginfo('Sending Image')
            image_pub.publish(bridge.imgmsg_to_cv2(frame, "passthrough"))
            

    except CvBridgeError as e:
        print(e)

import pyrealsense2 as rs
import cv2
import numpy as np
import matplotlib.pyplot as plt

pipeline = rs.pipeline()

# load config.json
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.start(config)

img_counter = 0

for _ in range(10):
    pipeline.wait_for_frames()

try:
    while True:
        frameset = pipeline.wait_for_frames()
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()

        image_converter(color_frame)

finally:
    pipeline.stop()