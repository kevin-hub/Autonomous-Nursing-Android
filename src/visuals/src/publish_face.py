#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rospy
import rospkg
import time
import sys
import threading

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

rospack = rospkg.RosPack()
path = rospack.get_path('visuals') + '/media/'

class face:
    """Publish a video as ROS messages. """

    def __init__(self):
        # Set up node.
        rospy.init_node("VideoPublisher")
        self.img_pub = rospy.Publisher("/visuals/image_raw", Image, queue_size=10)
        self.flag_pub = rospy.Publisher("face_ready", Bool, queue_size=10)
        self.video_file = '.mp4'
        self.flag_recieved = False
        rospy.Subscriber("speech_ready", Bool, self.flag_callback)
        rospy.Subscriber("file_out", String, self.face_callback)

    def face_callback(self, data):
        self.video_file = path + data.data
        height = 600
        width = 1024

        # Open video.
        video = cv2.VideoCapture(self.video_file)

        # Get frame rate.
        fps = video.get(cv2.CAP_PROP_FPS)
        rate = rospy.Rate(fps)
        self.flag_pub.publish(True)
        while self.flag_recieved == False:
            time.sleep(0.01)

        loop_thread = threading.Thread(target=self.loop_video, args=(self.video_file, height, width, video, rate))
        loop_thread.start()
        self.flag_recieved = False

    def flag_callback(self, data):
        if data.data == True:
            self.flag_recieved = True

    def loop_video(self, video_file, height, width, video, rate):
        video_stop = self.video_file
        # Loop through video frames.
        while not rospy.is_shutdown() and video.grab():
            tmp, img = video.retrieve()

            if video_stop != self.video_file:
                break

            if not tmp:
                print "Could not grab frame."
                break

            img_out = np.empty((height, width, img.shape[2]))

            # Compute input/output aspect ratios.
            aspect_ratio_in = np.float(img.shape[1]) / np.float(img.shape[0])
            aspect_ratio_out = np.float(width) / np.float(height)

            if aspect_ratio_in > aspect_ratio_out:
                # Output is narrower than input -> crop left/right.
                rsz_factor = np.float(height) / np.float(img.shape[0])
                img_rsz = cv2.resize(img, (0, 0), fx=rsz_factor, fy=rsz_factor,
                                     interpolation=cv2.INTER_AREA)

                diff = (img_rsz.shape[1] - width) / 2
                img_out = img_rsz[:, diff:-diff-1, :]
            elif aspect_ratio_in < aspect_ratio_out:
                # Output is wider than input -> crop top/bottom.
                rsz_factor = np.float(width) / np.float(img.shape[1])
                img_rsz = cv2.resize(img, (0, 0), fx=rsz_factor, fy=rsz_factor,
                                     interpolation=cv2.INTER_AREA)

                diff = (img_rsz.shape[0] - height) / 2

                img_out = img_rsz[diff:-diff-1, :, :]
            else:
                # Resize image.
                img_out = cv2.resize(img, (width, height))

            assert img_out.shape[0:2] == (height, width)

            try:
                # Publish image.
                img_msg = bridge.cv2_to_imgmsg(img_out, "bgr8")
                img_msg.header.stamp = rospy.Time.now()
                self.img_pub.publish(img_msg)
            except CvBridgeError as e:
                print(e)

            rate.sleep()


if __name__ == '__main__':
    try:
        face()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
