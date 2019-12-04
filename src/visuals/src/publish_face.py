#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rospy
import rospkg
import sys
import threading
from random import randrange

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

rospack = rospkg.RosPack()
path = rospack.get_path('visuals') + '/media/'


class face:
    """Publish a video as ROS messages. """

    # Constructor
    def __init__(self):
        # Set up node and publishers.
        rospy.init_node("VideoPublisher")
        rospy.on_shutdown(self.shutdown)
        self.img_pub = rospy.Publisher("/visuals/image_raw", Image, queue_size=10)
        self.flag_pub = rospy.Publisher("face_ready", Bool, queue_size=10)
        self.video_file = '.mp4'
        # Flags
        self.flag_received = False
        self.pause_idle = False
        # ROS subscribers
        rospy.Subscriber("speech_ready", Bool, self.flag_callback)
        rospy.Subscriber("file_out", String, self.face_callback)

        # Start-up face
        height = 600
        width = 1024
        video = cv2.VideoCapture(path + "startup.mp4")
        fps = video.get(cv2.CAP_PROP_FPS)
        rate = rospy.Rate(fps)
        # Loop through video frames
        while not rospy.is_shutdown() and video.grab():
            tmp, img = video.retrieve()

            if not tmp:
                print "Could not grab frame."
                break

            img_out = np.empty((height, width, img.shape[2]))
            # Resize image
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
        # Start idle face

    # Callback fucntion for incoming speech output
    def face_callback(self, data):
        # Set flag to pause the idle face
        self.pause_idle = True
        # Video path and dimensions
        self.video_file = path + data.data
        height = 600
        width = 1024
        # Open video.
        video = cv2.VideoCapture(self.video_file)
        # Get frame rate.
        fps = video.get(cv2.CAP_PROP_FPS)
        rate = rospy.Rate(fps)
        # Set flag and wait for confirmation...
        self.flag_pub.publish(True)
        while not self.flag_received:
            rospy.sleep(0.01)
        # Start video
        loop_thread = threading.Thread(target=self.play_video, args=(self.video_file, height, width, video, rate))
        loop_thread.start()
        # self.play_video(self.video_file, height, width, video, rate)
        self.flag_received = False

    # Callback for receiving flag
    def flag_callback(self, data):
        if data.data:
            self.flag_received = True

    # Function for playing a single video file
    def play_video(self, video_file, height, width, video, rate):
        # Make note of current video file playing
        video_stop = self.video_file
        # Loop through video frames
        while not rospy.is_shutdown() and video.grab():
            tmp, img = video.retrieve()

            # Cancel if new video is requested
            if video_stop != self.video_file:
                break

            if not tmp:
                print "Could not grab frame."
                break

            img_out = np.empty((height, width, img.shape[2]))
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
        # Restart the idle face
        self.flag_pub.publish(False)
        self.pause_idle = False

    # Idle face function
    def idle_video(self):
        # Dimensions and path
        height = 600
        width = 1024
        video = cv2.VideoCapture(path + "idle" + str(randrange(4) + 1) + ".mp4")
        # Get frame rate.
        fps = video.get(cv2.CAP_PROP_FPS)
        rate = rospy.Rate(fps)
        # Loop through video frames.
        while not rospy.is_shutdown() and video.grab():
            # Break when new video requested
            if self.pause_idle:
                return
            # Grab frame
            tmp, img = video.retrieve()
            if not tmp:
                print "Could not grab frame."
                break

            img_out = np.empty((height, width, img.shape[2]))
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
        # Loop idle face

    def shutdown(self):
        self.pause_idle = True


# End of face class


if __name__ == '__main__':
    try:
        f = face()
        while not rospy.is_shutdown():
            f.idle_video()
            rospy.sleep(0.1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
