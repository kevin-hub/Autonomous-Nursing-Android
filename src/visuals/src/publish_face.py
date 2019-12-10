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
        self.img_pub = rospy.Publisher("robot/xdisplay", Image, queue_size=10)
        self.flag_pub = rospy.Publisher("face_ready", Bool, queue_size=10)
        self.video_file = '.mp4'
        # Flags
        self.flag_received = False
        self.pause_idle = False
        # ROS subscribers
        rospy.Subscriber("speech_ready", Bool, self.flag_callback)
        rospy.Subscriber("file_out", String, self.face_callback)

        # Start-up face settings
        height = 600
        width = 1024
        video_file = path + "startup.mp4"
        video = cv2.VideoCapture(video_file)
        fps = video.get(cv2.CAP_PROP_FPS)
        rate = rospy.Rate(fps)
        # Play start-up visual
        self.play_video(video_file, height, width, video, rate)
        # Start idle face
        self.idle_thread = threading.Thread(target=self.idle_video, args=())
        self.idle_thread.start()

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

<<<<<<< HEAD
    # # Idle face function
    # def idle_video(self):
    #     # Dimensions and path
    #     height = 600
    #     width = 1024
    #     video = cv2.VideoCapture(path + "idle" + str(randrange(4) + 1) + ".mp4")
    #     # Get frame rate.
    #     fps = video.get(cv2.CAP_PROP_FPS)
    #     rate = rospy.Rate(fps)
    #     # Loop through video frames.
    #     while not rospy.is_shutdown() and video.grab():
    #         # Break when new video requested
    #         if self.pause_idle:
    #             return
    #         # Grab frame
    #         tmp, img = video.retrieve()
    #         if not tmp:
    #             print "Could not grab frame."
    #             break

    #         img_out = np.empty((height, width, img.shape[2]))
    #         # Resize image.
    #         img_out = cv2.resize(img, (width, height))
    #         assert img_out.shape[0:2] == (height, width)

    #         try:
    #             # Publish image.
    #             img_msg = bridge.cv2_to_imgmsg(img_out, "bgr8")
    #             img_msg.header.stamp = rospy.Time.now()
    #             self.img_pub.publish(img_msg)
    #         except CvBridgeError as e:
    #             print(e)

    #         rate.sleep()
    #     # Loop idle face

    def shutdown(self):
        self.pause_idle = True
=======
    # Idle face function
    def idle_video(self):
        # Loop idle face
        while not rospy.is_shutdown():
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
                    break
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
            while self.pause_idle:
                rospy.sleep(0.01)
>>>>>>> 08f76965f5e24a27b445e3c94e5a5eb2432759b2


# End of face class


if __name__ == '__main__':
    try:
        f = face()
<<<<<<< HEAD
        # while not rospy.is_shutdown():
        #     f.idle_video()
        #     rospy.sleep(0.1)
=======
>>>>>>> 08f76965f5e24a27b445e3c94e5a5eb2432759b2
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
