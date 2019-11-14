#!/usr/bin/env python

# Ros_speech_recognition Node

import time
import speech_recognition as sr
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('chatter', String, queue_size=10)

# this is called from the background thread
def callback(recognizer, audio):
# received audio data, now we'll recognize it using Google Speech Recognition
	try:
	# Using Google Default API 
		incoming_string = recognizer.recognize_google(audio)
		print(incoming_string)
		pub.publish(incoming_string)
	# Implement some error detection in these cases
	except sr.UnknownValueError:
		print("Google Speech Recognition could not understand audio")
	except sr.RequestError as e:
		print("Could not request results from Sphinx Speech Recognition service; {0}".format(e))


def listener():
	# Initial the publisher, initialise node, creates a microphone and recognizer

	# Starts the Node
	print('Initialising Node')
	rospy.init_node('Listen', anonymous=True)

	# Waits to setup the callback

	while not rospy.is_shutdown():
		continue

	
	



if __name__ == '__main__':
	try:
		print('Starting Recogniser and Microphone')
		r = sr.Recognizer()
		m = sr.Microphone()
		# Adjusts the microphone for the background noise
		with m as source:
			r.adjust_for_ambient_noise(source)  # we only need to calibrate once, before we start 
		# start listening in the background (note that we don't have to do this inside a `with` statement)
		stop_listening = r.listen_in_background(m, callback)
		time.sleep(5)
		listener()
	except rospy.ROSInterruptException:
		pass


