#!/usr/bin/env python
import rospy
from std_msgs.msg import String


import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# Ordered this way to minimize wait time.
# Creates a Single Node called 'say'
# Says Hello World

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


    voice = 'voice_kal_diphone'
    volume = 2.0
    s = data.data

    print ('Saying: %s' % s)
    print ('Voice: %s' % voice)
    print ('Volme: %s' % volume)

    soundhandle.say(s, voice, volume)
    rospy.sleep(3)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Output', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    soundhandle = SoundClient()
    rospy.sleep(2)
    listener()