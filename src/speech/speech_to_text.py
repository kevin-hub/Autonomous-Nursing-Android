#tts ROS node enables a robot to speak with a human voice by providing a Text-To-Speech service

#This will form the out-going dependency on the robot

import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# Ordered this way to minimize wait time.
# Creates a Single Node called 'say'
# Says Hello World
rospy.init_node('say', anonymous = True)
soundhandle = SoundClient()
rospy.sleep(1)

voice = 'voice_kal_diphone'
volume = 2.0

s = 'Hello World'


print ('Saying: %s' % s)
print ('Voice: %s' % voice)
print ('Volme: %s' % volume)

soundhandle.say(s, voice, volume)
rospy.sleep(1)
