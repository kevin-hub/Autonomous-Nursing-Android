#!/usr/bin/env python
import sys, time
import os
import roslib
import rospkg
from google.cloud import texttospeech
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

rospack = rospkg.RosPack()
path = rospack.get_path('speech') + '/utils/'


class TTSInterfaceClient:

    def __init__(self):
        self.client = texttospeech.TextToSpeechClient()
        self.sc = SoundClient()
        self.flag_recieved = False

        self.voice = texttospeech.types.cloud_tts_pb2.VoiceSelectionParams(
            language_code='en_US',
            name='en-US-Wavenet-F',
            ssml_gender=texttospeech.enums.SsmlVoiceGender.NEUTRAL)

        self.audio_config = texttospeech.types.cloud_tts_pb2.AudioConfig(
            audio_encoding=texttospeech.enums.AudioEncoding.MP3)

        rospy.init_node('SpeechOutput')
        rospy.Subscriber("speech_out", String, self.speech_callback)
        rospy.Subscriber("face_ready", Bool, self.flag_callback)
        self.flag_pub = rospy.Publisher("speech_ready", Bool, queue_size=10)
        print("Awaiting new phrase input...")

    def speech_callback(self, data):
        print('PLaying Sound Now')
        synthesis_input = texttospeech.types.SynthesisInput(text=data.data)
        response = self.client.synthesize_speech(synthesis_input, self.voice, self.audio_config)

        with open(path + 'output.mp3', 'wb') as out:
            out.write(response.audio_content)
            print('Audio content written to file "output.mp3"')

        self.flag_pub.publish(True)
        while self.flag_recieved == False:
            time.sleep(0.01)

        self.sc.playWave(path + 'output.mp3')
        # self. flag_recieved = False
        time.sleep(0.2)
        os.remove(path + 'output.mp3')
        print("Awaiting new phrase input...")

    def flag_callback(self, data):
        if data.data == True:
            self.flag_recieved = True

#end of tts class

def main():
    # declare TTS
    tts = TTSInterfaceClient()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
