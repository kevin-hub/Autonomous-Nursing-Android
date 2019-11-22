#!/usr/bin/env python
import sys
import roslib
from google.cloud import texttospeech
import rospy
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class TTSInterfaceClient:

    def __init__(self):
        self.client = texttospeech.TextToSpeechClient()
        self.publisher = rospy.Publisher('tts_publisher', String, queue_size=10)
        self.sc = SoundClient()
        rospy.init_node('texttospeech')
        self.text_input = "Hello, my name is Anna"

    def get_synthesis(self):
        synthesis_input = texttospeech.types.cloud_tts_pb2.SynthesisInput(text=self.text_input)

        voice = texttospeech.types.cloud_tts_pb2.VoiceSelectionParams(
            language_code='en_US',
            name='en-US-Wavenet-F',
            ssml_gender=texttospeech.enums.SsmlVoiceGender.NEUTRAL)

        audio_config = texttospeech.types.cloud_tts_pb2.AudioConfig(
            audio_encoding=texttospeech.enums.AudioEncoding.MP3)

        response = self.client.synthesize_speech(synthesis_input, voice, audio_config)

        with open('output.mp3', 'wb') as out:
            out.write(response.audio_content)
            print('Audio content written to file "output.mp3"')

        self.sc.playWave('/home/joe/catkin_ws/output.mp3')


if __name__ == '__main__':
    try:
        tts = TTSInterfaceClient()
        tts.get_synthesis()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)

