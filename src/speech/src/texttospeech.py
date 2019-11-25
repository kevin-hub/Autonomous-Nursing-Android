#!/usr/bin/env python
import sys
import roslib
import rospkg
from google.cloud import texttospeech
import rospy
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

rospack = rospkg.RosPack()
path = rospack.get_path('speech') + '/utils/'


class TTSInterfaceClient:

    def __init__(self):
        self.client = texttospeech.TextToSpeechClient()
        self.sc = SoundClient()

        self.voice = texttospeech.types.cloud_tts_pb2.VoiceSelectionParams(
            language_code='en_US',
            name='en-US-Wavenet-F',
            ssml_gender=texttospeech.enums.SsmlVoiceGender.NEUTRAL)

        self.audio_config = texttospeech.types.cloud_tts_pb2.AudioConfig(
            audio_encoding=texttospeech.enums.AudioEncoding.MP3)

        rospy.init_node('Output', anonymous=True)
        rospy.Subscriber("speech_out", String, self.callback)

    def callback(self, data):
        synthesis_input = texttospeech.types.SynthesisInput(text=data.data)
        response = self.client.synthesize_speech(synthesis_input, self.voice, self.audio_config)

        with open(path + 'output.mp3', 'wb') as out:
            out.write(response.audio_content)
            print('Audio content written to file "output.mp3"')

        self.sc.playWave(path + 'output.mp3')


if __name__ == '__main__':
    tts = TTSInterfaceClient()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
