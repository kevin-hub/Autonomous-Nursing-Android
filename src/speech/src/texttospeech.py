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
        print('PLaying Sound Now')
        synthesis_input = texttospeech.types.SynthesisInput(text=data.data)
        response = self.client.synthesize_speech(synthesis_input, self.voice, self.audio_config)

        with open('output.mp3', 'wb') as out:
            out.write(response.audio_content)
            print('Audio content written to file "output.mp3"')

        self.sc.playWave('/home/prl1/Documents/EE4-Human-Centered-Robotics/output.mp3')

        rospy.sleep(3)


if __name__ == '__main__':
    try:
        tts = TTSInterfaceClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
