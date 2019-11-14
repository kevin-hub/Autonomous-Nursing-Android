
import vlc
from time import sleep

p = vlc.MediaPlayer('/home/prl1/Documents/EE4-Human-Centered-Robotics/src/speech/utils/Intro.mp3')
print('Starting Sound')
p.play()

sleep(5) # Or however long you expect it to take to open vlc
while p.is_playing():
     sleep(1)
