#!/bin/sh
sudo apt-get update -y # To get the latest package 

# It is presumed that ROS Kinetic has been installed correctly on the device of choice

# Install the required packages for the project (System-Wide)

# Setup ROS environment

catkin_make

BASEDIR=$(dirname "$0")
echo "Current Directory = $BASEDIR"

echo “source $BASEDIR/devel/setup.bash” >> ~/.bashrc
source ~/.bashrc

# Speech Dependencies
sudo apt-get install python3 python3-all-dev python3-pip build-essential swig git libpulse-dev libasound2-dev portaudio19-dev python3-pyaudio -y
#etc.

pip3 install catkin_pkg

sudo apt-get install ros-kinetic-sound-play -y

# Installing the dependencies for sound
rosdep install sound_play