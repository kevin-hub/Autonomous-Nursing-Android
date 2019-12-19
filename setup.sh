#!/bin/sh
sudo apt-get update -y # To get the latest package

# It is presumed that ROS Kinetic has been installed correctly on the device of choice

# Install the required packages for the project (System-Wide)

# Setup ROS environment

#catkin_make

BASEDIR=$(dirname "$0")
echo "Current Directory = $BASEDIR"

# echo “source $BASEDIR/devel/setup.bash” >> ~/.bashrc
# source ~/.bashrc

# Speech Dependencies
sudo apt-get install python python-all-dev python-pip build-essential swig git libpulse-dev libasound2-dev portaudio19-dev python3-pyaudio -y
#etc.

# Vision Dependencies, Python 2.7.1
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

pip install -r requirements.txt

# Arm Movement Dependencies
sudo apt-get install ros-melodic-moveit-commander

# Base Movement Dependencies
sudo apt-get install ros-kinetic-ridgeback-navigation


# Video and sound dependencies
sudo apt-ge install sox
sudo apt-get install ros-kinetic-sound-play -y
sudo apt-get install ros-kinetic-video-stream-opencv

# Installing the dependencies for sound
sudo usr/bin/rosdep install sound_play
