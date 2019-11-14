/*
 * test.cpp
 *
 *  Created on: 12 Nov 2019
 *      Author: joe
 */
#include "ros/ros.h"
#include "sound_play/sound_play.h"
#include "std_msgs/String.h"
#include <iostream>

int main(int argc, char **argv)
{

	int i;
	ros::init(argc, argv, "tts");
	ros::NodeHandle nh;
	sound_play::SoundClient sc (nh, "robotsound");
	ros::Rate looprate(0.5);

	while(nh.ok())
	{
	std::string phrase;

	ROS_INFO_STREAM("Enter phrase:");

	bool done = false;
	int k = 0;

	std::getline(std::cin, phrase);
	sc.say(phrase);
	looprate.sleep();

	ROS_INFO_STREAM(i);

	i++;
	//looprate.sleep();
	}

	return 0;
}
