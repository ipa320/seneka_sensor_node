/*
 * vTester.cpp
 *
 *  Created on: 23.04.2013
 *      Author: Johannes Goth (cmm-jg)
 */

#include "ros/ros.h"
#include "videoOnDemand_on_sensor_msgs/getVideo.h"
#include "videoOnDemand_on_sensor_msgs/getSnapShots.h"
#include "videoOnDemand_on_sensor_msgs/getLiveStream.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vTester");
	ros::NodeHandle n;
	videoOnDemand_on_sensor_msgs::getVideo videoService;
	videoOnDemand_on_sensor_msgs::getSnapShots snapShotService;
	videoOnDemand_on_sensor_msgs::getLiveStream liveStreamService;
	ros::ServiceClient client;

	int mode = 0;

	if(argv[1] != NULL)
		mode = *argv[1] - '0';

	// interval in seconds
	int interval;
	// initialize  interval
	if(argv[2] != NULL)
		interval = *argv[2] - '0';
	else
		interval = 5;

	// choose mode
	switch(mode){
	case 1:
		ROS_INFO("MODE VideoOnDemand");
		client = n.serviceClient<videoOnDemand_on_sensor_msgs::getVideo>("seneka/VODNODE/getVideo");
		ROS_INFO("Connecting to VODNODE ...");
		// set request to 1 -> will create a video
		videoService.request.createVideo = 1;
		// connect to server
		client.call(videoService);

		// user feedback
		if((int)videoService.response.releasedVideo == 1)
			ROS_INFO("Creating video ...");
		else if((int)videoService.response.releasedVideo == -1)
			ROS_ERROR("Video recorder is busy !!");
		else if((int)videoService.response.releasedVideo == -2)
			ROS_ERROR("NO complete video available !!");
		else
			ROS_ERROR("Connection to Server failed");
		break;
	case 2:
		ROS_INFO("MODE SnapShot");
		client = n.serviceClient<videoOnDemand_on_sensor_msgs::getSnapShots>("seneka/VODNODE/getSnapShots");
		ROS_INFO("Connecting to VODNODE ...");
		// value 1 starts / value -1 stops service
		snapShotService.request.start = true;
		snapShotService.request.interval = interval;
		// connect to server
		client.call(snapShotService);

		if(snapShotService.response.notifier)
			ROS_INFO("STARTED creating SnapShot ...");
		else if(!snapShotService.response.notifier)
			ROS_INFO("STOPED creating SnapShot ...");
		else
			ROS_ERROR("Connection to VODNODE failed");
		break;
	case 3:
		ROS_INFO("MODE LiveStream");
		client = n.serviceClient<videoOnDemand_on_sensor_msgs::getLiveStream>("seneka/VODNODE/getLiveStream");
		ROS_INFO("Connecting to VODNODE ...");
		liveStreamService.request.start = true;
		client.call(liveStreamService);

		if(liveStreamService.response.notifier)
			ROS_INFO("STARTED creating liveStream ...");
		else if(!liveStreamService.response.notifier)
			ROS_INFO("STOPED creating liveStream ...");
		else
			ROS_ERROR("Connection to VODNODE failed");
		break;
	default:
		ROS_INFO("Unknown Mode ... (1) VideoOnDemand, (2) SnapShot, (3) LiveStream");
		break;
	}





	//	return 0;
}
