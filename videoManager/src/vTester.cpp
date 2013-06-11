/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: SENEKA
 * \note
 *   ROS stack name: SENEKA
 * \note
 *   ROS package name: VideoManager
 *
 * \author
 *   Author: Johannes Goth (cmm-jg)
 * \author
 *   Supervised by: Christophe Maufroy (cmm)
 *
 * \date Date of creation: 23.04.2013
 *
 * \brief
 *   vTester.cpp
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include "ros/ros.h"
#include "videoManager/getVideo.h"
#include "videoManager/getSnapShots.h"
#include "videoManager/getLiveStream.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vTester");
	ros::NodeHandle n;
	videoManager::getVideo videoService;
	videoManager::getSnapShots snapShotService;
	videoManager::getLiveStream liveStreamService;
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
		client = n.serviceClient<videoManager::getVideo>("seneka/VIDEO_MANAGER/getVideo");
		ROS_INFO("Connecting to VIDEO_MANAGER ...");
		// set request to 1 -> will create a video
		videoService.request.createVideo = 1;
		// connect to server
		client.call(videoService);

		// feedback by frameManager
		switch((int)videoService.response.releasedVideo){

		case 1 :	ROS_INFO("Creating video ...");
					break;

		case -1:	ROS_WARN("Video recorder is busy !!");
					break;

		case -2:	ROS_WARN("NO complete video available !!");
					break;

		case -3:	ROS_WARN("VideoOnDemand is in state LIVE_STREAM not available! "
					"Cancel state LIVE_STREAM to switch to state ON_DEMAND!");
					break;

		default:	ROS_ERROR("Connection to Server failed or unknown command!");
					break;
		}
		break;
	case 2:
		ROS_INFO("MODE SnapShot");
		client = n.serviceClient<videoManager::getSnapShots>("seneka/VIDEO_MANAGER/getSnapShots");
		ROS_INFO("Connecting to VIDEO_MANAGER ...");
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
			ROS_ERROR("Connection to VIDEO_MANAGER failed");
		break;
	case 3:
		ROS_INFO("MODE LiveStream");
		client = n.serviceClient<videoManager::getLiveStream>("seneka/VIDEO_MANAGER/getLiveStream");
		ROS_INFO("Connecting to VIDEO_MANAGER ...");
		liveStreamService.request.start = true;
		client.call(liveStreamService);

		if(liveStreamService.response.notifier)
			ROS_INFO("STARTED creating liveStream ...");
		else if(!liveStreamService.response.notifier)
			ROS_INFO("STOPED creating liveStream ...");
		else
			ROS_ERROR("Connection to VIDEO_MANAGER failed");
		break;
	default:
		ROS_INFO("Unknown mode of vTEster ... Enter one of the following modes: \n(1) create VideoOnDemand"
				"\n(2) start/stop SnapShot and optional an interval in seconds (e.g 5)"
				"\n(3) start/stop LiveStream");
		break;
	}
}
