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
 *   ROS package name: seneka_video_manager
 *
 * \author
 *   Author: Johannes Goth (cmm-jg)
 * \author
 *   Supervised by: Christophe Maufroy (cmm)
 *
 * \date Date of creation: 21.02.2013
 *
 * \brief
 *   videoManager.cpp
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
#include "sensor_msgs/Image.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include "frameManager.h"
#include "frameManager.cpp"
#include "seneka_video_manager/getVideo.h"
#include "seneka_video_manager/getSnapShots.h"
#include "seneka_video_manager/getLiveStream.h"

namespace enc = sensor_msgs::image_encodings;

// global attributes
FrameManager* fManager;

void processFrameCallback(const sensor_msgs::Image& img)
{
	fManager->processFrame(img);
}

bool getLiveStreamCallback(seneka_video_manager::getLiveStream::Request &req, seneka_video_manager::getLiveStream::Response &res){

	ROS_INFO("Remote getLiveStream call ...");

	// start snapShots
	if(!fManager->isLiveStreamRunning()){
		fManager->startLiveStream();
		res.notifier = true;
		return true;
	}
	else{
		fManager->stopLiveStream();
		res.notifier = false;
		res.notifier = false;
		return false;
	}
}

bool getSnapShotCallback(seneka_video_manager::getSnapShots::Request &req, seneka_video_manager::getSnapShots::Response &res){

	ROS_INFO("Remote getSnapShots call ...");

	// start snapShots
	if(!fManager->isSnapShotRunning()){
		fManager->startSnapshots(req.interval);
		res.notifier = true;
		return true;
	}
	else{
		fManager->stopSnapshots();
		res.notifier = false;
		return false;
	}
}

bool getVideoCallback(seneka_video_manager::getVideo::Request &req, seneka_video_manager::getVideo::Response &res){

	ROS_INFO("Remote getVideo call ...");

	// start video creation
	if(req.createVideo == 1){
		res.releasedVideo = fManager->getVideo();
		return true;
	}
	else{
		ROS_ERROR("Unknown getVideo-service command %d", (int)req.createVideo);
		return false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "VIDEO_MANAGER");
	// attributes
	ros::NodeHandle nHandler("VIDEO_MANAGER");
	fManager = new FrameManager(nHandler);

	std::string inputTopic;

	if(!nHandler.hasParam("inputTopic")){
		ROS_ERROR("No input topic in launch-file defined!!\n\n");
		return -1;
	}
	else{
		nHandler.getParam("inputTopic", inputTopic);
	}


	ROS_INFO("advertising getVideo service ...");
	ros::ServiceServer videoService = nHandler.advertiseService("getVideo", getVideoCallback);
	ros::ServiceServer snapShotService = nHandler.advertiseService("getSnapShots", getSnapShotCallback);
	ros::ServiceServer liveStreamService = nHandler.advertiseService("getLiveStream", getLiveStreamCallback);
	ROS_INFO("subscribing for thermal_image ...");
	// subscribed on topic THERMAL_IMAGE
	ros::Subscriber sub = nHandler.subscribe(inputTopic, 2, processFrameCallback);

	ros::spin();
	return 0;
}
