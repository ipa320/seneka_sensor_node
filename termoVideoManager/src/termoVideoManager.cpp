/*
 * termoVideoManager.cpp
 *
 *  Created on: 21.02.2013
 *      Author: Johannes Goth (cmm-jg)
 */

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
#include "termoVideoManager/getVideo.h"
#include "termoVideoManager/getSnapShots.h"
#include "termoVideoManager/getLiveStream.h"

namespace enc = sensor_msgs::image_encodings;

// global attributes
FrameManager* fManager;

void processFrameCallback(const sensor_msgs::Image& img)
{
	fManager->processFrame(img);
}

bool getLiveStreamCallback(termoVideoManager::getLiveStream::Request &req, termoVideoManager::getLiveStream::Response &res){

	std::cout << "Remote getLiveStream call ..." << std::endl;

	// start snapShots
	if(!fManager->isLiveStreamRunning()){
		fManager->startLiveStream();
		res.notifier = true;
		return true;
	}
	else{
		fManager->stopLiveStream();
		res.notifier = false;
		return false;
	}
}

bool getSnapShotCallback(termoVideoManager::getSnapShots::Request &req, termoVideoManager::getSnapShots::Response &res){

	std::cout << "Remote getSnapShots call ..." << std::endl;

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

bool getVideoCallback(termoVideoManager::getVideo::Request &req, termoVideoManager::getVideo::Response &res){

	int state = 0;
	std::cout << "Remote getVideo call ..." << std::endl;

	// start video creation
	if(req.createVideo == 1)
		state = fManager->getVideo();
	else
		ROS_ERROR("Unknown getVideo-service command %d", (int)req.createVideo);

	if (state == 1){
		res.releasedVideo = 1;
		return true;
	}
	else if(state == -1){
		res.releasedVideo = -1;
		return true;
	}
	else if(state == -2){
		res.releasedVideo = -2;
		return true;
	}
	return false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TERMO_VIDEO_MANAGER");
	// attributes
	ros::NodeHandle nHandler("TERMO_VIDEO_MANAGER");
	fManager = new FrameManager(nHandler);

	ROS_INFO("advertising getVideo service ...");
	ros::ServiceServer videoService = nHandler.advertiseService("getVideo", getVideoCallback);
	ros::ServiceServer snapShotService = nHandler.advertiseService("getSnapShots", getSnapShotCallback);
	ros::ServiceServer liveStreamService = nHandler.advertiseService("getLiveStream", getLiveStreamCallback);
	ROS_INFO("subscribing for thermal_image ...");
	// subscribed on topic THERMAL_IMAGE
	ros::Subscriber sub = nHandler.subscribe("/optris/thermal_image", 2, processFrameCallback);

	ros::spin();
	return 0;
}
