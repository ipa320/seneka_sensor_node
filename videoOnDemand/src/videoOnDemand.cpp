/*
 * videoOnDemand.cpp
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
#include "videoOnDemand/getVideo.h"

namespace enc = sensor_msgs::image_encodings;

// global attributes
FrameManager* fManager;

void processFrameCallback(const sensor_msgs::Image& img)
{
	fManager->processFrame(img);
}

bool getVideoCallback(videoOnDemand::getVideo::Request &req, videoOnDemand::getVideo::Response &res){

	int state = 0;
	std::cout << "Remote getVideo call ..." << std::endl;

	// start video creation
	if(req.creatVideo == 1)
		state = fManager->getVideo();
	else
		ROS_ERROR("Unknown getVideo-service command %d", (int)req.creatVideo);

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
	ros::init(argc, argv, "VODNODE");
	// attributes
	ros::NodeHandle nHandler("VODNODE");
	const float SNAPSHOT_INTERVAL = 25;
	fManager = new FrameManager(nHandler);

	ROS_INFO("advertising getVideo service ...");
	ros::ServiceServer service = nHandler.advertiseService("getVideo", getVideoCallback);
	ROS_INFO("subscribing for thermal_image_view ...");
	ros::Subscriber sub = nHandler.subscribe("/optris/thermal_image_view", 2, processFrameCallback);


	// frequency in Hz
	ros::Rate loop_rate(SNAPSHOT_INTERVAL);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
