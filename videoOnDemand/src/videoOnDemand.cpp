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

namespace enc = sensor_msgs::image_encodings;

// global attributes
frameManager* fManager;

void processFrameCallback(const sensor_msgs::Image& img)
{
	fManager->processFrame(img);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "videoOnDemandNode");
	// attributes
	fManager = new frameManager();
	const float SNAPSHOT_INTERVAL = 25;
	ros::NodeHandle n;

	ROS_INFO("subscribing for thermal_image_view ...");
	ros::Subscriber sub = n.subscribe("/optris/thermal_image_view", 2, processFrameCallback);

	// frequency in Hz
	ros::Rate loop_rate(SNAPSHOT_INTERVAL);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
