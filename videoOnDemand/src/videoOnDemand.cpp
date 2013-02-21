/*
 * author: Johannes Goth (cmm-jg)
 *
 */

#include "ros/ros.h"
#include <queue>
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


int main(int argc, char **argv)
{
	frameManager* fm = new frameManager();

	ros::init(argc, argv, "videoCreator");
	const float SNAPSHOT_INTERVAL = 25;
	// main access point to communications with the ROS system
	ros::NodeHandle n;

	//ROS_INFO("subscribing for thermal_image_view ...");
	//ros::Subscriber sub = n.subscribe("/optris/thermal_image_view", 2, videoCallback);

	// frequency in Hz
	ros::Rate loop_rate(SNAPSHOT_INTERVAL);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}
