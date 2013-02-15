/*
 * author: Johannes Goth (cmm-jg)
 *
 */

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

void imageCallback(const sensor_msgs::Image& img)
{
  ROS_INFO("On topic");

  // sensor_imgs -> OpenCV	Share
  	cv_bridge::CvImageConstPtr cvptrS;
  	try
  	{
  		//cvptrS = cv_bridge::toCvShare(img, enc::MONO8);
  	}
  	catch (cv_bridge::Exception& e)
  	{
  		ROS_ERROR("cv_bridge exception: %s", e.what());
  		return;
  	}


  	// sensor_imgs -> OpenCV  Copy
  	cv_bridge::CvImagePtr cvptrC;
  	try
  	{
  		cvptrC = cv_bridge::toCvCopy(img, enc::MONO8);
  		std::ostringstream name;
  		name << "output";
  		cv::imwrite(name.str() , cvptrC);
  	}
  	catch (cv_bridge::Exception& e)
  	{
  		ROS_ERROR("cv_bridge exception: %s", e.what());
  		return;
  	}

  	//Scheduler(cvptrS,cvptrC);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "imageCreator");

  // main access point to communications with the ROS system
  ros::NodeHandle n;
  ROS_INFO("subscribing for thermal_image_view ...");
  ros::Subscriber sub = n.subscribe("/optris/thermal_image_view", 2, imageCallback);

  // ros::spin() will enter a loop, pumping callbacks.
  ros::spin();




  return 0;
}
