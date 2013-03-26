#include "ros/ros.h"
#include "videoOnDemand/getVideo.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vTester");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<videoOnDemand::getVideo>("getVideo");
	videoOnDemand::getVideo srv;

	srv.request.creatVideo = 1;
	client.call(srv);
	ROS_INFO("vTester ... to ... vOnDemand");
	ROS_INFO("Sum: %d", (int)srv.response.releasedVideo);


//	if (client.call(srv))
//	{
//		ROS_INFO("Response: %d", (int)srv.response.releasedVideo);
//	}
//	else
//	{
//		ROS_ERROR("Failed ...");
//		return 1;
//	}

	return 0;
}
