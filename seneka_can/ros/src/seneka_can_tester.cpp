#include <ros/ros.h>
#include <seneka_srv/canSendMsg.h>
#include <seneka_srv/canReadMsg.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "seneka_can_tester");

	ros::NodeHandle nh;

	ros::ServiceClient transmission_client = nh.serviceClient<seneka_srv::canSendMsg>("/can_send");
	ros::ServiceClient receiving_client = nh.serviceClient<seneka_srv::canReadMsg>("/can_read");

	seneka_srv::canSendMsg transmission;
	seneka_srv::canReadMsg receiving;

	transmission.request.can_id = 0x55;
	transmission.request.can_dlc = 0x01;
	transmission.request.data[0] = 0x33;

	if (transmission_client.call(transmission)) {

		ROS_INFO("Bytes sent: ", transmission.response.bytes_sent);

	}

	else {

		ROS_ERROR("Something went wrong.");

	}

	if (receiving_client.call(receiving)) {

		ROS_INFO("Bytes received: ", receiving.response.bytes_read);

	}

	else {

		ROS_ERROR("Something went wrong.");

	}

	return 0;

}