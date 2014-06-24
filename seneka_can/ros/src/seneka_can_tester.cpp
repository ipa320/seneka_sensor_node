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

	transmission.request.can_id = 0x123;
	transmission.request.can_dlc = 2;

	transmission.request.data.clear();
	transmission.request.data.push_back(0x11);
	transmission.request.data.push_back(0x22);

	ros::Rate rate(1); // [] = Hz;

	while(nh.ok()) {

		transmission_client.call(transmission);

		ros::spinOnce();
		rate.sleep();

	}

	return 0;

}