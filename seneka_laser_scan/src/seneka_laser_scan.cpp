#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <laser_assembler/AssembleScans.h>

bool response(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool scan(void);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan");

  ros::NodeHandle nh1;
  ros::ServiceServer service = nh1.advertiseService("laser_scan", response);
  ROS_INFO("Ready to scan environment.\n");

  ros::spin();

  return 0;
}

bool response(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  scan();

  return true;
}


bool scan(void)
{
  ROS_INFO("Scanning environment...\n");

  /*
  ros::NodeHandle nh2;
  ros::service::waitForService("assemble_scans");
  ros::ServiceClient client = nh2.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
 
  laser_assembler::AssembleScans srv;
  srv.request.begin = ros::Time(0,0);
  srv.request.end = ros::Time::now();

  if (client.call(srv))
  {
    ROS_INFO("Service call succeeded\n");
  }
  else
  {
    ROS_ERROR("Service call failed\n");
  }
  */

  return true;
}

