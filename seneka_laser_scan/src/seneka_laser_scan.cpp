#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan");
  ros::NodeHandle nh;

  ros::service::waitForService("assemble_scans");
  ros::ServiceClient client = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
  laser_assembler::AssembleScans srv;

  srv.request.begin = ros::Time(0,0);
  srv.request.end   = ros::Time::now();

  if (client.call(srv))
  {
    ROS_INFO("Got cloud with %u points\n", srv.response.cloud.points.size());
  }
  else
  {
    ROS_ERROR("Service call failed\n");
  }

  return 0;
}
