/****************************************************************
*
* Copyright (c) 2014
*
* Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Project name: SeNeKa
* ROS metapackage: seneka_sensor_node
* ROS package: seneka_laser_scan
* GitHub repository: https://github.com/ipa320/seneka_sensor_node
* 
* Package description: The seneka_laser_scan package is part of the
* seneka_sensor_node metapackage, developed for the SeNeKa project
* at Fraunhofer IPA. It implements a ROS service node which triggers
* a (Hokuyo) laser scanner, rotated by an electric actuator, to take
* a 360 degrees 3D scan of its environment. This package might work
* with other hardware and can be used for other purposes, however the
* development has been specifically for this project and the deployed
* sensors.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Supervisor: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
* Author: Thorsten Andreas Kannacher, E-Mail:Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
*
* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Date of creation: January 2014
*
* Modified by: 
* Date of modification: 
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/

// ros includes
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <laser_assembler/AssembleScans.h>
// #include <...> // actuator driver

// function prototypes
bool scanEnvironment(std_srvs::Empty::Request& cRequest, std_srvs::Empty::Response& cResponse);	// callback function
bool driveRequest(float fTarget_position);
bool getCloud(ros::Time cTime_flag_begin, ros::Time cTime_flag_end);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("scan_environment", scanEnvironment);

  // waiting for all dependend services to be ready

  // service for building and merging point clouds
  ros::service::waitForService("assemble_scans");

  // node continues as soon as all dependend services are ready
  ROS_INFO("Ready to scan environment.\n");

  ros::spin();
  return 0;
}

bool scanEnvironment(std_srvs::Empty::Request& cRequest, std_srvs::Empty::Response& cResponse)
{
  ROS_INFO("Scanning environment...\n");

  const float fStart_position	= 0.0;
  const float fEnd_position	= 360.0;

  ros::Time cTime_flag_begin, cTime_flag_end;

  // assuming motor driver does not return true before reaching target position
  if (driveRequest(fStart_position))
  {
    ros::Time cTime_flag_begin = ros::Time::now();
    ROS_INFO("Reached starting position.\n");
  }
  // assuming motor driver returns false in case of an error or a timeout
  else
  {
    ROS_ERROR("Reaching starting position failed.\n");
    return false;
  }

  // assuming motor driver does not return true before reaching target position
  if (driveRequest(fEnd_position))
  {
    ros::Time cTime_flag_end = ros::Time::now();
    ROS_INFO("Reached end position.\n");
  }
  // assuming motor driver returns false in case of an error or a timeout
  else
  {
    ROS_ERROR("Reaching end position failed.\n");
    return false;
  }

  if (getCloud(cTime_flag_begin, cTime_flag_end))
  {
    ROS_INFO("Building point cloud succeeded.\n");
  }
  else
  {
    ROS_ERROR("Failed to build point cloud.\n");
    return false;
  }

  ROS_INFO("Scan succeeded.\n");
  return true;
}

bool driveRequest(float fTarget_position)
{
  /*
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<actuator_package::actuator_service>("service_name");

  actuator_package::actuator_service srv;
  srv.request.SOMETHING = target_position

  //Assuming motor driver does not return true before reaching target position
  if (client.call(srv))
    return true;

  else
    return false;
  */
  return true;
}

bool getCloud(ros::Time cStarting_time, ros::Time cEnd_time)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
 
  laser_assembler::AssembleScans srv;
  srv.request.begin = cStarting_time;
  srv.request.end   = cEnd_time;

  if (client.call(srv))
    return true;
  else
    return false;
}

/* This function will only be necessary if motor driver returns true before reaching target position
 *
bool getPosition(void)
{
  ros::ServiceClient client = nh4.serviceClient<MOTOR_DRIVER_PACKAGE::MOTOR_DRIVER_SERVICE>("MOTOR_DRIVER_SERVICE");

  MOTOR_DRIVER_PACKAGE::MOTOR_DRIVER_SERVICE srv;
  
  ...

  if (client.call(srv))
    return true;
  else
    return false;
}
 *
 */

