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
* ROS package: seneka_remote_control
* GitHub repository: https://github.com/ipa320/seneka_sensor_node
* 
* Package description: The seneka_remote_control package implements a 
* ROS node which acts as an interface to the nodes of the seneka_sensor_node
* metapackage, developed for the SeNeKa project at Fraunhofer IPA. This
* package might work with other hardware and can be used for other purposes,
* however the development has been specifically for this project and the
* deployed sensors and actuators.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Supervisor: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
* Author: Thorsten Andreas Kannacher, E-Mail:Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
*
* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Date of creation: February 2014
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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <Tower.h>

void remoteControl(const std_msgs::String msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "remote_control");
  ros::NodeHandle nh_("");

  Tower cTower;

  ros::service::waitForService("user_input");
  ROS_INFO("Control node is ready.\n");

  ros::Subscriber sub = nh_.subscribe("user_input", 1000, remoteControl);

  ros::spin();

  return 0;
}

void remoteControl(const std_msgs::String msg)
{

}