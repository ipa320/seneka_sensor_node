/*!
*****************************************************************
* seneka_control_interface.cpp
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_control_interface
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
* 
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
* Modified xx/20xx: 
*
* Description:
* The seneka_control_interface package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
* This package might work with other hardware and can be used for other purposes, 
* however the development has been specifically for this project and the deployed sensors.
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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
#include <seneka_socketcan/SocketCAN.h>
#include <seneka_laser_scan/SenekaLaserScan.h>
#include <seneka_leg/SenekaLeg.h>

#include <sensor_msgs/JointState.h>

#include <iostream>

using namespace std;

/*********************************************/
/*************** main function ***************/
/*********************************************/

class ControlNode {
	ros::NodeHandle nh_;
	ros::Publisher pub_joints_;
	sensor_msgs::JointState joint_state_;
public:
	
	ControlNode() {
		pub_joints_  = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
	}
	
	void add(SenekaGeneralCANDevice &dev, const std::string &name) {
		dev.setUpdateCallback((int)joint_state_.position.size(), boost::bind(&ControlNode::update_joint, this, _1, _2) );
		
		joint_state_.position.push_back(0.);
		joint_state_.name.push_back(name);
	}
	
	void update_joint(const int id, const double val) {
		if(id<0 || id>=(int)joint_state_.position.size())
			return;
			
		joint_state_.position[id] = val;
		pub_joints_.publish(joint_state_);
	}
};

int main(int argc, char *argv[]) {

  // ROS initialization; apply "seneka_control_interface" as node name;
  ros::init(argc, argv, "seneka_control_interface");
  ros::NodeHandle nh;

  SenekaLaserScan laser_scan;
  SenekaTurret    turret;
  SenekaTilt      tilt;
  SenekaLeg       leg1(1);
  SenekaLeg       leg2(2);
  SenekaLeg       leg3(3);
  
  ControlNode node;
  
  node.add(turret, "turret");
  node.add(tilt, "tilt");
  node.add(leg1, "leg1");
  node.add(leg2, "leg2");
  node.add(leg3, "leg3");
  
  ros::spin();

  return 0;

}

/********************************************/
/********************************************/
/********************************************/
