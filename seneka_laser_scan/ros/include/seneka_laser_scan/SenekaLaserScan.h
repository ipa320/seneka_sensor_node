/*!
*****************************************************************
* SenekaLaserScan.h
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_socketcan
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
*
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
* Modified xx/20xx: 
*
* Description:
* The seneka_laser_scan package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
* This package might work with other hardware and can be used for other purposes, 
* however the development has been specifically for this project and the deployed sensors.
*
* To-do:
* - Read/write functions covering BCM protocol;
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

#ifndef SENEKA_LASERSCAN_H_
#define SENEKA_LASERSCAN_H_

/***********************************************/
/*************** SenekaLaserScan ***************/
/***********************************************/

#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>

using namespace std;

class SenekaLaserScan {

	public:

		SenekaLaserScan();
		~SenekaLaserScan();

		ros::NodeHandle nh;
		ros::ServiceClient service_client;

		bool scan(ros::Time &time_flag_begin, ros::Time &time_flag_end);

};

SenekaLaserScan::SenekaLaserScan() {

	nh = ros::NodeHandle("~");
	service_client = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");

}

SenekaLaserScan::~SenekaLaserScan() {}

bool SenekaLaserScan::scan(ros::Time &time_flag_begin, ros::Time &time_flag_end) {

	laser_assembler::AssembleScans service;
	service.request.begin = time_flag_begin;
	service.request.end = time_flag_end;

	if (service_client.call(service))
		return true;
	else
		return false;

}

#endif // SENEKA_LASERSCAN_H_