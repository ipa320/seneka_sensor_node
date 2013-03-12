/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Project name:  SENEKA 
 * ROS stack name: DGPS
 * ROS package name: seneka_dgps
 * Description:
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Author: Ciby Mathew, email:Ciby.Mathew@ipa.fhg.de
 * Supervised by: Christophe Maufroy
 *
 * Date of creation: Jan 2013
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
//--

// ROS includes


// ROS message includes
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <diagnostic_msgs/DiagnosticArray.h>

// ROS service includes
//--
// external includes
#include <seneka_dgps/Dgps.h>

//####################
//#### node class ####
class NodeClass
{
public:
	ros::NodeHandle nh;
	// topics to publish
	ros::Publisher topicPub_position;
	ros::Publisher topicPub_Diagnostic_;

	// topics to subscribe, callback is called for new messages arriving
	//--

	// service servers
	//--

	// service clients
	//--

	// global variables
	std::string port;
	int baud;
	int rate;
	//		bool inverted;
	//		std::string frame_id;
	ros::Time syncedROSTime;
	//		unsigned int syncedSICKStamp;
	//		bool syncedTimeReady;

	// Constructor
	NodeClass()
	{
		// create a handle for this node, initialize node
		nh = ros::NodeHandle("~");
		if(!nh.hasParam("port"))ROS_WARN("Used default parameter for port");
		nh.param("port", port, std::string("/dev/ttyUSB0"));

		if(!nh.hasParam("baud")) ROS_WARN("Used default parameter for baud");
		nh.param("baud", baud, 38400);

		if(!nh.hasParam("rate")) ROS_WARN("Used default parameter for rate");
		nh.param("rate", rate, 50);

		syncedROSTime = ros::Time::now();
		//	syncedTimeReady = false;
		// implementation of topics to publish
		topicPub_position = nh.advertise<sensor_msgs::NavSatFix>("position", 1);
		topicPub_Diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
		// implementation of topics to subscribe
		//--

		// implementation of service servers
		//--
	}

	// Destructor
	~NodeClass()
	{
	}

	// topic callback functions
	// function will be called when a new message arrives on a topic
	//--
	// service callback functions
	// function will be called when a service is querried
	//--

	// other function declarations
	void publishposition(double* lat)
	{
		sensor_msgs::NavSatFix positions;
		positions.latitude= lat[0];
		positions.longitude= lat[1];
		positions.altitude= lat[2];
		topicPub_position.publish(positions);
		//			 ROS_INFO("...publishing position of DGps");

		//Diagnostics
		diagnostic_msgs::DiagnosticArray diagnostics;
		diagnostics.status.resize(1);
		diagnostics.status[0].level = 0;
		diagnostics.status[0].name = nh.getNamespace();
		diagnostics.status[0].message = "Dgps running";
		topicPub_Diagnostic_.publish(diagnostics);
	}

	void publishError(std::string error_str)
	{
		diagnostic_msgs::DiagnosticArray diagnostics;
		diagnostics.status.resize(1);
		diagnostics.status[0].level = 2;
		diagnostics.status[0].name = nh.getNamespace();
		diagnostics.status[0].message = error_str;
		topicPub_Diagnostic_.publish(diagnostics);
	}
};

//
////#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "Dgps");
	NodeClass nodeClass;
	Dgps dgps;
	int lrate = nodeClass.rate;
	int iBaudRate = nodeClass.baud;
	bool bOpenDgps = false, bRecScan = false;
	double lat[100]= {0};
	while (!bOpenDgps)
	{
		ROS_INFO("Opening DGPS... (port:%s)",nodeClass.port.c_str());
		bOpenDgps = dgps.open(nodeClass.port.c_str(), iBaudRate);
		// check, if it is the first try to open scanner
		if(!bOpenDgps)
		{
			ROS_ERROR("...DGPS not available on port %s. Will retry every second.",nodeClass.port.c_str());
			nodeClass.publishError("...DGPS not available on port");
		}
		sleep(1); // wait for Dgps to get ready if successfull, or wait befor retrying
	}
	//	ROS_INFO("...DGPS opened successfully on port %s",nodeClass.port.c_str());
	// main loop
	ros::Rate loop_rate(lrate); // Hz
	while(nodeClass.nh.ok())
	{
		// read values
		ROS_DEBUG("Reading DGPS...");
		dgps.latlong(lat);
		ROS_INFO("...publishing position of DGps %1f, %1f,%1f",lat[0],lat[1],lat[2]);
		nodeClass.publishposition(lat);
		if(!bRecScan)
		{
			ROS_DEBUG("...publishing position of DGps");
			nodeClass.publishposition(lat);
		}
		else
		{
			ROS_WARN("...no Values available");
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
