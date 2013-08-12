/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Project name: SENEKA
 * ROS stack name: windsensor
 * ROS package name: seneka_windsensor
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
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing
 *     Engineering and Automation (IPA) nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
#include <diagnostic_msgs/DiagnosticArray.h>
// ROS service includes
//--
// external includes
#include <seneka_windsensor/windsensor.h>
#include <sstream>
#include <seneka_windsensor/WindData.h>
//####################
//#### node class ####
class NodeClass
{
public:
	ros::NodeHandle nh;
	// topics to publish
	ros::Publisher topicPub_wind;
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
	ros::Time syncedROSTime;
	// Constructor
	NodeClass()
	{
		// create a handle for this node, initialize node
		nh = ros::NodeHandle("~");

		if(!nh.hasParam("port")) ROS_WARN("Used default parameter for port");
		nh.param("port", port, std::string("/dev/ttyUSB0"));

		if(!nh.hasParam("baud")) ROS_WARN("Used default parameter for baud");
		nh.param("baud", baud, 4800);

		if(!nh.hasParam("rate")) ROS_WARN("Used default parameter for rate");
		nh.param("rate", rate, 10);

		syncedROSTime = ros::Time::now();
		// implementation of topics to publish
		topicPub_wind = nh.advertise<seneka_windsensor::WindData>("wind", 1);
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
	void publishwind(float* dir)
	{
		seneka_windsensor::WindData value;
		value.speed =dir[0];
		value.direction = dir[1];
		topicPub_wind.publish(value);

		//	 ROS_INFO("...publishing wind of windsensor");

		diagnostic_msgs::DiagnosticArray diagnostics;
		diagnostics.status.resize(1);
		diagnostics.status[0].level = 0;
		diagnostics.status[0].name = nh.getNamespace();
		diagnostics.status[0].message = "Wind sensor running";
		topicPub_Diagnostic_.publish(diagnostics);
	}
	void publishError(std::string error_str) {
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
	ros::init(argc, argv, "windsensor");

	NodeClass nodeClass;
	windsensor windsensor;
	int iBaudRate = nodeClass.baud;
	int lrate = nodeClass.rate;
	bool bOpenwindsensor = false, bRecScan = false;
	float dir[100]= {0};
	while (!bOpenwindsensor)
	{
		ROS_INFO("Opening wind sensor... (port:%s)",nodeClass.port.c_str());
		bOpenwindsensor = windsensor.open(nodeClass.port.c_str(), iBaudRate);
		// check, if it is the first try to open scanner
		if(!bOpenwindsensor)
		{
			ROS_ERROR("...windsensor not available on port %s. Will retry every second.",nodeClass.port.c_str());
			nodeClass.publishError("...windsensor not available on port");
		}
		sleep(1); // wait for Windsensorto get ready if successfull, or wait befor retrying
	}
	ROS_INFO("...Windsensor opened successfully on port %s",nodeClass.port.c_str());
	// main loop
	ros::Rate loop_rate(lrate); // Hz

	while(nodeClass.nh.ok())
	{
		// read values
		ROS_DEBUG("Reading windsensor...");
		windsensor.direction(dir);
		ROS_INFO("...publishing direction and speed of windsensor %1f, %1f",dir[0],dir[1]);
		//     publish wind
		nodeClass.publishwind(dir);
		//     sleep and waiting for messages, callbacks
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

