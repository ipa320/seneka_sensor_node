/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Project name: SENEKA
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
#include <std_msgs/String.h>
#include <sstream>
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
    syncedROSTime = ros::Time::now();
    // implementation of topics to publish
    topicPub_wind = nh.advertise<std_msgs::String>("wind", 1);
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
  void publishwind(double* dir)
  {

    std_msgs::String msg;
    std::stringstream ss;
    ss << "angle" << dir[0]<<"speed"<<dir[1];
    msg.data = ss.str();
//    topicPub_wind.publish(msg);
    //       ROS_INFO("...publishing wind of DGps");
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
  bool bOpenwindsensor = false, bRecScan = false;
  double dir[100]= {0};
  while (!bOpenwindsensor)
  bool bOpenDgps = false, bRecScan = false;
  while (!bOpenwindsensor)
  {
    ROS_INFO("Opening wind sensor... (port:%s)",nodeClass.port.c_str());
    bOpenwindsensor = windsensor.open(nodeClass.port.c_str(), iBaudRate);
    cout<<bOpenwindsensor;
    // check, if it is the first try to open scanner
   /* if(!bOpenwindsensor)
    {
      ROS_ERROR("...DGPS not available on port %s. Will retry every second.",nodeClass.port.c_str());
      nodeClass.publishError("...DGPS not available on port");
    }*/
    sleep(1); // wait for Dgps to get ready if successfull, or wait befor retrying
  }
  //	ROS_INFO("...DGPS opened successfully on port %s",nodeClass.port.c_str());
  // main loop
  ros::Rate loop_rate(50); // Hz
  while(nodeClass.nh.ok())
  {
    // read values
    ROS_DEBUG("Reading Windsensor...");
    //		for(; ;)
    //		 publish position
    if(!bRecScan)
    {
      ROS_ERROR("...windsensor not available on port %s. Will retry every second.",nodeClass.port.c_str());
      //      nodeClass.publishError("...windsensor not available on port");
      //    }
      sleep(1); // wait for windsensor to get ready if successfull, or wait before retrying
    }
    ROS_INFO("...windsensor opened successfully on port %s",nodeClass.port.c_str());
    // main loop
    ros::Rate loop_rate(50); // Hz
    while(nodeClass.nh.ok())
    {
      // read values
      ROS_DEBUG("Reading windsensor...");
      windsensor.direction(dir);
      ROS_INFO("...publishing direction and speed of windsensor %1f, %1f",dir[0],dir[1]);
      //     publish wind
      nodeClass.publishwind(dir);
      if(!bRecScan)
      {
        ROS_DEBUG("...publishing wind of windsensor");
        nodeClass.publishwind(dir);
      }
      else
      {
        ROS_WARN("...no Values available");
      }
      //     sleep and waiting for messages, callbacks
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
  }
}
