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

#include <sstream>


// default parameters
string position_topic = "/position";
string diagnostics_topic = "/diagnostics";
string serial_port = "/dev/ttyUSB0";
int serial_baudrate = 38400;
int publishrate = 1;





string IntToString(int a) {
    //string str;
    ostringstream temp;
    temp << a;
    return temp.str();
}

//####################
//#### node class ####

class DgpsNode {
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

    DgpsNode() {
        // create a handle for this node, initialize node
        nh = ros::NodeHandle("~");
        if (!nh.hasParam("port"))ROS_WARN("Used default parameter for port (%s)",serial_port.c_str());
        nh.param("port", port, std::string(serial_port));

        if (!nh.hasParam("baud")) ROS_WARN("Used default parameter for baud (%i)",serial_baudrate);
        nh.param("baud", baud, serial_baudrate);


        if (!nh.hasParam("rate")) ROS_WARN("Used default parameter for rate (%i)",publishrate);
        nh.param("rate", rate, publishrate);

        syncedROSTime = ros::Time::now();
        //	syncedTimeReady = false;
        // implementation of topics to publish
        topicPub_position = nh.advertise<sensor_msgs::NavSatFix > (position_topic.c_str(), 1);
        topicPub_Diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticArray > (diagnostics_topic.c_str(), 1);
        // implementation of topics to subscribe
        //--

        // implementation of service servers
        //--
    }

    
    
    // Destructor

    ~DgpsNode() {
    }

    // topic callback functions
    // function will be called when a new message arrives on a topic
    //--
    // service callback functions
    // function will be called when a service is querried
    //--

    // other function declarations

    void publishPosition(double* lat) {
        sensor_msgs::NavSatFix positions;
        positions.latitude = lat[0];
        positions.longitude = lat[1];
        positions.altitude = lat[2];
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


int main(int argc, char** argv) {
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "Dgps");
    DgpsNode rosNode;
    Dgps dgps;

    


    
    
    int publishRate = rosNode.rate;
    int baudRate = rosNode.baud;
    bool dgpsSensor_opened = false, success_getPosition = false, connection_OK = false;
    double dgpsData[100] = {0};
    while (!dgpsSensor_opened) {

         
        ROS_INFO("Opening DGPS... (port: %s , baudrate: %s )", rosNode.port.c_str(), IntToString(baudRate).c_str());
        dgpsSensor_opened = dgps.open(rosNode.port.c_str(), baudRate);
        // check, if it is the first try to open scanner
        if (!dgpsSensor_opened) {
            ROS_ERROR("...DGPS not available on port %s. Will retry every second.", rosNode.port.c_str());
            rosNode.publishError("...DGPS not available on port");
        }
        sleep(1); // wait for Dgps to get ready if successfull, or wait before retrying
    }
    //	ROS_INFO("...DGPS opened successfully on port %s",nodeClass.port.c_str());
    // main loop
    ros::Rate loop_rate(publishRate); // Hz

    connection_OK = dgps.checkConnection();

    
    if (!connection_OK) {
        cout << "protocol request failed (05h): check cables, adapters, settings, ...";
    } else {

        
        while (rosNode.nh.ok()) {
            // read values
            ROS_DEBUG("Reading DGPS...");
            
            success_getPosition = dgps.getPosition(dgpsData);

            if (success_getPosition) {
                ROS_INFO("...publishing position of DGPS: %1f, %1f, %1f", dgpsData[0], dgpsData[1], dgpsData[2]);

                rosNode.publishPosition(dgpsData);

            } else {
                ROS_WARN("...no Values available");
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    return 0;
}
