/*!
*****************************************************************
* seneka_dgps.cpp
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_dgps
*
* Author: Ciby Mathew, E-Mail: Ciby.Mathew@ipa.fhg.de
* 
* Supervised by: Christophe Maufroy
*
* Date of creation: Jan 2013
* Modified 03/2014: David Bertram, E-Mail: davidbertram@gmx.de
* Modified 04/2014: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
*
* Description:
*
* To-Do:
*
* --> see seneka_dgps_node.cpp --> To-Do
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

#include <seneka_dgps/seneka_dgps.h>

/***************************************************************/
/*************** SenekaDgps class implementation ***************/
/***************************************************************/

// constructor
SenekaDgps::SenekaDgps() {

    // set default parameters
    position_topic      = "/position";
    diagnostics_topic   = "/diagnostics";
    serial_port         = "/dev/ttyUSB0";
    serial_baudrate     = 38400;            // [] = Bd
    publishrate         = 1;                // [] = Hz

    nh = ros::NodeHandle("~");

    // get parameters from the ROS parameter server
    // if there is no matching parameter on the server, the default value is used

    // serial port
    if (!nh.hasParam("port")) {

        ROS_WARN("Using default parameter for port: %s", getSerialPort().c_str());
        publishStatus("Using default parameter for port.", OK);
    }

        nh.param("port", port, getSerialPort());

    // baud rate for serial connection
    if (!nh.hasParam("baud")) {

        ROS_WARN("Using default parameter for baud rate: %i Bd", getSerialBaudRate());
        publishStatus("Using default parameter for baud rate.", OK);
    }   

        nh.param("baud", baud, getSerialBaudRate());

    // ROS publish rate
    if (!nh.hasParam("rate")) {

        ROS_WARN("Using default parameter for publish rate: %i Hz", getPublishRate());
        publishStatus("Using default parameter for publish rate.", OK);
    }

        nh.param("rate", rate, getPublishRate());

    syncedROSTime = ros::Time::now();

    topicPub_position       = nh.advertise<sensor_msgs::NavSatFix>              (position_topic.c_str(), 1);
    topicPub_Diagnostic_    = nh.advertise<diagnostic_msgs::DiagnosticArray>    (diagnostics_topic.c_str(), 1);
}

// destructor
SenekaDgps::~SenekaDgps(){}

// publishing functions

// takes position data from DGPS device and publishes it to given ROS topic
void SenekaDgps::publishPosition(Dgps::GpsData gps) {

    sensor_msgs::NavSatFix positions;

    positions.latitude          = gps.latitude_value;
    positions.longitude         = gps.longitude_value;
    positions.altitude          = gps.altitude_value;
    positions.header.frame_id   = "dgps_frame_id";
    positions.header.stamp      = ros::Time::now();

    topicPub_position.publish(positions);
}

// takes diagnostic statements and publishes them to given topic
void SenekaDgps::publishStatus(std::string status_str, int level) {
    
    diagnostic_msgs::DiagnosticArray diagnostics;

    diagnostics.status.resize(1);
    diagnostics.status[0].level     = level;
    diagnostics.status[0].name      = nh.getNamespace();
    diagnostics.status[0].message   = status_str;
    diagnostics.header.frame_id     = "dgps_frame_id";
    diagnostics.header.stamp        = ros::Time::now();

    topicPub_Diagnostic_.publish(diagnostics);
}

void SenekaDgps::extractDiagnostics(Dgps &obj) {

    Dgps::DiagnosticStatement statement;

    for (std::vector<Dgps::DiagnosticStatement>::iterator it = obj.diagnostic_array.begin(); it != obj.diagnostic_array.end(); it++) {

        statement = *it;

        publishStatus(statement.diagnostic_message, statement.diagnostic_flag);
    }

    obj.diagnostic_array.clear();
}

/***************************************************************/
/***************************************************************/
/***************************************************************/