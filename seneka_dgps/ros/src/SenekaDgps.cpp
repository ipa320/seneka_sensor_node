/*!
*****************************************************************
* SenekaDgps.cpp
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

#include <seneka_dgps/SenekaDgps.h>

/***************************************************************/
/*************** SenekaDgps class implementation ***************/
/***************************************************************/

// constructor
SenekaDgps::SenekaDgps() {

    // initialization of default parameters
    position_topic      = "/position";
    diagnostics_topic   = "/diagnostics";
    serial_port         = "/dev/ttyUSB0";
    serial_baudrate     = 38400;            // [] = Bd
    publishrate         = 1;                // [] = Hz

    nh = ros::NodeHandle("~");

    // gather all required parameters from the ROS parameter server
    // if there is no matching parameter on the server, use default value

    // gather serial port for connection establishment
    if (!nh.hasParam("port")) {

        ROS_WARN("Using default parameter for port: %s", getSerialPort().c_str());
        publishDiagnostics("Using default parameter for port.", INFO);
    }

        nh.param("port", port, getSerialPort());

    // gather baud rate for serial connection
    if (!nh.hasParam("baud")) {

        ROS_WARN("Using default parameter for baud rate: %i Bd", getSerialBaudRate());
        publishDiagnostics("Using default parameter for baud rate.", INFO);
    }   

        nh.param("baud", baud, getSerialBaudRate());

    // gather ROS publish rate
    if (!nh.hasParam("rate")) {

        ROS_WARN("Using default parameter for publish rate: %i Hz", getPublishRate());
        publishDiagnostics("Using default parameter for publish rate.", INFO);
    }

        nh.param("rate", rate, getPublishRate());

    syncedROSTime = ros::Time::now();

    position_publisher      = nh.advertise<sensor_msgs::NavSatFix>              (position_topic.c_str(), 1);
    diagnostics_publisher   = nh.advertise<diagnostic_msgs::DiagnosticArray>    (diagnostics_topic.c_str(), 1);
}

// destructor
SenekaDgps::~SenekaDgps(){}

// gathers all console output which occured due to execution of functions on Dgps instance
// by extracting the diagnostic statements from Dgps diagnostic_array;
// publishes extracted diagnostic statements on given topic by transmitting them to publishDiagnostics()-function;
// ROS diagnostics handling:
// see ROS diagnostics (http://wiki.ros.org/diagnostics and http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html);
void SenekaDgps::extractDiagnostics(Dgps &obj) {

    Dgps::DiagnosticStatement statement;

    for (std::vector<Dgps::DiagnosticStatement>::iterator it = obj.getDiagnosticArray().begin(); it != obj.getDiagnosticArray().end(); it++) {

        statement = *it;

        switch (statement.diagnostic_flag) {

            case Dgps::DEBUG:

                publishDiagnostics   (statement.diagnostic_message, DEBUG);
                break;

            case Dgps::INFO:

                publishDiagnostics   (statement.diagnostic_message, INFO);
                break;

            case Dgps::WARNING:

                publishDiagnostics   (statement.diagnostic_message, WARN);
                break;

            case Dgps::ERROR:

                publishDiagnostics   (statement.diagnostic_message, ERROR);
                break;

            default:

                message << "No matching ROS verbosity level for DGPS device diagnostics message: " << statement.diagnostic_message;
                publishDiagnostics(message.str(), WARN);
                break;
        }
    }

    obj.clearDiagnosticArray();
}

// takes diagnostic statements and publishes them to given topic
// enumerated DiagnosticFlag type for diagnostic statements
// see ROS verbosity levels (http://wiki.ros.org/Verbosity Levels)
// see ROS diagnostics (http://wiki.ros.org/diagnostics and http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html)
void SenekaDgps::publishDiagnostics(std::string message, DiagnosticFlag flag) {

    // allocates another element in diagnostics array
    diagnostics.status.resize(1);
    
    // diagnostics.status[0].level assignment below
    diagnostics.header.frame_id     = "dgps_frame_id";
    diagnostics.header.stamp        = ros::Time::now();
    diagnostics.status[0].name      = nh.getNamespace();
    diagnostics.status[0].message   = message;

    switch(flag) {

        case DEBUG:

            ROS_DEBUG   ("%s", message.c_str());
            break;

        case INFO:

            ROS_INFO    ("%s", message.c_str());
            diagnostics.status[0].level = 0;
            break;

        case WARN:

            ROS_WARN    ("%s", message.c_str());
            diagnostics.status[0].level = 1;
            break;

        case ERROR:

            ROS_ERROR   ("%s", message.c_str());
            diagnostics.status[0].level = 2;
            break;

        case FATAL:

            ROS_FATAL("");
            break;

        default:

            ROS_WARN("No matching ROS verbosity level for message: %s", message.c_str());

            break;
    }

    diagnostics_publisher.publish(diagnostics);
}

// takes position data from DGPS device and publishes it to given ROS topic
void SenekaDgps::publishPosition(Dgps::GpsData gps) {

    message << "Publishing GPS position on topic " << getPositionTopic() << "...";
    publishDiagnostics(message.str(), INFO);

    sensor_msgs::NavSatFix positions;

    positions.latitude          = gps.latitude_value;
    positions.longitude         = gps.longitude_value;
    positions.altitude          = gps.altitude_value;
    positions.header.frame_id   = "dgps_frame_id";
    positions.header.stamp      = ros::Time::now();

    position_publisher.publish(positions);
}

/***************************************************************/
/***************************************************************/
/***************************************************************/