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

    message << "Booting DGPS node...";
    publishDiagnostics(INFO);

    // initialization of default parameters
    message << "Initializing default parameters...";
    publishDiagnostics(INFO);
    position_topic      = "/position";
    diagnostics_topic   = "/diagnostics";
    serial_port         = "/dev/ttyUSB0";
    serial_baudrate     = 38400;            // [] = Bd
    publishrate         = 1;                // [] = Hz

    nh = ros::NodeHandle("~");

    // gathering all required parameters from the ROS parameter server
    // if there is no matching parameter on the server, use default value

    message << "Gathering parameters from parameter server...";
    publishDiagnostics(INFO);

    if (!nh.hasParam("port")) {
        message << "Using default parameter for port: " << getSerialPort();
        publishDiagnostics(WARN);
    }

    else {

        message << "Port: " << getPort();
        publishDiagnostics(INFO);

    }
        // gathering serial port for connection establishment
        nh.param("port", port, getSerialPort());

    if (!nh.hasParam("baud")) {

        message << "Using default parameter for baud rate: " << getSerialBaudRate() << " Bd";
        publishDiagnostics(WARN);

    }

    else {

        message << "Baud rate: " << getBaud() << "Bd";
        publishDiagnostics(INFO);

    }
        
        // gathering baud rate of serial connection
        nh.param("baud", baud, getSerialBaudRate());

    if (!nh.hasParam("rate")) {

        message << "Using default parameter for publish rate: " << getPublishRate() << " Hz";
        publishDiagnostics(WARN);

    }

    else {

        message << "Publish rate: " << getRate() << " Hz";
        publishDiagnostics(INFO);

    }
        
        // gathering ROS publish rate
        nh.param("rate", rate, getPublishRate());


    // advertising ROS topics
    position_publisher      = nh.advertise<sensor_msgs::NavSatFix>              (position_topic.c_str(), 1);
    diagnostics_publisher   = nh.advertise<diagnostic_msgs::DiagnosticArray>    (diagnostics_topic.c_str(), 1);

    message << "DGPS node is ready. Calling DGPS device driver for action...";
    publishDiagnostics(INFO);

}

/**************************************************/
/**************************************************/
/**************************************************/

// destructor
SenekaDgps::~SenekaDgps(){}

/**************************************************/
/**************************************************/
/**************************************************/

// gathers all console output which occured due to execution of functions on Dgps instance;
// done by extracting the diagnostic statements from Dgps diagnostic_array;
// publishes extracted diagnostic statements on given topic by transmitting them to publishDiagnostics()-function;
// ROS diagnostics handling:
// see ROS diagnostics (http://wiki.ros.org/diagnostics and http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html);
void SenekaDgps::extractDiagnostics(Dgps &obj) {

    message << "Extracting latest diagnostic statements from DGPS device driver...";
    publishDiagnostics(INFO);

    Dgps::DiagnosticStatement statement;

    for (std::vector<Dgps::DiagnosticStatement>::iterator it = obj.diagnostic_array.begin(); it != obj.diagnostic_array.end(); it++) {

        statement = * it;

        switch (statement.diagnostic_flag) {

            case Dgps::DEBUG:

                message << statement.diagnostic_message;
                publishDiagnostics(DEBUG);
                break;

            case Dgps::INFO:

                message << statement.diagnostic_message;
                publishDiagnostics(INFO);
                break;

            case Dgps::WARNING:

                message << statement.diagnostic_message;
                publishDiagnostics(WARN);
                break;

            case Dgps::ERROR:

                message << statement.diagnostic_message;
                publishDiagnostics(ERROR);
                break;

            default:

                message << "No matching ROS verbosity level for message: " << statement.diagnostic_message;
                publishDiagnostics(WARN);
                break;
        }

    }

    // clears all elements in Dgps::diagnostic_array;
    // needs to be done after extracting, so that old messages don't get published again;
    obj.diagnostic_array.clear();

}

/**************************************************/
/**************************************************/
/**************************************************/

// takes diagnostic statements and publishes them to given topic;
// enumerated DiagnosticFlag type for diagnostic statements;
// see ROS verbosity levels (http://wiki.ros.org/Verbosity Levels);
// see ROS diagnostics (http://wiki.ros.org/diagnostics and http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html);
void SenekaDgps::publishDiagnostics(DiagnosticFlag flag) {

    // uncomment this and use message_extended instead of message for attaching further information or formatted output;
    // std::stringstream message_extended;
    // example of formatted output: message_extended << "\n" << message.str() << "\n";

    // allocates another element in diagnostics array
    diagnostics.status.resize(1);
    
    // diagnostics.status[0].level assignment below
    diagnostics.header.frame_id     = "dgps_frame_id";
    diagnostics.header.stamp        = ros::Time::now();
    diagnostics.status[0].name      = nh.getNamespace();
    diagnostics.status[0].message   = message.str();

    switch(flag) {

        case DEBUG:

            ROS_DEBUG   ("%s", message.str().c_str());
            break;

        case INFO:

            ROS_INFO    ("%s", message.str().c_str());
            diagnostics.status[0].level = 0;
            break;

        case WARN:

            ROS_WARN    ("%s", message.str().c_str());
            diagnostics.status[0].level = 1;
            break;

        case ERROR:

            ROS_ERROR   ("%s", message.str().c_str());
            diagnostics.status[0].level = 2;
            break;

        case FATAL:

            ROS_FATAL   ("%s", message.str().c_str());
            break;

        default:

            ROS_WARN("No matching ROS verbosity level for message: %s", message.str().c_str());
            break;
    }

    diagnostics_publisher.publish(diagnostics);

    // this expression clears the stringstream instance "message" after each transmit process;
    // if it doesn't get cleared, every new diagnostic statement will get attached
    // to the existing ones within the object "message", so that it grows and grows...;
    message.str("");

}

/**************************************************/
/**************************************************/
/**************************************************/

// takes position data from DGPS device and publishes it to given ROS topic
void SenekaDgps::publishPosition(Dgps::GpsData gps_data) {

    message << "Publishing GPS position...";
    publishDiagnostics(INFO);

    positions.latitude          = gps_data.latitude_value;
    positions.longitude         = gps_data.longitude_value;
    positions.altitude          = gps_data.altitude_value;
    positions.header.frame_id   = "dgps_frame_id";
    positions.header.stamp      = ros::Time::now();

    position_publisher.publish(positions);

}

/**************************************************/
/**************************************************/
/**************************************************/