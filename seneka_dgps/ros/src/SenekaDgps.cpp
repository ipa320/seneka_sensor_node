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
* TODO:
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

// constructor;
SenekaDgps::SenekaDgps() {

    message << "Initializing...";
    publishDiagnostics(INFO);

    // initialize default parameters
    message << "Initializing default parameters...";
    publishDiagnostics(DEBUG);
    position_topic      = "/position";
    diagnostics_topic   = "/diagnostics";
    serial_port         = "/dev/ttyUSB0";
    serial_baudrate     = 38400;            // [] = Bd
    publishrate         = 1;                // [] = Hz

    nh = ros::NodeHandle("~");

    // gather all required parameters from ROS parameter server;
    // if there is no matching parameter on the server, use default value;

    message << "Gathering parameters from parameter server...";
    publishDiagnostics(DEBUG);

    if (!nh.hasParam("port")) {
        message << "Using default parameter for port: " << getSerialPort();
        publishDiagnostics(WARN);
    }

    else {

        message << "Port: " << getPort();
        publishDiagnostics(INFO);

    }
        // gather serial port identifier;
        nh.param("port", port, getSerialPort());

    if (!nh.hasParam("baud")) {

        message << "Using default parameter for baud rate: " << getSerialBaudRate() << " Bd";
        publishDiagnostics(WARN);

    }

    else {

        message << "Baud rate: " << getBaud() << "Bd";
        publishDiagnostics(INFO);

    }
        
        // gather baud rate of serial connection;
        nh.param("baud", baud, getSerialBaudRate());

    if (!nh.hasParam("rate")) {

        message << "Using default parameter for publish rate: " << getPublishRate() << " Hz";
        publishDiagnostics(WARN);

    }

    else {

        message << "Publish rate: " << getRate() << " Hz";
        publishDiagnostics(INFO);

    }
        
        // gather ROS publish rate;
        nh.param("rate", rate, getPublishRate());


    // advertise given ROS topics;
    position_publisher      = nh.advertise<seneka_msg::dgpsPosition>            (position_topic.c_str(), 1);
    diagnostics_publisher   = nh.advertise<diagnostic_msgs::DiagnosticArray>    (diagnostics_topic.c_str(), 1);

    message << "Finnished. Calling DGPS device driver for action...";
    publishDiagnostics(INFO);

}

/**************************************************/
/**************************************************/
/**************************************************/

// destructor;
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

    // allocates another element in diagnostics array;
    diagnostics.status.resize(1);
    
    // diagnostics.status[0].level  assignment below;
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

// takes position data from DGPS device and publishes it to given ROS topic;
void SenekaDgps::publishPosition(Dgps::GpsData gps_data) {

    message << "Publishing GPS position...";
    publishDiagnostics(INFO);

    position.header.frame_id           = "dgps_frame_id";
    position.header.stamp              = ros::Time::now();

    position.NavSatFix.header.frame_id = "dgps_frame_id";
    position.NavSatFix.header.stamp    = ros::Time::now();
    position.NavSatFix.latitude        = gps_data.latitude_value;
    position.NavSatFix.longitude       = gps_data.longitude_value;
    position.NavSatFix.altitude        = gps_data.altitude_value;

    position.clock_offset              = gps_data.clock_offset;
    position.frequency_offset          = gps_data.frequency_offset;
    position.pdop                      = gps_data.pdop;
    position.latitude_rate             = gps_data.latitude_rate;
    position.longitude_rate            = gps_data.longitude_rate;
    position.altitude_rate             = gps_data.altitude_rate;
    position.gps_msec_of_week          = gps_data.gps_msec_of_week;
    position.position_flags            = gps_data.position_flags;
    position.number_of_SVs             = gps_data.number_of_SVs;

    // get rid of old values;
    position.channel_numbers.clear();
    position.prn.clear();

    std::vector<char>::iterator it1 = gps_data.channel_numbers.begin();
    std::vector<char>::iterator it2 = gps_data.prn.begin();

    // gather new values according to number of satellites;
    for (int i = 0; i< gps_data.number_of_SVs; i++) {

        position.channel_numbers.push_back(*it1);
        position.prn.push_back(*it2);
        it1++;
        it2++;

    }

    position_publisher.publish(position);

}

/**************************************************/
/**************************************************/
/**************************************************/