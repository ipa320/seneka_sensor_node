/*!
*****************************************************************
* seneka_dgps.h
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
* --> see seneka_dgps_node.cpp To-Do
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

#ifndef _SENEKADGPS_H
#define _SENEKADGPS_H

/********************/
/***** includes *****/
/********************/

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <diagnostic_msgs/DiagnosticArray.h>

// internal includes
#include <seneka_dgps/Dgps.h>

// standart includes
#include <sstream>

/****************************/
/***** SenekaDgps class *****/
/****************************/

class SenekaDgps
{
    private:

        // default parameters
        std::string position_topic;
        std::string diagnostics_topic;
        std::string serial_port;
        int serial_baudrate;            // [] = Bd
        int publishrate;                // [] = Hz

        // parameters getting initialized by parameter server in constructor
        std::string     port;   // serial port
        int             baud;   // serial port baud rate
        int             rate;   // ROS publish rate

    public:

        // public member variables
        ros::NodeHandle nh;
        ros::Publisher  topicPub_position;
        ros::Publisher  topicPub_Diagnostic_;
        ros::Time       syncedROSTime;

        // diagnostics handling
        // see ROS diagnostics (http://wiki.ros.org/diagnostics and http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html)
        enum Status
        {
            OK          = 0,
            WARNING     = 1,
            ERROR       = 2
        };

        // constructor
        SenekaDgps();

        // destructor
        ~SenekaDgps();

        // getters
        std::string getPositionTopic    (void)  {return position_topic;}
        std::string getDiagnosticsTopic (void)  {return diagnostics_topic;}
        std::string getSerialPort       (void)  {return serial_port;}
        int         getSerialBaudRate   (void)  {return serial_baudrate;}
        int         getPublishRate      (void)  {return publishrate;}
        std::string getPort             (void)  {return port;};
        int         getBaud             (void)  {return baud;};
        int         getRate             (void)  {return rate;};

        // setters
        void        setPort (std::string port)  {this->port = port;};
        void        setBaud (int baud)          {this->baud = baud;};
        void        setRate (int rate)          {this->rate = rate;};

        // publishing functions
        void publishPosition(Dgps::gps_data gps);
        void publishStatus(std::string status_str, int level);
};

#endif //_SENEKADGPS_H