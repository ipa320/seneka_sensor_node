/*!
*****************************************************************
* SenekaCan.h
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_can
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
* 
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
* Modified xx/20xx: 
*
* Description:
* The seneka_can package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
* By implementing a simple ROS wrapper node for SocketCAN, it offers several services to communicate with CAN devices.
* This package might work with other hardware and can be used for other purposes, 
* however the development has been specifically for this project and the deployed sensors.
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

#ifndef SENEKA_CAN_H_
#define SENEKA_CAN_H_

/****************************************/
/*************** includes ***************/
/****************************************/

#include <ros/ros.h>

#include <seneka_srv/canSendMsg.h>
#include <seneka_srv/canReadMsg.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
 
/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif
 
#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

/*****************************************/
/*************** SenekaCan ***************/
/*****************************************/

class SenekaCan {

    private:

        std::string transmission_srv_ident;
        std::string receiving_srv_ident;

        int     skt;
        struct  ifreq ifr;
        struct  sockaddr_can addr;

    public:

        // constructor;
        SenekaCan();

        // destructor;
        ~SenekaCan();

        // ROS instances (need to be public);
        ros::NodeHandle nh;

        // ROS services;
        ros::ServiceServer  transmission_srv,
                            receiving_srv;

        // service callback function
        bool SendMsg(seneka_srv::canSendMsg::Request  &req,
                     seneka_srv::canSendMsg::Response &res);

        // service callback function
        bool ReadMsg(seneka_srv::canReadMsg::Request  &req,
                     seneka_srv::canReadMsg::Response &res);

};

#endif // SENEKA_CAN_H_