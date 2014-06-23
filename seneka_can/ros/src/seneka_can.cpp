/*!
*****************************************************************
* seneka_can.cpp
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

#include <seneka_can/SenekaCan.h>

/**************************************************/
/**************************************************/
/**************************************************/

// constructor
SenekaCan::SenekaCan() {

  nh = ros::NodeHandle("~");

  // initialize default parameters;
  transmission_srv_ident  = "/can_send";
  receiving_srv_ident     = "/can_read";

  transmission_srv  = nh.advertiseService(transmission_srv_ident, &SenekaCan::SendMsg, this);
  receiving_srv     = nh.advertiseService(receiving_srv_ident, &SenekaCan::ReadMsg, this);

  /*****************************************/
  /*************** SocketCAN ***************/
  /*****************************************/

  // create the socket;
  skt = socket(PF_CAN, SOCK_RAW, CAN_RAW);
 
  // locate the interface you wish to use;
  strcpy(ifr.ifr_name, "can0");
  // ifr.ifr_ifindex gets filled with that device's index;
  ioctl(skt, SIOCGIFINDEX, &ifr);
 
  // select that CAN interface, and bind the socket to it;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind( skt, (struct sockaddr*)&addr, sizeof(addr) );

  /*****************************************/
  /*****************************************/
  /*****************************************/

}

// destructor
SenekaCan::~SenekaCan(){}

/**************************************************/
/**************************************************/
/**************************************************/

// function to send a message to the CAN bus;
bool SenekaCan::SendMsg(seneka_srv::canSendMsg::Request  &req,
                        seneka_srv::canSendMsg::Response &res) {

  struct can_frame frame;

  frame.can_id  = req.can_id;
  frame.can_dlc = req.can_dlc;

  for (int i = 0; i < 8; i++) {

    frame.data[i] = req.data[i];

  }

  res.bytes_sent  = write(skt, &frame, sizeof(frame));

  if (res.bytes_sent != 0) {

    return true;

  }

}

/**************************************************/
/**************************************************/
/**************************************************/

// function to read a message from the CAN bus;
bool SenekaCan::ReadMsg(seneka_srv::canReadMsg::Request  &req,
                        seneka_srv::canReadMsg::Response &res) {

  struct can_frame frame;

  res.bytes_read = read(skt, &frame, sizeof(frame));

  if (res.bytes_read != 0) {

    res.can_id  = frame.can_id;
    res.can_dlc = frame.can_dlc;

    for (int i = 0; i < 8; i++) {

      res.data[i] = frame.data[i];

    }

    return true;

  }

  else {

    return false;

  }

}

/*********************************************/
/*************** main function ***************/
/*********************************************/

int main(int argc, char** argv) {

  // ROS initialization; apply "seneka_can" as node name;
  ros::init(argc, argv, "seneka_can");

  SenekaCan cSenekaCan;

  while(cSenekaCan.nh.ok()) {

    ros::spin();

  }

  return 0;

}

/********************************************/
/********************************************/
/********************************************/