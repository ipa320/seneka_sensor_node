/*!
*****************************************************************
* SocketCAN.h
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_socketcan
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

#ifndef SOCKETCAN_H_
#define SOCKETCAN_H_

/****************************************/
/*************** includes ***************/
/****************************************/

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

// SocketCAN
#include <linux/can.h>
#include <linux/can/raw.h>

// standard c++ library
#include <string>

using namespace std;

/*****************************************/
/*************** SocketCAN ***************/
/*****************************************/

class SocketCAN {

  public:

    // default constructor;
    SocketCAN();
    // constructor;
    SocketCAN(std::string interface);
    
    // destructor;
    ~SocketCAN();

    // member functions;
    struct can_frame * readFrame(void);
    void writeFrame(struct can_frame *pFrame);

  private:

    int         skt;
    struct      sockaddr_can addr;
    struct      ifreq ifr;
    std::string interface;

};

/*****************************************/
/*****************************************/
/*****************************************/

// default constructor;
SocketCAN::SocketCAN() {

  // initialize default CAN interface;
  this->interface = "can0";

  skt = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  strcpy(ifr.ifr_name, this->interface.c_str());
  ioctl(skt, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  bind(skt, (struct sockaddr*)&addr, sizeof(addr));

}

/*****************************************/
/*****************************************/
/*****************************************/

// constructor;
SocketCAN::SocketCAN(std::string interface) {

  // initialize CAN interface;
  this->interface = interface;

  skt = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  strcpy(ifr.ifr_name, this->interface.c_str());
  ioctl(skt, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  bind(skt, (struct sockaddr*)&addr, sizeof(addr));

}

/*****************************************/
/*****************************************/
/*****************************************/

// destructor;
SocketCAN::~SocketCAN() {}

/*****************************************/
/*****************************************/
/*****************************************/

// function reads a CAN frame from the CAN device;
struct can_frame * SocketCAN::readFrame(void) {

  struct can_frame *pFrame = new can_frame;

  int bytes_read = read(this->skt, pFrame, sizeof(struct can_frame));

  return pFrame;

}

/*****************************************/
/*****************************************/
/*****************************************/

// function writes a CAN frame to the CAN device;
void SocketCAN::writeFrame(can_frame *pFrame) {

  int bytes_sent = write(this->skt, pFrame, sizeof(struct can_frame));

}

/*****************************************/
/*****************************************/
/*****************************************/

#endif // SOCKETCAN_H_
