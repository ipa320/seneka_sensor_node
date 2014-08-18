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

    // used in constructor to decide whether to open only one CAN socket of a specific type or both;
    enum SocketType {

      RAW   = 0,
      BCM   = 1,
      BOTH  = 2,

    };

    // constructor;
    SocketCAN(std::string interface, SocketCAN::SocketType type);
    
    // destructor;
    ~SocketCAN();

    /*****************************************/
    /*****************************************/
    /*****************************************/

    void newIdea(void);

    void createSocket(SocketCAN::SocketType type);

    // functions covering the SocketCAN RAW protocol;

    void createRAWSocket(void);
    struct can_frame * readRAW(void);
    void writeRAW(struct can_frame *pFrame);

    /*****************************************/
    /*****************************************/
    /*****************************************/

    // functions covering the SocketCAN BCM protocol; BCM = "Broadcast Manager";

    void createBCMSocket(void);
    void readBCM(void);
    void writeBCM(void);

  private:

    std::string interface;  // CAN interface; e.g. "can0", "can1", "vcan0", ...;

    int         RAW_socket; // SocketCAN RAW socket;
    int         BCM_socket; // SocketCAN BCM socket;
    struct      sockaddr_can addr;
    struct      ifreq ifr;

};

/*****************************************/
/*****************************************/
/*****************************************/

// constructor;
SocketCAN::SocketCAN(std::string interface = "can0", SocketCAN::SocketType type = BOTH) {

  // initialize default CAN interface; then start up;
  this->interface = interface;

  if (type == RAW)
    createRAWSocket();

  else if (type == BCM)
    createBCMSocket();

  else {

    createRAWSocket();
    createBCMSocket();

  }
  

}

/*****************************************/
/*****************************************/
/*****************************************/

// destructor;
SocketCAN::~SocketCAN() {}

/*****************************************/
/*****************************************/
/*****************************************/

int * newIdea(std::string interface) {

  int * pSocket = new int;

  * pSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);


 
   /* Locate the interface you wish to use */
   struct ifreq ifr;
   strcpy(ifr.ifr_name, interface.c_str());
   ioctl(*pSocket, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled 
                                  * with that device's index */
 
   /* Select that CAN interface, and bind the socket to it. */
   struct sockaddr_can addr;
   addr.can_family = AF_CAN;
   addr.can_ifindex = ifr.ifr_ifindex;
   bind( *pSocket, (struct sockaddr*)&addr, sizeof(addr) );

   return pSocket;

}

/*****************************************/
/*****************************************/
/*****************************************/

void SocketCAN::createSocket(SocketCAN::SocketType type) {

  strcpy(ifr.ifr_name, interface.c_str());

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (type == RAW) {

    RAW_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    ioctl(RAW_socket, SIOCGIFINDEX, &ifr);
    bind(RAW_socket, (struct sockaddr*)&addr, sizeof(addr));

  }

  else if (type == BCM) {

    BCM_socket = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
    ioctl(BCM_socket, SIOCGIFINDEX, &ifr);
    connect(BCM_socket, (struct sockaddr *)&addr, sizeof(addr));

  }

  else {

    RAW_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    ioctl(RAW_socket, SIOCGIFINDEX, &ifr);
    bind(RAW_socket, (struct sockaddr*)&addr, sizeof(addr));

    BCM_socket = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
    ioctl(BCM_socket, SIOCGIFINDEX, &ifr);
    connect(BCM_socket, (struct sockaddr *)&addr, sizeof(addr));

  }

}

void SocketCAN::createRAWSocket(void) {



}

void SocketCAN::createBCMSocket(void) {



}

/*****************************************/
/*****************************************/
/*****************************************/

// function reads a single RAW SocketCAN frame from the given CAN device interface;
struct can_frame * SocketCAN::readRAW(void) {

  struct can_frame *pFrame = new can_frame;

  int bytes_read = read(RAW_socket, pFrame, sizeof(struct can_frame));

  return pFrame;

}

/*****************************************/
/*****************************************/
/*****************************************/

// function writes a single RAW SocketCAN frame to the given CAN device interface;
void SocketCAN::writeRAW(can_frame *pFrame) {

  int bytes_sent = write(RAW_socket, pFrame, sizeof(struct can_frame));

}

/*****************************************/
/*****************************************/
/*****************************************/

#endif // SOCKETCAN_H_
