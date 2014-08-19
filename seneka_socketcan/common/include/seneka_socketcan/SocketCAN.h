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
* To-do:
* - Read/write functions covering BCM protocol;
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

/*****************************************/
/*************** SocketCAN ***************/
/*****************************************/

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/bcm.h>

#include <stdio.h>

using namespace std;

namespace SocketCAN {

  /*****************************************/
  /*****************************************/
  /*****************************************/

  // RAW protocol;
  bool openRAW(int &skt, const char * can_interface) {

    // open socket;
    if((skt = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
      perror("Failed to open socket.");
      return false;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface);
    ioctl(skt, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // bind socket;
    if((bind(skt, (struct sockaddr *)&addr, sizeof(addr))) < 0) {
      perror("Failed to bind socket.");
      return false;
    }

    return true;

  }

  /*****************************************/
  /*****************************************/
  /*****************************************/

  // BCM protocol;
  bool openBCM(int &skt, const char * can_interface) {

    // open socket;
    if((skt = socket(PF_CAN, SOCK_DGRAM, CAN_BCM)) < 0) {
      perror("Failed to open socket.");
      return false;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface);
    ioctl(skt, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // connect socket;
    if((connect(skt, (struct sockaddr *)&addr, sizeof(addr))) < 0) {
      perror("Failed to connect socket.");
      return false;
    }

    return true;

  }

  /*****************************************/
  /*****************************************/
  /*****************************************/

  // reads a single CAN frame from a bound CAN_RAW socket;
  bool readRAW(const int &skt, struct can_frame &frame) {

    int nbytes;

    if((nbytes = read(skt, &frame, sizeof(struct can_frame))) < sizeof(struct can_frame)) {
      perror("Failed to read CAN frame.");
      return false;
    }

    else
      return true;

  }

  /*****************************************/
  /*****************************************/
  /*****************************************/

  // writes a single CAN frame to a bound CAN_RAW socket;
  bool writeRAW(const int &skt, const struct can_frame &frame) {

    int nbytes;

    if((nbytes = write(skt, &frame, sizeof(struct can_frame))) < sizeof(struct can_frame)) {
      perror("Failed to write CAN frame.");
      return false;
    }

    else
      return true;

  }

  /*****************************************/
  /*****************************************/
  /*****************************************/

}

/*****************************************/
/*****************************************/
/*****************************************/

#endif // SOCKETCAN_H_
