/*!
*****************************************************************
* SenekaLeg.h
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_leg_control
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
* 
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
* Modified xx/20xx: 
*
* Description:
* The seneka_leg_control package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
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

#ifndef SENEKA_LEG_H_
#define SENEKA_LEG_H_

/*******************************************/
/*************** SenekaLeg *****************/
/*******************************************/

#define LEG_TX_COMMAND_ID         0x196 // "extend/retract"-command CAN-ID; also priority of CAN message (bus arbitration);
#define LEG_TX_LEG_BYTE_NR        0     // position of data byte in CAN frame;
#define LEG_TX_CMD_TYPE_BYTE_NR   1

#define LEG_TX_INTERRUPT_ID       0x196   // "interrupt"-command CAN-ID; also priority of CAN message (bus arbitration);

#define LEG_RX_POSITION_ID        0x35E   // CAN-ID of CAN frame containing current leg position; also priority of CAN message (bus arbitration);
#define LEG_RX_POSITION_BYTE_NR   7       // position of data byte in CAN frame;
#define LEG_RX_LEG_BYTE_NR        0       // position of data byte in CAN frame;

#include <seneka_socketcan/SocketCAN.h>
#include <string>

using namespace std;

class SenekaLeg {

  public:

    SenekaLeg(unsigned char leg_nr, string can_interface);
    ~SenekaLeg();

    // available commands;
    enum Command {
      EXTEND = 0,
      RETRACT = 1,
    };

    bool executeCommand(Command command, unsigned char leg_nr);
    bool interrupt(unsigned char leg_nr);

  private:

    int socket; // socket for CAN communication;

};

SenekaLeg::SenekaLeg(unsigned char leg_nr, string can_interface = "can0") {

  // open socket for CAN communication; respect optionally given differing CAN interface;
  SocketCAN::openRAW(socket, can_interface);

  // create receive filter; only need to receive CAN frames of CAN_ID <LEG_RX_POSITION_ID>;
  struct can_filter filter[1];
  filter[1].can_id = TILT_RX_POSITION_ID;
  filter[1].can_mask = CAN_SFF_MASK;

  // set receive filter;
  SocketCAN::setFilter(socket, filter);

};

SenekaLeg::~SenekaLeg() {};

bool SenekaLeg::executeCommand(Command command, unsigned char leg_nr) {

  struct can_frame frame;

  frame.can_id  = LEG_TX_COMMAND_ID;
  frame.can_dlc = 8; // count of data bytes;

  for (int i = 0; i < frame.can_dlc; i++) {
    frame.data[i] = 0x00;
  }

  frame.data[LEG_TX_LEG_BYTE_NR] = leg_nr;        // leg number;
  frame.data[LEG_TX_CMD_TYPE_BYTE_NR] = command;  // command type (extend/retract);

  return SocketCAN::writeRAW(socket, frame);

}

bool SenekaLeg::interrupt(unsigned char leg_nr) {

  struct can_frame frame;

  frame.can_id  = LEG_TX_INTERRUPT_ID;
  frame.can_dlc = 8; // count of data bytes;

  for (int i = 0; i < frame.can_dlc; i++) {
    frame.data[i] = 0x00;
  }

  frame.data[LEG_TX_LEG_BYTE_NR] = leg_nr; // leg number;

  return SocketCAN::writeRAW(socket, frame);

}

/********************************************/
/********************************************/
/********************************************/

#endif // SENEKA_LEG_H_