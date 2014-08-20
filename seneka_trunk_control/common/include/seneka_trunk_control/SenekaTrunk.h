/*!
*****************************************************************
* SenekaTrunk.h
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_trunk_control
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
* 
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
* Modified xx/20xx: 
*
* Description:
* The seneka_trunk_control package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
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

#ifndef SENEKA_TRUNK_H_
#define SENEKA_TRUNK_H_

#define TX_MOTION_ID        0x196   // "execute motion"-command CAN-ID; also priority of CAN message (bus arbitration);
#define TX_MODE_BYTENR      0       // position of data byte in CAN frame;
#define TX_DIRECTION_BYTENR 1       // position of data byte in CAN frame;
#define TX_VELOCITY_BYTENR  2       // position of data byte in CAN frame;
#define TX_TARGETPOS_BYTENR 3       // position of data byte in CAN frame;

#define TX_INTERRUPT_ID     0x196   // "interrupt motion"-command CAN-ID; also priority of CAN message (bus arbitration);

#define RX_POSITION_ID      0x35E   // CAN-ID of CAN frame containing current trunk position; also priority of CAN message (bus arbitration);
#define RX_POSITION_BYTENR  7       // position of data byte in CAN frame;

/*********************************************/
/*************** SenekaTrunk *****************/
/*********************************************/

#include <seneka_socketcan/SocketCAN.h>

class SenekaTrunk {

  public:

    SenekaTrunk(char * can_interface);
    ~SenekaTrunk();

    // available modes of trunk rotation;
    // each mode respects optionally given parameters (cf. execution function parameters);
    enum Mode {
      ENDLESS = 0,  // endless trunk rotation;
      CUSTOM  = 1,  // rotation to given target position;
      ONCE    = 2,  // single entire rotation;
    };

    // mathematical directions of rotation (counter-clockwise);
    enum Direction {
      NEGATIVE = 0,
      POSITIVE = 1,
    };

    // executes rotation movement in respect of <mode> according to optionally given parameters;
    void executeMotion(Mode mode, Direction direction, unsigned char target_velocity, unsigned char target_position);
    // interrupts rotation movement; no emergency stop;
    void interruptMotion(void);
    // returns updated position value;
    unsigned char getPosition(void);

};

/*****************************************/
/*****************************************/
/*****************************************/

// constructor;
SenekaTrunk::SenekaTrunk(char * can_interface = "can0") {

  // open socket for CAN communication; respect optionally given differing CAN interface;
  socket = SocketCAN::openRAW(can_interface);

  // create receive filter; only need to receive CAN frames of CAN_ID <RX_POSITION_ID>;
  struct can_filter filter[1];
  filter[1].can_id = RX_POSITION_ID;
  filter[1].can_mask = CAN_SFF_MASK;

  // set receive filter;
  setFilter(socket, &filter)

};

// destructor;
SenekaTrunk::~SenekaTrunk() {};

// executes rotation movement in respect of <mode> according to optionally given parameters;
void SenekaTrunk::executeMotion(Mode &mode, Direction &direction = NEGATIVE, unsigned char &target_velocity = 25, unsigned char &target_position = 0.00) {

  struct can_frame frame;

  frame.can_id  = TX_MOTION_ID;
  frame.can_dlc = 8; // count of data bytes;

  for (int i = 0; i < frame.can_dlc; i++) {
    frame.data[i] = 0x00;
  }

  frame.data[TX_MODE_BYTENR]      = mode;             // mode;
  frame.data[TX_DIRECTION_BYTENR] = direction;        // direction;
  frame.data[TX_VELOCITY_BYTENR]  = target_velocity;  // target velocity; [] = %; [0%; 100%]; increment = 1%;
  frame.data[TX_TARGETPOS_BYTENR] = target_position;  // target position; [] = °; [0°; 360°]; increment = 1°;

  SocketCAN::writeRAW(socket, frame);

}

// interrupts rotation movement; no emergency stop;
void SenekaTrunk::interruptMotion(void) {

  struct can_frame frame;

  frame.can_id  = TX_INTERRUPT_ID;
  frame.can_dlc = 8; // count of data bytes;

  for (int i = 0; i < frame.can_dlc; i++) {
    frame.data[i] = 0x00;
  }

  SocketCAN::writeRAW(socket, frame);

}

// returns updated position value;
unsigned char SenekaTrunk::getPosition(void) {

  struct can_frame frame;
  SocketCAN::readRaw(socket, frame);

  return frame.data[RX_POSITION_BYTENR];

}

#endif // SENEKA_TRUNK_H_