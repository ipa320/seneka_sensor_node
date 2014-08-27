/*!
*****************************************************************
* SenekaTurret.h
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_turret
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
* 
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
* Modified xx/20xx: 
*
* Description:
* The seneka_turret package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
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

#ifndef SENEKA_TURRET_H_
#define SENEKA_TURRET_H_

/**********************************************/
/*************** SenekaTurret *****************/
/**********************************************/

#define TRUNK_TX_ROTATE_ID          0x196   // "execute rotation"-command CAN-ID; also priority of CAN message (bus arbitration);
#define TRUNK_TX_MODE_BYTE_NR       0       // position of data byte in CAN frame;
#define TRUNK_TX_DIRECTION_BYTE_NR  1       // position of data byte in CAN frame;
#define TRUNK_TX_VELOCITY_BYTE_NR   2       // position of data byte in CAN frame;
#define TRUNK_TX_TARGETPOS_BYTE_NR  3       // position of data byte in CAN frame;

#define TRUNK_TX_INTERRUPT_ID       0x196   // "interrupt rotation"-command CAN-ID; also priority of CAN message (bus arbitration);

#define TRUNK_RX_POSITION_ID        0x35E   // CAN-ID of CAN frame containing current trunk position; also priority of CAN message (bus arbitration);
#define TRUNK_RX_POSITION_BYTE_NR   7       // position of data byte in CAN frame;

#include <seneka_socketcan/SocketCAN.h>
#include <string>

using namespace std;

class SenekaTurret {

  public:

    SenekaTurret(string can_interface);
    ~SenekaTurret();

    // available modes of turret rotation;
    // each mode respects optionally given parameters (cf. executeRotation() parameters);
    enum Mode {
      ENDLESS = 0,  // endless turret rotation;
      CUSTOM  = 1,  // rotation to given target position;
      SINGLE  = 2,  // single entire rotation;
    };

    // mathematical directions of rotation (counter-clockwise);
    enum Direction {
      NEGATIVE = 0,
      POSITIVE = 1,
    };

    // executes rotation in respect of <mode> according to optionally given parameters;
    bool rotate(Mode &mode, Direction direction, unsigned char target_velocity, unsigned char target_position);
    // interrupts rotation; no emergency stop;
    bool interrupt(void);
    // returns updated position value;
    unsigned char getPosition(void);

  private:

    int socket; // socket for CAN communication;

};

/*****************************************/
/*****************************************/
/*****************************************/

// constructor;
SenekaTurret::SenekaTurret(string can_interface = "can0") {

  // open socket for CAN communication; respect optionally given differing CAN interface;
  SocketCAN::openRAW(socket, can_interface);

  // create receive filter; only need to receive CAN frames of CAN_ID <TRUNK_RX_POSITION_ID>;
  struct can_filter filter[1];
  filter[1].can_id = TRUNK_RX_POSITION_ID;
  filter[1].can_mask = CAN_SFF_MASK;

  // set receive filter;
  SocketCAN::setFilter(socket, filter);

}

// destructor;
SenekaTurret::~SenekaTurret() {};

// executes rotation in respect of <mode> according to optionally given parameters;
bool SenekaTurret::rotate(Mode &mode, Direction direction = NEGATIVE, unsigned char target_velocity = 25, unsigned char target_position = 0) {

  struct can_frame frame;

  frame.can_id  = TRUNK_TX_ROTATE_ID;
  frame.can_dlc = 8; // count of data bytes;

  for (int i = 0; i < frame.can_dlc; i++) {
    frame.data[i] = 0x00;
  }

  frame.data[TRUNK_TX_MODE_BYTE_NR]      = mode;             // mode;
  frame.data[TRUNK_TX_DIRECTION_BYTE_NR] = direction;        // direction;
  frame.data[TRUNK_TX_VELOCITY_BYTE_NR]  = target_velocity;  // target velocity; [] = %; [0%; 100%]; increment = 1%;
  frame.data[TRUNK_TX_TARGETPOS_BYTE_NR] = target_position;  // target position; [] = °; [0°; 360°]; increment = 1°;

  return SocketCAN::writeRAW(socket, frame);

}

// interrupts rotation; no emergency stop;
bool SenekaTurret::interrupt(void) {

  struct can_frame frame;

  frame.can_id  = TRUNK_TX_INTERRUPT_ID;
  frame.can_dlc = 8; // count of data bytes;

  for (int i = 0; i < frame.can_dlc; i++) {
    frame.data[i] = 0x00;
  }

  return SocketCAN::writeRAW(socket, frame);

}

// returns updated position value;
unsigned char SenekaTurret::getPosition(void) {

  struct can_frame frame;
  SocketCAN::readRAW(socket, frame);

  return frame.data[TRUNK_RX_POSITION_BYTE_NR];

}

#endif // SENEKA_TURRET_H_