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

#ifndef SENEKA_TRUNK_H_
#define SENEKA_TRUNK_H_

/****************************************/
/*************** includes ***************/
/****************************************/

#include <seneka_socketcan/SocketCAN.h>

/***************************************************/
/*************** SenekaTrunk Class *****************/
/***************************************************/

class SenekaTrunk {

  public:

    // default constructor;
    SenekaTrunk();

    // destructor;
    ~SenekaTrunk();

    // enum data types;

    enum TrunkMode {

      ENDLESS = 0,
      CUSTOM  = 1,
      ONCE    = 2,

    };

    enum TrunkDirection {

      NEGATIVE = 0,
      POSITIVE = 1,

    };

    // member functions;
    void run(void);
    void stop(void);
    void turnNegative(void);
    void turnPositive(void);

    // getters;
    unsigned char   getCurrentPosition(void)  {return this->current_position;}
    unsigned char   getTargetPosition(void)   {return this->target_position;}
    unsigned char   getCurrentVelocity(void)  {return this->current_velocity;}
    unsigned char   getTargetVelocity(void)   {return this->target_velocity;}
    unsigned char   getSensitivity(void)      {return this->sensitivity;}
    TrunkDirection  getDirection(void)        {return this->direction;}
    TrunkMode       getMode(void)             {return this->mode;}

    // setters;
    void setTargetPosition(unsigned char position) {
      this->target_position = position;
    }

    void setTargetVelocity(unsigned char velocity) {
      this->target_velocity = velocity;
    }

    void setSensitivity(unsigned char sensitivity) {
      this->sensitivity = sensitivity;
    }

    void setDirection(TrunkDirection direction) {
      this->direction = direction;
    }

    void setMode(TrunkMode mode) {
      this->mode = mode;
    }

  private:

    SocketCAN socketCAN;

    unsigned char   current_position;
    unsigned char   target_position;
    unsigned char   current_velocity;
    unsigned char   target_velocity;
    unsigned char   sensitivity;
    TrunkDirection  direction;
    TrunkMode       mode;

};

/******************************************************************/
/*************** SenekaTrunk Class Implementation *****************/
/******************************************************************/

// default constructor;
SenekaTrunk::SenekaTrunk() {

  setTargetPosition(0);
  setTargetVelocity(25);
  setSensitivity(15);
  setDirection(NEGATIVE);
  setMode(CUSTOM);

};

// destructor;
SenekaTrunk::~SenekaTrunk() {};

void SenekaTrunk::run(void) {

  struct can_frame frame;

  frame.can_id  = 0x196;  // trunk CAN-ID; also priority of CAN message (bus arbitration);
  frame.can_dlc = 8;      // count of data bytes;

  if (getMode() != ONCE) {

    frame.data[0] = getMode();            // mode;
    frame.data[1] = getDirection();       // direction;
    frame.data[3] = getTargetVelocity();  // target velocity; [] = %; [0%; 100%]; increment = 1%;
    frame.data[2] = getTargetPosition();  // target position; [] = °; [0°; 360°]; increment = 1°;
    frame.data[4] = 0x00;                 // void;
    frame.data[5] = 0x00;                 // void;
    frame.data[6] = 0x00;                 // void;
    frame.data[7] = 0x00;                 // void;

    socketCAN.writeRAW(&frame);

  }

  else {

    return;

  }

}

// interrupt trunk rotation;
void SenekaTrunk::stop(void) {

  struct can_frame frame;

  frame.can_id  = 0x196;  // trunk CAN-ID; also priority of CAN message;
  frame.can_dlc = 8;      // count of data bytes;
  frame.data[0] = 0x02;

  for (int i = 0+1; i < frame.can_dlc; i++) {

    frame.data[i] = 0x00; // void;

  }

  socketCAN.writeRAW(&frame);

}

// trunk rotates in negative direction, depending on given sensitivity;
void SenekaTrunk::turnNegative(void) {

  setMode(CUSTOM);
  setTargetPosition(getCurrentPosition() - getSensitivity());
  run();

}

// trunk rotates in positive direction, depending on given sensitivity;
void SenekaTrunk::turnPositive(void) {

  setMode(CUSTOM);
  setTargetPosition(getCurrentPosition() + getSensitivity());
  run();

}

#endif // SENEKA_TRUNK_H_