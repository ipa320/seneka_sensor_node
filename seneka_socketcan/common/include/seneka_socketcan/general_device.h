/*!
*****************************************************************
* SenekaTilt.h
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_tilt
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
* 
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
* Modified xx/20xx: 
*
* Description:
* The seneka_tilt package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
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

#pragma once

#include <seneka_socketcan/SocketCAN.h>
#include <string>

class SenekaGeneralCANDevice {
  protected:

    SenekaGeneralCANDevice(const int can_id, const std::string &can_interface = "can0") {
	  // open socket for CAN communication; respect optionally given differing CAN interface;
	  SocketCAN::openRAW(socket_, can_interface);

	  // create receive filter; only need to receive CAN frames of CAN_ID <TILT_RX_POSITION_ID>;
	  struct can_filter filter[1];
	  filter[1].can_id = can_id;
	  filter[1].can_mask = CAN_SFF_MASK;

	  // set receive filter;
	  SocketCAN::setFilter(socket_, filter);
	}
    virtual ~SenekaGeneralCANDevice() {}

    struct can_frame readFrame() const {
	  struct can_frame frame;
	  SocketCAN::readRAW(socket_, frame);
	  return frame;
	}
	
    struct can_frame fillFrame(const int can_id, const int can_dlc=8) const {
	  struct can_frame frame = {};

	  frame.can_id  = can_id;
	  frame.can_dlc = can_dlc; // count of data bytes;
	  
	  return frame;
	}
	
	bool sendFrame(const struct can_frame & frame) const {
		return SocketCAN::writeRAW(socket_, frame);
	}

  private:

    int socket_; // socket for CAN communication;
};
