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
* ROS package name: seneka_leg
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
* 
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
* Modified xx/20xx: 
*
* Description:
* The seneka_leg package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
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

#include <seneka_socketcan/general_device.h>

class SenekaLeg : public SenekaGeneralCANDevice {
	enum {BN_STATUS=0, BN_M0=1, BN_M1=3, BN_SERVO=5};
	struct can_frame send_frame_;
	
	uint16_t _read16(const uint8_t *buf) {
		return ( (uint16_t)buf[0] | (((uint16_t)buf[1])<<8) );
	}
	void _write16(uint8_t *buf, uint16_t val) {
		buf[0] = (val&0xff);
		buf[1] = (val>>8);
	}
	
	void readPosition(const struct can_frame &frame) {
		if(frame.data[BN_STATUS]&0x80)
			return;
		const bool sw = (frame.data[BN_STATUS]&0x02)!=0;
		updated(_read16(frame.data+BN_M0), 0);
		updated(_read16(frame.data+BN_M1), 1);
		updated(_read16(frame.data+BN_SERVO), 2);
		updated(!sw);
	}
	
	virtual void _setTarget(const int joint, const double val) {
		_write16(send_frame_.data+(joint*2+1), (uint16_t)std::floor(val));
		sendFrame(send_frame_);
		
		if(joint==2)	//servo cannot be read!
			updated(val, 2);
	}

  public:

    SenekaLeg(const std::string &can_interface = "can0") :
		SenekaGeneralCANDevice(3, can_interface) {
	}
	
	virtual void init(const int can_id = LEG_RX_POSITION_ID) {
		addListener(can_id, boost::bind(&SenekaLeg::readPosition, this, _1));
		send_frame_ = fillFrame(can_id);
		memset(send_frame_.data, 0xff, sizeof(send_frame_.data)); //set to "stop"-value
	}
};


#endif // SENEKA_LEG_H_
