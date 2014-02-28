/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: SENEKA
 * ROS stack name: SENEKA
 * ROS package name: Dgps
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: ciby mathew, email:ciby.mathew@ipa.fhg.de
 * Supervised by: ciby mathew, email:ciby.mathew@ipa.fhg.de
 *
 * Date of creation: Jan 2009
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
#ifndef _Dgps_H
#define _Dgps_H
#include "SerialIO.h"
#include <math.h>
#include <iostream>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <cstdlib>
#include <string>
#include<stdio.h>
#include<stdlib.h>
#include <cstring>
using namespace std;





// see BD982 user guide, packet type 57h
// this struct contains all bytes of a 57h
struct packet_data{
    // --- header --- p.132: 4 bytes packet header
    char stx;
    char status;
    char packet_type;
    char length;
    // --- data_part --- including 4 bytes of paging and interpretation data
    char record_type;
    char page_counter;                                                          // see user guide for bit interpretation!
    char reply_number;
    char record_interpretation_flags;
    char * data_bytes;                                                          // maximum of 244 bytes for data; concatenate pages if needed!
    // --- footer ---
    char checksum;                                                              // calculated over: all bytes between stx and checksum
    char etx;
};

// see BD982 user guide, packet type 57h: Position Record; p. 139
// this struct contains all the interpreted values of the data fields of a Position Record packet
//
// interpretation of data bytes follows:
// ... latitude; 8 char bytes --> Motorola Byte-Order -> IEEE Double Precision Floating Point Format
// ... ...
struct gps_data{
    double latitude_value;                                                      // in semi-circles
    double longitude_value;                                                     // in semi-circles
    double altitude_value;                                                      // in meters
    double clock_offset;                                                        // in meters
    double frequency_offset;                                                    // in Hz
    double pdop;
    double latitude_rate;                                                       // in radians per second
    double longitude_rate;                                                      // in radians per second
    double altitude_rate;                                                       // in meters  per second
    long gps_msec_of_week;                                                      // in msec
    char position_flags;                                                        // see page 140
    char number_of_SVs;                                                         // number of used satellites
    char * channel_number;                                                      // 1 char for each satellite
    char * prn;                                                                 // 1 char for each satellite
}; 



class Dgps
{
public:

	// Constructor
	Dgps();

	// Destructor
	~Dgps();

	/**
	 * Opens serial port.
	 * @param pcPort used "COMx" or "/dev/ttyUSB0"
	 * @param iBaudRate baud rate
	 */
	bool open(const char* pcPort, int iBaudRate);

        bool receiveData(unsigned char * incoming_data, 
                int incoming_data_length,
                packet_data incoming_packet,
                gps_data &position_record);               // gets data from serial.IO, gives packet data
        bool extractGPS(packet_data &incoming_packet, gps_data &position_record );        // gets packet data, gives gps data

	bool getPosition(gps_data &position_record);
        bool checkConnection();

private:
	// Constants
	// Components
	SerialIO m_SerialIO;
	// Functions

	unsigned int getUnsignedWord(unsigned char msb, unsigned char lsb)
	{
		return (msb << 8) | lsb;
	}

	unsigned int createCRC(unsigned char *ptrData, int Size);

};
#endif //

