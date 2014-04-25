/*!
*****************************************************************
* seneka_dgps.cpp
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_dgps
*
* Author: Ciby Mathew, E-Mail: Ciby.Mathew@ipa.fhg.de
* 
* Supervised by: Christophe Maufroy
*
* Date of creation: Jan 2013
* Modified 03/2014: David Bertram, E-Mail: davidbertram@gmx.de
* Modified 04/2014: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
*
* Description:
*
* To-Do:
*
* --> see seneka_dgps_node.cpp To-Do
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

#ifndef _DGPS_H
#define _DGPS_H

/********************/
/***** includes *****/
/********************/

// internal includes
#include <seneka_dgps/SerialIO.h>

// standart includes

#include <iostream>
#include <string>
#include <stdio.h>
#include <cstring>

#include <math.h>

#include <errno.h>
#include <fcntl.h>
#include <cstdlib>
#include <stdlib.h>

// miscellaneous includes

#include <endian.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

/**********************/
/***** DGPS class *****/
/**********************/

class Dgps
{

    public:

        // constructor
        Dgps();

        // destructor
        ~Dgps();

        // packet type 57h: position record (see Trimble BD982 GNSS receiver manual, page 139)
        // this struct contains all the interpreted values of the data fields of a position record packet
        // interpretation of data bytes follows:
        // latitude: 8 char bytes --> Motorola Byte-Order --> IEEE Double Precision Floating Point Format
        // ...
        struct gps_data
        {
            double  latitude_value;      // in semi-circles
            double  longitude_value;     // in semi-circles
            double  altitude_value;      // [] = m
            double  clock_offset;        // [] = m
            double  frequency_offset;    // [] = Hz
            double  pdop;                //
            double  latitude_rate;       // [] = rad/s
            double  longitude_rate;      // [] = rad/s
            double  altitude_rate;       // [] = rad/s
            long    gps_msec_of_week;    // [] = ms
            char    position_flags;      // see Trimble BD982 GNSS receiver manual, page 140
            char    number_of_SVs;       // number of used satellites
            char *  channel_number;      // 1 char for each satellite
            char *  prn;                 // 1 char for each satellite
        };

        // this struct contains all bytes of a 57h packet (see Trimble BD982 GNSS receiver manual, page 132)
        struct packet_data
        {
            // header
            // see Trimble BD982 GNSS receiver manual, page 132: 4 bytes packet header
            char stx;
            char status;
            char packet_type;
            char length;
            // data_part
            // including 4 bytes of paging and interpretation data
            char record_type;
            char page_counter;                  // see user guide for bit interpretation!
            char reply_number;
            char record_interpretation_flags;
            char * data_bytes;                  // maximum of 244 bytes for data; concatenate pages if needed!
            // footer
            char checksum;                      // calculated over: all bytes between stx and checksum
            char etx;
        };

        // transfers a status statement to SenekaDgps instance for publishing a status message
        void publishStatus(std::string status_str, int level);

        // opens serial port at given baud rate
        bool open(const char* pcPort, int iBaudRate);

        // gets data from serial.IO and serves packet_data
        bool interpretData(unsigned char * incoming_data,
                           int incoming_data_length,
                           packet_data incoming_packet,
                           gps_data &position_record);

        // takes packet data from interpretData(), serves gps_data
        bool extractGPS(packet_data &incoming_packet,
                        gps_data &position_record);
        
        /*  function getPosition():
        *
        *   - requests position record packet from receiver (see Trimble BD982 GNSS receiver manual, page 139)
        *   - appends incoming data to ringbuffer
        *   - tries to extract valid packets (incl. checksum verification)
        *   - tries to read "position record"-fields from valid packets
        *   - writes "position record"-data into gps_data struct
        */                            
        bool getPosition(gps_data &position_record);
    
        // tests the communications link by sending protocol request ENQ (05h) (see Trimble BD982 GNSS receiver manual, page 65)
        // sends 0x052 and expects to receive 0x06
        bool checkConnection();

    private:
    
	   SerialIO m_SerialIO;
};

#endif