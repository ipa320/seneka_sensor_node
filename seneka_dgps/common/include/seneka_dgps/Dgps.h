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
* Description: The seneka_dgps package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
* It implements a GNU/Linux driver for the Trimble BD982 GNSS Receiver Module as well as a ROS publisher node "DGPS", which acts as a wrapper for the driver.
* The ROS node "DGPS" publishes GPS data gathered by the DGPS device driver.
* This package might work with other hardware and can be used for other purposes, however the development has been specifically for this project and the deployed sensors.
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

#ifndef DGPS_H_
#define DGPS_H_

/****************************************/
/*************** includes ***************/
/****************************************/

#include <seneka_dgps/SerialIO.h>

#include <sstream>
#include <string>
#include <cstring>
#include <vector>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <math.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <cstdlib>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

/******************************************/
/*************** DGPS class ***************/
/******************************************/

class Dgps {

    public:

        // constructor
        Dgps();

        // destructor
        ~Dgps();

        /****************************************************/
        /*************** diagnostics handling ***************/
        /****************************************************/

        enum DiagnosticFlag {

            DEBUG,
            INFO,
            WARNING,
            ERROR,
            FATAL

        };

        struct DiagnosticStatement {

            std::string     diagnostic_message;
            DiagnosticFlag  diagnostic_flag;

        };

        std::vector<DiagnosticStatement> diagnostic_array;

        /*********************************************/
        /*************** data handling ***************/
        /*********************************************/

        // RAWDATA (57h) PACKET - POSITION RECORD;
        // this struct data type represents the structure of a "RAWDATA" (57h) position record packet;
        // base unit is 1 char = 1 byte = 8 bit;
        // position record (packet type: 57h, see Trimble BD982 GNSS Receiver manual, p. 139;
        struct PacketData {

            // header;
            char stx;
            char status;
            char packet_type;
            char length;
            
            // data part;
            char record_type;
            char page_counter;                  // see user guide for bit interpretation!;
            char reply_number;
            char record_interpretation_flags;

            std::vector<char> data_bytes;                  // maximum of 244 bytes for data; concatenate pages if needed!;
            
            // footer;
            char checksum;                      // calculated over: all bytes between stx and checksum;
            char etx;
        
        };

       
        // RAWDATA (57h) PACKET - POSITION RECORD - DATA PART;
        // the structure below contains all the INTERPRETED data bytes of a position record packet data field; 
        // position record (packet type: 57h, see Trimble BD982 GNSS Receiver manual, p. 139);
        struct GpsData {

            double  latitude_value;             // in semi-circles
            double  longitude_value;            // in semi-circles
            double  altitude_value;             // [] = m
            double  clock_offset;               // [] = m
            double  frequency_offset;           // [] = Hz
            double  pdop;                       //
            double  latitude_rate;              // [] = rad/s
            double  longitude_rate;             // [] = rad/s
            double  altitude_rate;              // [] = rad/s
            long    gps_msec_of_week;           // [] = ms
            char    position_flags;             // see Trimble BD982 GNSS receiver manual, p. 140
            char    number_of_SVs;              // number of used satellites
            std::vector<char> channel_numbers;  // 1 char for each satellite
            std::vector<char> prn;              // 1 char for each satellite
        
        };

        /**************************************************/
        /**************************************************/
        /**************************************************/

        // establishes serial connection;
        bool open(const char* pcPort, int iBaudRate);

        // tests the communication link by sending protocol request "ENQ" (05h);
        // expects to receive "ACK" (06h);
        // see Trimble BD982 GNSS receiver manual, p. 65;
        bool checkConnection();

        // requests GPS data from GPS device;
        // hereby called functions analyze the received packet in-depth, structure it, extract and finnaly serve GPS data;
        // if everything works fine, GPS data is getting stored in Dgps::GpsData gps_data;
        bool getDgpsData();

        // getters;
        GpsData getPosition() {return gps_data;}

        // helper functions;

        // for some reason, instead of just responding the requested "RAWDATA" (57h) position record packet, 
        // the receiver module applys additional 16 bytes of data on top of the reply packet;
        // these 16 bytes of data form another separate packet, including header, data part and tail;
        // for now, I couldn't figure out why;
        // to avoid this kind of mistake, the following workaround is necessary;
        std::vector<unsigned char> debugBuffer(unsigned char * buffer);

        enum DataType {

            CHAR,
            SHORT,
            LONG,
            FLOAT,
            DOUBLE,

        };

        // function to reorder incoming bits;
        std::vector<bool>  invertBitOrder      (bool * bits, DataType data_type, bool invertBitsPerByte = true, bool invertByteOrder = false);
        // function to extract numbers of data type CHAR;
        char    getCHAR             (unsigned char byte);
        // function to extract numbers of data type LONG from an 4-byte array;
        long    getLONG             (unsigned char * bytes);
        // function to extract IEEE DOUBLE precision number values from an 8-byte array;
        double  getDOUBLE           (unsigned char * bytes, int exponent_bias = 1023);

        /**************************************************/
        /**************************************************/
        /**************************************************/

    private:

        // serial input/output instance;
        SerialIO m_SerialIO;

        /*********************************************/
        /*************** data handling ***************/
        /*********************************************/

        // see comments at corresponding structure definition above;
        PacketData          temp_packet;
        GpsData             gps_data;

        // analyzes received data packets in-depth, structures it and serves PacketData incoming_packet;
        bool analyzeData(unsigned char *  incoming_data,
                         int              incoming_data_length);

        // takes PacketData incoming_packet from analyzeData(),
        // extracts and finally serves GpsData gps_data;
        bool extractGpsData();

        /****************************************************/
        /*************** diagnostics handling ***************/
        /****************************************************/

        DiagnosticStatement diagnostic_statement;
        
        // helper variables which store diagnostic messages temporarily;
        std::stringstream msg;
        std::stringstream msg_tagged;

        void transmitStatement(DiagnosticFlag flag);

        /****************************************************/
        /****************************************************/
        /****************************************************/

};

#endif // DGPS_H_
