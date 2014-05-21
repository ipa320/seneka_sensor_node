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

        // enables/disables console output;
        // default = false (initialized in constructor);
        bool cout_enabled;

        /****************************************************/
        /*************** diagnostics handling ***************/
        /****************************************************/

        enum DiagnosticFlag {

            DEBUG,
            INFO,
            WARNING,
            ERROR,

        };

        struct DiagnosticStatement {

            std::string     diagnostic_message;
            DiagnosticFlag  diagnostic_flag;

        };

        std::vector<DiagnosticStatement> diagnostic_array;

        /*********************************************/
        /*************** data handling ***************/
        /*********************************************/

        // POSITION RECORD PACKET;
        // the strucutre below contains all bytes of an UNINTERPRETED position record packet;
        // base unit is 1 char = 1 byte = 8 bit;
        // position record packet (packet type: 57h, see Trimble BD982 GNSS Receiver manual, p. 139;
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

            char * data_bytes;                  // maximum of 244 bytes for data; concatenate pages if needed!;
            
            // footer;
            char checksum;                      // calculated over: all bytes between stx and checksum;
            char etx;
        
        };

        // POSITION RECORD PACKET;
        // the variables below represent the starting positions of associated data bytes in a position record packet;
        // base unit is 1 char = 1 byte = 8 bit;
        // position record packet (packet type: 57h, see Trimble BD982 GNSS Receiver manual, p. 139);
        struct PacketDataStructure {

            int stx_index;                          // index: starting position of data element in incoming data frame
            int status_index;
            int packet_type_index;
            int length_index;
            int record_type_index;
            int page_counter_index;                 // split byte in two parts! its <page> of <total>, each 4 bit
            int reply_number_index;
            int record_interpretation_flags_index;     
            int data_bytes_index;
            // checksum_index see helper function checksum_index()
            // etx_index see helper function etx_index()

        };

        // POSITION RECORD PACKET - DATA PART;
        // the structure below contains all the INTERPRETED data bytes of a position record packet data field; 
        // position record packet (packet type: 57h, see Trimble BD982 GNSS Receiver manual, p. 139);
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

        // this structure contains all variables to index the positions of position record packet data field bytes;
        // index is relative to begin of data_part, so it is byte #: index+8... of packet-bytes (stx = 0);
        // position record packet (packet type: 57h, see Trimble BD982 GNSS receiver manual, p. 132/139);
        struct GpsDataStructure {

            int latitude_value_index;
            int longitude_value_index;
            int altitude_value_index;
            int clock_offset_index;
            int frequency_offset_index;
            int pdop_index;
            int latitude_rate_index;
            int longitude_rate_index;
            int altitude_rate_index;
            int gps_msec_of_week_index;
            int position_flags_index;
            int number_of_SVs_index;
            int channel_number_index;
            int prn_index;
        
        };

        /**************************************************/
        /**************************************************/
        /**************************************************/

        // establishes serial connection;
        bool open(const char* pcPort, int iBaudRate);

        // tests the communications link by sending protocol request "ENQ" (05h);
        // expects to receive "ACK" (0x06h);
        // see Trimble BD982 GNSS receiver manual, p. 65;
        bool checkConnection();

        /*  function getDgpsData():
        *
        *   --> requests position record packet from receiver (see Trimble BD982 GNSS receiver manual, p. 132/139)
        *   --> appends incoming data to ringbuffer
        *   --> tries to extract valid packets (incl. checksum verification)
        *   --> tries to read position-record-fields from valid packets
        *   --> writes position-record-data into GpsData struct
        */  
        bool getDgpsData();

        // getters;
        GpsData getPosition() {return gps_data;}

        // helper functions;

        enum DataType {

            CHAR,
            SHORT,
            LONG,
            FLOAT,
            DOUBLE,

        };

        // function to reorder incoming bits;
        bool *  invertBitOrder      (bool * bits, DataType data_type, bool invertBitsPerByte = true, bool invertByteOrder = false);
        // function to extract numbers of data type CHAR;
        char    getCHAR             (unsigned char byte);
        // function to extract numbers of data type LONG from an 4-byte array;
        long    getLONG             (unsigned char * bytes);
        // function to extract IEEE DOUBLE precision number values from an 8-byte array;
        double  getDOUBLE           (unsigned char * bytes, int exponent_bias = 1023);

        int     data_bytes_length   (int length_value);
        int     checksum_index      (int length_value);      
        int     etx_index           (int length_value);

        /**************************************************/
        /**************************************************/
        /**************************************************/

    private:

        // serial input/output instance;
        SerialIO m_SerialIO;

        /*********************************************/
        /*************** data handling ***************/
        /*********************************************/

        unsigned char   ringbuffer[4096 * 4];   // ! important: change int ringbuffer_size = ... (in constructor) too, when changing number of ringbuffer elements ringbuffer[...]!
        int             ringbuffer_size;        // ! ==> must be euqal to number of elements in ringbuffer array!
        int             ringbuffer_start;
        int             ringbuffer_length;

        // see comments at corresponding structure definition above;
        PacketDataStructure packet_data_structure;
        PacketData          incoming_packet;
        GpsDataStructure    gps_data_structure;
        GpsData             gps_data;

        // helper variable, used to find meaning of semi-circles in this case...;
        // (it's just 0-180 normalized to 0.0-1.0);
        double semi_circle_factor;

        // gets data from serial.IO and serves PacketData;
        bool interpretData(unsigned char *  incoming_data,
                           int              incoming_data_length);

        // takes packet data from interpretData(), serves GpsData;
        bool extractGPS();

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