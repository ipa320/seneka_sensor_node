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

#ifndef DGPS_H_
#define DGPS_H_

/****************************************/
/*************** includes ***************/
/****************************************/

#include <seneka_dgps/SerialIO.h>

#include <string>
#include <cstring>
#include <vector>
#include <iostream>
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

            OK          = 0,
            WARNING     = 1,
            ERROR       = 2
        };

        struct DiagnosticStatement {

            std::string     diagnostic_message;
            DiagnosticFlag  diagnostic_flag;
        };

        /****************************************************/
        /****************************************************/
        /****************************************************/



        /***************************************************/
        /*************** data frame handling ***************/
        /***************************************************/

        // this structure contains all bytes of an UNINTERPRETED position record packet
        // position record packet (packet type: 57h, see Trimble BD982 GNSS receiver manual, p. 132/139)
        struct PacketData {

            // header (4 bytes packet header, see Trimble BD982 GNSS receiver manual, p. 132)
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

        // this structure contains all variables to index the positions of position record packet bytes
        // position record packet (packet type: 57h, see Trimble BD982 GNSS receiver manual, p. 132/139)
        struct PacketDataStructure {

            int stx_index;                          // index: starting position of data element in incoming data frame
            int stx_length;                         // length of data field in incoming data frame ([] = byte)

            int status_index;
            int status_length;

            int packet_type_index;
            int packet_type_length;

            int length_index;
            int length_length;

            int record_type_index;
            int record_type_length;

            int page_counter_index;                 // split byte in two parts! its <page> of <total>, each 4 bit
            int page_counter_length;

            int reply_number_index;
            int reply_number_length;

            int record_interpretation_flags_index;     
            int record_interpretation_flags_length;

            int data_bytes_index;
        };

        // this structure contains all the INTERPRETED data field bytes of a position record packet
        // interpretation of data bytes follows:
        // latitude: 8 char bytes --> Motorola Byte-Order --> IEEE Double Precision Floating Point Format
        // ...
        // position record packet (packet type: 57h, see Trimble BD982 GNSS receiver manual, p. 132/139)
        struct GpsData {

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
            char    position_flags;      // (see Trimble BD982 GNSS receiver manual, p. 140)
            char    number_of_SVs;       // number of used satellites
            char *  channel_number;      // 1 char for each satellite
            char *  prn;                 // 1 char for each satellite
        };

        // this structure contains all variables to index the positions of position record packet data field bytes
        // index is relative to begin of data_part, so it is byte #: index+8... of packet-bytes (stx = 0)
        // position record packet (packet type: 57h, see Trimble BD982 GNSS receiver manual, p. 132/139)
        struct GpsDataStrucutre {

            int latitude_value_index;
            int latitude_value_length;

            int longitude_value_index;
            int longitude_value_length;

            int altitude_value_index;
            int altitude_value_length;

            int clock_offset_index;
            int clock_offset_length;

            int frequency_offset_index;
            int frequency_offset_length;

            int pdop_index;
            int pdop_length;

            int latitude_rate_index;
            int latitude_rate_length;

            int longitude_rate_index;
            int longitude_rate_length;

            int altitude_rate_index;
            int altitude_rate_length;

            int gps_msec_of_week_index;
            int gps_msec_of_week_length;

            int position_flags_index;
            int position_flags_length;

            int number_of_SVs_index;
            int number_of_SVs_length;

            // 78 .. 80 .. 82 .. 84 ..
            int * channel_number_index;
            int channel_number_length;

            // 79 .. 81 .. 83 .. 85 ..
            int * prn_index;
            int prn_length;

            // helper variable, used to find meaning of semi-circles in this case... 
            //(it's just 0-180 normalized to 0.0-1.0)
            double semi_circle_factor;
        };

        /***************************************************/
        /***************************************************/
        /***************************************************/



        /****************************************************/
        /*************** public class methods ***************/
        /****************************************************/

        // opens serial port at given baud rate
        bool open(const char* pcPort, int iBaudRate);

        // tests the communications link by sending protocol request ENQ (05h)
        // sends 0x05h and expects to receive 0x06h
        // (see Trimble BD982 GNSS receiver manual, p. 65)
        bool checkConnection();

        /*  function getGpsData():
        *
        *   --> requests position record packet from receiver (see Trimble BD982 GNSS receiver manual, p. 132/139)
        *   --> appends incoming data to ringbuffer
        *   --> tries to extract valid packets (incl. checksum verification)
        *   --> tries to read position-record-fields from valid packets
        *   --> writes position-record-data into GpsData struct
        */  
        bool getGpsData();

        // getters
        GpsData                             getPosition()           {return gps_data;}
        std::vector<DiagnosticStatement>    getDiagnosticArray()    {return diagnostic_array;}

        // setters
        void clearDiagnosticArray() {diagnostic_array.clear();}

        /*****************************************************/
        /*****************************************************/
        /*****************************************************/


        
    private:

        // serial input/output
        SerialIO m_SerialIO;

        /***************************************************/
        /*************** data frame handling ***************/
        /***************************************************/

        PacketDataStructure     packet_data_structure;
        GpsDataStrucutre        gps_data_structure;
        GpsData                 gps_data;

        // gets data from serial.IO and serves PacketData
        bool interpretData(unsigned char * incoming_data,
                           int incoming_data_length,
                           PacketData incoming_packet,
                           GpsData &gps_data);

        // takes packet data from interpretData(), serves GpsData
        bool extractGPS(PacketData &incoming_packet,
                        GpsData &gps_data);

        /***************************************************/
        /***************************************************/
        /***************************************************/



        /****************************************************/
        /*************** diagnostics handling ***************/
        /****************************************************/

        DiagnosticStatement                 diagnostic_statement;
        std::vector<DiagnosticStatement>    diagnostic_array;

        void transmitStatement(std::string message, DiagnosticFlag flag);

        /****************************************************/
        /****************************************************/
        /****************************************************/
};

#endif // DGPS_H_