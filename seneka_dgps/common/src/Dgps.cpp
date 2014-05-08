/*!
*****************************************************************
* Dgps.cpp
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

#include <seneka_dgps/Dgps.h>

/*********************************************************/
/*************** Dgps class implementation ***************/
/*********************************************************/

/*******************************************/
/*************** Constructor ***************/
/*******************************************/

Dgps::Dgps() {

    /***************************************************/
    /*************** data frame handling ***************/
    /***************************************************/

    int             ringbuffer_size         = 4096 * 4; // ! important: change next line too (array init), when changing buffer size!
    unsigned char   ringbuffer[4096 * 4]    = {0};
    int             ringbuffer_start        = 0;
    int             ringbuffer_length       = 0;

    // the following variables allow to extract associated packet fields from incoming data frames as described in Trimble BD982 GNSS receiver manual
    // base unit is 1 char = 8 Bit

    // packet_data_structure initialization

    packet_data_structure.stx_index                           = 0;    // index: starting position of data element in incoming data frame
    packet_data_structure.stx_length                          = 1;    // length of data field in incoming data frame ([] = byte)

    packet_data_structure.status_index                        = 1;
    packet_data_structure.status_length                       = 1;

    packet_data_structure.packet_type_index                   = 2;
    packet_data_structure.packet_type_length                  = 1;

    packet_data_structure.length_index                        = 3;
    packet_data_structure.length_length                       = 1;

    packet_data_structure.record_type_index                   = 4;
    packet_data_structure.record_type_length                  = 1;

    packet_data_structure.page_counter_index                  = 5;    // split byte in two parts! its <page> of <total>, each 4 bit
    packet_data_structure.page_counter_length                 = 1;

    packet_data_structure.reply_number_index                  = 6;
    packet_data_structure.reply_number_length                 = 1;

    packet_data_structure.record_interpretation_flags_index   = 7;     
    packet_data_structure.record_interpretation_flags_length  = 1;

    packet_data_structure.data_bytes_index                    = 8;

    // gps_data_structure initialization

    // variables for index positions of byte indizes for the fields of a position record packet
    // index is relative to begin of data_part, so it is byte #: index+8... of packet-bytes (stx = 0)

    gps_data_structure.latitude_value_index     = 0;
    gps_data_structure.latitude_value_length    = 8;

    gps_data_structure.longitude_value_index    = 8;
    gps_data_structure.longitude_value_length   = 8;

    gps_data_structure.altitude_value_index     = 16;
    gps_data_structure.altitude_value_length    = 8;

    gps_data_structure.clock_offset_index       = 24;
    gps_data_structure.clock_offset_length      = 8;

    gps_data_structure.frequency_offset_index   = 32;
    gps_data_structure.frequency_offset_length  = 8;

    gps_data_structure.pdop_index               = 40;
    gps_data_structure.pdop_length              = 8;

    gps_data_structure.latitude_rate_index      = 48;
    gps_data_structure.latitude_rate_length     = 8;

    gps_data_structure.longitude_rate_index     = 56;
    gps_data_structure.longitude_rate_length    = 8;

    gps_data_structure.altitude_rate_index      = 64;
    gps_data_structure.altitude_rate_length     = 8;

    gps_data_structure.gps_msec_of_week_index   = 72;
    gps_data_structure.gps_msec_of_week_length  = 4;

    gps_data_structure.position_flags_index;
    gps_data_structure.position_flags_length    = 76;

    gps_data_structure.number_of_SVs_index;
    gps_data_structure.number_of_SVs_length     = 77;

    // 78 .. 80 .. 82 .. 84 ..
    gps_data_structure.channel_number_index;
    gps_data_structure.channel_number_length    = 1;

    // 79 .. 81 .. 83 .. 85 ..
    gps_data_structure.prn_index;
    gps_data_structure.prn_length               = 1;

    // helper variable, used to find meaning of semi-circles in this case... (it's just 0-180 normalized to 0.0-1.0)
    gps_data_structure.semi_circle_factor       = 180.0;

    /***************************************************/
    /***************************************************/
    /***************************************************/

}

/*******************************************/
/*******************************************/
/*******************************************/

// destructor
Dgps::~Dgps() {

    m_SerialIO.close();

}

/****************************************************/
/*************** diagnostics handling ***************/
/****************************************************/

// takes diagnostic statements and stores them in diagnostic_array
// if diagnostic_array holds more than 100 elements, the oldest stored element will get erased
void Dgps::transmitStatement(std::string msg, DiagnosticFlag flag) {

    std::cout << "\n";

    switch(flag) {

        case DEBUG:

            std::cout << "DGPS [DEBUG]: "   << msg;
            break;

        case INFO:

            std::cout << "DGPS [INFO]: "    << msg;
            break;

        case WARNING:

            std::cout << "DGPS [WARNING]: " << msg;
            break;

        case ERROR:

            std::cout << "DGPS [ERROR]: "   << msg;
            break;
    
    }

    std::cout << "\n";

    diagnostic_statement.diagnostic_message = msg;
    diagnostic_statement.diagnostic_flag    = flag;

    if (diagnostic_array.size() >= 100) {

        std::vector<DiagnosticStatement>::iterator it = diagnostic_array.begin();

        diagnostic_array.erase(it);
    
    }

    else {

        diagnostic_array.push_back(diagnostic_statement);

    }

}

/****************************************************/
/****************************************************/
/****************************************************/

/********************************************/
/*************** Dgps::open() ***************/
/********************************************/

// opens serial connection
bool Dgps::open(const char * pcPort, int iBaudRate) {

    int serial_open;
    m_SerialIO.setBaudRate(iBaudRate);
    m_SerialIO.setDeviceName(pcPort);
    serial_open = m_SerialIO.open();

    if (serial_open == 0) {

        m_SerialIO.purge();

        msg << "DGPS: Opened port " << pcPort << " at " << iBaudRate <<" Bd.";
        transmitStatement(msg.str(), INFO);
        return true;

    }

    else {

        msg << "DGPS: Opening port " << pcPort << " at " << iBaudRate <<" Bd failed. Device is not available.";
        transmitStatement(msg.str(), ERROR);
        return false;

    }

}

/********************************************/
/********************************************/
/********************************************/

/*******************************************************/
/*************** Dgps::checkConnection() ***************/
/*******************************************************/

// tests the communications link by sending protocol request "ENQ" (05h) (see Trimble BD982 GNSS receiver manual, page 65)
// returns success response "ACK" (06h) (see Trimble BD982 GNSS receiver manual, page 65)
bool Dgps::checkConnection() {

    // test command "ENQ" (05h)
    char message[]  = {0x05};
    int length      = sizeof (message) / sizeof (message[0]);

    // send connection check message
    int bytesWritten = m_SerialIO.write(message, length);

    unsigned char Buffer[1024] = {0};

    bool    success     = false;    // connection check response
    int     count       = 0;        // count of how many times connection check has been retried
    int     max_tries   = 5;        // number of maximum tries to check connection
    int     retry_delay = 1000000;  // [] = us

    // expected test response "ACK" (06h)
    while (!success && (count < max_tries)) {

        count += 1;

        usleep(retry_delay);

        int bytesRead = m_SerialIO.readNonBlocking((char*) Buffer, 1020);

        if (bytesRead > 0 && Buffer[0] == 6) {

            success = true;

        }

        else {

            success = false;

        }

    }

    return success;

}

/*******************************************************/
/*******************************************************/
/*******************************************************/

/***************************************************/
/*************** Dgps::getDgpsData() ***************/
/***************************************************/

bool Dgps::getDgpsData() {

    // is set to true if extracting DGPS position values succeeded
    bool success = false;

    unsigned char Buffer[1024]  = {0};
    int buffer_index            =  0;
    int bytesread;
    int byteswrite;

    // generation of request message (see Trimble BD982 GNSS receiver manual, p. 73)
    unsigned char stx_          = 0x02;
    unsigned char status_       = 0x00;
    unsigned char packet_type_  = 0x56;
    unsigned char length_       = 0x03;
    unsigned char data_type_    = 0x01;
    unsigned char etx_          = 0x03;
    unsigned char checksum_     = status_ + packet_type_ + data_type_ + length_;

    // 56h command packet; expects 57h reply packet (basic coding)
    char message[]  = {stx_, status_, packet_type_, length_, data_type_, 0x00, 0x00, checksum_, etx_};
    int length      = sizeof (message) / sizeof (message[0]);

    // send request message to serial port
    byteswrite = m_SerialIO.write(message, length);

    msg << "Sent request message to serial port.";
    transmitStatement(msg.str(), INFO);

    #ifndef NDEBUG

    msg << "Total number of bytes sent: " << byteswrite;
    transmitStatement(msg.str(), DEBUG);

    #endif // NDEBUG

    // read response from serial port
    bytesread = m_SerialIO.readNonBlocking((char*) Buffer, 1020);

    msg << "Received reply packet.";
    transmitStatement(msg.str(), INFO);    

    #ifndef NDEBUG

    msg << "Total number of bytes received: " << bytesread;
    transmitStatement(msg.str(), DEBUG);

    for (int i = 0; i < bytesread; i++) {

        msg << "Buffer[" << i << "]: " << Buffer[buffer_index + i];
        transmitStatement(msg.str(), DEBUG);
    }

    #endif // NDEBUG

    // create data structure for the extracted data packets from serial port
    // this is not needed, so it could be removed and only used internally by interpretData function
    // left from dev code... ;)
    Dgps::PacketData incoming_packet;

    // put received data into buffer, extract packets, extract gps data if available
    success = interpretData(Buffer, bytesread, incoming_packet, gps_data);

    return success;
}

/***************************************************/
/***************************************************/
/***************************************************/

/*****************************************************/
/*************** Dgps::interpretData() ***************/
/*****************************************************/

bool Dgps::interpretData(unsigned char *    incoming_data,          // int array from serial.IO
                         int                incoming_data_length,   // count of received bytes
                         Dgps::PacketData   incoming_packet,
                         GpsData            &gps_data) {            // function writes to this data address 
                
    bool                success = false;
    Dgps::PacketData    temp_packet; // = new PacketData;

    if ((ringbuffer_size - ringbuffer_length) >= incoming_data_length) {

        for (int i = 0; i < incoming_data_length; i++) {

            ringbuffer[(ringbuffer_start + ringbuffer_length + 1) % ringbuffer_size] = (char) incoming_data[i];
            ringbuffer_length = (ringbuffer_length + 1) % ringbuffer_size;
        }
    }

    else {

        msg << "Buffer is full! Cannot insert data!";
        transmitStatement(msg.str(), WARNING);
    }

    #ifndef NDEBUG

    msg << "Buffer start: " << ringbuffer_start;
    transmitStatement(msg.str(), DEBUG);

    msg << "Buffer length: " << ringbuffer_length;
    transmitStatement(msg.str(), DEBUG);    

    msg << "Content of ringbuffer: " << ringbuffer_length;
    transmitStatement(msg.str(), DEBUG);
    
    for (int i = 0; i < ringbuffer_size; i++) {

        msg << "Ringbuffer[" << ringbuffer[i] << "]: " << ringbuffer[i];
        transmitStatement(msg.str(), DEBUG);
        // printf("%.2x\t", ringbuffer[i]);
    }

    #endif // NDEBUG

    // find stx, try to get length and match checksum + etx
    for (int y = 0; y < ringbuffer_length; y++) {

        // find stx: (byte 0 == 0x02)
        if (ringbuffer[(ringbuffer_start + y + packet_data_structure.stx_index) % ringbuffer_size] != 0x02) {

            msg << "First byte in received data frame was not stx: " << ringbuffer[(ringbuffer_start + y + packet_data_structure.stx_index) % ringbuffer_size] << "!";
            transmitStatement(msg.str(), WARNING);
            continue;
        }

        else {

            msg << "Found stx in received data frame.";
            transmitStatement(msg.str(), INFO);

            // --- header ---
            temp_packet.stx         = ringbuffer[(ringbuffer_start + y + packet_data_structure.stx_index)         % ringbuffer_size] % 256;
            temp_packet.status      = ringbuffer[(ringbuffer_start + y + packet_data_structure.status_index)      % ringbuffer_size] % 256;
            temp_packet.packet_type = ringbuffer[(ringbuffer_start + y + packet_data_structure.packet_type_index) % ringbuffer_size] % 256;
            temp_packet.length      = ringbuffer[(ringbuffer_start + y + packet_data_structure.length_index)      % ringbuffer_size] % 256;
            
            // --- data part ---
            temp_packet.record_type                 = ringbuffer[(ringbuffer_start + y + packet_data_structure.record_type_index)                 % ringbuffer_size] % 256;
            temp_packet.page_counter                = ringbuffer[(ringbuffer_start + y + packet_data_structure.page_counter_index)                % ringbuffer_size] % 256;
            temp_packet.reply_number                = ringbuffer[(ringbuffer_start + y + packet_data_structure.reply_number_index)                % ringbuffer_size] % 256;
            temp_packet.record_interpretation_flags = ringbuffer[(ringbuffer_start + y + packet_data_structure.record_interpretation_flags_index) % ringbuffer_size] % 256;
            
            // --- --- data bytes --- ---
            temp_packet.data_bytes = new char[temp_packet.length];

            for (int j = 0; j < data_bytes_length(temp_packet.length); j++) {

                temp_packet.data_bytes[j] = ringbuffer[(ringbuffer_start + y + packet_data_structure.data_bytes_index + j) % ringbuffer_size] % 256;
            }

            // --- footer ---
            temp_packet.checksum    = ringbuffer[(ringbuffer_start + y + checksum_index(temp_packet.length))    % ringbuffer_size] % 256;
            temp_packet.etx         = ringbuffer[(ringbuffer_start + y + etx_index(temp_packet.length))         % ringbuffer_size] % 256;

            // verify checksum + etx
            char checksum = 0x00;
            checksum = checksum + (temp_packet.status                       % 256);
            checksum = checksum + (temp_packet.packet_type                  % 256);
            checksum = checksum + (temp_packet.length                       % 256);
            checksum = checksum + (temp_packet.record_type                  % 256);
            checksum = checksum + (temp_packet.page_counter                 % 256);
            checksum = checksum + (temp_packet.reply_number                 % 256);
            checksum = checksum + (temp_packet.record_interpretation_flags  % 256);
            
            // calculate checksum over data bytes
            for (int z = 0; z < data_bytes_length(temp_packet.length); z++) {

                checksum = (checksum + temp_packet.data_bytes[z]);
            }

            // wrap checksum into 1 byte
            checksum = checksum % 256;

            bool error_occured = false;

            if (checksum != temp_packet.checksum) {

                msg << "Checksum mismatch! Calculated checksum: " << checksum << ". Received checksum: " << temp_packet.checksum << ".";
                transmitStatement(msg.str(), WARNING);
                error_occured = true;
            }

            if (temp_packet.etx != 0x03) {

                msg << "Etc was not 0x03. Received etx: " << temp_packet.etx << ".";
                transmitStatement(msg.str(), WARNING);
                error_occured = true;
            }

            // calculate new ringbuffer pointers
            int ringbuffer_old_start = ringbuffer_start;
            
            ringbuffer_start = (ringbuffer_start + y + etx_index(temp_packet.length) + 1) % ringbuffer_size;

            if (ringbuffer_old_start < ringbuffer_start)
                ringbuffer_length = ringbuffer_length - (ringbuffer_start - ringbuffer_old_start);

            if (ringbuffer_old_start >= ringbuffer_start)
                ringbuffer_length = ringbuffer_length - (ringbuffer_start + (ringbuffer_size - ringbuffer_old_start));

            if (!error_occured) {

                incoming_packet = temp_packet;

                // if data is okay -> success = true, then update ringbuffer pointers and write new packet to parameter... see  few lines below
                success = extractGPS(incoming_packet, gps_data);
            }

            if (ringbuffer_length > 0) {

                msg << "Ringbuffer was not empty after reading one packet: " << ringbuffer_length << " bytes left! Calling function to receive data again...";
                transmitStatement(msg.str(), WARNING);
                
                if (ringbuffer_old_start != ringbuffer_start)

                    if (!success) {

                        // call without data to process rest of buffered data
                        success = interpretData(NULL, 0, incoming_packet, gps_data);
                    } 

                    else {

                        interpretData(NULL, 0, incoming_packet, gps_data);
                    }

                    else {

                        msg << "Stopped interpreting remaining buffer to avoid infinite loop.";
                        transmitStatement(msg.str(), WARNING);
                    }
            }

            else {

                msg << "Successfully extracted packet.";
                transmitStatement(msg.str(), INFO);
            }

            return success;
        }
    }

    return success;
}

/**************************************************/
/*************** Dgps::extractGPS() ***************/
/**************************************************/

bool Dgps::extractGPS(Dgps::PacketData &incoming_packet, GpsData &gps_data) {

    if (incoming_packet.packet_type != 0x057) {

        msg << "Received data packet has wrong type! Received packet type: " << incoming_packet.packet_type << ". Expected packet type: 0x057.";
        transmitStatement(msg.str(), WARNING);
        return false;

    }

    else if (incoming_packet.record_type != 0x01) {

        msg << "Received data packet has wrong record type! Received record type: " << incoming_packet.record_type << ". Expected record type: 0x01.";
        transmitStatement(msg.str(), WARNING);
        return false;

    }

    else {

        msg << ("Received data packet is ok.");
        transmitStatement(msg.str(), INFO);

    }

    // get all double fields (8 bytes)

    unsigned char latitude_bytes            [8];
    unsigned char longitude_bytes           [8];
    unsigned char altitude_bytes            [8];
    unsigned char clock_offset_bytes        [8];
    unsigned char frequency_offset_bytes    [8];
    unsigned char pdop_bytes                [8];
    unsigned char latitude_rate_bytes       [8];
    unsigned char longitude_rate_bytes      [8];
    unsigned char altitude_rate_bytes       [8];

    for (int i = 0; i < 8; i++) {

        latitude_bytes          [i] = incoming_packet.data_bytes[i + gps_data_structure.latitude_value_index];
        longitude_bytes         [i] = incoming_packet.data_bytes[i + gps_data_structure.longitude_value_index];
        altitude_bytes          [i] = incoming_packet.data_bytes[i + gps_data_structure.altitude_value_index];
        clock_offset_bytes      [i] = incoming_packet.data_bytes[i + gps_data_structure.clock_offset_index];
        frequency_offset_bytes  [i] = incoming_packet.data_bytes[i + gps_data_structure.frequency_offset_index];
        pdop_bytes              [i] = incoming_packet.data_bytes[i + gps_data_structure.pdop_index];
        latitude_rate_bytes     [i] = incoming_packet.data_bytes[i + gps_data_structure.latitude_rate_index];
        longitude_rate_bytes    [i] = incoming_packet.data_bytes[i + gps_data_structure.longitude_rate_index];
        altitude_rate_bytes     [i] = incoming_packet.data_bytes[i + gps_data_structure.altitude_rate_index];

    }

    gps_data.latitude_value     = getDOUBLE(latitude_bytes) * gps_data_structure.semi_circle_factor;
    gps_data.longitude_value    = getDOUBLE(longitude_bytes) * gps_data_structure.semi_circle_factor;
    gps_data.altitude_value     = getDOUBLE(altitude_bytes);
    gps_data.clock_offset       = getDOUBLE(clock_offset_bytes);
    gps_data.frequency_offset   = getDOUBLE(frequency_offset_bytes);
    gps_data.pdop               = getDOUBLE(pdop_bytes);
    gps_data.latitude_rate      = getDOUBLE(latitude_rate_bytes);
    gps_data.longitude_rate     = getDOUBLE(longitude_rate_bytes);
    gps_data.altitude_rate      = getDOUBLE(altitude_rate_bytes);

    // get all long fields (4 bytes)

    unsigned char gps_msec_of_week_bytes[4];

    for (int i = 0; i < 4; i++) {

        gps_msec_of_week_bytes[i]   = incoming_packet.data_bytes[i + gps_data_structure.gps_msec_of_week_index];

    }

    gps_data.gps_msec_of_week = getLONG(gps_msec_of_week_bytes);

    // get all char fields (1 byte)

    gps_data.position_flags = incoming_packet.data_bytes[gps_data_structure.position_flags_index];
    gps_data.number_of_SVs  = incoming_packet.data_bytes[gps_data_structure.number_of_SVs_index];
    //gps_data.channel_number = incoming_packet.data_bytes[gps_data_structure.channel_number_index];
    //gps_data.prn            = incoming_packet.data_bytes[gps_data_structure.prn_index];

    // extract satellite number
    // generate arrays of this size for channel_number prn_index
    // extract value and generate arrays for the fields: channel_number_index and prn_index

    #ifndef NDEBUG

    msg << "Calculated longitude: " << longitude_value;
    transmitStatement(msg.str(), DEBUG);

    msg << "Calculated latitude: "  << latitude_value;
    transmitStatement(msg.str(), DEBUG);

    msg << "Calculated altitude: "  << altitude_value;
    transmitStatement(msg.str(), DEBUG);

    msg << "Clock_offset: "         << clock_offset;
    transmitStatement(msg.str(), DEBUG);

    msg << "Frequency_offset: "     << frequency_offset;
    transmitStatement(msg.str(), DEBUG);

    msg << "Pdop: "                 << pdop;
    transmitStatement(msg.str(), DEBUG);

    #endif // NDEBUG

    // write extracted values to GpsData struct; this is the returned data.


    // implement value checking if needed, e.g. value of longitude and latitude between 0 and 180; ...
    // return false if any check fails

    return true;
}

/**************************************************/
/**************************************************/
/**************************************************/

/************************************************/
/*************** helper functions ***************/
/************************************************/

// function to reorder incoming bits
bool * Dgps::invertBitOrder(bool * bits, Dgps::DataType data_type, bool invertBitsPerByte = true, bool invertByteOrder = false) {

bool * reversed_CHAR    = new bool[ 8];
bool * reversed_SHORT   = new bool[16];
bool * reversed_LONG    = new bool[32];
bool * reversed_FLOAT   = new bool[32];
bool * reversed_DOUBLE  = new bool[64];

    switch (data_type) {

        case CHAR:

            for (int i = 0; i < 8; i++) {

            if      (invertBitsPerByte)   reversed_SHORT[i] = bits[7-i];
            else if (!invertBitsPerByte)  reversed_SHORT[i] = bits[i  ];

            }

            return reversed_CHAR;

        case SHORT:

            for (int k = 0; k < 2; k++) {

                for (int i = 0; i < 8; i++) {

                if      (!invertByteOrder   && invertBitsPerByte)   reversed_SHORT[k * 8 + i] = bits[(k)      * 8 + (7 - i)   ];
                else if (!invertByteOrder   && !invertBitsPerByte)  reversed_SHORT[k * 8 + i] = bits[(k)      * 8 + (i)       ];
                else if (invertByteOrder    && invertBitsPerByte)   reversed_SHORT[k * 8 + i] = bits[(1 - k)  * 8 + (7 - i)   ];
                else if (invertByteOrder    && !invertBitsPerByte)  reversed_SHORT[k * 8 + i] = bits[(1 - k)  * 8 + (i)       ];
        
                }
    
            }

            return reversed_SHORT;

        case LONG:
         
            for (int k = 0; k < 4; k++) {

                for (int i = 0; i < 8; i++) {

                if      (!invertByteOrder   && invertBitsPerByte)   reversed_LONG[k * 8 + i] = bits[(k)      * 8 + (7 - i)   ];
                else if (!invertByteOrder   && !invertBitsPerByte)  reversed_LONG[k * 8 + i] = bits[(k)      * 8 + (i)       ];
                else if (invertByteOrder    && invertBitsPerByte)   reversed_LONG[k * 8 + i] = bits[(3 - k)  * 8 + (7 - i)   ];
                else if (invertByteOrder    && !invertBitsPerByte)  reversed_LONG[k * 8 + i] = bits[(3 - k)  * 8 + (i)       ];
        
                }
    
            }

            return reversed_LONG;

        case FLOAT:
         
            for (int k = 0; k < 4; k++) {

                for (int i = 0; i < 8; i++) {

                if      (!invertByteOrder   && invertBitsPerByte)   reversed_LONG[k * 8 + i] = bits[(k)      * 8 + (7 - i)   ];
                else if (!invertByteOrder   && !invertBitsPerByte)  reversed_LONG[k * 8 + i] = bits[(k)      * 8 + (i)       ];
                else if (invertByteOrder    && invertBitsPerByte)   reversed_LONG[k * 8 + i] = bits[(3 - k)  * 8 + (7 - i)   ];
                else if (invertByteOrder    && !invertBitsPerByte)  reversed_LONG[k * 8 + i] = bits[(3 - k)  * 8 + (i)       ];
        
                }
    
            }

            return reversed_FLOAT;

        case DOUBLE:

            for (int k = 0; k < 8; k++) {

                for (int i = 0; i < 8; i++) {

                if      (!invertByteOrder   && invertBitsPerByte)   reversed_DOUBLE[k * 8 + i] = bits[(k)      * 8 + (7 - i)   ];
                else if (!invertByteOrder   && !invertBitsPerByte)  reversed_DOUBLE[k * 8 + i] = bits[(k)      * 8 + (i)       ];
                else if (invertByteOrder    && invertBitsPerByte)   reversed_DOUBLE[k * 8 + i] = bits[(7 - k)  * 8 + (7 - i)   ];
                else if (invertByteOrder    && !invertBitsPerByte)  reversed_DOUBLE[k * 8 + i] = bits[(7 - k)  * 8 + (i)       ];
        
                }
    
            }

            return reversed_DOUBLE;

    }

}

long Dgps::getLONG(unsigned char * bytes) {

    bool bits[32] = {0};

    for (int k = 0; k < 4; k++) {

        for (int i = 0; i < 8; i++) {

            bits[((k * 4) + i)] = 0 != (bytes[k] & (1 << i));

        }

    }

    bool * resulting_bits;
    resulting_bits = invertBitOrder(bits, LONG);

    for (int i = 0; i < 32; i++) {

        bits[i] = resulting_bits[i];

    }

    long value = 0;

    for (int i = 0; i < 32; i++) {

        value = value + bits[i] * pow(2, 31 - i);

    }

    return value;

}

/* function to extract IEEE double precision number values from an 8-byte array
 * (8 Bit per Byte; array size is expected to be 8; ==> 64 Bit)
 *
 * bias is 1023 as default for standard numbers
 * ...(IEEE example -25.25 reads ok with invertedBitsPerByte and bias 1023)
 *
 * example data for -25.25:
 *
 * (8 bytes, last 5 bytes are 0x00, so written at initialization of array...)
 *
 *    unsigned char test_minus_25_25[8] = {0x00};
 *    test_minus_25_25[0]               = 0xc0;
 *    test_minus_25_25[1]               = 0x39;
 *    test_minus_25_25[2]               = 0x40;
 *
 * int exponent_bias = 1023;
 * hopefully working as default parameter... */
double Dgps::getDOUBLE(unsigned char * bytes, int exponent_bias = 1023) {

    // init with zero/false
    bool bits[64] = {false};

    // get bits of DOUBLE
    for (int k = 0; k < 8; k++) {

        for (int i = 0; i < 8; i++) {

            bits[((k * 8) + i)] = 0 != (bytes[k] & (1 << i));

        }

    }

    bool * resulting_bits;
    resulting_bits = invertBitOrder(bits, DOUBLE);
    for (int i = 0; i < 64; i++) bits[i] = resulting_bits[i];

    #ifndef NDEBUG

    msg << "Bits: ";
    transmitStatement(msg.str(), DEBUG);

    for (int i = 0; i < 64; i++) {

        msg << bits[i];
        transmitStatement(msg.str(), DEBUG);

    }

    std::cout << "\n";

    #endif // NDEBUG

    // calculate sign, fraction and exponent
    int sign_bit = bits[0];
    double exponent = 0;
    double fraction = 0;
    for (int i = 0; i < 11; i++) {

        exponent = exponent + bits[i + 1] * pow(2, 10 - i);

    }

    for (int i = 0; i < 52; i++) {

        fraction = fraction + bits[i + 12] * pow(0.5, i + 1);

    }

    // fraction is value between 1.0 and 2.0.. see IEEE spec for details
    fraction += 1;

    #ifndef NDEBUG

    printf("\nExponent_lat: %f\n", exponent_lat-exponent_bias);
    printf("\nfraction_lat: %f\n", fraction_lat);

    #endif // NDEBUG

    int sign_lat = 1;
    if (sign_bit == 1) sign_lat = -1;

    // calculate number value from extracted values
    double double_value = sign_lat * (fraction * pow(2, exponent - exponent_bias));

    #ifndef NDEBUG

    printf("\nCalculated value: %f\n", latitude_value);

    #endif // NDEBUG

    return double_value;

}

int Dgps::data_bytes_length(int length_value) {

    return length_value - 4;

}

int Dgps::checksum_index(int length_value) {

    return 4 + length_value;

}

int checksum_length = 1;

// call with value of byte #3 (length-field)
int Dgps::etx_index(int length_value) {

    return 4 + length_value + 1;

}

int etx_length = 1;

/************************************************/
/************************************************/
/************************************************/

/*********************************************************/
/*********************************************************/
/*********************************************************/