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
* TODO:
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

// constructor
Dgps::Dgps() {

    msg << "Initializing...";
    transmitStatement(INFO);

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // enable/disable console output;
    cout_enabled = false;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    ringbuffer_size         = 4096 * 4; // ! ==> must be equal to number of elements in ringbuffer array (ringbuffer[...], see Dgps.h);

    for (int i = 0; i < ringbuffer_size; i++) {
        ringbuffer[i] = 0;
    }

    ringbuffer_start        = 0;
    ringbuffer_length       = 0;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // POSITION RECORD PACKET;
    // the variables below represent the starting positions of associated data bytes in a position record packet;
    // base unit is 1 char = 1 byte = 8 bit;
    // position record packet (packet type: 57h, see Trimble BD982 GNSS Receiver manual, p. 139);

    // packet_data_structure initialization;

    packet_data_structure.stx_index                         = 0; // index: starting position in incoming data frame;
    packet_data_structure.status_index                      = 1;
    packet_data_structure.packet_type_index                 = 2;
    packet_data_structure.length_index                      = 3;
    packet_data_structure.record_type_index                 = 4;
    packet_data_structure.page_counter_index                = 5; // byte splitted in two parts! its <page> of <total>, each 4 bit;
    packet_data_structure.reply_number_index                = 6;
    packet_data_structure.record_interpretation_flags_index = 7;     
    packet_data_structure.data_bytes_index                  = 8;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // gps_data_structure initialization;

    // POSITION RECORD PACKET - DATA PART;
    // the variables below represent the starting positions of associated data bytes in a position record packet data field;
    // the index is relative to begin of the data part of a position record packet, 
    // so it is byte # index+8  of position record packet (stx = 0);
    // position record packet (packet type: 57h, see Trimble BD982 GNSS Receiver manual, p. 139);

    gps_data_structure.latitude_value_index     = 0;
    gps_data_structure.longitude_value_index    = 8;
    gps_data_structure.altitude_value_index     = 16;
    gps_data_structure.clock_offset_index       = 24;
    gps_data_structure.frequency_offset_index   = 32;
    gps_data_structure.pdop_index               = 40;
    gps_data_structure.latitude_rate_index      = 48;
    gps_data_structure.longitude_rate_index     = 56;
    gps_data_structure.altitude_rate_index      = 64;
    gps_data_structure.gps_msec_of_week_index   = 72;
    gps_data_structure.position_flags_index     = 76;
    gps_data_structure.number_of_SVs_index      = 77;
    gps_data_structure.channel_number_index     = 78;
    gps_data_structure.prn_index                = 79;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // helper variable; used to find the meaning of semi-circles in this case...
    // (it's just 0-180 normalized to 0.0-1.0)
    semi_circle_factor = 180.0;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    msg << "Ready.";
    transmitStatement(INFO);

}

/**************************************************/
/**************************************************/
/**************************************************/

// destructor
Dgps::~Dgps() {

    m_SerialIO.close();

}

/**************************************************/
/**************************************************/
/**************************************************/

// takes diagnostic statements and stores them in diagnostic_array;
// if diagnostic_array holds more than 100 elements, the oldest stored element will get erased;
void Dgps::transmitStatement(DiagnosticFlag flag) {

    if (cout_enabled) std::cout << "\n";

    switch(flag) {

        case DEBUG:

            msg_tagged  << "DGPS [DEBUG]: "     << msg.str();
            if (cout_enabled) std::cout         << msg_tagged.str();
            break;

        case INFO:

            msg_tagged  << "DGPS [INFO]: "      << msg.str();
            if (cout_enabled) std::cout         << msg_tagged.str();
            break;

        case WARNING:

            msg_tagged  << "DGPS [WARNING]: "   << msg.str();
            if (cout_enabled) std::cout         << msg_tagged.str();
            break;

        case ERROR:

            msg_tagged  << "DGPS [ERROR]: "     << msg.str();
            if (cout_enabled) std::cout         << msg_tagged.str();
            break;

        default:

            if (cout_enabled) std::cout         << msg.str();
            break;
    
    }

    diagnostic_statement.diagnostic_message = msg_tagged.str();
    diagnostic_statement.diagnostic_flag    = flag;

    if (diagnostic_array.size() >= 100) {

        std::vector<DiagnosticStatement>::iterator it = diagnostic_array.begin();

        diagnostic_array.erase(it);
    
    }

    diagnostic_array.push_back(diagnostic_statement);

    msg.str("");
    msg_tagged.str("");

}

/**************************************************/
/**************************************************/
/**************************************************/

// establishes serial connection
bool Dgps::open(const char * pcPort, int iBaudRate) {

    msg << "Establishing connection to DGPS device...";
    transmitStatement(INFO);

    msg << "Port: " << pcPort;
    transmitStatement(INFO);

    msg << "Baud rate: " << iBaudRate;
    transmitStatement(INFO);

    m_SerialIO.setBaudRate  (iBaudRate);
    m_SerialIO.setDeviceName(pcPort);

    int serial_open = m_SerialIO.open();

    if (serial_open == 0) {

        m_SerialIO.purge();

        msg << "Successfully connected.";
        transmitStatement(INFO);

        return true;

    }

    else {

        msg << "Establishing connection failed. Device is not available on given port.";
        transmitStatement(ERROR);
        return false;

    }

}

/**************************************************/
/**************************************************/
/**************************************************/

// tests the communications link by sending protocol request "ENQ" (05h);
// expects success response "ACK" (06h); 
// see Trimble BD982 GNSS receiver manual, p. 65;
bool Dgps::checkConnection() {

    msg << "Testing the communications link...";
    transmitStatement(INFO);

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // creating test connection link command "ENQ" (05h)
    char    message[]   = {0x05};
    int     length      = sizeof (message) / sizeof (message[0]);

    msg << "Sending test command ENQ (05h)...";
    transmitStatement(INFO);

    int bytes_sent  = 0;
    bytes_sent      = m_SerialIO.write(message, length);

    if (bytes_sent > 0) {

        msg << "Successfully sent test command. Waiting for response...";
        transmitStatement(INFO);

    }

    else {

        msg << "Could not send test command.";
        transmitStatement(ERROR);

        msg << "Testing the communications link failed!";
        transmitStatement(ERROR);

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    msg << "Requesting test response...";
    transmitStatement(INFO);

    // creating buffer for reply packet
    unsigned char buffer[1024] = {0};

    int bytes_received  = 0;
    bytes_received      = m_SerialIO.readNonBlocking((char *) buffer, 1020);

    if (bytes_received > 0) {

        msg << "Received test response.";
        transmitStatement(INFO);

        if (buffer[0] == 6) {

            msg << "Test response packet is \"ACK\" (06h) as expected.";
            transmitStatement(INFO);

            return true;

        }

        else if (buffer[0] == 15) {

            msg << "Test response packet is \"NAK\" (15h). Device is not yet ready.";
            transmitStatement(WARNING);

            return false;

        }

        else {

            msg << "Unknown test reponse packet. Device is not yet ready.";
            transmitStatement(WARNING);

            return false;

        }

    }

    else {

        msg << "Device does not respond.";
        transmitStatement(ERROR);
        
        return false;

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

}

/*******************************************************/
/*******************************************************/
/*******************************************************/

bool Dgps::getDgpsData() {

    msg << "Requesting GPS data...";
    transmitStatement(INFO);

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // generation of request command packet (56h);
    // expects 57h reply packet (basic coding);
    // see Trimble BD982 GNSS receiver manual, p. 73;
    unsigned char stx_          = 0x02;
    unsigned char status_       = 0x00;
    unsigned char packet_type_  = 0x56;
    unsigned char length_       = 0x03;
    unsigned char data_type_    = 0x01;
    unsigned char etx_          = 0x03;
    unsigned char checksum_     = status_ + packet_type_ + data_type_ + length_;

    char message[]  = {stx_, status_, packet_type_, length_, data_type_, 0x00, 0x00, checksum_, etx_};
    int length      = sizeof (message) / sizeof (message[0]);

    msg << "Sending request command...";
    transmitStatement(INFO);

    int bytes_sent  = 0;
    bytes_sent      = m_SerialIO.write(message, length);

    if (bytes_sent > 0) {

        msg << "Successfully sent request command. Waiting for response...";
        transmitStatement(INFO);

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // creating buffer for reply packet
    unsigned char buffer[1024]  = {0};
    
    // reading response from serial port
    int bytes_received  = 0;
    bytes_received      = m_SerialIO.readNonBlocking((char*) buffer, 1020);

    if (bytes_received > 0) {

        msg << "Received reply packet.";
        transmitStatement(INFO);

        if (buffer[0] == 15) {

            msg << "Response packet is \"NAK\" (15h). Device cannot fullfill request.";
            transmitStatement(WARNING);

            return false;

        }

        else {

            // put received data into buffer, extract packets, extract gps data if available
            return interpretData(buffer, bytes_received);

        }

    }

    else {

        msg << "Device does not respond.";
        transmitStatement(ERROR);

        return false;

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

}

/**************************************************/
/**************************************************/
/**************************************************/

bool Dgps::interpretData(unsigned char *    incoming_data,          // int array from serial.IO
                         int                incoming_data_length) { // count of received bytes            // function writes to this data address 

    // preparing for output of hex numbers
    msg << showbase         // show the 0x prefix
        << internal         // fill between the prefix and the number
        << setfill('0');    // fill with 0s

    bool                success         = false;
    Dgps::PacketData    temp_packet;             // = new PacketData;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    if ((ringbuffer_size - ringbuffer_length) >= incoming_data_length) {

        msg << "Putting new data into buffer...";
        transmitStatement(INFO);

        for (int i = 0; i < incoming_data_length; i++) {

            ringbuffer[(ringbuffer_start + ringbuffer_length + 1) % ringbuffer_size] = (char) incoming_data[i];
            ringbuffer_length = (ringbuffer_length + 1) % ringbuffer_size;

        }

    }

    else {

        msg << "Buffer is full! Cannot insert new data!";
        transmitStatement(WARNING);

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    msg << "Interpreting received data...";
    transmitStatement(INFO);

    // find stx, try to get length and match checksum + etx
    for (int y = 0; y < ringbuffer_length; y++) {

        // find stx: (byte 0 == 0x02)
        if (ringbuffer[(ringbuffer_start + y + packet_data_structure.stx_index)] != 0x02) {

            msg << "First byte was not stx!"; // << ringbuffer[(ringbuffer_start + y + packet_data_structure.stx_index) % ringbuffer_size] << "!";
            transmitStatement(WARNING);

        }

        else {

            msg << "Found stx.";
            transmitStatement(INFO);

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

                msg << "Checksum mismatch!";
                transmitStatement(WARNING);
                msg << "Calculated checksum: "  << std::hex << setw(4) << int(checksum);
                transmitStatement(WARNING);
                msg << "Received checksum: "    << std::hex << setw(4) << int(temp_packet.checksum);
                transmitStatement(WARNING);

                error_occured = true;

            }

            if (temp_packet.etx != 0x03) {

                msg << "Etx was not 0x03!";
                transmitStatement(WARNING);
                msg << "Received etx: " << std::hex << setw(4) << int(temp_packet.etx);
                transmitStatement(WARNING);
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
                success = extractGPS();
            }

            if (ringbuffer_length > 0) {

                msg << "Ringbuffer was not empty after reading one packet: " << ringbuffer_length << " bytes left! Calling function to receive data again...";
                transmitStatement(WARNING);
                
                if (ringbuffer_old_start != ringbuffer_start)

                    if (!success) {

                        // call without data to process rest of buffered data
                        success = interpretData(NULL, 0);
                    } 

                    else {

                        interpretData(NULL, 0);
                    }

                    else {

                        msg << "Stopped interpreting remaining buffer to avoid infinite loop.";
                        transmitStatement(WARNING);
                    }
            }

            else {

                //msg << "Successfully extracted packet.";
                //transmitStatement(INFO);
            }

            return success;
        }

    }

    return success;
}

/**************************************************/
/**************************************************/
/**************************************************/

bool Dgps::extractGPS() {

    // preparing for output of hex numbers
    msg << showbase         // show the 0x prefix
        << internal         // fill between the prefix and the number
        << setfill('0');    // fill with 0s

    msg << "Extracting DGPS data...";
    transmitStatement(INFO);

    /**************************************************/
    /**************************************************/
    /**************************************************/

    msg << "Checking types...";
    transmitStatement(INFO);

    if (incoming_packet.packet_type != 0x57) {

        msg << "Received reply packet has wrong type!";
        transmitStatement(WARNING);
        msg << "Received reply packet of type: " << std::hex << setw(4) << int(incoming_packet.packet_type);
        transmitStatement(WARNING);
        msg << "Expected reply packet of type: 0x57";
        transmitStatement(WARNING);

        return false;

    }

    else if (incoming_packet.record_type != 0x01) {

        msg << "Received reply packet has wrong record type!";
        transmitStatement(WARNING);
        msg << "Received reply packet of record type: " << std::hex << setw(4) << int(incoming_packet.record_type);
        transmitStatement(WARNING);
        msg << "Expected reply packet of record type: 0x01";
        transmitStatement(WARNING);

        return false;

    }

    else {

        msg << ("Received data packet is ok.");
        transmitStatement(INFO);

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // getting all fields of type DOUBLE (8 bytes)

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

    // getDOUBLE includes inverting of bit order
    gps_data.latitude_value     = getDOUBLE(latitude_bytes) * semi_circle_factor;
    gps_data.longitude_value    = getDOUBLE(longitude_bytes) * semi_circle_factor;
    gps_data.altitude_value     = getDOUBLE(altitude_bytes);
    gps_data.clock_offset       = getDOUBLE(clock_offset_bytes);
    gps_data.frequency_offset   = getDOUBLE(frequency_offset_bytes);
    gps_data.pdop               = getDOUBLE(pdop_bytes);
    gps_data.latitude_rate      = getDOUBLE(latitude_rate_bytes);
    gps_data.longitude_rate     = getDOUBLE(longitude_rate_bytes);
    gps_data.altitude_rate      = getDOUBLE(altitude_rate_bytes);

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // getting all fields of type LONG (4 bytes)

    unsigned char gps_msec_of_week_bytes[4];

    for (int i = 0; i < 4; i++) {

        gps_msec_of_week_bytes[i]   = incoming_packet.data_bytes[i + gps_data_structure.gps_msec_of_week_index];

    }

    // getLONG includes inverting of bit order
    gps_data.gps_msec_of_week = getLONG(gps_msec_of_week_bytes);

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // getting all fields of type CHAR (1 byte)

    gps_data.position_flags = getCHAR(incoming_packet.data_bytes[gps_data_structure.position_flags_index]);
    gps_data.number_of_SVs  = getCHAR(incoming_packet.data_bytes[gps_data_structure.number_of_SVs_index]);

    // first getting rid of old values
    gps_data.channel_numbers.clear();
    gps_data.prn.clear();

    // gathering new values according to number of satellites
    for (int i = 0; i < gps_data.number_of_SVs; i++) {

        gps_data.channel_numbers.push_back  (getCHAR(incoming_packet.data_bytes[gps_data_structure.channel_number_index + i*2]));
        gps_data.prn.push_back              (getCHAR(incoming_packet.data_bytes[gps_data_structure.prn_index            + i*2]));

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    #ifndef NDEBUG

    msg << "Calculated longitude: " << longitude_value;
    transmitStatement(DEBUG);

    msg << "Calculated latitude: "  << latitude_value;
    transmitStatement(DEBUG);

    msg << "Calculated altitude: "  << altitude_value;
    transmitStatement(DEBUG);

    msg << "Clock_offset: "         << clock_offset;
    transmitStatement(DEBUG);

    msg << "Frequency_offset: "     << frequency_offset;
    transmitStatement(DEBUG);

    msg << "Pdop: "                 << pdop;
    transmitStatement(DEBUG);

    #endif // NDEBUG

    /**************************************************/
    /**************************************************/
    /**************************************************/

    return true;

    // remaining TODO's in withing this function:
    // implement value checking if needed, e.g. value of longitude and latitude between 0 and 180; ...
    // return false if any check fails

}

/**************************************************/
/**************************************************/
/**************************************************/

// function to reorder incoming bits
bool * Dgps::invertBitOrder(bool * bits, Dgps::DataType data_type, bool invertBitsPerByte, bool invertByteOrder) {

bool * reversed_8_bit   = new bool[ 8];
bool * reversed_16_bit  = new bool[16];
bool * reversed_32_bit  = new bool[32];
bool * reversed_64_bit  = new bool[64];

    switch (data_type) {

        case CHAR:

            for (int i = 0; i < 8; i++) {

            if      (invertBitsPerByte)   reversed_8_bit[i] = bits[7-i];
            else if (!invertBitsPerByte)  reversed_8_bit[i] = bits[i  ];

            }

            return reversed_8_bit;

        case SHORT:

            for (int k = 0; k < 2; k++) {

                for (int i = 0; i < 8; i++) {

                if      (!invertByteOrder   && invertBitsPerByte)   reversed_16_bit[k * 8 + i] = bits[(k)      * 8 + (7 - i)   ];
                else if (!invertByteOrder   && !invertBitsPerByte)  reversed_16_bit[k * 8 + i] = bits[(k)      * 8 + (i)       ];
                else if (invertByteOrder    && invertBitsPerByte)   reversed_16_bit[k * 8 + i] = bits[(1 - k)  * 8 + (7 - i)   ];
                else if (invertByteOrder    && !invertBitsPerByte)  reversed_16_bit[k * 8 + i] = bits[(1 - k)  * 8 + (i)       ];
        
                }
    
            }

            return reversed_16_bit;

        case LONG:
        case FLOAT:
         
            for (int k = 0; k < 4; k++) {

                for (int i = 0; i < 8; i++) {

                if      (!invertByteOrder   && invertBitsPerByte)   reversed_32_bit[k * 8 + i] = bits[(k)      * 8 + (7 - i)   ];
                else if (!invertByteOrder   && !invertBitsPerByte)  reversed_32_bit[k * 8 + i] = bits[(k)      * 8 + (i)       ];
                else if (invertByteOrder    && invertBitsPerByte)   reversed_32_bit[k * 8 + i] = bits[(3 - k)  * 8 + (7 - i)   ];
                else if (invertByteOrder    && !invertBitsPerByte)  reversed_32_bit[k * 8 + i] = bits[(3 - k)  * 8 + (i)       ];
        
                }
    
            }

            return reversed_32_bit;

        case DOUBLE:

            for (int k = 0; k < 8; k++) {

                for (int i = 0; i < 8; i++) {

                if      (!invertByteOrder   && invertBitsPerByte)   reversed_64_bit[k * 8 + i] = bits[(k)      * 8 + (7 - i)   ];
                else if (!invertByteOrder   && !invertBitsPerByte)  reversed_64_bit[k * 8 + i] = bits[(k)      * 8 + (i)       ];
                else if (invertByteOrder    && invertBitsPerByte)   reversed_64_bit[k * 8 + i] = bits[(7 - k)  * 8 + (7 - i)   ];
                else if (invertByteOrder    && !invertBitsPerByte)  reversed_64_bit[k * 8 + i] = bits[(7 - k)  * 8 + (i)       ];
        
                }
    
            }

            return reversed_64_bit;

        default:

            return NULL;

    }

}

/**************************************************/
/**************************************************/
/**************************************************/

char Dgps::getCHAR(unsigned char byte) {

    bool bits[8] = {0};

    for (int i = 0; i < 8; i++) {

        bits[i] = 0 != (byte & (1 << i));

    }

    bool * resulting_bits;
    resulting_bits = invertBitOrder(bits, CHAR);

    for (int i = 0; i < 8; i++) {

        bits[i] = resulting_bits[i];

    }

    char value = 0;

    for (int i = 0; i < 8; i++) {

        value = value + bits[i] * pow(2, 7 - i);

    }

    return value;

}

// function to extract numbers of data type LONG INTEGER from an 4-byte array;
// 8 bits per byte; array size is expected to be 4; ==> 32 bit;
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

/**************************************************/
/**************************************************/
/**************************************************/

/* function to extract IEEE double precision number values from an 8-byte array
 * (8 bits per byte; array size is expected to be 8; ==> 64 Bit)
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
double Dgps::getDOUBLE(unsigned char * bytes, int exponent_bias) {

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

    // fraction is value between 1.0 and 2.0... see IEEE spec for details
    fraction += 1;

    int sign_lat = 1;
    if (sign_bit == 1) sign_lat = -1;

    // calculate number value from extracted values
    double double_value = sign_lat * (fraction * pow(2, exponent - exponent_bias));

    return double_value;

}

/**************************************************/
/**************************************************/
/**************************************************/

int Dgps::data_bytes_length(int length_value) {

    return length_value - 4;

}

/**************************************************/
/**************************************************/
/**************************************************/

int Dgps::checksum_index(int length_value) {

    return 4 + length_value;

}

/**************************************************/
/**************************************************/
/**************************************************/

// call with value of byte #3 (length-field)
int Dgps::etx_index(int length_value) {

    return 4 + length_value + 1;

}

/**************************************************/
/**************************************************/
/**************************************************/