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
Dgps::Dgps() {}

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

    diagnostic_statement.diagnostic_message = msg.str();
    diagnostic_statement.diagnostic_flag    = flag;

    if (diagnostic_array.size() >= 100) {

        std::vector<DiagnosticStatement>::iterator it = diagnostic_array.begin();

        diagnostic_array.erase(it);
    
    }

    diagnostic_array.push_back(diagnostic_statement);

    // this expression clears the stringstream instance after each transmit process;
    // if it doesn't get cleared, every new diagnostic statement will get attached
    // to the existing ones within the object, so that it grows and grows...;
    msg.str("");

}

/**************************************************/
/**************************************************/
/**************************************************/

// establishes serial connection
bool Dgps::open(const char * pcPort, int iBaudRate) {

    msg << "Establishing serial connection to GPS device...";
    transmitStatement(INFO);

    msg << "Port: " << pcPort;
    transmitStatement(INFO);

    msg << "Baud rate: " << iBaudRate;
    transmitStatement(INFO);

    // gather parameters;
    m_SerialIO.setBaudRate  (iBaudRate);
    m_SerialIO.setDeviceName(pcPort);

    // open port;
    int port_opened = m_SerialIO.open();

    if (port_opened != 0) {

        msg << "Failed to establish connection. Device is not available on given port.";
        transmitStatement(ERROR);

        return false;

    }

    else {

        m_SerialIO.purge();

        msg << "Connection established.";
        transmitStatement(INFO);

        return true;
        
    }

}

/**************************************************/
/**************************************************/
/**************************************************/

// checks the communication link by transmission of protocol request "ENQ" (05h);
// expected result is either "ACK" (06h) or "NAK" (05h); 
// see Trimble BD982 GNSS receiver manual, p. 65;
bool Dgps::checkConnection() {

    msg << "Testing the communication link...";
    transmitStatement(INFO);

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // test command "ENQ" (05h)
    char message[]      = {0x05};
    int message_size    = sizeof(message) / sizeof(message[0]);

    int bytes_sent  = 0;
    bytes_sent      = m_SerialIO.write(message, message_size); // function returns number of transmitted bytes;

    if (!(bytes_sent > 0)) {

        msg << "Could not send test command.";
        transmitStatement(ERROR);

        msg << "Communication link check failed. Device is not available.";
        transmitStatement(ERROR);

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // test result;
    unsigned char result[] = {0};

    int bytes_received  = 0;
    bytes_received      = m_SerialIO.readNonBlocking((char *) result, 1); // function returns number of received bytes;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // analyzing received data;

    // checking if there is a response at all;
    if (!(bytes_received > 0)) {

        msg << "Device does not respond.";
        transmitStatement(ERROR);

        return false;

    }

    // checking for possible response "NAK" (15h);
    if (!(result[0] != 0x15)) {

        msg << "Test result is \"NAK\" (15h). Device is not ready yet.";
        transmitStatement(WARNING);

        return false;

    }

    // checking for expected result "ACK" (06h);
    if (!(result[0] != 0x06)) {

        msg << "Test result is \"ACK\" (06h) as expected.";
        transmitStatement(INFO);

        msg << "Testing the communications link succeeded. Device is available.";
        transmitStatement(INFO);

        return true;

    }

    // checking for unexpected reponse;
    else {

        msg << "Unknown test reponse packet. Device is not ready yet.";
        transmitStatement(WARNING);

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

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // generation of request command packet "GETRAW" (56h);
    // expected reply packet is "RAWDATA" (57h); 
    // see Trimble BD982 GNSS Receiver Manual, p. 73/132;

    unsigned char stx           = 0x02; // head
    unsigned char status        = 0x00;
    unsigned char packet_type   = 0x56; // this command packet is of type "GETRAW" (56h);
    unsigned char length        = 0x03; // length of data part;
    unsigned char data_type     = 0x01; // raw data type:   position record --> type    = 00000001 (binary);
    unsigned char flags         = 0x00; // raw data format: concise         --> flags   = 00000001 (binary);
    unsigned char reserved      = 0x00; // tail
    unsigned char checksum;
    unsigned char etx           = 0x03;

    checksum = (status + packet_type + length + data_type + flags + reserved) % 256;

    char message[]      = {stx, status, packet_type, length, data_type, flags, reserved, checksum, etx};
    int message_size    = sizeof(message) / sizeof(message[0]);

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // transmission of request command
    int bytes_sent  = 0;
    bytes_sent      = m_SerialIO.write(message, message_size); // function returns number of bytes which have been sent;

    if (bytes_sent != message_size) {

        msg << "Failed to transmit request command.";
        transmitStatement(WARNING);

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // buffer for expected reply packet;
    // the size of the buffer is equal to the maximum size a "RAWDATA" (57h) reply packet 
    // of type 57h position record can get;
    // maximum_size = 8 bytes + 78 bytes + 2 * N bytes + 2 bytes = 112  bytes; (where N, the number of used sattelites, is 12);
    // minimum_size = 8 bytes + 78 bytes + 2 * N bytes + 2 bytes = 88   bytes; (where N, the number of used sattelites, is 0);
    // see Trimble BD982 GNSS Receiver Manual, p. 139;
    unsigned char buffer[112+16] = {0};
    
    // reading possible response from serial port;
    int bytes_received  = 0;
    bytes_received      = m_SerialIO.readNonBlocking((char*) buffer, (112+16)); // function returns number of bytes which have been received;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    int debug_bytes_received = bytes_received - 16;

    unsigned char debug_buffer[112] = {0};

    int j = 16;

    for (int i = 0; i < 112; i++) {

        debug_buffer[i] = buffer[j];

        j++;

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    #ifndef NDEBUG

    std::cout << endl << endl << "\t\t\t------------------------------" << endl << endl;

    std::cout << "bytes_received:\t\t" << bytes_received << endl;
    std::cout << "debug_bytes_received:\t" << debug_bytes_received << endl;


    int k = 16;

    for (int i = 0; i < (112+16); i++) {

        std::cout << "\nbuffer[" << i << "]:\t";
        printf("%x", buffer[i]);

        if (!(i >= 112)) {

            std::cout << "\tbuffer[" << k << "]:\t";
            printf("%x", buffer[k]);

            std::cout << "\tdebug_buffer[" << i <<"]:\t";
            printf("%x", debug_buffer[i]);

        }

        k++;
        
    }

    std::cout << endl << endl << "\t\t\t------------------------------" << endl << endl;

    #endif

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // raw analysis of received data;

    // checking if there has been a response at all;
    if (!(debug_bytes_received > 0)) {

        msg << "Device does not respond.";
        transmitStatement(ERROR);

        return false;

    }

    // checking for possible "NAK" (15h) reply packet;
    else if (!(debug_buffer[0] != 15)) {

        msg << "Response packet is \"NAK\" (15h). Device cannot fullfill request.";
        transmitStatement(WARNING);

        return false;

    }

    // checking for legal packet size;
    else if (!(88 >= debug_bytes_received <= 112)) {

        msg << "Received packet has wrong size.";
        transmitStatement(WARNING);

        return false;

    }

    // checking for packet head (stx);
    else if (debug_buffer[0] != 0x02) {

        msg << "First byte of received packet is not stx (0x02).";
        transmitStatement(WARNING);

        return false;

    }

    // checking for packet tail (etx)
    else if (debug_buffer[debug_bytes_received-1] != 0x03) {

        msg << "Last byte of received packet is not etx (0x03).";
        transmitStatement(WARNING);

        return false;

    }

    // checking for packet type;
    // must be "RAWDATA" (57h);
    else if (debug_buffer[2] != 0x57) {

        msg << "Received packet has wrong type.";
        transmitStatement(WARNING);

        return false;

    }

    // checking for record type;
    // must be "Position Data" (01h);
    else if (debug_buffer[4] != 0x01) {

        msg << "Received packet has wrong record type.";
        transmitStatement(WARNING);

        return false;

    }

    // checking pager counter;
    // must be 11h for position records;
    // 11 hex = 00010001 binary --> means page 01 of 01;
    else if (debug_buffer[5] != 0x11) {

        msg << "Received packet has wrong page counter value.";
        transmitStatement(WARNING);

        return false;

    }

    // another raw checking may be applied here...;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    else {

        // hereby called functions analyze the received packet in-depth, structure it, extract and finnaly serve GPS data;
        // if everything works fine, GPS data is getting stored in Dgps::GpsData gps_data;
        if (!analyzeData(debug_buffer, debug_bytes_received)) {

            msg << "Failed to gather GPS data.";
            transmitStatement(WARNING);

            return false;

        }

        else {

            return true;

        }

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

}

/**************************************************/
/**************************************************/
/**************************************************/

bool Dgps::analyzeData(unsigned char *  buffer,         // points to received packet from serialIO (char array);
                       int              buffer_size) {  // number of received bytes;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // --- header ---
    temp_packet.stx         = buffer[0] % 256;
    temp_packet.status      = buffer[1] % 256;
    temp_packet.packet_type = buffer[2] % 256;
    temp_packet.length      = buffer[3] % 256;

    // --- data part header ---
    temp_packet.record_type                 = buffer[4] % 256;
    temp_packet.page_counter                = buffer[5] % 256;
    temp_packet.reply_number                = buffer[6] % 256;
    temp_packet.record_interpretation_flags = buffer[7] % 256;

    // --- data part ---

    temp_packet.data_bytes = new char[temp_packet.length];

    for (int i = 0; i < (temp_packet.length - 4); i++) {

        temp_packet.data_bytes[i] = buffer[8 + i] % 256;

    }

    // --- footer ---
    temp_packet.checksum    = buffer[temp_packet.length + 4]        % 256;
    temp_packet.etx         = buffer[temp_packet.length + 4 + 1]    % 256;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // verify checksum;
    char checksum = 0x00;
    checksum = checksum + (temp_packet.status                       % 256);
    checksum = checksum + (temp_packet.packet_type                  % 256);
    checksum = checksum + (temp_packet.length                       % 256);
    checksum = checksum + (temp_packet.record_type                  % 256);
    checksum = checksum + (temp_packet.page_counter                 % 256);
    checksum = checksum + (temp_packet.reply_number                 % 256);
    checksum = checksum + (temp_packet.record_interpretation_flags  % 256);
            
    // calculate checksum over data bytes;
    for (int i = 0; i < (temp_packet.length - 4); i++) {

        checksum = checksum + temp_packet.data_bytes[i];

    }

    // wrap checksum into 1 byte;
    checksum = checksum % 256;

    // check for checksum mismatch;
    if (checksum != temp_packet.checksum) {

        msg << "Checksum mismatch.";
        transmitStatement(WARNING);

        return false;

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    else {

        return extractGpsData();

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

}

/**************************************************/
/**************************************************/
/**************************************************/

bool Dgps::extractGpsData() {

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // get all fields of type DOUBLE (8 bytes);

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

        latitude_bytes          [i] = temp_packet.data_bytes[i + 0];
        longitude_bytes         [i] = temp_packet.data_bytes[i + 8];
        altitude_bytes          [i] = temp_packet.data_bytes[i + 16];
        clock_offset_bytes      [i] = temp_packet.data_bytes[i + 24];
        frequency_offset_bytes  [i] = temp_packet.data_bytes[i + 32];
        pdop_bytes              [i] = temp_packet.data_bytes[i + 40];
        latitude_rate_bytes     [i] = temp_packet.data_bytes[i + 48];
        longitude_rate_bytes    [i] = temp_packet.data_bytes[i + 56];
        altitude_rate_bytes     [i] = temp_packet.data_bytes[i + 64];

    }

    // helper variable; used to find the meaning of semi-circles in this case...
    // (it's just 0-180 normalized to 0.0-1.0)
    double semi_circle_factor = 180.0;

    // getDOUBLE includes inverting of bit order;
    // neccessary because of motorola format;
    // see Trimbel BD982 GNSS Receiver Manual, p. 66ff;
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

    // get all fields of type LONG (4 bytes)

    unsigned char gps_msec_of_week_bytes[4];

    for (int i = 0; i < 4; i++) {

        gps_msec_of_week_bytes[i]   = temp_packet.data_bytes[i + 72];

    }

    // getLONG includes inverting of bit order
    gps_data.gps_msec_of_week = getLONG(gps_msec_of_week_bytes);

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // get all fields of type CHAR (1 byte)

    gps_data.position_flags = getCHAR(temp_packet.data_bytes[76]);
    gps_data.number_of_SVs  = getCHAR(temp_packet.data_bytes[77]);

    // first getting rid of old values
    gps_data.channel_numbers.clear();
    gps_data.prn.clear();

    // gathering new values according to number of satellites
    for (int i = 0; i < gps_data.number_of_SVs; i++) {

        gps_data.channel_numbers.push_back  (getCHAR(temp_packet.data_bytes[78 + i*2]));
        gps_data.prn.push_back              (getCHAR(temp_packet.data_bytes[79 + i*2]));

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    if (!(gps_data.number_of_SVs > 0)) {

        msg << "Currently no tracked sattelites.";
        transmitStatement(WARNING);

        return true; // return true, because this is a possible state and not an error!;

    }

    // some further checking may be added here...;
    // do not forget to return false in case of errors!;

    return true;

}

/**************************************************/
/**************************************************/
/**************************************************/

// function to reorder incoming bits
// neccessary because of motorola format;
// see Trimbel BD982 GNSS Receiver Manual, p. 66ff;
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