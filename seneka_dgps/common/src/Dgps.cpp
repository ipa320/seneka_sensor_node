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
 * ROS stack name: DGPS
 * ROS package name: seneka_dgps
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Ciby Mathew, email:Ciby.Mathew@ipa.fhg.de
 * Supervised by: Christophe Maufroy
 *
 * Date of creation: Jan 2013
 * modified 03/2014: David Bertram, email: davidbertram@gmx.de
 *
 * ToDo:
 * - see seneka_dgps.cpp for todo list
 *
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
#include <seneka_dgps/Dgps.h>

#include <endian.h>

using namespace std;

//-----------------------------------------------

typedef unsigned char BYTE;





// following variable allow to extract packet fields from incoming data bytes as described in bd982 user guide..
// - base unit is 1 char = 8 Bit

int stx_index = 0; // index
int stx_length = 1; // length / bytes

int status_index = 1;
int status_length = 1;

int packet_type_index = 2;
int packet_type_length = 1;

int length_index = 3;
int length_length = 1; // bytelength of packet-field "length"

int record_type_index = 4;
int record_type_length = 1;

int page_counter_index = 5; // split byte in two parts! its <page> of <total>, each 4 bit
int page_counter_length = 1;

int reply_number_index = 6;
int reply_number_length = 1;

int record_interpretation_flags_index = 7;
int record_interpretation_flags_length = 1;

int data_bytes_index = 8;

int data_bytes_length(int length_value) {
    return length_value - 4;
}

int checksum_index(int length_value) {
    return 4 + length_value;
}
int checksum_length = 1;

// call with value of byte #3 (length-field)

int etx_index(int length_value) {
    return 4 + length_value + 1;
}
int etx_length = 1;


// class methods
Dgps::Dgps() {

}

Dgps::~Dgps() {
    m_SerialIO.close();
}

// open serial connection
bool Dgps::open(const char* pcPort, int iBaudRate) {
    int serial_open;
    m_SerialIO.setBaudRate(iBaudRate);
    m_SerialIO.setDeviceName(pcPort);
    serial_open = m_SerialIO.open();
    if (serial_open == 0) {
        m_SerialIO.purge();
        return true;
    } else {
        return false;
    }
}

// send protocol request to serial and return success
bool Dgps::checkConnection() {
    bool success = false;
    int max_tries = 5;
    int retry_delay = 1000000; // in microSeconds
    // test command "05h"       // expects test reply "06h"
    char message[] = {0x05};
    int length = sizeof (message) / sizeof (message[0]);
    unsigned char Buffer[1024] = {0};
    int bytesWritten = m_SerialIO.write(message, length);
    // retry few times
    int count = 0;
    while (!success && (count < max_tries)) {
        count += 1;
        usleep(retry_delay);
        int bytesRead = m_SerialIO.readNonBlocking((char*) Buffer, 1020);
        if (bytesRead > 0 && Buffer[0] == 6) {
            success = true;
            cout << "protocol request successful.\n";
        } else {
            success = false;
            cout << "protocol request failed. retrying...\n";
        };
    }
    return success;
}




int ringbuffer_size = 4096 * 4; // important:  - change next line too (array init), when changing buffer size!!
unsigned char ringbuffer[4096 * 4] = {0};
int ringbuffer_start = 0;
int ringbuffer_length = 0;

bool Dgps::interpretData(unsigned char * incoming_data, // int array from serial.IO
        int incoming_data_length, // count of received bytes
        packet_data incoming_packet,
        gps_data &position_record) { // .. function writes to this data address


    bool success = false;
    packet_data temp_packet; // = new packet_data;
    if ((ringbuffer_size - ringbuffer_length) >= incoming_data_length) {
        for (int i = 0; i < incoming_data_length; i++) {
            ringbuffer[(ringbuffer_start + ringbuffer_length + 1) % ringbuffer_size] = (char) incoming_data[i];
            ringbuffer_length = (ringbuffer_length + 1) % ringbuffer_size;
        }
    } else {
        cout << "Buffer is full! .. cannot insert data.." << endl;
        // received too much data, buffer is full
    }
    printf(" buffer_start: %i\n", ringbuffer_start);
    printf(" buffer_length: %i\n", ringbuffer_length);
    printf("content of ringbuffer:\n");
    for (int i = 0; i < ringbuffer_size; i++) {
        printf(" %.2x", ringbuffer[i]);
    }
    cout << endl;

    // find stx, try to get length, and match checksum + etx
    for (int y = 0; y < ringbuffer_length; y++) {
        // find stx: (byte 0 == 0x02)
        if (ringbuffer[(ringbuffer_start + y + stx_index) % ringbuffer_size] != 0x02) {
            printf("first byte was not stx: %i\n", ringbuffer[(ringbuffer_start + y + stx_index) % ringbuffer_size]);
            continue;
        } else {
            //            printf("found stx @ byte: %i\n", i);
            // --- header ---
            temp_packet.stx = ringbuffer[(ringbuffer_start + y + stx_index) % ringbuffer_size] % 256;
            temp_packet.status = ringbuffer[(ringbuffer_start + y + status_index) % ringbuffer_size] % 256;
            temp_packet.packet_type = ringbuffer[(ringbuffer_start + y + packet_type_index) % ringbuffer_size] % 256;
            temp_packet.length = ringbuffer[(ringbuffer_start + y + length_index) % ringbuffer_size] % 256;
            // --- data_part ---
            temp_packet.record_type = ringbuffer[(ringbuffer_start + y + record_type_index) % ringbuffer_size] % 256;
            temp_packet.page_counter = ringbuffer[(ringbuffer_start + y + page_counter_index) % ringbuffer_size] % 256;
            temp_packet.reply_number = ringbuffer[(ringbuffer_start + y + reply_number_index) % ringbuffer_size] % 256;
            temp_packet.record_interpretation_flags = ringbuffer[(ringbuffer_start + y + record_interpretation_flags_index) % ringbuffer_size] % 256;
            // --- --- data_bytes --- ---
            temp_packet.data_bytes = new char[temp_packet.length];
            for (int j = 0; j < data_bytes_length(temp_packet.length); j++) {
                temp_packet.data_bytes[j] = ringbuffer[(ringbuffer_start + y + data_bytes_index + j) % ringbuffer_size] % 256;
            }
            // --- footer ---
            temp_packet.checksum = ringbuffer[(ringbuffer_start + y + checksum_index(temp_packet.length)) % ringbuffer_size] % 256;
            temp_packet.etx = ringbuffer[(ringbuffer_start + y + etx_index(temp_packet.length)) % ringbuffer_size] % 256;

            // verify checksum + etx
            char checksum = 0x00;
            checksum = checksum + (temp_packet.status % 256);
            checksum = checksum + (temp_packet.packet_type % 256);
            checksum = checksum + (temp_packet.length % 256);
            checksum = checksum + (temp_packet.record_type % 256);
            checksum = checksum + (temp_packet.page_counter % 256);
            checksum = checksum + (temp_packet.reply_number % 256);
            checksum = checksum + (temp_packet.record_interpretation_flags % 256);
            // calculate checksum over data bytes
            for (int z = 0; z < data_bytes_length(temp_packet.length); z++) {
                checksum = (checksum + temp_packet.data_bytes[z]);
            }
            // wrap checksum into 1 byte
            checksum = checksum % 256;

            bool error_occured = false;
            if (checksum != temp_packet.checksum) {
                printf("\n\n  new parser: checksum mismatch %x (calculated) - %x (received)\n\n", checksum, temp_packet.checksum);
                error_occured = true;
            }
            if (temp_packet.etx != 0x03) {
                printf("\n\n  new parser: etx was not 0x03 - %x (received)\n\n", temp_packet.etx);
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

                // if data is okay -> success = true;  ..then update ringbuffer pointers and write new packet to parameter. ..see  few lines below
                success = extractGPS(incoming_packet, position_record);
            }
            if (ringbuffer_length > 0) {
                printf("ringbuffer was not empty after reading one packet! (%i bytes left).. calling receiveData again\n", ringbuffer_length);
                if (ringbuffer_old_start != ringbuffer_start)
                    if (!success) {
                        // call without data to process rest of buffered data
                        success = interpretData(NULL, 0, incoming_packet, position_record);
                    } else {
                        interpretData(NULL, 0, incoming_packet, position_record);
                    } else {
                    printf("..stopped interpreting remaining buffer to avoid infinite-loop\n");
                }
            } else {
                printf(" extracted packet successfully \n\n");
            }
            return success;
        }
    }
    return success;
}


// function to reorder incoming bits.
bool * invertBitOrder_Double(bool* bits, bool invertBitsPerByte = false, bool invertByteOrder = true) {
    bool * reversed = new bool[64];
    for (int k = 0; k < 8; k++) {
        for (int i = 0; i < 8; i++) {
            if (!invertByteOrder && invertBitsPerByte) reversed[k * 8 + i] = bits[(k) * 8 + (7 - i)];
            else if (!invertByteOrder && !invertBitsPerByte) reversed[k * 8 + i] = bits[(k) * 8 + (i) ];
            else if (invertByteOrder && invertBitsPerByte) reversed[k * 8 + i] = bits[(7 - k) * 8 + (7 - i)];
            else if (invertByteOrder && !invertBitsPerByte) reversed[k * 8 + i] = bits[(7 - k) * 8 + (i)];
        }
    }
    return reversed;
}



// function to extract IEEE double precision number values from an 8-byte array (8 Bit per Byte; array size is expected to be 8; ==> 64 Bit)
//
// bias is 1023 as default for standard numbers
// ...(IEEE example -25.25 reads ok with invertedBitsPerByte and bias 1023)
//
// example data for -25.25:          ( 8 bytes, last 5 bytes are 0x00, so written at initialization of array..)
//    unsigned char test_minus_25_25[8] = {0x00};
//    test_minus_25_25[0] = 0xc0;
//    test_minus_25_25[1] = 0x39;
//    test_minus_25_25[2] = 0x40;

//int exponent_bias = 1023;         // hopefully working as default parameter..
double getDOUBLE(unsigned char* bytes, int exponent_bias = 1023) {
    // init with zero/false
    bool bits[64] = {false};
    // get bits of DOUBLE
    for (int k = 0; k < 8; k++) {
        for (int i = 0; i < 8; i++) {
            bits[((k * 8) + i)] = 0 != (bytes[k] & (1 << i));
        }
    }
    bool * resulting_bits;
    resulting_bits = invertBitOrder_Double(bits);
    for (int i = 0; i < 64; i++) bits[i] = resulting_bits[i];

// debug output
//    cout << "bits:\n";
//    for (int i = 0; i < 64; i++) {
//        printf("%i", bits[i]);
//    }
//    cout << "\n";

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

// debug output
//printf("exponent_lat: %f\n", exponent_lat-exponent_bias);
//printf("fraction_lat: %f\n", fraction_lat);

    int sign_lat = 1;
    if (sign_bit == 1) sign_lat = -1;

    // calculate number value from extracted values
    double double_value = sign_lat * (fraction * pow(2, exponent - exponent_bias));
// debug output
//printf("  calculated value: %f \n", latitude_value);
    return double_value;
}





// variables for index positions of byte indizes for the fields of a position record packet
// 
// index is relative to begin of data_part; so it is byte #: index+8       ... of packet-bytes (stx = 0)
int latitude_value_index = 0;
int latitude_value_length = 8;
int longitude_value_index = 8;
int longitude_value_length = 8;
int altitude_value_index = 16;
int altitude_value_length = 8;
int clock_offset_index = 24;
int clock_offset_length = 8;
int frequency_offset_index = 32;
int frequency_offset_length = 8;
int pdop_index = 40;
int pdop_length = 8;
int latitude_rate_index = 48;
int latitude_rate_length = 8;
int longitude_rate_index = 56;
int longitude_rate_length = 8;
int altitude_rate_index = 64;
int altitude_rate_length = 8;

int gps_msec_of_week_index = 72;
int gps_msec_of_week_length = 4;

int position_flags_index;
int position_flags_length = 76;

int number_of_SVs_index;
int number_of_SVs_length = 77;
// 78 .. 80 .. 82 .. 84 ..
int *channel_number_index;
int channel_number_length = 1;
// 79 .. 81 .. 83 .. 85 ..
int *prn_index;
int prn_length = 1;

// helper variable, used to find meaning of semi-circles in this case.. (it's just 0-180 normalized to 0.0-1.0)
double semi_circle_factor = 180.0;


// TODO: extract ALL fields!!
bool Dgps::extractGPS(packet_data &incoming_packet, gps_data &position_record) {

    if (incoming_packet.packet_type != 0x057) {
        printf("wrong packet type: %.10x\n", incoming_packet.packet_type);
        return false;
    }
    if (incoming_packet.record_type != 0x01) {
        printf("wrong record type: %.10x\n", incoming_packet.record_type);
        return false;
    }

    printf("packet ok\n..reading values\n\n");

    unsigned char latitude_bytes[8] = {0};
    unsigned char longitude_bytes[8] = {0};
    unsigned char altitude_bytes[8] = {0};
    unsigned char clock_offset_bytes[8] = {0};
    unsigned char frequency_offset_bytes[8] = {0};
    unsigned char pdop_bytes[8] = {0};

    // get all double fields ( 8 bytes )
    for (int i = 0; i < 8; i++) {
        latitude_bytes[i] = incoming_packet.data_bytes[i + latitude_value_index];
        longitude_bytes[i] = incoming_packet.data_bytes[i + longitude_value_index];
        altitude_bytes[i] = incoming_packet.data_bytes[i + altitude_value_index];
        clock_offset_bytes[i] = incoming_packet.data_bytes[i + clock_offset_index];
        frequency_offset_bytes[i] = incoming_packet.data_bytes[i + frequency_offset_index];
        pdop_bytes[i] = incoming_packet.data_bytes[i + pdop_index];
    }

    // get all long fields ( 4 bytes )

    // get all char fields ( 1 byte )


    // extract satellite number
    // generate arrays of this size for channel_number prn_index
    // extract value and generate arrays for the fields: channel_number_index and prn_index


    // extract number values
    double latitude_value = getDOUBLE(latitude_bytes) * semi_circle_factor;
    double longitude_value = getDOUBLE(longitude_bytes) * semi_circle_factor;
    double altitude_value = getDOUBLE(altitude_bytes);
    double clock_offset = getDOUBLE(clock_offset_bytes);
    double frequency_offset = getDOUBLE(frequency_offset_bytes);
    double pdop = getDOUBLE(pdop_bytes);

// debug output
//    printf("calculated longitude: %f\n\n\n", longitude_value);
//    printf("calculated latitude: %f\n\n\n", latitude_value);
//    printf("calculated altitude: %f\n\n\n", altitude_value);
//    printf("clock_offset: %f\n\n\n", clock_offset);
//    printf("frequency_offset: %f\n\n\n", frequency_offset);
//    printf("pdop: %f\n\n\n", pdop);

    // write extracted values to gps_data struct; this is the returned data
    position_record.latitude_value = latitude_value;
    position_record.longitude_value = longitude_value;
    position_record.altitude_value = altitude_value;

    // implement value checking if needed:  e.g. value of longitude&latitude between 0 and 180; ...
    // ... return false if any check fails..
    return true;
}

bool Dgps::getPosition(gps_data &position_record) {
    // set to true after extracting position values. method return value.
    bool success = false;
    unsigned char Buffer[1024] = {0};
    int buffer_index = 0;
    int bytesread, byteswrite;

    // generate request message
    //
    // see page 73 in BD982 user guide for packet specification
    //  start tx,
    //      status,
    //          packet type,
    //              length,
    //                  type raw data,      [0x00: Real-Time Survey Data Record; 0x01: Position Record]
    //                      flags,
    //                          reserved,
    //                              checksum,
    //                                  end tx
    unsigned char stx_ = 0x02;
    unsigned char status_ = 0x00;
    unsigned char packet_type_ = 0x56;
    unsigned char length_ = 0x03;
    unsigned char data_type_ = 0x01;
    unsigned char etx_ = 0x03;
    unsigned char checksum_ = status_ + packet_type_ + data_type_ + length_;
    char message[] = {stx_, status_, packet_type_, length_, data_type_, 0x00, 0x00, checksum_, etx_}; // 56h command packet       // expects 57h reply packet (basic coding)
    int length = sizeof (message) / sizeof (message[0]);

    // send request message to serial
    byteswrite = m_SerialIO.write(message, length);

// debug output
//    printf("Total number of bytes sent: %i\n", byteswrite);

    // read response from serial
    bytesread = m_SerialIO.readNonBlocking((char*) Buffer, 1020);
    
// debug output
//    printf("\nTotal number of bytes received: %i\n", bytesread);
//    cout << "-----------\n";
//    for (int i = 0; i < bytesread; i++) {
//        printf(" %.2x", Buffer[buffer_index + i]);
//
//    }
//    cout << std::dec << "\n";
//    cout << "-----------\n";

    // create data structure for the extracted data packets from serial
    // .. this is not needed.. could be removed and only used internally by interpretData function..
    // .. .. left from dev code ;)
    packet_data incoming_packet;

    // put received data into buffer; extract packets; ..extract gps data when available
    success = interpretData(Buffer, bytesread, incoming_packet, position_record);

    return success;
}


