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
 * ToDo:
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

// allows to extract bytes as described in bd982 user guide..
// - base unit is 1 char = 8 Bit

int stx_index = 0; // index
int stx_length = 1; // length / bytes

int status_index = 1;
int status_length = 1;

int packet_type_index = 2;
int packet_type_length = 1;

int length_index = 3;
int length_length = 1; // byte-length of packet-field "length"

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

// call with value from byte 3: "length"

int etx_index(int length_value) {
    return 4 + length_value + 1;
}
int etx_length = 1;

//-----------------------------------------------

Dgps::Dgps() {

}


//-------------------------------------------

Dgps::~Dgps() {
    m_SerialIO.close();
}


// ---------------------------------------------------------------------------

bool Dgps::open(const char* pcPort, int iBaudRate) {
    int bRetSerial;
    // forDGPs :default is 38400
    //	if (iBaudRate != 38400)
    //            ROS_WARN("Baudrate is not set to 38400, might be too slow to transmit all data..");
    //	return false;

    // update scan id (id=8 for slave scanner, else 7)

    // initialize Serial Interface
    m_SerialIO.setBaudRate(iBaudRate);
    m_SerialIO.setDeviceName(pcPort);
    //	m_SerialIO.setBufferSize(READ_BUF_SIZE - 10 , WRITE_BUF_SIZE -10 );
    //	m_SerialIO.setHandshake(SerialIO::HS_NONE);
    //	m_SerialIO.setMultiplier(m_dBaudMult);
    bRetSerial = m_SerialIO.open();
    //	m_SerialIO.setTimeout(0.0);
    //	m_SerialIO.SetFormat(8, SerialIO::PA_NONE, SerialIO::SB_ONE);
    //	cout<<"test"<<bRetSerial<<endl;
    if (bRetSerial == 0) {
        // Clears the read and transmit buffer.
        //	    m_iPosReadBuf2 = 0;
        m_SerialIO.purge();
        return true;
    } else {
        return false;
    }
}

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

        if (bytesRead <= 0) {
            // error, nothing received --> wait for at least 500ms and try again few times
        } else {
            for (int i = 0; i < bytesRead; i++) {
                printf("%.2x ", Buffer[i]);
                //cout << std::hex << Buffer[i];
                //            if (i % 2 == 1) {
                //                cout << " ";
                //            } else
                if (i % 20 == 19) {
                    cout << "\n";
                }
            }
            cout << std::dec << "\n";
        }

        if (Buffer[0] == 6) {
            success = true;
            cout << "protocol request successful.\n";
        } else {
            success = false;
            cout << "protocol request failed. retrying...\n";

        };
    }
    return success;
}

int ringbuffer_size = 256;
unsigned char ringbuffer[256] = {0};
int ringbuffer_start = 0;
int ringbuffer_length = 0;

bool Dgps::receiveData(unsigned char * incoming_data, // int array from serial.IO
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

    printf("content of ringbuffer:\n");
    for (int i = 0; i < ringbuffer_size; i++) {
        printf(" %.2x", ringbuffer[i]);
    }
    cout << endl;
    printf(" buffer_start: %i", ringbuffer_start);
    printf(" buffer_length: %i", ringbuffer_length);

    // find stx, try to get length, and match checksum + etx
    for (int i = 0; i < ringbuffer_length; i++) {
        // find stx: (byte 0 == 0x02)
        if (ringbuffer[(ringbuffer_start + i + stx_index) % ringbuffer_size] != 0x02) {
            printf("first byte was not stx: %i\n", ringbuffer[(ringbuffer_start + i + stx_index) % ringbuffer_size]);
            continue;
        } else {
            //            printf("found stx @ byte: %i\n", i);
            // --- header ---
            temp_packet.stx = ringbuffer[(ringbuffer_start + i + stx_index) % ringbuffer_size] % 256;
            temp_packet.status = ringbuffer[(ringbuffer_start + i + status_index) % ringbuffer_size] % 256;
            temp_packet.packet_type = ringbuffer[(ringbuffer_start + i + packet_type_index) % ringbuffer_size] % 256;
            temp_packet.length = ringbuffer[(ringbuffer_start + i + length_index) % ringbuffer_size] % 256;
            // --- data_part ---
            temp_packet.record_type = ringbuffer[(ringbuffer_start + i + record_type_index) % ringbuffer_size] % 256;
            temp_packet.page_counter = ringbuffer[(ringbuffer_start + i + page_counter_index) % ringbuffer_size] % 256;
            temp_packet.reply_number = ringbuffer[(ringbuffer_start + i + reply_number_index) % ringbuffer_size] % 256;
            temp_packet.record_interpretation_flags = ringbuffer[(ringbuffer_start + i + record_interpretation_flags_index) % ringbuffer_size] % 256;
            // --- --- data_bytes --- ---
            temp_packet.data_bytes = new char[temp_packet.length];
            for (int j = 0; j < data_bytes_length(temp_packet.length); j++) {
                temp_packet.data_bytes[j] = ringbuffer[(ringbuffer_start + i + data_bytes_index + j) % ringbuffer_size] % 256;
            }
            // --- footer ---
            temp_packet.checksum = ringbuffer[(ringbuffer_start + i + checksum_index(temp_packet.length)) % ringbuffer_size] % 256;
            temp_packet.etx = ringbuffer[(ringbuffer_start + i + etx_index(temp_packet.length)) % ringbuffer_size] % 256;

            // verify checksum + etx
            char checksum = 0x00;
            checksum = checksum + (temp_packet.status % 256);
            checksum = checksum + (temp_packet.packet_type % 256);
            checksum = checksum + (temp_packet.length % 256);
            checksum = checksum + (temp_packet.record_type % 256);
            checksum = checksum + (temp_packet.page_counter % 256);
            checksum = checksum + (temp_packet.reply_number % 256);
            checksum = checksum + (temp_packet.record_interpretation_flags % 256);
            for (int i = 0; i < data_bytes_length(temp_packet.length); i++) {
                checksum = (checksum + temp_packet.data_bytes[i]);

            }
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

            int ringbuffer_old_start = ringbuffer_start;
            ringbuffer_start = (ringbuffer_start + i + etx_index(temp_packet.length) + 1) % ringbuffer_size;

            if (ringbuffer_old_start < ringbuffer_start)
                ringbuffer_length = ringbuffer_length - (ringbuffer_start - ringbuffer_old_start);
            if (ringbuffer_old_start >= ringbuffer_start)
                ringbuffer_length = ringbuffer_length - (ringbuffer_start + ringbuffer_old_start);
            if (!error_occured) {
                incoming_packet = temp_packet;
                // if data is okay, update ringbuffer pointers and write new packet to parameter
                success = extractGPS(incoming_packet, position_record);
            }
            if (ringbuffer_length > 0) {
                printf("ringbuffer was not empty after reading one packet! (%i bytes left).. calling receiveData again\n", ringbuffer_length);
                if (ringbuffer_old_start != ringbuffer_start)
                    if (!success) {
                        // call without data to process rest of buffered data
                        success = receiveData(NULL, 0, incoming_packet, position_record);
                    } else {
                        receiveData(NULL, 0, incoming_packet, position_record);
                    }
                else {
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


// use:
//          else reversed[k * 8 + i] = bits[(k) * 8 + (i)];
bool * invertBitOrder_Double(bool* bits, bool invertBitsPerByte = true, bool invertByteOrder = false) {
    bool * reversed = new bool[64];
    for (int k = 0; k < 8; k++) {
        for (int i = 0; i < 8; i++) {
            if      (!invertByteOrder &&  invertBitsPerByte) reversed[k * 8 + i] = bits[(k)   * 8 + (7-i)];
            else if (!invertByteOrder && !invertBitsPerByte) reversed[k * 8 + i] = bits[(k)   * 8 + (i)  ];
            else if ( invertByteOrder &&  invertBitsPerByte) reversed[k * 8 + i] = bits[(7-k) * 8 + (7-i)];
            else if ( invertByteOrder && !invertBitsPerByte) reversed[k * 8 + i] = bits[(k)   * 8 + (7-i)];
        }
    }
    return reversed;
}



//
int exponent_bias = 1023;
double getDOUBLE(unsigned char* bytes) {
    bool bits[64] = {false};
    // latitude_bits[bit_count] = {0};
    // get bits of DOUBLE
    for (int k = 0; k < 8; k++) {
        for (int i = 0; i < 8; i++) {
            bits[((k * 8) + i)] = 0 != (bytes[k] & (1 << i));
        }
    }
    bool * bits_reversed;
    bits_reversed = invertBitOrder_Double(bits);
    for (int i = 0; i < 64; i++) bits[i] = bits_reversed[i];

    cout << "bits:\n";
    for (int i = 0; i < 64; i++) {
        printf("%i", bits[i]);
    }
    cout << "\n";

    int sign_bit_lat = bits[0];
    double exponent_lat = 0;
    double fraction_lat = 0;
    for (int i = 0; i < 11; i++) {
        exponent_lat = exponent_lat + bits[i + 1] * pow(2, 10 - i);
    }
    for (int i = 0; i < 52; i++) {
        fraction_lat = fraction_lat + bits[i + 12] * pow(0.5, i + 1);
    }
    fraction_lat += 1;
    //printf("exponent_lat: %f\n", exponent_lat-exponent_bias);
    //printf("fraction_lat: %f\n", fraction_lat);

    int sign_lat = 1;
    if (sign_bit_lat == 1) sign_lat = -1;

    double latitude_value = sign_lat * (fraction_lat * pow(2, exponent_lat - exponent_bias));
    //printf("  calculated value: %f \n", latitude_value);
    return latitude_value;
}

// index is relative to begin of data_part; so it is byte number: index+8 in packet-bytes
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

    unsigned char test_minus_25_25[8] = {0x00};
    test_minus_25_25[0] = 0xc0;
    test_minus_25_25[1] = 0x39;
    test_minus_25_25[2] = 0x40;




    double semi_circle_factor = (pow(2, 31));
    printf("semi-circle factor: %f\n", semi_circle_factor);
    semi_circle_factor = 180.0;
    printf("semi-circle factor: %f\n", semi_circle_factor);

    double latitude_value = getDOUBLE(latitude_bytes) * semi_circle_factor;
    printf("calculated latitude semi-circles: %f\n", latitude_value);
    printf("calculated latitude: %f\n\n\n", latitude_value);

    double longitude_value = getDOUBLE(longitude_bytes) * semi_circle_factor;
    printf("calculated longitude semi-circles: %f\n", longitude_value);
    printf("calculated longitude: %f\n\n\n", longitude_value);

    double altitude_value = getDOUBLE(altitude_bytes);
    printf("calculated altitude: %f\n\n\n", altitude_value);


    double clock_offset = getDOUBLE(clock_offset_bytes);
    printf("clock_offset: %f\n\n\n", clock_offset);
    double frequency_offset = getDOUBLE(frequency_offset_bytes);
    printf("frequency_offset: %f\n\n\n", frequency_offset);
    double pdop = getDOUBLE(pdop_bytes);
    printf("pdop: %f\n\n\n", pdop);




    printf("calculated -25.25: %f\n\n\n", getDOUBLE(test_minus_25_25));


    position_record.latitude_value = latitude_value;
    position_record.longitude_value = longitude_value;
    position_record.altitude_value = altitude_value;


    return true;
}

bool Dgps::getPosition(gps_data &position_record) {
    // set to true after extracting position values. method return value.
    bool success = false;
    unsigned char Buffer[1024] = {0};
    int buffer_index = 0;
    int bytesread, byteswrite;


    // generate request message

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

    //SerialIO dgps;
    //open = dgps.open();
    byteswrite = m_SerialIO.write(message, length);
    printf("Total number of bytes sent: %i\n", byteswrite);
    //sleep(1);
    bytesread = m_SerialIO.readNonBlocking((char*) Buffer, 1020);

    //bytesread = 118;  //m_SerialIO.readNonBlocking((char*) Buffer, 1020);
    //    string test_packet = " |02|  |20|  |57|  |0a|  |0c|  |11|  |00|  |00|  |00|  |4d|  |01|  |e1|  |01|  |e1|  |af|  |03|  |02|  |20|  |57|  |60|  |01|  |11|  |00|  |00|  |3f|  |d1|  |54|  |8a|  |b6|  |cf|  |c6|  |8d|  |3f|  |a9|  |e0|  |bd|  |3f|  |29|  |c8|  |f7|  |40|  |80|  |ae|  |2a|  |c9|  |7b|  |b7|  |11|  |c0|  |fd|  |d3|  |79|  |61|  |fb|  |23|  |99|  |c0|  |92|  |ca|  |3b|  |46|  |c7|  |05|  |15|  |3f|  |ff|  |9f|  |23|  |e0|  |00|  |00|  |00|  |be|  |44|  |16|  |1f|  |0d|  |84|  |d5|  |33|  |be|  |2c|  |8b|  |3b|  |bb|  |46|  |eb|  |85|  |3f|  |b5|  |ec|  |f0|  |c0|  |00|  |00|  |00|  |08|  |06|  |f0|  |d8|  |d4|  |07|  |0f|  |0d|  |13|  |01|  |0c|  |07|  |04|  |07|  |0d|  |08|  |09|  |0a|  |1a|  |1c|  |5c|  |03|";
    //           test_packet = " |02|  |20|  |57|  |0a|  |0c|  |11|  |00|  |00|  |00|  |4d|  |01|  |e1|  |01|  |e1|  |af|  |03|  |02|  |20|  |57|  |60|  |01|  |11|  |00|  |00|  |3f|  |d1|  |54|  |8a|  |b6|  |cf|  |c6|  |8d|  |3f|  |a9|  |e0|  |bd|  |3f|  |29|  |c8|  |f7|  |40|  |80|  |ae|  |2a|  |c9|  |7b|  |b7|  |11|  |c0|  |fd|  |d3|  |79|  |61|  |fb|  |23|  |99|  |c0|  |92|  |ca|  |3b|  |46|  |c7|  |05|  |15|  |3f|  |ff|  |9f|  |23|  |e0|  |00|  |00|  |00|  |be|  |44|  |16|  |1f|  |0d|  |84|  |d5|  |33|  |be|  |2c|  |8b|  |3b|  |bb|  |46|  |eb|  |85|  |3f|  |b5|  |ec|  |f0|  |c0|  |00|  |00|  |00|  |08|  |06|  |f0|  |d8|  |d4|  |07|  |0f|  |0d|  |13|  |01|  |0c|  |07|  |04|  |07|  |0d|  |08|  |09|  |0a|  |1a|  |1c|  |5c|  |03|";
    //
    //    for (int i = 0; i < bytesread; i++) {
    //        char hex_byte1 = test_packet[i * 6 + 2];
    //        char hex_byte2 = test_packet[i * 6 + 3];
    //
    //        if (hex_byte1 > 96) hex_byte1 -= 87; // 96-9
    //        else hex_byte1 -= 48;
    //        if (hex_byte2 > 96) hex_byte2 -= 87; // 96-9
    //        else hex_byte2 -= 48;
    //
    //        Buffer[i] = hex_byte1 * 16 + hex_byte2;
    //        printf("%x%x-%i  ", hex_byte1, hex_byte2, Buffer[i]);
    //
    //    }
    //    cout << "\n";

    printf("\nTotal number of bytes received: %i\n", bytesread);
    cout << "-----------\n";
    for (int i = 0; i < bytesread; i++) {
        printf(" %.2x", Buffer[buffer_index + i]);

    }
    cout << std::dec << "\n";
    cout << "-----------\n";

    packet_data incoming_packet;
    //Dgps temp_gps_dev = Dgps();

    success = receiveData(Buffer, bytesread, incoming_packet, position_record);

    // need to check if values were ok, right now just hardcoded true..
    //    success = true;
    return success;
}


