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

int ringbuffer_size = 4096;
char ringbuffer[4096] = {0};
int ringbuffer_start = 0;
int ringbuffer_length = 0;

//int ringbuffer_filled(){
//    if (ringbuffer_length == 0) return 0;
//    else if (ringbuffer_start < ringbuffer_end) return ringbuffer_end - ringbuffer_start;
//    else if (ringbuffer_start > ringbuffer_end) return (ringbuffer_size - ringbuffer_start) + ringbuffer_end;
//}

bool Dgps::receiveData(unsigned char * incoming_data, // int array from serial.IO
        int incoming_data_length, // count of received bytes
        packet_data incoming_packet) { // .. function writes to this data address
    bool success = false;

    packet_data temp_packet; // = new packet_data;



    if ((ringbuffer_size - ringbuffer_length) >= incoming_data_length) {
        for (int i = 0; i < incoming_data_length; i++) {

            ringbuffer[(ringbuffer_start + ringbuffer_length + 1) % ringbuffer_size] = incoming_data[i];
            ringbuffer_length = (ringbuffer_length + 1) % ringbuffer_size;
            printf("inserting byte# %i, buffer_end: %i\n", i, (ringbuffer_start + ringbuffer_length) % ringbuffer_size);
        }
    } else {
        // received too much data, buffer is full
    }

    // find stx, try to get length, and match checksum + etx


    for (int i = 0; i < ringbuffer_length; i++) {

        // find stx: (byte 0 == 0x02)
        if (ringbuffer[(ringbuffer_start + i + stx_index) % ringbuffer_size] != 0x02) {
            printf("first byte was not stx: %i\n", ringbuffer[(ringbuffer_start + i + stx_index) % ringbuffer_size]);
            continue;
        } else {
            printf("found stx @ byte: %i\n", i);
            // prepare packet

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

            char checksum = 0x0;
            printf(" generating checksum:\n");
            printf("    %.10x    + %.10x\n", checksum, temp_packet.status);
            checksum = checksum + (temp_packet.status % 256);
            printf("    %.10x    + %.10x\n", checksum, temp_packet.packet_type);
            checksum = checksum + (temp_packet.packet_type % 256);
            printf("    %.10x    + %.10x\n", checksum, temp_packet.length);

            checksum = checksum + (temp_packet.length % 256);

            printf("    %.10x    + %.10x\n", checksum, temp_packet.record_type);
            checksum = checksum + (temp_packet.record_type % 256);
            printf("    %.10x    + %.10x\n", checksum, temp_packet.page_counter);
            checksum = checksum + (temp_packet.page_counter % 256);
            printf("    %.10x    + %.10x\n", checksum, temp_packet.reply_number);
            checksum = checksum + (temp_packet.reply_number % 256);
            printf("    %.10x    + %.10x\n", checksum, temp_packet.record_interpretation_flags);
            checksum = checksum + (temp_packet.record_interpretation_flags % 256);
            printf("    %.10x    + %.10x\n\n", checksum, temp_packet.status);
            //                          + temp_packet.packet_type  % 256
            //                          + temp_packet.length  % 256
            //                          + temp_packet.record_type  % 256
            //                          + temp_packet.page_counter  % 256
            //                          + temp_packet.reply_number  % 256
            //                          + temp_packet.record_interpretation_flags % 256) % 256;
            for (int i = 0; i < data_bytes_length(temp_packet.length); i++) {
                checksum = (checksum + temp_packet.data_bytes[i]);
                printf("    %.10x    + %.10x\n", checksum, temp_packet.data_bytes[i]);
            }
            checksum = checksum % 256;
            printf(" -----------\n");
            printf("    %.10x:\n", checksum);

            if (checksum != temp_packet.checksum) {
                printf("\n\n  new parser: checksum mismatch %x (calculated) - %x (received)\n\n", checksum, temp_packet.checksum);
                return false;
            }
            if (temp_packet.etx != 0x03) {
                printf("\n\n  new parser: etx was not 0x03 - %x (received)\n\n", temp_packet.etx);
                return false;
            } //-----------


            int ringbuffer_old_start = ringbuffer_start;
            ringbuffer_start = (ringbuffer_start + i + etx_index(temp_packet.length) + 1) % ringbuffer_size;

            if (ringbuffer_old_start < ringbuffer_start)
                ringbuffer_length = ringbuffer_length - (ringbuffer_start - ringbuffer_old_start);
            if (ringbuffer_old_start >= ringbuffer_start)
                ringbuffer_length = ringbuffer_length - (ringbuffer_start + ringbuffer_old_start);

            incoming_packet = temp_packet;
            // if okay, update ringbuffer pointers and write new packet to parameter

            gps_data incoming_gps;
            extractGPS(incoming_packet, incoming_gps);

            if (ringbuffer_length > 0) {
                printf("ringbuffer is not empty after reading one packet! (%i bytes left).. calling receiveData again\n", ringbuffer_length);
                if (ringbuffer_old_start != ringbuffer_start)


                    receiveData(NULL, 0, incoming_packet);

                else {
                    printf("..stopped interpreting remaining buffer to avoid infinite-loop\n");
                }
            } else {

                printf(" extracted packet successfully \n\n");
            }
            return true;
        }
    }

    return success;
}


// obsolete! use BitOrder and k-1 or i-1 to define new ordering.. bytewise + bitwise
unsigned char * invertByteOrder_Double(unsigned char* in) {
    unsigned char * temp = new unsigned char[8];
    for (int i = 0; i < 8; i++) {

        temp[i] = in[i];

    }
    return temp;
}


bool * invertBitOrder_Double(bool* bits, bool invertByteOrder = false) {
    bool * reversed = new bool[64];
    for (int k = 0; k < 8; k++) {
        for (int i = 0; i < 8; i++) {
            if (!invertByteOrder)  reversed[k * 8 + i] = bits[(k) * 8 + (7-i)];
            else reversed[k * 8 + i] = bits[(k) * 8 + (i)];
        }
    }
    return reversed;
}

int exponent_bias = 1023;
double getDOUBLE(unsigned char* bytes, bool invertByteOrder = false){
        bool bits[64] = {false};
    // latitude_bits[bit_count] = {0};
    // get bits of DOUBLE
    for (int k = 0; k < 8; k++) {
        for (int i = 0; i < 8; i++) {
            bits[((k * 8) + i)] = 0 != (bytes[k] & (1 << i));
        }
    }
    bool * bits_reversed;
    bits_reversed = invertBitOrder_Double(bits, invertByteOrder);
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


bool Dgps::extractGPS(packet_data incoming_packet, gps_data incoming_gps) {

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

    for (int i = 0; i < 8; i++) {
        latitude_bytes[i] = incoming_packet.data_bytes[i];
        longitude_bytes[i] = incoming_packet.data_bytes[i + 8];
        altitude_bytes[i] = incoming_packet.data_bytes[i + 16];
    }

    unsigned char test_minus_25_25[8] ={0x00};
    test_minus_25_25[0] = 0xc0;
    test_minus_25_25[1] = 0x39;
    test_minus_25_25[2] = 0x40;


    

    double sc_factor = (pow(2,31));
    printf("semi-circle factor: %f\n", sc_factor );
    sc_factor = 180.0 / 1;
    printf("semi-circle factor: %f\n", sc_factor );

    double latitude_value = getDOUBLE(latitude_bytes);
    printf("calculated latitude semi-circles: %f\n", latitude_value);
    printf("calculated latitude: %f\n\n\n", latitude_value * sc_factor);

    double longitude_value = getDOUBLE(longitude_bytes);
    printf("calculated longitude semi-circles: %f\n", longitude_value);
    printf("calculated longitude: %f\n\n\n", longitude_value * sc_factor);

    double altitude_value = getDOUBLE(altitude_bytes);
    printf("calculated altitude semi-circles: %f\n", longitude_value);
    printf("calculated altitude: %f\n\n\n", altitude_value);

    printf("calculated -25.25: %f\n\n\n", getDOUBLE(test_minus_25_25) );



//
//
//    printf(" latitude bytes:\n");
//    for (int i = 0; i < 8; i++) {
//        printf("%.4x ", latitude_bytes[i]);
//    }
//    printf("\n");
//    unsigned char *latitude_bytes_reversed;
//    latitude_bytes_reversed = invertByteOrder_Double(latitude_bytes);
//    printf(" latitude bytes reversed:\n");
//    for (int i = 0; i < 8; i++) {
//        printf("%.4x ", latitude_bytes_reversed[i]);
//    }
//    printf("\n");
//    bool latitude_bits[64] = {false};
//    // latitude_bits[bit_count] = {0};
//    // get bits of DOUBLE
//    for (int k = 0; k < 8; k++) {
//        for (int i = 0; i < 8; i++) {
//            latitude_bits[((k * 8) + i)] = 0 != (latitude_bytes_reversed[k] & (1 << i));
//        }
//    }
//
//
//
//
//
//    //    for (int i = 0; i < 64; i++) {
//    //        latitude_bits[i] = 0;
//    //    }
//
//    // = 1.0
//    //bool latitude_bits [64] = {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//
//    // = -0.6875
//    //bool latitude_bits [64] =   {1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//
//    // = 1.5
//    //bool latitude_bits [64] =   {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//
//    // = -1.5
//    //bool latitude_bits [64] =   {1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//
//    // = -25.25
//    //    bool latitude_bits [64] =   {1,
//    //                                 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
//    //                                 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//
//     // 0 ; //1023;
//
//
//    cout << "latitude bits:\n";
//    for (int i = 0; i < 64; i++) {
//        printf("%i", latitude_bits[i]);
//    }
//    cout << "\n";
//
//
//
//
//
//    printf(" longitude bytes:\n");
//    for (int i = 0; i < 8; i++) {
//        printf("%.4x ", longitude_bytes[i]);
//    }
//    printf("\n");
//    unsigned char *longitude_bytes_reversed;
//    longitude_bytes_reversed = invertByteOrder_Double(longitude_bytes);
//    printf(" longitude bytes reversed:\n");
//    for (int i = 0; i < 8; i++) {
//        printf("%.4x ", longitude_bytes_reversed[i]);
//    }
//    printf("\n");
//    bool longitude_bits[64] = {false};
//    // latitude_bits[bit_count] = {0};
//    // get bits of DOUBLE
//    for (int k = 0; k < 8; k++) {
//        for (int i = 0; i < 8; i++) {
//            longitude_bits[((k * 8) + i)] = 0 != (longitude_bytes_reversed[k] & (1 << i));
//        }
//    }
//    cout << "longitude bits:\n";
//    for (int i = 0; i < 64; i++) {
//        printf(" %i", longitude_bits[i]);
//    }
//    cout << "\n";
//
//
//
//
//
//
//    printf(" altitude bytes:\n");
//    for (int i = 0; i < 8; i++) {
//        printf("%.4x ", altitude_bytes[i]);
//    }
//    printf("\n");
//    unsigned char *altitude_bytes_reversed;
//    altitude_bytes_reversed = invertByteOrder_Double(altitude_bytes);
//    printf(" altitude bytes reversed:\n");
//    for (int i = 0; i < 8; i++) {
//        printf("%.4x ", altitude_bytes_reversed[i]);
//    }
//    printf("\n");
//    bool altitude_bits[64] = {false};
//    // latitude_bits[bit_count] = {0};
//    // get bits of DOUBLE
//    for (int k = 0; k < 8; k++) {
//        for (int i = 0; i < 8; i++) {
//            altitude_bits[((k * 8) + i)] = 0 != (altitude_bytes_reversed[k] & (1 << i));
//        }
//    }
//    cout << "altitude bits:\n";
//    for (int i = 0; i < 64; i++) {
//        printf(" %i", altitude_bits[i]);
//    }
//    cout << "\n\n\n";
//
//
//
//
//
//
//
//
//    bool * latitude_bits_reversed;
//    latitude_bits_reversed = invertBitOrder_Double(latitude_bits);
//    for (int i = 0; i < 64; i++) latitude_bits[i] = latitude_bits_reversed[i];
//
//    cout << "latitude bits:\n";
//    for (int i = 0; i < 64; i++) {
//        printf("%i", latitude_bits[i]);
//    }
//    cout << "\n";
//
//
//    int sign_bit_lat = latitude_bits[0];
//    int exponent_lat = 0;
//    double fraction_lat = 0;
//    for (int i = 0; i < 11; i++) {
//        exponent_lat = exponent_lat + latitude_bits[i + 1] * pow(2, 10 - i);
//    }
//    for (int i = 0; i < 52; i++) {
//        fraction_lat = fraction_lat + latitude_bits[i + 12] * pow(0.5, i + 1);
//    }
//    fraction_lat += 1;
//    printf("exponent_lat: %i\n", exponent_lat-exponent_bias);
//    printf("fraction_lat: %f\n", fraction_lat);
//
//    int sign_lat = 1;
//    if (sign_bit_lat == 1) sign_lat = -1;
//
//    double latitude_value = sign_lat * (fraction_lat * pow(2, exponent_lat - exponent_bias)) * (180/2^31);
//    printf("  calculated latitude: %f \n", latitude_value);
//
//
//
//        bool * longitude_bits_reversed;
//    longitude_bits_reversed = invertBitOrder_Double(longitude_bits);
//    for (int i = 0; i < 64; i++) longitude_bits[i] = longitude_bits_reversed[i];
//
//    cout << "longitude bits:\n";
//    for (int i = 0; i < 64; i++) {
//        printf("%i", longitude_bits[i]);
//    }
//    cout << "\n";
//
//    int sign_bit_long = longitude_bits[0];
//    int exponent_long = 0;
//    double fraction_long = 0;
//    for (int i = 1; i < 11; i++) {
//        exponent_long = exponent_long + longitude_bits[i+1] * pow(2, i);
//    }
//    for (int i = 0; i < 52; i++) {
//        fraction_long = fraction_long + longitude_bits[i+12] * pow(0.5, i+1);
//    }
//    fraction_long += 1;
//        printf("exponent_long: %i\n", exponent_long-exponent_bias);
//    printf("fraction_long: %f\n", fraction_long);
//        int sign_long = 1;
//    if (sign_bit_long == 1) sign_long = -1;
//    double longitude_value = sign_long * (fraction_long * pow(2, exponent_long-exponent_bias)) * (180/2^31);
//    printf("  calculated longitude: %f \n", longitude_value);
//
//
//
//    bool * altitude_bits_reversed;
//    altitude_bits_reversed = invertBitOrder_Double(altitude_bits);
//    for (int i = 0; i < 64; i++) altitude_bits[i] = altitude_bits_reversed[i];
//    cout << "altitude bits:\n";
//    for (int i = 0; i < 64; i++) {
//        printf("%i", altitude_bits[i]);
//    }
//    cout << "\n";
//    int sign_bit_alt = altitude_bits[0];
//    int exponent_alt = 0;
//    double fraction_alt = 0;
//    for (int i = 1; i < 11; i++) {
//        exponent_alt = exponent_alt + altitude_bits[i+1] * pow(2, i);
//    }
//    for (int i = 11; i < 52; i++) {
//        fraction_alt = fraction_alt + altitude_bits[i+12] * pow(0.5, i+1);
//    }
//    fraction_alt +=1;
//
//        printf("exponent_alt: %i\n", exponent_alt-exponent_bias);
//    printf("fraction_alt: %f\n", fraction_alt);
//        int sign_alt = 1;
//    if (sign_bit_alt == 1) sign_alt = -1;
//    double altitude_value = sign_alt * (fraction_alt *  pow(2, exponent_alt-exponent_bias));
//    printf("  calculated altitude: %f \n", altitude_value);
//
//















    return true;
}

bool Dgps::getPosition(double* latt) {
    // set to true after extracting position values. method return value.
    bool success = false;

    int length;
    unsigned char Buffer[1024] = {0};
    for (int i = 0; i < 1024; i++) Buffer[i] = '0';
    char str[10];
    char binary[10000] = {0};
    int value[1000] = {0};
    int open, y, bytesread, byteswrite, bin;

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
    //(status_ + packet_type_ + data_type_ + 0 + 0 + length_)%256;



    char message[] = {stx_, status_, packet_type_, length_, data_type_, 0x00, 0x00, checksum_, etx_}; // 56h command packet       // expects 57h reply packet (basic coding)

    //        char message[]={ 0x05 };
    length = sizeof (message) / sizeof (message[0]);

    cout << "length of command: " << length << "\n";

    //SerialIO dgps;
    //open = dgps.open();
    byteswrite = m_SerialIO.write(message, length);
    printf("Total number of bytes written: %i\n", byteswrite);
    std::cout << "command was: " << std::hex << message << "\n";
    sleep(1);
    bytesread = m_SerialIO.readNonBlocking((char*) Buffer, 1020);

    printf("\nTotal number of bytes read: %i\n", bytesread);
    cout << "-----------\n";
    //    int buffer_size = sizeof (Buffer) / sizeof (Buffer[0]);
    for (int i = 0; i < bytesread; i++) {
        printf(" |%.2x| ", Buffer[i]);
        //cout << std::hex << Buffer[i];
        //        if (i % 2 == 1) {
        //            cout << " ";
        //        } else
        //        if (i % 20 == 19) {
        //            cout << "\n";
        //        }
    }
    cout << std::dec << "\n";

    cout << "-----------\n";

    // NEW parsing code

    // parse a position record (p. 139 in BD982 userguide)
    // ------------ header --------------
    int stx = Buffer[0];
    int rx_status = Buffer[1]; // big-endian; bit 1 signals low battery; rest is reserved
    //                cut most left bit; check if next bit is 1
    int low_battery = (rx_status % 128) / 64;

    int packet_type = Buffer[2];
    int data_length = Buffer[3];
    // ---------- data records -----------      CHAR: 1 byte, INT: 2 byte, LONG: 4 byte, DOUBLE: 8 byte

    int record_type = Buffer[4];
    int paging_information = Buffer[5]; // bit 7-4 is current page number; bit 3-0 is total page number
    int page_total = paging_information / (2 * 2 * 2 * 2); // bits 0-3
    int current_page = paging_information % (2 * 2 * 2 * 2); // bits 4-7

    int reply_number = Buffer[6]; // this number is identical for every page of one reply
    int record_interpretation = Buffer [7]; // CHAR  --> bitflags



    int latitude_index = 8;
    int longitude_index = 16;
    int altitude_index = 24;


    // - get all 8 bytes of latitude
    // - reverse byte order (NOT bit order!)
    // - calculate value according to IEEE double-precision format (DOUBLE)
    int latitude_msg[8] = {0}; // DOUBLE
    for (int i = 0; i < 8; i++) {
        latitude_msg[i] = Buffer[latitude_index + 7 - i];
    }
    cout << "latitude bytes, reversed:\n";
    for (int i = 0; i < 8; i++) {
        printf(" %.2x", latitude_msg[i]);
    }
    cout << "\n";



    static int byte_count = 8;

    bool latitude_bits[64] = {false};
    bool latitude_bits_reversed[64] = {false};

    // latitude_bits[bit_count] = {0};
    // get bits of DOUBLE
    for (int k = 0; k < 8; k++) {
        for (int i = byte_count - 1; i >= 0; i--) {
            latitude_bits[((k * 8) + i)] = 0 != (latitude_msg[k] & (1 << i));
        }
    }
    cout << "latitude bits:\n";
    for (int i = 0; i < 64; i++) {
        printf(" %i", latitude_bits[i]);
    }
    cout << "\n";





    cout << "latitude bits per byte reversed:\n";
    for (int i = 0; i < 64; i++) {
        printf(" %i", latitude_bits[i]);
    }
    cout << "\n";



    int sign_bit = latitude_bits[0];
    int exponent = -1023;
    int fraction = 0;
    for (int i = 1; i < 11; i++) {
        exponent = exponent + latitude_bits[i] * pow(2, i);
    }
    for (int i = 11; i < 64; i++) {
        fraction = latitude_bits[i] * pow(0.5, i);
    }

    double latitude_value = pow(-1, sign_bit) * pow(fraction, exponent);

    cout << std::dec << "  calculated latitude: " << latitude_value << " \n";




    int longitude_msg[8] = {0}; // DOUBLE
    int altitude_msg[8] = {0}; // DOUBLE
    int clock_offset[8] = {0}; // DOUBLE
    int freq_offset[8] = {0}; // DOUBLE
    int pdop[8] = {0}; // DOUBLE
    int latitude_rate[8] = {0}; // DOUBLE
    int longitude_rate[8] = {0}; // DOUBLE
    int altitude_rate[8] = {0}; // DOUBLE
    int GPS_msec_of_week[4] = {0}; // LONG
    int position_flags[4] = {0}; // CHAR
    int number_of_SVs = {0}; // CHAR

    //

    int *data = new int[bytesread];
    for (int i = 0; i < data_length; i++) {
        data[i] = Buffer[i + 8];
    }


    // ------------ footer -----------
    int checksum = Buffer[bytesread - 2];
    // check if data_length "hits" checksum
    int checksum2 = Buffer[4 + data_length];

    int etx = Buffer[bytesread - 1];

    for (int i = 0; i < length_; i++) {
        checksum_ += Buffer[i + 4];
    }

    // log to console
    printf("STX (expects 02): %.2x\n", stx);
    printf("rx_status: %.2x %i\n", rx_status, rx_status);
    printf("  low_battery: %i\n", low_battery);
    printf("packet_type: %.2x\n", packet_type);
    printf("LENGTH: %i\n", data_length);
    printf("record_type: %i\n", record_type);
    printf("paging_information: %.2x\n", paging_information);
    printf("  page_total: %i\n", page_total);
    printf("  current_page: %i\n", current_page);
    printf("reply_number: %.2x\n", reply_number);
    printf("record_interpretation: %.2x\n", record_interpretation);
    cout << "packet data: ";
    cout << std::dec << data << "\n";
    printf("checksum: %.2x %.2x\n", checksum, checksum2);
    printf("ETX (expects 03): %.2x\n", etx);
    cout << "----\n";
    //printf("latitude_msg: %.2x\n", latitude_msg);
    //printf("longitude_msg: %.2x\n", longitude_msg);
    //printf("altitude_msg: %.2x\n", altitude_msg);
    cout << "----\n";


    int data_sum = 0;
    for (int i = 0; i < data_length; i++) data_sum += Buffer[4 + i];
    int checksum_r = (rx_status + packet_type + data_sum + data_length) % 256;

    printf("checksum comparison: %.2x %.2x\n", checksum, checksum_r);
    // OLD parsing code

    for (int i = 0; i < bytesread; i++) {
        //	printf(" %.2x Hexa-decimal",(unsigned char)Buffer[i]);
        bin = (unsigned char) Buffer[i]; //
        m_SerialIO.binary(bin, binary); // binary conversion
    }

    m_SerialIO.alphatointeg(binary, value);
    cout << "value: " << value << "\n";
    cout << "-----------\n";

    double lat_fract = 0.0, lat_exp = 0.0, lon_fract = 0.0, lon_exp = 0.0, alt_fract = 0.0, alt_exp = 0.0;
    for (int j = 1; j <= 52; j++) {
        lat_fract = lat_fract + (value[75 + j]) * pow(0.5, j);
        lon_fract = lon_fract + (value[139 + j]) * pow(0.5, j);
        alt_fract = alt_fract + (value[203 + j]) * pow(0.5, j);
    }
    // write a function to convert from character to integer
    for (int h = 0; h <= 10; h++) {
        lat_exp = lat_exp + (value[75 - h]) * pow(2, h);
        lon_exp = lon_exp + (value[139 - h]) * pow(2, h);
        alt_exp = alt_exp + (value[203 - h]) * pow(2, h);
    }

    cout << value[64] << value[128] << value[192] << "\n";
    cout << value[64] << value[65] << value[66] << value[67] << "..\n";


    latt[0] = pow((-1), value[64])* ((lat_fract + 1) * pow(2, (lat_exp - 1023))*180);
    latt[1] = pow((-1), value[128])* ((lon_fract + 1) * pow(2, (lon_exp - 1023))*180);
    latt[2] = pow((-1), value[192])* ((alt_fract + 1) * pow(2, (alt_exp - 1023)));
    cout << "       latitude= " << latt[0] << "\tlongitude= " << latt[1] << "\taltitude= " << latt[2] << endl;

    // need to check if values were ok, right now just hardcoded true..
    success = true;
    return success;
}

