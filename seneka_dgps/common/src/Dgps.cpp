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
    int retry_delay = 1000000;      // in microSeconds

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
            cout << "protocol request successful.";
        } else {
            success = false;
            cout << "protocol request failed. retrying..." << "\n";
            
        };
    }
    return success;
}

bool Dgps::getPosition(double* latt) {
    // set to true after extracting position values. method return value.
    bool success = false;

    int length;
    unsigned char Buffer[1024] = {0};
    for (int i = 0; i<1024; i++) Buffer[i]='0';
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

    unsigned char checksum_ = (status_ + packet_type_ + data_type_ + 0 + 0 + length_)%256;

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
    int rx_status = Buffer[1];      // big-endian; bit 1 signals low battery; rest is reserved
    //                cut most left bit; check if next bit is 1
       int low_battery = (rx_status % 128)  / 64;

    int packet_type = Buffer[2];
    int data_length = Buffer[3];
// ---------- data records -----------

    int record_type = Buffer[4];
    int paging_information = Buffer[5];  // bit 7-4 is current page number; bit 3-0 is total page number
       int page_total = paging_information /(2*2*2*2); // bits 0-3
       int current_page = paging_information % (2*2*2*2); // bits 4-7

       int reply_number = Buffer[6]; // this number is identical for every page of one reply
    int record_interpretation = Buffer [7];

    int latitude_msg = Buffer[8];
    int longitude_msg = Buffer[9];
    int altitude_msg = Buffer[10];
    int clock_offset = Buffer[11];
    int freq_offset = Buffer[12];
    int pdop = Buffer[13];
    int latitude_rate = Buffer[14];

    int *data = new int[data_length];
    for (int i = 0; i < data_length; i++) {
        data[i] = Buffer[i + 8];
    }


// ------------ footer -----------
    int checksum = Buffer[bytesread - 2];
    // check if data_length "hits" checksum
    int checksum2 = Buffer[4+data_length];
    int etx = Buffer[bytesread -1];



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
    printf( "ETX (expects 03): %.2x\n", etx);
    cout << "----\n";
    printf("latitude_msg: %.2x\n", latitude_msg);
    printf("longitude_msg: %.2x\n", longitude_msg);
    printf("altitude_msg: %.2x\n", altitude_msg);
    cout << "----\n";


    int data_sum = 0;
    for (int i = 0; i<data_length; i++) data_sum += Buffer[4+i];
    int checksum_r = (rx_status + packet_type + data_sum + data_length)%256;

    printf("checksum comparison: %.2x %.2x\n", checksum, checksum_r);
    // OLD parsing code

    for (int i = 0; i < bytesread; i++) {
        //	printf(" %.2x Hexa-decimal",(unsigned char)Buffer[i]);
        bin = (unsigned char) Buffer[i]; //
        m_SerialIO.binary(bin, binary); // binary conversion
    }

    m_SerialIO.alphatointeg(binary, value);
    cout <<"value: " << value << "\n";
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

