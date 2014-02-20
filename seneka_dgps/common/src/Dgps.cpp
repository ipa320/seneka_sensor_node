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

    // test command "05h"       // expects test reply "06h"
    char message[] = {0x05};
    int length = sizeof (message) / sizeof (message[0]);
    unsigned char Buffer[1024] = {0};
    int bytesWritten = m_SerialIO.write(message, length);
    // wait a second
    sleep(1);
    int bytesRead = m_SerialIO.readNonBlocking((char*) Buffer, 1020);

    if (bytesRead <= 0) {
        // error, nothing received --> wait for at least 500ms and try again few times
        cout << "error, nothing received";
    } else {
        cout << "received:\n";
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

    if (Buffer[0]==6) success = true;
    else success = false;

    return success;
}

bool Dgps::getPosition(double* latt) {
    // set to true after extracting position values. method return value.
    bool success = false;

    int length;
    unsigned char Buffer[1024] = {0};
    char str[10];
    char binary[10000] = {0};
    int value[1000] = {0};
    int open, y, bytesread, byteswrite, bin;

    //  start tx,
    //      status,
    //          packet type,
    //              length,
    //                  type raw data,
    //                      flags,
    //                          reserved,
    //                              checksum,
    //                                  end tx
    char message[] = {0x02, 0x00, 0x56, 0x03, 0x01, 0x00, 0x00, 0x5a, 0x03}; // 56h command packet       // expects 57h reply packet (basic coding)

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
    cout << Buffer << "\n";
    cout << "-----------\n";
//    int buffer_size = sizeof (Buffer) / sizeof (Buffer[0]);
    for (int i = 0; i < bytesread; i++) {
        printf("%.2x ", Buffer[i]);
        //cout << std::hex << Buffer[i];
//        if (i % 2 == 1) {
//            cout << " ";
//        } else
            if (i % 20 == 19) {
            cout << "\n";
        }
    }
    cout << std::dec << "\n";

    cout << "-----------\n";

// NEW parsing code

    int stx = Buffer[0];
    int rx_status = Buffer[1];
    int packet_type = Buffer[2];
    int data_length = Buffer[3];
    int checksum = Buffer[data_length+3];
    int etx = Buffer[data_length+4];

    int *data = new int[data_length];
    
    for (int i = 0;i < data_length; i++){
        data[i] = Buffer[i+4];
    }
    cout << "packet data:" << "\n";
    cout << std::dec << data << "\n";
    cout << "----\n";


// OLD parsing code

    for (int i = 0; i < bytesread; i++) {
        //	printf(" %.2x Hexa-decimal",(unsigned char)Buffer[i]);
        bin = (unsigned char) Buffer[i]; //
        m_SerialIO.binary(bin, binary); // binary conversion
    }

    m_SerialIO.alphatointeg(binary, value);
    cout << value << "\n";
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
    latt[0] = pow((-1), value[64])* ((lat_fract + 1) * pow(2, (lat_exp - 1023))*180);
    latt[1] = pow((-1), value[128])* ((lon_fract + 1) * pow(2, (lon_exp - 1023))*180);
    latt[2] = pow((-1), value[192])* ((alt_fract + 1) * pow(2, (alt_exp - 1023)));
    cout << "latitude= " << latt[0] << "\tlongitude= " << latt[1] << "\taltitude= " << latt[2] << endl;

    // need to check if values were ok, right now just hardcoded true..
    success = true;
    return success;
}

