/*!
*****************************************************************
* seneka_dgps_node.cpp
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
* done          - Generation and publishing of error messages
* done          - Extract all fields of a position record message (especially dynamic length of sat-channel_numbers and prns...)
* done          - Publish all gps values to ros topic (maybe need a new message if navsatFix cannot take all provided values...)
* done          - Rewrite function structure of interpretData and connected functions.. (still in dev state... double check for memory leaks etc...!!!)
* unnecessary   - Extracting multi page messages from buffer...  (not needed for position records)
*               - Monitor frequency/quality/... of incoming data packets... --> inform ROS about bad settings (publishing rate <-> receiving rate)
* unnecessary   - Add more parameter handling (commandline, ...); document parameters and configuration
* done          - Clean up and improve code readability
* done          - Add/improve comments
* in progress   - Testing!
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

#include <seneka_dgps/SenekaDgps.h>
#include <seneka_dgps/Dgps.h>

/*****************************************************************/
/*************** main program seneka_dgps_node.cpp ***************/
/*****************************************************************/

int main(int argc, char** argv) {

    // ROS initialization; apply "DGPS" as node name;
    ros::init(argc, argv, "DGPS");

    SenekaDgps      cSenekaDgps;
    Dgps            cDgps;

    cSenekaDgps.extractDiagnostics(cDgps);

    int counter = 0;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // establish serial connection;

    bool port_opened = false;
    
    while (!port_opened && (counter < 10)) {

        // just executed if connection establishment needs a retry;
        if (counter > 0) {

            cSenekaDgps.message << "Retrying...";
            cSenekaDgps.publishDiagnostics(SenekaDgps::WARN);

        }

        port_opened = cDgps.open(cSenekaDgps.getPort().c_str(), cSenekaDgps.getBaud());
        cSenekaDgps.extractDiagnostics(cDgps);      

        counter++;
        sleep(1); // delay

    }

    // return false if connection establishment failed 10 times;
    if (!port_opened) {

        return 0;

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // test the communications link by sending protocol request "ENQ" (05h);
    // see BD982 manual, p. 65;

    bool    connection_is_ok    = false;    // connection check response
            counter             = 0;        // reset counter; count of how many times connection check has been retried;

    while (!connection_is_ok && (counter < 10)) {

        connection_is_ok = cDgps.checkConnection();
        cSenekaDgps.extractDiagnostics(cDgps);

        if (counter > 0) {

            cSenekaDgps.message << "Retrying...";
            cSenekaDgps.publishDiagnostics(SenekaDgps::WARN);

        }

        counter++;
        sleep(1); // delay

    }

    // return false if test of connection link failed 10 times;
    if (!connection_is_ok) {

        cSenekaDgps.message << "Testing the communications link finally failed! Device is not available!";
        cSenekaDgps.publishDiagnostics(SenekaDgps::ERROR);

        return 0;

    }

    /*************************************************/
    /*************** main program loop ***************/
    /*************************************************/

    ros::Rate loop_rate(cSenekaDgps.getRate());

    cSenekaDgps.message << "Initiating continuous requesting and publishing of DGPS data...";
    cSenekaDgps.publishDiagnostics(SenekaDgps::INFO);

    while (cSenekaDgps.nh.ok()) {

        // this...
        // -> requests position record packet from receiver
        // -> appends incoming data to ringbuffer
        // -> tries to extract valid packets (incl. checksum verification)
        // -> tries to read position record fields from valid packets
        // -> writes position record data into struct of type gps_data
        if(cDgps.getDgpsData()) {

            cSenekaDgps.extractDiagnostics(cDgps);

            // gathering data from DGPS instance and publishing it to given ROS topic
            cSenekaDgps.publishPosition(cDgps.getPosition());

        }

        else {

            cSenekaDgps.extractDiagnostics(cDgps);

        }

        // #ifndef NDEBUG

        // TESTING/DEBUGGING
        ROS_ERROR("Press ENTER to continue.");
        if (cin.get() == '\n') {}

        // #endif NDEBUG

        ros::spinOnce();
        loop_rate.sleep();

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    return 0;

}

/**************************************************/
/**************************************************/
/**************************************************/

#ifndef NDEBUG

// ##########################################################################
// ## dev-methods -> can be removed when not needed anymore                ##
// ##########################################################################

// context of this function needs to be created!!
//bool getFakePosition(double* latt) {
//    // set to true after extracting position values. method return value.
//    bool success = false;
//
//    int length;
//    unsigned char Buffer[1024] = {0};
//    int buffer_index = 0;
//    unsigned char data_buffer[1024] = {0};
//    int data_index = 0;
//    for (int i = 0; i < 1024; i++) Buffer[i] = '0';
//    char str[10];
//    char binary[10000] = {0};
//    int value[1000] = {0};
//    int open, y, bytesread, byteswrite, bin;
//
//    // see page 73 in BD982 user guide for packet specification
//    //  start tx,
//    //      status,
//    //          packet type,
//    //              length,
//    //                  type raw data,      [0x00: Real-Time Survey Data Record; 0x01: Position Record]
//    //                      flags,
//    //                          reserved,
//    //                              checksum,
//    //                                  end tx
//    unsigned char stx_ = 0x02;
//    unsigned char status_ = 0x00;
//    unsigned char packet_type_ = 0x56;
//    unsigned char length_ = 0x03;
//    unsigned char data_type_ = 0x01;
//    unsigned char etx_ = 0x03;
//
//    unsigned char checksum_ = status_ + packet_type_ + data_type_ + length_;
//    //(status_ + packet_type_ + data_type_ + 0 + 0 + length_)%256;
//
//
//
//    char message[] = {stx_, status_, packet_type_, length_, data_type_, 0x00, 0x00, checksum_, etx_}; // 56h command packet       // expects 57h reply packet (basic coding)
//
//    //        char message[]={ 0x05 };
//    length = sizeof (message) / sizeof (message[0]);
//
//    cout << "length of command: " << length << "\n";
//
//    //SerialIO dgps;
//    //open = dgps.open();
//    byteswrite = 9; //m_SerialIO.write(message, length);
//    printf("Total number of bytes written: %i\n", byteswrite);
//    std::cout << "command was: " << std::hex << message << "\n";
//    sleep(1);
//    bytesread = 118; //m_SerialIO.readNonBlocking((char*) Buffer, 1020);
//
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
//
//    printf("\nTotal number of bytes read: %i\n", bytesread);
//    cout << "-----------\n";
//    for (int i = 0; i < bytesread; i++) {
//        printf(" |%.2x| ", Buffer[buffer_index + i]);
//
//    }
//    cout << std::dec << "\n";
//
//    cout << "-----------\n";
//
//    packet_data incoming_packet;
//    Dgps temp_gps_dev = Dgps();
//    temp_gps_dev.interpretData(Buffer, bytesread, incoming_packet);
//
//
//    // need to check if values were ok, right now just hardcoded true..
//    success = true;
//    return success;
//}

#endif // NDEBUG