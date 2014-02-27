/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Project name:  SENEKA 
 * ROS stack name: DGPS
 * ROS package name: seneka_dgps
 * Description:
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Author: Ciby Mathew, email:Ciby.Mathew@ipa.fhg.de
 * Supervised by: Christophe Maufroy
 *
 * Date of creation: Jan 2013
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
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

//##################
//#### includes ####

// standard includes
//--

// ROS includes


// ROS message includes
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <diagnostic_msgs/DiagnosticArray.h>

// ROS service includes
//--
// external includes
#include <seneka_dgps/Dgps.h>

#include <sstream>


// default parameters
string position_topic = "/position";
string diagnostics_topic = "/diagnostics";
string serial_port = "/dev/ttyUSB0";
int serial_baudrate = 38400;
int publishrate = 1;

string IntToString(int a) {
    //string str;
    ostringstream temp;
    temp << a;
    return temp.str();
}

//####################
//#### node class ####

class DgpsNode {
public:
    ros::NodeHandle nh;
    // topics to publish
    ros::Publisher topicPub_position;
    ros::Publisher topicPub_Diagnostic_;

    // topics to subscribe, callback is called for new messages arriving
    //--

    // service servers
    //--

    // service clients
    //--


    // global variables
    std::string port;
    int baud;
    int rate;
    //		bool inverted;
    //		std::string frame_id;
    ros::Time syncedROSTime;
    //		unsigned int syncedSICKStamp;
    //		bool syncedTimeReady;



    // Constructor

    DgpsNode() {
        // create a handle for this node, initialize node
        nh = ros::NodeHandle("~");
        if (!nh.hasParam("port"))ROS_WARN("Used default parameter for port (%s)", serial_port.c_str());
        nh.param("port", port, std::string(serial_port));

        if (!nh.hasParam("baud")) ROS_WARN("Used default parameter for baud (%i)", serial_baudrate);
        nh.param("baud", baud, serial_baudrate);


        if (!nh.hasParam("rate")) ROS_WARN("Used default parameter for rate (%i)", publishrate);
        nh.param("rate", rate, publishrate);

        syncedROSTime = ros::Time::now();
        //	syncedTimeReady = false;
        // implementation of topics to publish
        topicPub_position = nh.advertise<sensor_msgs::NavSatFix > (position_topic.c_str(), 1);
        topicPub_Diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticArray > (diagnostics_topic.c_str(), 1);
        // implementation of topics to subscribe
        //--

        // implementation of service servers
        //--
    }



    // Destructor

    ~DgpsNode() {
    }

    // topic callback functions
    // function will be called when a new message arrives on a topic
    //--
    // service callback functions
    // function will be called when a service is querried
    //--

    // other function declarations

    void publishPosition(double* lat) {
        sensor_msgs::NavSatFix positions;
        positions.latitude = lat[0];
        positions.longitude = lat[1];
        positions.altitude = lat[2];
        topicPub_position.publish(positions);
        //			 ROS_INFO("...publishing position of DGps");

        //Diagnostics
        diagnostic_msgs::DiagnosticArray diagnostics;
        diagnostics.status.resize(1);
        diagnostics.status[0].level = 0;
        diagnostics.status[0].name = nh.getNamespace();
        diagnostics.status[0].message = "Dgps running";
        topicPub_Diagnostic_.publish(diagnostics);
    }

    void publishError(std::string error_str) {
        diagnostic_msgs::DiagnosticArray diagnostics;
        diagnostics.status.resize(1);
        diagnostics.status[0].level = 2;
        diagnostics.status[0].name = nh.getNamespace();
        diagnostics.status[0].message = error_str;
        topicPub_Diagnostic_.publish(diagnostics);
    }
};

void to_bin(int dec, char* binary) {
    int dec_in = dec;
    char bin8[] = "00000000";
    // sure we want to use --pos instead of pos-- ??
    for (int pos = 7; pos >= 0; pos--) {
        if (dec % 2)
            bin8[pos] = '1';
        dec /= 2;
    }

    cout << "binary of: " << dec_in << " --> " << bin8 << "\n";
    strcat(binary, bin8);
}

void alphatointeg(char* binary, int* value) {
    for (int i = 0; i < strlen(binary); i++) {
        value[i] = (binary[i] - '0');
    }
}

bool getFakePosition(double* latt) {
    // set to true after extracting position values. method return value.
    bool success = false;

    int length;
    unsigned char Buffer[1024] = {0};
    int buffer_index = 0;
    unsigned char data_buffer[1024] = {0};
    int data_index = 0;
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
    byteswrite = 9; //m_SerialIO.write(message, length);
    printf("Total number of bytes written: %i\n", byteswrite);
    std::cout << "command was: " << std::hex << message << "\n";
    sleep(1);
    bytesread = 118; //m_SerialIO.readNonBlocking((char*) Buffer, 1020);

    string test_packet = " |02|  |20|  |57|  |0a|  |0c|  |11|  |00|  |00|  |00|  |4d|  |01|  |e1|  |01|  |e1|  |af|  |03|  |02|  |20|  |57|  |60|  |01|  |11|  |00|  |00|  |3f|  |d1|  |54|  |8a|  |b6|  |cf|  |c6|  |8d|  |3f|  |a9|  |e0|  |bd|  |3f|  |29|  |c8|  |f7|  |40|  |80|  |ae|  |2a|  |c9|  |7b|  |b7|  |11|  |c0|  |fd|  |d3|  |79|  |61|  |fb|  |23|  |99|  |c0|  |92|  |ca|  |3b|  |46|  |c7|  |05|  |15|  |3f|  |ff|  |9f|  |23|  |e0|  |00|  |00|  |00|  |be|  |44|  |16|  |1f|  |0d|  |84|  |d5|  |33|  |be|  |2c|  |8b|  |3b|  |bb|  |46|  |eb|  |85|  |3f|  |b5|  |ec|  |f0|  |c0|  |00|  |00|  |00|  |08|  |06|  |f0|  |d8|  |d4|  |07|  |0f|  |0d|  |13|  |01|  |0c|  |07|  |04|  |07|  |0d|  |08|  |09|  |0a|  |1a|  |1c|  |5c|  |03|";
           test_packet = " |02|  |20|  |57|  |0a|  |0c|  |11|  |00|  |00|  |00|  |4d|  |01|  |e1|  |01|  |e1|  |af|  |03|  |02|  |20|  |57|  |60|  |01|  |11|  |00|  |00|  |3f|  |d1|  |54|  |8a|  |b6|  |cf|  |c6|  |8d|  |3f|  |a9|  |e0|  |bd|  |3f|  |29|  |c8|  |f7|  |40|  |80|  |ae|  |2a|  |c9|  |7b|  |b7|  |11|  |c0|  |fd|  |d3|  |79|  |61|  |fb|  |23|  |99|  |c0|  |92|  |ca|  |3b|  |46|  |c7|  |05|  |15|  |3f|  |ff|  |9f|  |23|  |e0|  |00|  |00|  |00|  |be|  |44|  |16|  |1f|  |0d|  |84|  |d5|  |33|  |be|  |2c|  |8b|  |3b|  |bb|  |46|  |eb|  |85|  |3f|  |b5|  |ec|  |f0|  |c0|  |00|  |00|  |00|  |08|  |06|  |f0|  |d8|  |d4|  |07|  |0f|  |0d|  |13|  |01|  |0c|  |07|  |04|  |07|  |0d|  |08|  |09|  |0a|  |1a|  |1c|  |5c|  |03|";

    for (int i = 0; i < bytesread; i++) {
        char hex_byte1 = test_packet[i * 6 + 2];
        char hex_byte2 = test_packet[i * 6 + 3];

        if (hex_byte1 > 96) hex_byte1 -= 87; // 96-9
        else hex_byte1 -= 48;
        if (hex_byte2 > 96) hex_byte2 -= 87; // 96-9
        else hex_byte2 -= 48;

        Buffer[i] = hex_byte1 * 16 + hex_byte2;
        printf("%x%x-%i  ", hex_byte1, hex_byte2, Buffer[i]);

    }
    cout << "\n";

    printf("\nTotal number of bytes read: %i\n", bytesread);
    cout << "-----------\n";
    for (int i = 0; i < bytesread; i++) {
        printf(" |%.2x| ", Buffer[buffer_index + i]);

    }
    cout << std::dec << "\n";

    cout << "-----------\n";

    packet_data incoming_packet;
    Dgps temp_gps_dev = Dgps();
    temp_gps_dev.receiveData(Buffer, bytesread, incoming_packet);


    // need to check if values were ok, right now just hardcoded true..
    success = true;
    return success;
}


//
////#######################
//#### main programm ####

int main(int argc, char** argv) {
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "Dgps");
    DgpsNode rosNode;
    Dgps dgps;




    cout << "modified to use fakeGPS data.. a recorded sample (70569 Stuttgart, Nobelstraße 12)" << endl;

    int publishRate = rosNode.rate;
    int baudRate = rosNode.baud;
    bool dgpsSensor_opened = true; //false;
    bool success_getPosition = false, connection_OK = false;
    double dgpsData[100] = {0};
    while (!dgpsSensor_opened) {


        ROS_INFO("Opening DGPS... (port: %s , baudrate: %s )", rosNode.port.c_str(), IntToString(baudRate).c_str());
        dgpsSensor_opened = dgps.open(rosNode.port.c_str(), baudRate);
        // check, if it is the first try to open scanner
        if (!dgpsSensor_opened) {
            ROS_ERROR("...DGPS not available on port %s. Will retry every second.", rosNode.port.c_str());
            rosNode.publishError("...DGPS not available on port");
        }
        sleep(1); // wait for Dgps to get ready if successfull, or wait before retrying
    }  
    //	ROS_INFO("...DGPS opened successfully on port %s",nodeClass.port.c_str());
    // main loop
    ros::Rate loop_rate(publishRate); // Hz

    connection_OK = true; //dgps.checkConnection();


    if (!connection_OK) {
        cout << "protocol request failed (05h): check cables, adapters, settings, ...";
    } else {

        bool once = false;
        while (rosNode.nh.ok()) {
            // read values
            ROS_DEBUG("Reading DGPS...");

            //success_getPosition = dgps.getPosition(dgpsData);


            if (!once) {success_getPosition = getFakePosition(dgpsData);
            once = true;
            }else{
            if (success_getPosition) {
                ROS_INFO("...publishing position of DGPS: %1f, %1f, %1f", dgpsData[0], dgpsData[1], dgpsData[2]);

                rosNode.publishPosition(dgpsData);

            } else {
                ROS_WARN("...no Values available");
            }
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    return 0;
}


