/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Project name: SENEKA
 * ROS stack name: windsensor
 * ROS package name: seneka_windsensor
 * Description:
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Ciby Mathew, email:Ciby.Mathew@ipa.fhg.de
 * Supervised by: Christophe Maufroy
 *
 * modified by: David Bertram, David.Bertram@ipa.fhg.de
 *
 * Date of creation: Jan 2013
 * Date of modification: Dec 2013
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing
 *     Engineering and Automation (IPA) nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <seneka_windsensor/windsensor.h>
#include <sstream>
#include <seneka_windsensor/WindData.h>

#include <stdio.h>
#include <stdlib.h>

using namespace std;
//####################
//#### node class ####

class WindSensorNode {
public:
    ros::NodeHandle nh;
    // Publishers
    ros::Publisher topicPub_wind;
    ros::Publisher topicPub_Diagnostic_;
    ros::Time syncedROSTime;
    // Constructor:              rate can use fractions of 1.0 ( 0.5 publishes once every 2 seconds )

    WindSensorNode(string topic = "") {
        nh = ros::NodeHandle("~");
        // handle topic parameter
        string default_topic = "/wind";
        if (topic == "") {
            if (nh.hasParam("windsensor_topic")) {
                nh.getParam((string)"windsensor_topic", topic);
            } else {
                topic = default_topic;
                ROS_WARN("Used default parameter for windsensor_topic ( \"/wind\" )");
                nh.setParam("windsensor_topic", topic);
            }
        }

        syncedROSTime = ros::Time::now();
        // advertise topics
        topicPub_wind = nh.advertise<seneka_windsensor::WindData > (topic, 1);
        topicPub_Diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticArray > ("/diagnostics", 1);
    }

    // Destructor

    ~WindSensorNode() {
    }

    ros::NodeHandle get_NodeHandle() {
        return nh;
    }

    void publishWindsensor(float* sensor_data, string *windsensor_units) {
        seneka_windsensor::WindData value;
        value.wind_speed = sensor_data[0];
        value.wind_speed_unit = windsensor_units[0];
        value.wind_direction = sensor_data[1];
        value.wind_direction_unit = windsensor_units[1];
        value.temperature = sensor_data[2];
        value.temperature_unit = windsensor_units[2];

        topicPub_wind.publish(value);

        //	 ROS_INFO("...publishing wind of windsensor");
        diagnostic_msgs::DiagnosticArray diagnostics;
        diagnostics.status.resize(1);
        diagnostics.status[0].level = 0;
        diagnostics.status[0].name = nh.getNamespace();
        diagnostics.status[0].message = "Wind sensor running";
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

int main(int argc, char** argv) {

    // set default parameters
    string topic = "/wind"; // ros topic
    string serial_port = "/dev/ttyUSB0"; // serial device/port
    int baudrate = 4800; // Baud
    double publishrate = 1.0; // Hz

    int speed_unit = 0;
    // knots = 0
    // m/s   = 1
    // km/h  = 2

    int direction_unit = 0;
    // degree = 0
    // radian = 1

    int temperature_unit = 0;
    // centigrade = 0
    // fahrenheit = 1


    // reading parameters has following priorities:
    //  highest:    - command line
    //              - parameter server
    //              - default values

    // read given parameters from commandline and overwrite default values
    bool topic_given = false;
    bool sp_given = false;
    bool br_given = false;
    bool pr_given = false;
    bool su_given = false;
    bool du_given = false;
    bool tu_given = false;

    for (int i = 1; i < argc; i = i + 2) {
        string parameter;
        string value_string;

        if (argv[i][0] == '-') {

            parameter = ((string) argv[i]).substr(1, ((string) argv[i]).length());
            value_string = argv[i + 1];

            if (parameter == "topic") {
                topic = (string) value_string;
                topic_given = true;
            }
            if (parameter == "serial_port") {
                serial_port = (string) value_string;
                sp_given = true;
            }
            if (parameter == "baudrate") {
                baudrate = atoi(value_string.c_str());
                br_given = true;
            }
            if (parameter == "publishrate") {
                publishrate = (double) atof(value_string.c_str());
                pr_given = true;
            }
            if (parameter == "speed_unit") {
                speed_unit = atoi(value_string.c_str());
                su_given = true;
            }
            if (parameter == "direction_unit") {
                direction_unit = atoi(value_string.c_str());
                du_given = true;
            }
            if (parameter == "temperature_unit") {
                temperature_unit = atoi(value_string.c_str());
                tu_given = true;
            }
        }

    }

    // initialize ROS node, specify name of node
    ros::init(argc, argv, "windsensor");

    // spawn ros node
    WindSensorNode rosNode;    
    if (topic_given)   { rosNode = WindSensorNode(topic);}
    else               { rosNode = WindSensorNode();}
    ros::NodeHandle nh = rosNode.get_NodeHandle();
    
    // try to read parameters from rosNode, if not in argv..
    if (!sp_given && !nh.hasParam("windsensor_port")) ROS_WARN("Used default parameter for windsensor_port ( \"/dev/ttyUSB0\" )");
    else if (!sp_given && nh.hasParam("windsensor_port")) nh.getParam("windsensor_port", serial_port);
    else    nh.setParam("windsensor_port", serial_port);


    if (!br_given && !nh.hasParam("windsensor_baudrate")) ROS_WARN("Used default parameter for windsensor_baudrate( \"4800\" [Baud])");
    else if (!br_given && nh.hasParam("windsensor_baudrate")) nh.getParam("windsensor_baudrate", baudrate);
    else nh.setParam("windsensor_baudrate", baudrate);


    if (!pr_given && !nh.hasParam("windsensor_publishrate")) ROS_WARN("Used default parameter for windsensor_publishrate ( \"1.0\" [Hz] )");
    else if (!pr_given && nh.hasParam("windsensor_publishrate")) nh.getParam("windsensor_publishrate", publishrate );
    else { nh.setParam("windsensor_publishrate", publishrate); }

    if (!su_given && !nh.hasParam("windsensor_speed_unit")) ROS_WARN("Used default parameter for windsensor_speed_unit ( \"knots\" )");
    else if (!su_given && nh.hasParam("windsensor_speed_unit")) nh.getParam("windsensor_speed_unit", speed_unit);
    else nh.setParam("windsensor_speed_unit", speed_unit);


    if (!du_given && !nh.hasParam("windsensor_direction_unit")) ROS_WARN("Used default parameter for windsensor_direction_unit ( \"degree\" )");
    else if (!du_given && nh.hasParam("windsensor_direction_unit")) nh.getParam("windsensor_direction_unit", direction_unit);
    else nh.setParam("windsensor_direction_unit", direction_unit);


    if (!tu_given && !nh.hasParam("windsensor_temperature_unit")) ROS_WARN("Used default parameter for windsensor_temperature_unit ( \"centigrade\" )");
    else if (!tu_given && nh.hasParam("windsensor_temperature_unit")) nh.getParam("windsensor_temperature_unit", temperature_unit);
    else nh.setParam("windsensor_temperature_unit", temperature_unit);

    // spawn sensor object and initialize with units for: speed, direction, temperature
    windsensor sensor(speed_unit, direction_unit, temperature_unit);

    // display configuration
    stringstream configuration_message;
    configuration_message << "windsensor is using following configuration:\n topic: " << topic;
    configuration_message << "\n serial_port: " << serial_port;
    configuration_message << "\n baudrate: " << baudrate;
    configuration_message << "\n publishrate: " << publishrate;
    configuration_message << "\n speed_unit: " << speed_unit;
    configuration_message << "\n direction_unit: " << direction_unit;
    configuration_message << "\n temperature_unit: " << temperature_unit << "\n";
    ROS_INFO(configuration_message.str().c_str());

    // main part
    bool windsensor_opened = false;
    float windsensor_values[3] = {0};
    string windsensor_units[3] = {""};
    bool publishing = false;
    // try to connect to sensor
    while (!windsensor_opened) // open connection to windsensor
    {
        ROS_INFO("opening wind sensor... (port:%s)", serial_port.c_str());
        windsensor_opened = sensor.open(serial_port.c_str(), baudrate);
        if (!windsensor_opened) // check, connection was successfully opened
        {
            ROS_ERROR("no device available on port %s. Will retry every second.", serial_port.c_str());
            rosNode.publishError("windsensor not available");
        }
        sleep(1); // wait for Windsensor to get ready if successfull, or wait before retrying
    }
    ROS_INFO("serial connection opened successfully on port %s with baudrate = %i", serial_port.c_str(), baudrate);
    ros::Rate loop_rate(publishrate); // Hz
    // main loop:
    //  - read values
    //  - publish values
    //  - wait for next iteration
    while (rosNode.nh.ok()) {
        ROS_DEBUG("trying to access windsensor");
        // try to read values
        bool bSuccess = sensor.read(windsensor_values, windsensor_units);
        if (bSuccess == true) {
            // publish values
            ROS_DEBUG("publishing direction and speed of windsensor");
            rosNode.publishWindsensor(windsensor_values, windsensor_units);
            if (publishing == false) {
                publishing = true;
                stringstream message;
                message << (string) "started publishing to topic: ";
                message << (string) topic;
                ROS_INFO(message.str().c_str());
            }
        } else {
            publishing = false;

            ROS_ERROR("error reading from windsensor");
        }
        //     sleep and wait for next iteration
        ros::spinOnce();
        loop_rate.sleep();
    }
    // close connection to windsensor
    sensor.close();
    return 0;
}

