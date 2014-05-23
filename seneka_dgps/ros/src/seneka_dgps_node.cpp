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
* Description: The seneka_dgps package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
* It implements a GNU/Linux driver for the Trimble BD982 GNSS Receiver Module as well as a ROS publisher node "DGPS", which acts as a wrapper for the driver.
* The ROS node "DGPS" publishes GPS data gathered by the DGPS device driver.
* This package might work with other hardware and can be used for other purposes, however the development has been specifically for this project and the deployed sensors.
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

    // gather latest diagnostic messages;
    cSenekaDgps.extractDiagnostics(cDgps);

    int counter = 0;

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // establish serial connection;

    bool port_opened = false;
    
    while (!port_opened && (counter < 10)) {

        // is just getting executed if connection establishment needs a retry;
        if (counter > 0) {

            cSenekaDgps.message << "Retrying...";
            cSenekaDgps.publishDiagnostics(SenekaDgps::WARN);

        }

        port_opened = cDgps.open(cSenekaDgps.getPort().c_str(), cSenekaDgps.getBaud());
        cSenekaDgps.extractDiagnostics(cDgps);      

        counter++;

        sleep(1); // delay;

    }

    // return false if connection establishment failed 10 times;
    if (!port_opened) {

        cSenekaDgps.message << "Establishing serial connection finally failed. Device is not available.";
        cSenekaDgps.publishDiagnostics(SenekaDgps::ERROR);

        return 0;

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    // test the communication link by sending protocol request "ENQ" (05h);
    // see Trimble BD982 GNSS Receiver manual, p. 65;

    bool    connection_is_ok    = false;    // connection check response;
            counter             = 0;        // reset counter; count of how many times connection check has been retried;

    int i = 0;

    while (!connection_is_ok && (counter < 10)) {

        if (counter > 0) {

            cSenekaDgps.message << "Retrying...";
            cSenekaDgps.publishDiagnostics(SenekaDgps::WARN);

            cDgps.open(cSenekaDgps.getPort().c_str(), cSenekaDgps.getBaud());

        }

        connection_is_ok = cDgps.checkConnection();
        cSenekaDgps.extractDiagnostics(cDgps);

        counter++;

        sleep(1); // delay;

    }

    // return false if test of connection link failed 10 times;
    if (!connection_is_ok) {

        cSenekaDgps.message << "Testing the communication link finally failed. Device is not available.";
        cSenekaDgps.publishDiagnostics(SenekaDgps::ERROR);

        return 0;

    }

    /*************************************************/
    /*************** main program loop ***************/
    /*************************************************/

    ros::Rate loop_rate(cSenekaDgps.getRate());

    cSenekaDgps.message << "Initiate continuous requesting and publishing of DGPS data...";
    cSenekaDgps.publishDiagnostics(SenekaDgps::INFO);

    while (cSenekaDgps.nh.ok()) {

        // request GPS data from GPS device;
        // hereby called functions analyze the received packet in-depth, structure it, extract and finnaly serve GPS data;
        // if everything works fine, GPS data is getting stored in Dgps::GpsData gps_data;
        if(cDgps.getDgpsData()) {

            cSenekaDgps.extractDiagnostics(cDgps);

            // gathering data from DGPS instance and publishing it to given ROS topic;
            cSenekaDgps.publishPosition(cDgps.getPosition());

        }

        else {

            cSenekaDgps.extractDiagnostics(cDgps);

        }

        #ifndef NDEBUG

        // stop here after one cycle;
        ROS_ERROR("Press ENTER to continue.");
        if (cin.get() == '\n') {}

        #endif

        ros::spinOnce();
        loop_rate.sleep(); // delay;

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    return 0;

}

/**************************************************/
/**************************************************/
/**************************************************/