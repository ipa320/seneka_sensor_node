/*!
*****************************************************************
* seneka_control_interface.cpp
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_control_interface
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
* 
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
* Modified xx/20xx: 
*
* Description:
* The seneka_can package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
* This package might work with other hardware and can be used for other purposes, 
* however the development has been specifically for this project and the deployed sensors.
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

#include <ros/ros.h>
#include <seneka_trunk_control/SenekaTrunk.h>
#include <seneka_leg_control/SenekaLeg.h>
#include <seneka_tilt_control/SenekaTilt.h>

#include <iostream>
#include <string>

using namespace std;

/*********************************************/
/*************** main function ***************/
/*********************************************/

int main(int argc, char *argv[]) {

  // ROS initialization; apply "seneka_control_interface" as node name;
  ros::init(argc, argv, "seneka_control_interface");

  ros::NodeHandle nh;

  SenekaTrunk trunk;
  SenekaTilt  tilt;
  SenekaLeg   leg1;
  SenekaLeg   leg2;
  SenekaLeg   leg3;

  ros::Rate loop_rate(1); // [] = Hz;

  while(nh.ok()) {

    string command;
    string answer;

    cout << "\nEnter command: ";
    getline(cin, command);

    /**************************************************/
    /**************************************************/
    /**************************************************/

    if (command == "trunk run") {
      trunk.run();
    }

    else if (command == "trunk stop") {
      trunk.stop();
    }

    else if (command == "trunk turn negative") {
      trunk.turnNegative();
    }

    else if (command == "trunk turn positive") {
      trunk.turnPositive();
    }

    else if (command == "trunk set mode") {
      cout << "\nEnter trunk mode: ";
      getline(cin, answer);
      if (answer == "endless") {
        trunk.setMode(SenekaTrunk::ENDLESS);
      }
      else if (answer == "custom") {
        trunk.setMode(SenekaTrunk::CUSTOM);
      }
      else if (answer == "once") {
        trunk.setMode(SenekaTrunk::ONCE);
      }
      else {
        cout << "\nUnknown command";
      }
    }

    else if (command == "trunk set direction") {
      cout << "\nEnter trunk direction: ";
      getline(cin, answer);
      if (answer == "negative") {
        trunk.setDirection(SenekaTrunk::NEGATIVE);
      }
      else if (answer == "positive") {
        trunk.setDirection(SenekaTrunk::POSITIVE);
      }
      else {
        cout << "\nUnknown command";
      }
    }

    else if (command == "trunk set target position") {
      cout << "\nEnter trunk target position: ";
      unsigned char target_position;
      cin >> target_position;
      trunk.setTargetPosition(target_position);
    }

    else if (command == "trunk set target velocity") {
      cout << "\nEnter trunk target velocity: ";
      unsigned char target_velocity;
      cin >> target_velocity;
      trunk.setTargetVelocity(target_velocity);
    }

    else if (command == "trunk set sensitivity") {
      cout << "\nEnter trunk sensitivity: ";
      unsigned char sensitivity;
      cin >> sensitivity;
      trunk.setSensitivity(sensitivity);
    }

    else if (command == "trunk get current position") {
      cout << "\nCurrent trunk position: " << trunk.getCurrentPosition();
    }

    else if (command == "trunk get target position") {
      cout << "\nTarget trunk position: " << trunk.getTargetPosition();
    }

    else if (command == "trunk get current velocity") {
      cout << "\nCurrent trunk velocity: " << trunk.getCurrentVelocity();
    }

    else if (command == "trunk get target velocity") {
      cout << "\nTarget trunk velocity: " << trunk.getTargetVelocity();
    }

    else if (command == "trunk get sensitivity") {
      cout << "\nCurrent trunk sensitivity: " << trunk.getSensitivity();
    }

    /**************************************************/
    /**************************************************/
    /**************************************************/

    else {
      cout << "\nUnknown command";
    }

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;

}

/********************************************/
/********************************************/
/********************************************/