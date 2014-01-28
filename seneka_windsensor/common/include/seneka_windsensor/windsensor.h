/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_sick_s300
 * Description:
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
#ifndef _windsensor_H
#define _windsensor_H
#include "SerialIO.h"
#include <math.h>
#include <iostream>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <cstdlib>
#include <string>
#include<stdio.h>
#include<stdlib.h>
#include <cstring>
using namespace std;

class windsensor
{
public:
	// Constructor
	windsensor(int in_speed_unit, int in_direction_unit, int in_temperature_unit);

	// Destructor
	~windsensor();

        int sensor_port;
        int sensor_baudrate;
        bool connected;
        int speed_unit;         // knots = 0
                                // m/s   = 1
                                // km/h  = 2
        int direction_unit;     // degree = 0
                                // radian = 1
        int temperature_unit;   // centigrade = 0
                                // fahrenheit = 1

        float convert_speed_from_knots(float knots, int unit);
        float convert_direction_from_degree(float degree, int unit);
        float convert_temperature_from_centigrade(float centigrade, int unit);

        bool compare_checksum(string fields[], std::string line_for_checksum);
        bool extract_sensordata_from_buffer(unsigned char *input, float sensor_values[]);
        
        bool read(float sensor_values[], string sensor_units[]);
        bool open(const char* pcPort, int iBaudRate);
        void close();

private:

	SerialIO m_SerialIO;

};
#endif //

