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
 * ROS stack name: SENEKA
 * ROS package name: Dgps
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: ciby mathew, email:ciby.mathew@ipa.fhg.de
 * Supervised by: ciby mathew, email:ciby.mathew@ipa.fhg.de
 *
 * Date of creation: Jan 2009
 * ToDo:
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
#ifndef _Dgps_H
#define _Dgps_H
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


class Dgps
{
public:

	// Constructor
	Dgps();

	// Destructor
	~Dgps();

	/**
	 * Opens serial port.
	 * @param pcPort used "COMx" or "/dev/ttyUSB0"
	 * @param iBaudRate baud rate
	 */
	bool open(const char* pcPort, int iBaudRate);
	bool getPosition(double* lat);
        bool checkConnection();

private:
	// Constants
	// Components
	SerialIO m_SerialIO;
	// Functions

	unsigned int getUnsignedWord(unsigned char msb, unsigned char lsb)
	{
		return (msb << 8) | lsb;
	}

	unsigned int createCRC(unsigned char *ptrData, int Size);

};
#endif //

