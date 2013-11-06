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
 * ROS stack name: windsensor
 * ROS package name: seneka_windsensor
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
 * Date of modification: Oct 2013
 *
 * ToDo:
 * -- clean up
 * -- restructure code
 * -- test
 *
 * ToDo - extra features:
 * ++ writing to windsensor/ setting windsensor-internal parameters possible?
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
#include <seneka_windsensor/windsensor.h>
using namespace std;

//-----------------------------------------------

typedef unsigned char BYTE;

//-----------------------------------------------

windsensor::windsensor() {
    //set the units of wind sped and direction
    //	speed=1;// to change to knot
    //	angle=1; //to change to degree
}

//-------------------------------------------

windsensor::~windsensor() {
    m_SerialIO.close();
}

void windsensor::close() {
    m_SerialIO.close();
}

// ---------------------------------------------------------------------------

bool windsensor::open(const char* pcPort, int iBaudRate) {
    int bRetSerial;
    // forwindsensor :default is 4800
    if (iBaudRate != 4800)
        return false;
    // initialize Serial Interface
    m_SerialIO.setBaudRate(iBaudRate);
    m_SerialIO.setDeviceName(pcPort);
    bRetSerial = m_SerialIO.open();
    if (bRetSerial == 0) {
        // Clears the read and transmit buffer.
        //	    m_iPosReadBuf2 = 0;
        m_SerialIO.purge();
        return true;
    } else {
        return false;
    }
}



            // TODO: David:     NMEA messages have max length of 80 bytes??
            //              --> is that true?
            //              --> can use smaller buffer??

int windsensor::direction(float* sensor_values)                                // read from serial and put values into
{

    unsigned char Buffer[102400] = {0};                                           // increased Buffer from 512 to 1024	// 31.10.2013 David:	buffer[512] too small to read  1020 bytes ..??
    int bytesread;                                                                                // buffer increased to 102400 to analyze incoming data
    int iResultsFound = 0;

    //	SerialIO windsensor_serialConn;
    //	open = windsensor_serialConn.open();
    //bytesread = windsensor_serialConn.readNonBlocking((char*)Buffer,1020);
    
    bytesread = m_SerialIO.readNonBlocking((char*) Buffer, 102400);

cout<<"Total number of bytes read: "<<bytesread<<"\n"<<endl;;
cout<<"Buffer:"<<endl<< Buffer << endl;



// TODO: David:     review this loop! looks like it tries to find begin of command for each character in buffer..
//                  this will overwrite if any "$" is found.. and use the later command..
//      --> recode this loop! think about that before changing any code here..
//          - maybe implement a stop after first successfull read, and pass rest of buffer into another read-Function.. or whatever other solution gets "defined"

    for (int i = 0; i < bytesread; i++)                                         //the wind angle is in degrees and speed is in knots by default
    {
        if ((Buffer[i]) == '$') {
//printf(" %.2x Hexa-decimal",(unsigned char)Buffer[i]);
//cout<<endl;

            if ((Buffer[i + 1] == 'I') && (Buffer[i + 2] == 'I') && (Buffer[i + 3] == 'M') && (Buffer[i + 4] == 'W') && (Buffer[i + 14] == ',') && (Buffer[i + 23] == 'A')) {
                sensor_values[0] = ((Buffer[i + 7] - 48)*100 + (Buffer[i + 8] - 48)*10 + (Buffer[i + 9] - 48) + .1 * (Buffer[i + 11] - 48)); // extracting the value for the angle and speed
                sensor_values[1] = ((Buffer[i + 15] - 48)*100 + (Buffer[i + 16] - 48)*10 + (Buffer[i + 17] - 48) + .1 * (Buffer[i + 19] - 48));
                if ((sensor_values[0] < 0.001) || (sensor_values[0] > 360)) // To check one more time if the value is in bounds
                    sensor_values[0] = -1;
                if (sensor_values[1] < 0)
                    sensor_values[1] = -1;
                if ((sensor_values[0] != -1) || ((sensor_values[1] != -1))) {
                    switch (speed) {
                        case 1:
                            cout << "Result #" <<  iResultsFound+1  << ": knots, ";
                            break;
                        case 2:
                            sensor_values[1] = sensor_values[1]* 0.514444;
                            cout << "m/s, ";
                            break;

                        default:
                            sensor_values[1] = sensor_values[1]*1.852;
                            cout << "km/h, ";
                            break;
                    }
                    switch (angle) {
                        case 1:
                            cout << "degree: ";
                            break;

                        default:
                            sensor_values[0] = sensor_values[0]*0.0174532925;
                            cout << "radian: \t\t";
                            break;
                    }
                } else{
                    cout << "Incorrect values" << endl;
                }
                cout << "direction and speed: " << sensor_values[0] << "\t" << sensor_values[1] << endl;
                iResultsFound++;
            }

            //cout << "\n";
        }

    }

return iResultsFound;
}

