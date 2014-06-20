/*!
*****************************************************************
* seneka_can.cpp
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_can
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
* 
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
*
* Modified xx/20xx:
*
* Description:
* The seneka_can package is part of the seneka_sensor_node metapackage, 
* developed for the SeNeKa project at Fraunhofer IPA. 
* It implements a ROS wrapper for the PEAK System PCAn-Basic API in form of a ROS service node "CAN".
* The ROS node advertises services to communicate with CAN devices.
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

/**************************************************/
/**************************************************/
/**************************************************/

//  source code sections
//
//      -> pcanwrite
//      -> pcanread
//      -> pcaneventread
//
//  copied from PEAK System PCAN-Basic CAN-Software-API (Linux, C++)

//  see http://www.peak-system.com/PCAN-Basic.239.0.html (20.06.2014)
//  and consider the following lines
//
//  ------------------------------------------------------------------
//  Author : Thomas Haber (thomas@toem.de)
//  Last change: 18.06.2010
//
//  Language: C++
//  ------------------------------------------------------------------
//
//  Copyright (C) 1999-2010  PEAK-System Technik GmbH, Darmstadt
//  more Info at http://www.peak-system.com
//  ------------------------------------------------------------------
//
// linux@peak-system.com
// www.peak-system.com
//
//  ------------------------------------------------------------------
//  History:
//  07-11-2013 Stephane Grosjean
//  - Move DWORD definition from "unsigned long" to "__u32" to run on 64-bits
//    Kernel
//  - Change initital bitrate from 250K to 500K (default pcan driver bitrate)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

/**************************************************/
/**************************************************/
/**************************************************/

#include <ros/ros.h>

#include <stdio.h>
#include <unistd.h>
#include <asm/types.h>

#define DWORD  __u32
#define WORD   unsigned short
#define BYTE   unsigned char
#define LPSTR  char*
#include <pcanbasic/PCANBasic.h>

#include <pthread.h> 

/***********************************************************/
/*************** main program seneka_can.cpp ***************/
/***********************************************************/

int main(int argc, char** argv) {

    // ROS initialization; apply "CAN" as node name;
    ros::init(argc, argv, "CAN");

    TPCANMsg Message;
    TPCANStatus Status;
    unsigned long ulIndex = 0;
    fd_set Fds;

    Status = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_500K, 0, 0, 0);
    printf("Initialize CAN: %i\n",(int)Status);

    /**************************************************/
    /**************************************************/
    /**************************************************/
/* 
    // pcanwrite

    Message.ID = 0x77;
    Message.LEN = 8;
    Message.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    Message.DATA[0]=0;

    while(1)
        while ((Status=CAN_Write(PCAN_USBBUS1,&Message)) == PCAN_ERROR_OK) {
            Message.DATA[0]++;
            ulIndex++;
            if ((ulIndex % 1000) == 0)
                printf("  - T Message %i\n", (int)ulIndex);
        }

    printf("STATUS %i\n", (int)Status);
*/
    /**************************************************/
    /**************************************************/
    /**************************************************/
/*
    // pcanread

    while (1) {
        while ((Status=CAN_Read(PCAN_USBBUS1,&Message,NULL)) == PCAN_ERROR_QRCVEMPTY)
            usleep(1000);
        if (Status != PCAN_ERROR_OK) {
            printf("Error 0x%x\n",(int)Status);
            break;
        }

        printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
            (int)Message.ID, (int)Message.LEN,
            (int)Message.DATA[0], (int)Message.DATA[1],
            (int)Message.DATA[2], (int)Message.DATA[3],
            (int)Message.DATA[4], (int)Message.DATA[5],
            (int)Message.DATA[6], (int)Message.DATA[7]);
    }
*/
    /**************************************************/
    /**************************************************/
    /**************************************************/
/*
    // pcaneventread

    int fd;
    CAN_GetValue(PCAN_USBBUS1, PCAN_RECEIVE_EVENT, &fd,sizeof(int));

    // Watch stdin (fd 0) to see when it has input.
    FD_ZERO(&Fds);
    FD_SET(fd, &Fds);

    while (select(fd+1, &Fds, NULL, NULL, NULL) > 0) {
        Status = CAN_Read(PCAN_USBBUS1, &Message, NULL);
        if (Status != PCAN_ERROR_OK) {
            printf("Error 0x%x\n", (int) Status);
            break;
        }

        printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
                (int) Message.ID, (int) Message.LEN, (int) Message.DATA[0],
                (int) Message.DATA[1], (int) Message.DATA[2],
                (int) Message.DATA[3], (int) Message.DATA[4],
                (int) Message.DATA[5], (int) Message.DATA[6],
                (int) Message.DATA[7]);
    }
*/
    /**************************************************/
    /**************************************************/
    /**************************************************/

        ros::spinOnce();

        return 0;

    }

    /**************************************************/
    /**************************************************/
    /**************************************************/