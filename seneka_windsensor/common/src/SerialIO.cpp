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
 * ROS stack name: Windsensor
 * ROS package name: seneka_windsensor
 * Description:
 *
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
 * ToDo:
 * -- test
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
/* Autor: Ciby Mathew
 * Fraunhofer IPA
 *
 */

#include "seneka_windsensor/SerialIO.h"
#include <math.h>
#include <iostream>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <ros/ros.h>
using namespace std;

char str[10];
char value[1000], lat[64], longt[64];

bool getBaudrateCode(int iBaudrate, int* iBaudrateCode)                         // get termios:BaudrateCode from iBaudrate
{
    // baudrate codes are defined in termios.h
    // currently up to B1000000
    const int baudTable[] = {
        0, 50, 75, 110, 134, 150, 200, 300, 600,
        1200, 1800, 2400, 4800,
        9600, 19200, 38400, 57600, 115200, 230400,
        460800, 500000, 576000, 921600, 1000000
    };
    const int baudCodes[] = {
        B0, B50, B75, B110, B134, B150, B200, B300, B600,
        B1200, B1800, B2400, B4800,
        B9600, B19200, B38400, B57600, B115200, B230400,
        B460800, B500000, B576000, B921600, B1000000
    };
    const int iBaudsLen = sizeof (baudTable) / sizeof (int);
    bool bReturn = false;
    for (int i = 0; i < iBaudsLen; i++) {
        if (baudTable[i] == iBaudrate) {
            *iBaudrateCode = baudCodes[i];
            bReturn = true;
            break;
        }
    }
    return bReturn;
}

//////////////////////////////////////////////////////////////////////
// Konstruktion/Destruktion
//////////////////////////////////////////////////////////////////////

SerialIO::SerialIO()
: m_DeviceName("/dev/ttyUSB0"),
m_Device(-1),
m_BaudRate(4800),
m_Multiplier(1.0),
m_ByteSize(8),
m_StopBits(SB_ONE),
m_Parity(PA_NONE),
m_Handshake(HS_NONE),
m_ReadBufSize(1024),
m_WriteBufSize(m_ReadBufSize),
m_Timeout(0),
m_ShortBytePeriod(false) {
m_BytePeriod.tv_sec = 0;
m_BytePeriod.tv_usec = 0;
}

SerialIO::~SerialIO() {
    close();
}

int SerialIO::open()                                                            // open serial connection
{
    int iResult;
    m_Device = ::open(m_DeviceName.c_str(),                                     // open device
            O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_Device < 0) {
        //RF_ERR("Open " << m_DeviceName << " failed, error code " << errno);
        ROS_ERROR("Trying to open %s failed: %s (Error code: %i )", m_DeviceName.c_str(), strerror(errno), errno);
        return -1;
    }
    iResult = tcgetattr(m_Device, &m_tio); // get parameters
    if (iResult == -1) {

        ROS_ERROR("tcgetattr %s failed: %s (Error code: %i )", m_DeviceName.c_str(), strerror(errno), errno);

        ::close(m_Device);
        m_Device = -1;
        return -1;
    }
    m_tio.c_iflag = 0;                                                          // set Default values
    m_tio.c_oflag = 0;
    m_tio.c_cflag = B38400 | CS8 | CREAD | HUPCL | CLOCAL;
    m_tio.c_lflag = 0;
    m_tio.c_cc[VINTR] = 3; // Interrupt
    m_tio.c_cc[VQUIT] = 28; // Quit
    m_tio.c_cc[VERASE] = 127; // Erase
    m_tio.c_cc[VKILL] = 21; // Kill-line
    m_tio.c_cc[VEOF] = 4; // End-of-file
    m_tio.c_cc[VTIME] = 0; // Time to wait for data (tenths of seconds)
    m_tio.c_cc[VMIN] = 1; // Minimum number of characters to read
    m_tio.c_cc[VSWTC] = 0;
    m_tio.c_cc[VSTART] = 17;
    m_tio.c_cc[VSTOP] = 19;
    m_tio.c_cc[VSUSP] = 26;
    m_tio.c_cc[VEOL] = 0; // End-of-line
    m_tio.c_cc[VREPRINT] = 18;
    m_tio.c_cc[VDISCARD] = 15;
    m_tio.c_cc[VWERASE] = 23;
    m_tio.c_cc[VLNEXT] = 22;
    m_tio.c_cc[VEOL2] = 0; // Second end-of-line
    int iNewBaudrate = int(m_BaudRate * m_Multiplier + 0.5);
    int iBaudrateCode = 0;
    bool bBaudrateValid = getBaudrateCode(iNewBaudrate, &iBaudrateCode);
    ROS_DEBUG("Setting Baudrate to %i", iNewBaudrate);
    if (bBaudrateValid) {                                                       // if baudrateCode was  found -> set via ctlset..
        cfsetispeed(&m_tio, iBaudrateCode);                                         // set input baudrate
        cfsetospeed(&m_tio, iBaudrateCode);                                         // set output baudrate
    } else {                                                                    // else -> set directly via "ioctl"
        ROS_WARN("Baudrate code not available - setting baudrate directly");
        struct serial_struct ss;
        ioctl(m_Device, TIOCGSERIAL, &ss);
        ss.flags |= ASYNC_SPD_CUST;
        ss.custom_divisor = ss.baud_base / iNewBaudrate;
        ioctl(m_Device, TIOCSSERIAL, &ss);
    }
    m_tio.c_cflag &= ~CSIZE;                                                    // set data format
    switch (m_ByteSize)                                                         // set ByteSize
    {
        case 5:
            m_tio.c_cflag |= CS5;
            break;
        case 6:
            m_tio.c_cflag |= CS6;
            break;
        case 7:
            m_tio.c_cflag |= CS7;
            break;
        case 8:
        default:
            m_tio.c_cflag |= CS8;
    }
    m_tio.c_cflag &= ~(PARENB | PARODD);
    switch (m_Parity)                                                           // set ParityBits
    {
        case PA_ODD:
            m_tio.c_cflag |= PARODD;
            //break;                                                            // break must not be active here as we need the combination of PARODD and PARENB on odd parity.
        case PA_EVEN:
            m_tio.c_cflag |= PARENB;
            break;
        case PA_NONE:
        default:
        {
        }
    }
    switch (m_StopBits)                                                         // set StopBits
    {
        case SB_TWO:
            m_tio.c_cflag |= CSTOPB;
            break;
        case SB_ONE:
        default:
            m_tio.c_cflag &= ~CSTOPB;
    }
    switch (m_Handshake)                                                        // configure hardware handshake
    {
        case HS_NONE:
            m_tio.c_cflag &= ~CRTSCTS;
            m_tio.c_iflag &= ~(IXON | IXOFF | IXANY);
            break;
        case HS_HARDWARE:
            m_tio.c_cflag |= CRTSCTS;
            m_tio.c_iflag &= ~(IXON | IXOFF | IXANY);
            break;
        case HS_XONXOFF:
            m_tio.c_cflag &= ~CRTSCTS;
            m_tio.c_iflag |= (IXON | IXOFF | IXANY);
            break;
    }
    m_tio.c_oflag &= ~OPOST;
    m_tio.c_lflag &= ~ICANON;
    iResult = tcsetattr(m_Device, TCSANOW, &m_tio);                             // write parameters via tcsetattr
    if (iResult == -1) {
        ROS_ERROR("tcsetattr %s failed: %s (Error code: %i )", m_DeviceName.c_str(), strerror(errno), errno);
        ::close(m_Device);
        m_Device = -1;
        return -1;
    }
    setTimeout(m_Timeout);                                                      // set timeout
    return 0;
}

void SerialIO::close(){                                                          // close serial connection
    if (m_Device != -1) {
        ::close(m_Device);
        m_Device = -1;
    }
}

void SerialIO::setTimeout(double Timeout){                                       // set serialIO timeout
    m_Timeout = Timeout;
    if (m_Device != -1) {
        m_tio.c_cc[VTIME] = cc_t(ceil(m_Timeout * 10.0));
        ::tcsetattr(m_Device, TCSANOW, &m_tio);
    }
}

int SerialIO::readNonBlocking(char *Buffer, int Length){                         // read from serial into Buffer
    int iAvailableBytes = getSizeRXQueue();
    int iBytesToRead = (Length < iAvailableBytes) ? Length : iAvailableBytes;
    ssize_t BytesRead;
    BytesRead = ::read(m_Device, Buffer, iBytesToRead);
    return BytesRead;
}

int SerialIO::getSizeRXQueue(){                                                  // check how many bytes are waiting in RX queue
    int cbInQue;
    int Res = ioctl(m_Device, FIONREAD, &cbInQue);
    if (Res == -1) {
        return 0;
    }
    return cbInQue;
}




// ################################################################################################################################
// stuff below is NOT USED for windsensor   (..yet)
// ################################################################################################################################

void SerialIO::setBytePeriod(double Period) {
    m_ShortBytePeriod = false;
    m_BytePeriod.tv_sec = time_t(Period);
    m_BytePeriod.tv_usec = suseconds_t((Period - m_BytePeriod.tv_sec) * 1000);
}

int SerialIO::write(const char *Buffer, int Length) {
    ssize_t BytesWritten;

    if (m_BytePeriod.tv_usec || m_BytePeriod.tv_sec) {
        int i;
        for (i = 0; i < Length; i++) {
            BytesWritten = ::write(m_Device, Buffer + i, 1);
            if (BytesWritten != 1)
                break;
            ::select(0, 0, 0, 0, &m_BytePeriod);
        }
        BytesWritten = i;
    } else
        BytesWritten = ::write(m_Device, Buffer, Length);


#ifdef PRINT_BYTES
    printf("%2d Bytes sent:", BytesWritten);
    for (int i = 0; i < BytesWritten; i++)
        printf(" %.2x", (unsigned char) Buffer[i]);
    printf("\n");
#endif

    return BytesWritten;
}

int SerialIO::readBlocking(char *Buffer, int Length) {
    //ssize_t
    int BytesRead;
    BytesRead = ::read(m_Device, Buffer, Length);

    printf("%2d Bytes read:", BytesRead);
    for (int i = 0; i < BytesRead; i++) printf(" %.2x", (unsigned char) Buffer[i]);
    printf("\n");

#ifdef PRINT_BYTES
    printf("%2d Bytes read:", BytesRead);
    for (int i = 0; i < BytesRead; i++)
        printf(" %.2x", (unsigned char) Buffer[i]);
    printf("\n");
#endif
    
    if (BytesRead < 0) {
        printf("Reading error\n");
        printf("Error no is : %d\n", errno);
        printf("Error description is : %s\n", strerror(errno));
        //				return leng;
    } else
        return BytesRead;
}