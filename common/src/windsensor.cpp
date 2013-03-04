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
 * ROS stack name: DGPS
 * ROS package name: seneka_dgps
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Ciby Mathew, email:Ciby.Mathew@ipa.fhg.de
 * Supervised by: Christophe Maufroy
 *
 * Date of creation: Jan 2013
 * ToDo:
 *
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
windsensor::windsensor()
{

}

//-------------------------------------------
windsensor::~windsensor()
{
  m_SerialIO.close();
}


// ---------------------------------------------------------------------------
bool windsensor::open(const char* pcPort, int iBaudRate)
{
  int bRetSerial;
  // forDGPs :default is 38400
  if (iBaudRate != 4800)
    return false;

  // update scan id (id=8 for slave scanner, else 7)

  // initialize Serial Interface
  m_SerialIO.setBaudRate(iBaudRate);
  m_SerialIO.setDeviceName(pcPort);
  bRetSerial = m_SerialIO.open();
  if(bRetSerial == 0)
  {
    // Clears the read and transmit buffer.
    //	    m_iPosReadBuf2 = 0;
    m_SerialIO.purge();
    return true;
  }
  else
  {
    return false;
  }
}

void windsensor::direction(double* dir)
{
<<<<<<< HEAD:common/src/windsensor.cpp
  unsigned char Buffer[512] ={0};
  char str[10];
  char value[10000]={0};
  int open,bytesread,byteswrite;
  SerialIO windsensor;
  open = windsensor.open();
=======
  int length;
  cout<<"hello"<<endl;
  unsigned char Buffer[512];
  char str[10];
  char value[10000]={0};
  char ch[]={ 0x02, 0x00, 0x56, 0x03, 0x01, 0x00, 0x00, 0x5a, 0x03 }; //57h reply (basic coding)
  length = strlen(ch);
  int open,y,bytesread,byteswrite,bin;
  SerialIO dgps;
  open = dgps.open();
  byteswrite = dgps.write(ch,length);
  cout<<"Total number of bytes write "<<byteswrite<<"\n";
>>>>>>> fbcc55d54032a6d5dbd80dff76df2d0bf4c23b2b:seneka_dgps/common/src/Dgps.cpp
  sleep(1);
  bytesread = windsensor.readNonBlocking((char*)Buffer,1020);
  cout<<"Total number of bytes read"<<bytesread<<"\n"<<endl;;

  for(int i=0; i < bytesread; i++)
  {
<<<<<<< HEAD:common/src/windsensor.cpp
    if((Buffer[i])=='$')
    {
=======
    printf(" %.2x Hexa-decimal",(unsigned char)Buffer[i]);
    cout<<endl;
    bin = (unsigned char)Buffer[i];//
    dgps.binary(bin,value); // binary conversion
  }
  double lat_fract= 0.0,lat_exp= 0.0,lon_fract= 0.0,lon_exp= 0.0,alt_fract= 0.0,alt_exp= 0.0;
  for(int j=1;j<=52; j++)
  {
    lat_fract = lat_fract + (value[75+j]-'0')*pow (0.5,j);
    lon_fract = lon_fract + (value[139+j]-'0')*pow (0.5,j);
    alt_fract = alt_fract + (value[203+j]-'0')*pow (0.5,j);
>>>>>>> fbcc55d54032a6d5dbd80dff76df2d0bf4c23b2b:seneka_dgps/common/src/Dgps.cpp

      if((Buffer[i+1]=='I')&&(Buffer[i+2]=='I')&&(Buffer[i+3]=='M')&&(Buffer[i+4]=='W'))
      {

        dir[0] = ((Buffer[i+7]-48)*100 + (Buffer[i+8]-48)*10 +(Buffer[i+9]-48) +.1*(Buffer[i+11]-48));// extracting the value for the angle and speed
        if(Buffer[i+14]==',')
         dir[1] = ((Buffer[i+15]-48)*100 + (Buffer[i+16]-48)*10 +(Buffer[i+17]-48) +.1*(Buffer[i+19]-48))*1.852;
        else
          dir[1]=-1;
        if((dir[0] < 0) || (dir[0] > 360))
          dir[0]=-1;
        if(dir[1] < 0)
          dir[1]=-1;
        cout<<"direction and angle"<<dir[0]<<"\t"<<dir[1]<<endl;
      }
      cout<<"\n";
    }

//            printf(" %2.x reading values",(unsigned char)Buffer[i]);
        cout<<Buffer[i]<<endl;
  }
<<<<<<< HEAD:common/src/windsensor.cpp
=======
  for(int h=0;h<=10; h++)
  {
    lat_exp = lat_exp  + (value[75-h]-'0')*pow(2,h);
    lon_exp = lon_exp  + (value[139-h]-'0')*pow(2,h);
    alt_exp = alt_exp  + (value[203-h]-'0')*pow(2,h);
  }
  latt[0] = ((lat_fract+1)* pow(2,(lat_exp-1023))*180);
  latt[1] = ((lon_fract+1)* pow(2,(lon_exp-1023))*180);
  latt[2] = ((alt_fract+1) * pow(2,(alt_exp-1023)));
  cout << "latitude="<<latt[0]<<"\t longitude"<<latt[1]<<"\t altitude"<<"\t"<<latt[2]<<endl;
>>>>>>> fbcc55d54032a6d5dbd80dff76df2d0bf4c23b2b:seneka_dgps/common/src/Dgps.cpp

}
