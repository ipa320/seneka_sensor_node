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
#include <ros/ros.h>
#include <seneka_dgps/Dgps.h>
using namespace std;

//-----------------------------------------------

typedef unsigned char BYTE;

//-----------------------------------------------
Dgps::Dgps()
{

}


//-------------------------------------------
Dgps::~Dgps()
{
  m_SerialIO.close();
}


// ---------------------------------------------------------------------------
bool Dgps::open(const char* pcPort, int iBaudRate)
{
  int bRetSerial;
  // forDGPs :default is 38400
  if (iBaudRate != 38400)
    return false;

  // update scan id (id=8 for slave scanner, else 7)


  // initialize Serial Interface
  m_SerialIO.setBaudRate(iBaudRate);
  m_SerialIO.setDeviceName(pcPort);
  //	m_SerialIO.setBufferSize(READ_BUF_SIZE - 10 , WRITE_BUF_SIZE -10 );
  //	m_SerialIO.setHandshake(SerialIO::HS_NONE);
  //	m_SerialIO.setMultiplier(m_dBaudMult);
  bRetSerial = m_SerialIO.open();
  //	m_SerialIO.setTimeout(0.0);
  //	m_SerialIO.SetFormat(8, SerialIO::PA_NONE, SerialIO::SB_ONE);
  //	cout<<"test"<<bRetSerial<<endl;
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

void Dgps::latlong(double* latt)
{
  cout<<"hello"<<endl;
  unsigned char Buffer[512];
  char str[10];
  char value[10000]={0};
  char ch[]={ 0x02, 0x00, 0x06, 0x00, 0x06, 0x03};
  int length = 6; //No.of bits sending;
  int open,y,bytesread,byteswrite,bin,k=9;
  SerialIO dgps;
  open = dgps.open();
  byteswrite = dgps.write(ch,length);
  cout<<"Total number of bytes write "<<byteswrite<<"\n";
  sleep(1);
  bytesread = dgps.readNonBlocking((char*)Buffer, 1020);
  cout<<"Total number of bytes read"<<bytesread<<"\n"<<endl;;

  for(int i=0; i < bytesread; i++)
  {
    printf(" %.2x helloman",(unsigned char)Buffer[i]);
    cout<<endl;
    bin = (unsigned char)Buffer[i];//
    dgps.binary(bin,value); // error
  }
  double lat_fract= 0.0,lat_exp= 0.0,lon_fract= 0.0,lon_exp= 0.0,alt_fract= 0.0,alt_exp;
  for(int j=1;j<=52; j++)
  {
    lat_fract = lat_fract + (value[75+j]-'0')*pow (0.5,j);
    lon_fract = lon_fract + (value[139+j]-'0')*pow (0.5,j);
    alt_fract = alt_fract + (value[203+j]-'0')*pow (0.5,j);

  }
  for(int h=0;h<=10; h++)
  {
    lat_exp = lat_exp  + (value[75-h]-'0')*pow(2,h);
    lon_exp = lon_exp  + (value[139-h]-'0')*pow(2,h);
    alt_exp = lat_exp  + (value[203-h]-'0')*pow(2,h);
  }
  latt[0] = ((lat_fract+1)* pow(2,(lat_exp-1023))*180);
  latt[1] = ((lon_fract+1)* pow(2,(lon_exp-1023))*180);
  latt[2] = ((alt_fract+1)* pow(2,(alt_exp-1023))*180);
  //			latt[2] =25;
  //			ROS_INFO("...publishing position of DGpsss %1f, %1f,%1f",latt[0],latt[1],latt[2]);
  cout << "latitude="<<latt[0]<<"\t longitude"<<latt[1]<<"\t altitude"<<"\t"<<latt[2]<<endl;

}
