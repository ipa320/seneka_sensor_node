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
#include <seneka_windsensor/windsensor.h>
#include <ros/ros.h>
#include <boost/optional.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp> 
#include <boost/asio.hpp>
#include <boost/thread.hpp>

using namespace std;

typedef unsigned char BYTE;

void set_result1(boost::optional<boost::system::error_code>* a, boost::system::error_code b) 
{ 
	a->reset(b);
} 
void set_result2(boost::optional<boost::system::error_code>* a, boost::system::error_code b, size_t *bytes_transferred, size_t _bytes_transferred) 
{ 
	a->reset(b);
	*bytes_transferred = _bytes_transferred;
} 


template <typename MutableBufferSequence> 
int read_with_timeout(boost::asio::serial_port& sock, 
  const MutableBufferSequence& buffers) 
{ 
	boost::optional<boost::system::error_code> timer_result; 
	boost::asio::deadline_timer timer(sock.io_service()); 
	timer.expires_from_now(boost::posix_time::milliseconds(500)); 
	timer.async_wait(boost::bind(set_result1, &timer_result, _1)); 


	size_t bytes_to_transfer = 0;
	boost::optional<boost::system::error_code> read_result; 
	boost::asio::async_read(sock, buffers, 
		boost::bind(set_result2, &read_result, _1, &bytes_to_transfer, boost::asio::placeholders::bytes_transferred)); 

	sock.io_service().reset(); 
	while (sock.io_service().run_one()) 
	{ 
	  if (read_result) 
		timer.cancel(); 
	  else if (timer_result) 
		sock.cancel(); 
	} 


	if (*read_result && !(*read_result==boost::asio::error::operation_aborted && bytes_to_transfer>0) ) {
		std::cout<<"error code: "<<(*read_result)<<std::endl;
	  return 0; //throw boost::system::system_error(*read_result);
	  }
	  
	return bytes_to_transfer;
} 

int sensor_port = 0;
int sensor_baudrate = 0;
bool connected = false;
int speed_unit = 0;         // knots = 0
                            // m/s   = 1
                            // km/h  = 2
int direction_unit = 0;     // degree = 0
                            // radian = 1
int temperature_unit = 0;   // centigrade = 0
                            // fahrenheit = 1

windsensor::windsensor(int in_speed_unit, int in_direction_unit, int in_temperature_unit) : m_SerialIO(io_service_) {
    speed_unit = in_speed_unit;
    direction_unit = in_direction_unit;
    temperature_unit = in_temperature_unit;
    
	boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));
}

windsensor::~windsensor() {
    m_SerialIO.close();
}

bool windsensor::open(const char* pcPort, int iBaudRate) {

    //now comes a hack...
    char buf[256];
    sprintf(buf, "stty -F %s ispeed %d ospeed %d", pcPort, iBaudRate, iBaudRate);
    system(buf);

    // open port;
    boost::system::error_code ec;
    m_SerialIO.open(pcPort, ec);
    
    if(!ec) {
        connected = true;
        ROS_DEBUG("serial connection opened successfully");
        return true;
    } else {
        connected = false;
        ROS_ERROR("could not connect to serial device");
        return false;
    }
    
    m_SerialIO.set_option(boost::asio::serial_port_base::baud_rate(iBaudRate), ec);
    if(ec) ROS_ERROR("failed apply settings (%s, %d)", ec.category().name(), (int)ec.value());
	m_SerialIO.set_option(boost::asio::serial_port_base::character_size(8), ec);
    if(ec) ROS_ERROR("failed apply settings (%s, %d)", ec.category().name(), (int)ec.value());
	m_SerialIO.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one), ec);
    if(ec) ROS_ERROR("failed apply settings (%s, %d)", ec.category().name(), (int)ec.value());
	m_SerialIO.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none), ec);
    if(ec) ROS_ERROR("failed apply settings (%s, %d)", ec.category().name(), (int)ec.value());
	m_SerialIO.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none), ec);
    if(ec) ROS_ERROR("failed apply settings (%s, %d)", ec.category().name(), (int)ec.value());
}

void windsensor::close() {
    m_SerialIO.close();
}

float convert_speed_from_knots(float knots, int unit=speed_unit){
    float result = 0;
    switch (unit) {
        case 0:
            ROS_DEBUG("converting speed to knots");
            result = knots;
            break;
        case 1:
            ROS_DEBUG("converting speed to m/s");
            result = knots* 0.514444;
            break;
        case 2:
            ROS_DEBUG("converting speed to km/h");
            result = knots *1.852;
            break;            
        default:
            ROS_ERROR("wrong value for speed unit [0= knots, 1= m/s, 2= km/h]");
        }
    return result;

}

float convert_direction_from_degree(float degree, int unit=direction_unit){
    float result = 0;
    switch (unit) {
        case 0:
            ROS_DEBUG("converting direction to degree");
            result = degree;
            break;
        case 1:
            ROS_DEBUG("converting direction to radian");
            result = degree* 0.514444;
            break;
        default:
            ROS_ERROR("wrong value for direction unit [0= degree, 1= radian]");
        }
    return result;
}

float convert_temperature_from_centigrade(float centigrade, int unit=temperature_unit){
    float result = 0;
    switch (unit) {
        case 0:
            ROS_DEBUG("converting temperature to centigrade");
            result = centigrade;
            break;
        case 1:
            ROS_DEBUG("converting temperature to fahrenheit");
            result = (centigrade* 1.8000) + 32.00;
            break;
        default:
            ROS_ERROR("wrong value for temperature unit [0= centigrade, 1= fahrenheit]");
        }
    return result;
}

bool windsensor::compare_checksum(string fields[], std::string line_for_checksum){
            bool checksum_ok = false;
            // calculate checksum and verify message
            /*      A sentence may contain up to 80 characters plus "$" and CR/LF.
             *       If data for a field is not available, the
             *       field is omitted, but the delimiting commas are still sent, with no space between them. The checksum
             *       field consists of a "*" and two hex digits representing the exclusive OR of all characters between, but not
             *       including, the "$" and "*"
             */
            std::string checksum = fields[5].substr(fields[5].length()-2, fields[5].length());
            // calculate checksum (XOR over int-values of all characters between $ and *
            std::string message = line_for_checksum.substr(1,line_for_checksum.length()-4);
            // calculate XOR
            int bitwise_xor = 0;
            for (int i = 0; i<message.length(); i++){
                bitwise_xor ^= message[i];
            }
            // convert received checksum value (hex) to int
            int checksum_int;
            checksum_int = (int)strtol(checksum.c_str(), NULL, 16);
            // compare checksums
            if (checksum_int == bitwise_xor){
                ROS_DEBUG("checksum verfied for message from windsensor");
                checksum_ok = true;
            }else {
                ROS_ERROR("checksum error on message from windsensor");
                checksum_ok = false;
            }
            return checksum_ok;
}

bool windsensor::extract_sensordata_from_buffer(unsigned char *input, float sensor_values[]){
    // bool for return value
    bool success = false;
    // other variables
    float wind_speed = 0;
    bool got_wind_speed = false;
    float wind_direction = 0;
    bool got_wind_direction = false;
    float temperature = 0;
    bool got_temperature = false;
    bool wind_checksum_ok = false;
    bool temperature_checksum_ok = false;
    // convert char* to string
    std::string fields[6] = {""};
    std::string s = reinterpret_cast<const char*>(input);

    // extract values
    std::string delimiter = "\r\n";             // line endings
    std::string second_delimiter = ",";          // field delimiter
    size_t pos = 0;
    std::string line;
    // find lines in buffer
    while ((pos = s.find(delimiter)) != std::string::npos) {
        line = s.substr(0, pos);
        std::string line_for_checksum = s.substr(0, line.length());
        // check if message starts with '$'  (start character for NMEA0183)
        if (line[0] == '$'){
        // extract fields
        size_t second_pos = 0;
        std::string field;
        int field_counter = 0;
        // find fields in line
        while ((second_pos = line.find(second_delimiter)) != std::string::npos) {
            // get substring between delimiters (',')
            field = line.substr(0, second_pos);
            // put field value into fields array
            fields[field_counter] = field;
            field_counter++;
            // delete processed data from line-buffer
            line.erase(0, second_pos + second_delimiter.length());
        }
        // add the last field to fields array ( to allow comparing data to received checksum )
        fields[field_counter] = line.substr(0, line.length());
        // now analyze values
        // if line is wind_data (starts with "$IIMWV")
        if (fields[0] == "$IIMWV"){
            // extract wind direction
            char * direction_char = new char[fields[1].length()];
            strcpy(direction_char,fields[1].c_str());
            wind_direction = strtof(direction_char, NULL);
            // extract wind speed
            char * speed_char = new char[fields[3].length()];
            strcpy(speed_char,fields[3].c_str());
            wind_speed = strtof(speed_char, NULL);
            // verify checksum
            if (compare_checksum(fields, line_for_checksum) == true){
                got_wind_direction = true;
                got_wind_speed = true;
                wind_checksum_ok = true;
            }
        // else if line is temperature_data (starts with "$WIXDR")
        }else if (fields[0] == "$WIXDR"){
            // extract temperature value
            char * temperature_char = new char[fields[2].length()];
            strcpy(temperature_char,fields[2].c_str());
            temperature = strtof(temperature_char, NULL);
            // verify checksum
            if (compare_checksum(fields, line_for_checksum) == true){
                got_temperature = true;
                temperature_checksum_ok = true;
            }
        }
        // remove processed line from 'buffer'
        s.erase(0, pos + delimiter.length());
        }
        else{
            // clean 'buffer' after failed extraction
            int start_index = s.find("$");
            s.erase(0, start_index);
            ROS_DEBUG("message from windsensor did not start with \'$\'. skipping some bytes and retrying.");
        }
    }
    // check if all values were ok
    if (got_wind_speed && got_wind_direction && got_temperature && wind_checksum_ok && temperature_checksum_ok){
        sensor_values[0] = wind_speed;
        sensor_values[1] = wind_direction;
        sensor_values[2] = temperature;
        ROS_DEBUG("extracted all values from windsensor");
        success = true;
    }else{
        ROS_WARN("could not extract windsensor values (publishrate too high?)");
    }
    return success;
}

bool windsensor::read(float sensor_values[], string sensor_units[]){
    bool success = false;
    if (connected != true){
        success = false;
        ROS_ERROR("could not read from windsensor: %i, %i", sensor_port, sensor_baudrate);
    }
    else{
        unsigned char Buffer[1024] = {0};                                           // increased Buffer from 512 to 1024	// 31.10.2013 David:	buffer[512] too small to read  1020 bytes ..??
        int bytesread;
        int iResultsFound = 0;
        // read from serial
		bytesread = read_with_timeout(m_SerialIO, boost::asio::buffer(Buffer,sizeof(Buffer)));
        success = true;
        ROS_DEBUG("read %i bytes from windsensor: %i, %i", bytesread, sensor_port, sensor_baudrate);
        // extract sensor values and set sensor units
        if (extract_sensordata_from_buffer(Buffer, sensor_values)){
            switch(speed_unit){
                case 0:
                    sensor_units[0] = "knots";
                    break;
                case 1:
                    sensor_units[0] = "m/s";
                    break;
                case 2:
                    sensor_units[0] = "km/h";
                    break;
                default:
                    success = false;
                    ROS_ERROR("bad speed unit");
            }
            switch(direction_unit){
                case 0:
                    sensor_units[1] = "degree";
                    break;
                case 1:
                    sensor_units[1] = "radian";
                    break;
                default:
                    success = false;
                    ROS_ERROR("bad direction unit");
            }
            switch(temperature_unit){
                case 0:
                    sensor_units[2] = "centigrade";
                    break;
                case 1:
                    sensor_units[2] = "fahrenheit";
                    break;
                default:
                    success = false;
                    ROS_ERROR("bad temperature unit");
            }
        }
    }
    return success;
}





