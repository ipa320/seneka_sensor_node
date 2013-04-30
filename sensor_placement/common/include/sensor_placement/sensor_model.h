/****************************************************************
 *
 * Copyright (c) 2013
 *
 * Fraunhofer Institute for Manufacturing Engineering  
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: SeNeKa
 * ROS stack name: seneka
 * ROS package name: sensor_placement
 *  							
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  		
 * Author: Florian Mirus, email:Florian.Mirus@ipa.fhg.de
 *
 * Date of creation: April 2013
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *  	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *  	 notice, this list of conditions and the following disclaimer in the
 *  	 documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing 
 *  	 Engineering and Automation (IPA) nor the names of its
 *  	 contributors may be used to endorse or promote products derived from
 *  	 this software without specific prior written permission.
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

#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

// standard includes
#include <iostream>
#include <math.h>
#include <stdlib.h>

// ros includes
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

#define PI 3.14159265

namespace seneka_sensor_model{

class sensor_model // interface class for each sensor model
{

protected:

  // member variables
  std::string name_;

  // velocity
  geometry_msgs::Twist vel_;

  // maximal velocity
  geometry_msgs::Twist max_vel_;

  // sensor position
  geometry_msgs::Pose sensor_pose_;

  // sensor_range
  double range_;

  // sensor opening angles
  std::vector<double> open_angles_;

public:

  //destructor
  virtual ~sensor_model() {};

  // ************************ setter functions ************************

  // function to set the name
  virtual void setName(std::string new_name) = 0;

  // function to set actual velocity
  virtual void setVelocity(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z) = 0;
  virtual void setVelocity(geometry_msgs::Twist new_vel) = 0;

  // function to set maximal velocity
  virtual void setMaxVelocity(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z) = 0;
  virtual void setMaxVelocity(geometry_msgs::Twist new_max_vel) = 0;

  // functions to set general sensor pose
  virtual void setSensorPose(double x, double y, double z, double quat_x, double quat_y, double quat_z, double quat_w) = 0;
  virtual void setSensorPose(double x, double y, double z, geometry_msgs::Quaternion orientation) = 0;
  virtual void setSensorPose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation) = 0;
  virtual void setSensorPose(geometry_msgs::Pose new_pos) = 0;

  // function to set sensor opening angles
  virtual void setOpenAngles(double open_ang1, double open_ang2) = 0;

  // function to set sensor range
  virtual void setRange(double new_range) = 0;

  // ************************ getter functions ************************

  // functgion to get actual velocity
  virtual geometry_msgs::Twist getVelocity() = 0;

  // functgion to get the maximal velocity
  virtual geometry_msgs::Twist getMaxVelocity() = 0;

  // function to get actual sensor pose
  virtual geometry_msgs::Pose getSensorPose() = 0;

  // functions to get sensor opening angles
  virtual std::vector<double> getOpenAngles() = 0;

  // function to set sensor range
  virtual double getRange() = 0;


  // ************************* help functions *************************

  // function to generate random number in given interval
  virtual double randomNumber(double low, double high) = 0;

  // signum function
  virtual int signum(double x) = 0;

  // draws a visualization of the respective sensor model
  virtual visualization_msgs::MarkerArray getVisualizationMarkers(unsigned int id) = 0;
};

class FOV_2D_model : public sensor_model
{

public:

  // standard constructor
  FOV_2D_model();

  // constructor with arguments
  FOV_2D_model(geometry_msgs::Twist new_vel, geometry_msgs::Twist new_max_vel, geometry_msgs::Pose new_pos, double new_range, double new_angle1, double new_angle2, std::string new_name);

  // destructor
  virtual ~FOV_2D_model();

  // ************************ setter functions ************************

  // function to set the name
  virtual void setName(std::string new_name);

  // functions to set actual velocity
  virtual void setVelocity(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z);
  virtual void setVelocity(geometry_msgs::Twist new_vel);

  // functions to set maximal velocity
  virtual void setMaxVelocity(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z);
  virtual void setMaxVelocity(geometry_msgs::Twist new_max_vel);

  // functions to set general sensor pose
  virtual void setSensorPose(double x, double y, double z, double quat_x, double quat_y, double quat_z, double quat_w);
  virtual void setSensorPose(double x, double y, double z, geometry_msgs::Quaternion orientation);
  virtual void setSensorPose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation);
  virtual void setSensorPose(geometry_msgs::Pose new_pos);

  // function to set sensor opening angles
  virtual void setOpenAngles(double open_ang1, double open_ang2);

  // function to set sensor range
  virtual void setRange(double new_range);

  // ************************ getter functions ************************

  // function to get actual velocity
  virtual geometry_msgs::Twist getVelocity();

  // functgion to get the maximal velocity
  virtual geometry_msgs::Twist getMaxVelocity();

  // function to get actual sensor pose
  virtual geometry_msgs::Pose getSensorPose();

  // function to get sensor opening angles
  virtual std::vector<double> getOpenAngles();

  // function to set sensor range
  virtual double getRange();

  // ************************* help functions *************************

  // function to generate random number in given interval
  virtual double randomNumber(double low, double high);

  // signum function
  virtual int signum(double x);

  // returns the visualization markers of the respective sensor model
  virtual visualization_msgs::MarkerArray getVisualizationMarkers(unsigned int id);
};

class FOV_3D_model : public sensor_model
{

public:
  // To be done!!  

};

}
#endif
