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
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing 
 *     Engineering and Automation (IPA) nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

// includes
#include <seneka_utilities.h>

using namespace seneka_utilities;

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

  //lookup table for raytracing
  const std::vector< std::vector<geometry_msgs::Point32> > * pLookup_table_;

  //vector of end points of the rays for visualization;
  std::vector<geometry_msgs::Point> end_of_rays_;

public:

  //destructor
  virtual ~sensor_model() {};

  // ************************ setter functions ************************

  // function to set the name
  virtual void setName(std::string new_name) = 0;

  // functions to set actual velocity
  virtual void setVelocity(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z) = 0;
  virtual void setVelocity(geometry_msgs::Twist new_vel) = 0;

  // functions to set maximal velocity
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

  // function to set the lookup table
  virtual void setLookupTable(const std::vector< std::vector<geometry_msgs::Point32> > * pLookup_table) = 0;

  virtual void addRayEndPoint(geometry_msgs::Point new_end_point) = 0;

  virtual void clearRayEndPoints() = 0;

  // ************************ getter functions ************************

  // function to get actual velocity
  virtual geometry_msgs::Twist getVelocity() = 0;

  // function to get the maximal velocity
  virtual geometry_msgs::Twist getMaxVelocity() = 0;

  // function to get actual sensor pose
  virtual geometry_msgs::Pose getSensorPose() = 0;

  // function to get sensor opening angles
  virtual std::vector<double> getOpenAngles() = 0;

  // function to get sensor range
  virtual double getRange() = 0;

  // function to get the lookup table
  virtual const std::vector< std::vector<geometry_msgs::Point32> > * getLookupTable() = 0;

  // function to get the index of the lookup table for the corresponding angle 
  virtual int rayOfAngle(double angle) = 0;

  // ************************* help functions *************************

  // draws a visualization of the respective sensor model
  virtual visualization_msgs::MarkerArray getVisualizationMarkersOld(unsigned int id) = 0;
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
  ~FOV_2D_model();

  // ************************ setter functions ************************

  // function to set the name
  void setName(std::string new_name);

  // functions to set actual velocity
  void setVelocity(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z);
  void setVelocity(geometry_msgs::Twist new_vel);

  // functions to set maximal velocity
  void setMaxVelocity(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z);
  void setMaxVelocity(geometry_msgs::Twist new_max_vel);

  // functions to set general sensor pose
  void setSensorPose(double x, double y, double z, double quat_x, double quat_y, double quat_z, double quat_w);
  void setSensorPose(double x, double y, double z, geometry_msgs::Quaternion orientation);
  void setSensorPose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation);
  void setSensorPose(geometry_msgs::Pose new_pos);

  // function to set sensor opening angles
  void setOpenAngles(double open_ang1, double open_ang2);

  // function to set sensor range
  void setRange(double new_range);

  // function to set the lookup table
  void setLookupTable(const std::vector< std::vector<geometry_msgs::Point32> > * pLookup_table);

  // function to set a point as last visible cell of a ray for visualization purposes
  void addRayEndPoint(geometry_msgs::Point new_end_point);

  // function to clear the vector of last visible ray points
  void clearRayEndPoints();

  // ************************ getter functions ************************

  // function to get actual velocity
  geometry_msgs::Twist getVelocity();

  // function to get the maximal velocity
  geometry_msgs::Twist getMaxVelocity();

  // function to get actual sensor pose
  geometry_msgs::Pose getSensorPose();

  // function to get sensor opening angles
  std::vector<double> getOpenAngles();

  // function to get sensor range
  double getRange();

  // function to get the lookup table
  const std::vector< std::vector<geometry_msgs::Point32> > * getLookupTable();

  // function to get the index of the lookup table for the corresponding angle
  int rayOfAngle(double angle);

  // returns the visualization markers of the respective sensor model
  visualization_msgs::MarkerArray getVisualizationMarkersOld(unsigned int id);
  visualization_msgs::MarkerArray getVisualizationMarkers(unsigned int id);
};

class FOV_3D_model : public sensor_model
{

public:
  // To be done!!  

};

#endif
