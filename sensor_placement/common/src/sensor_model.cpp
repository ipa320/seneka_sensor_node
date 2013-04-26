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

#include <sensor_model.h>

using namespace seneka_sensor_model;

// standard constructor for derived class
FOV_2D_model::FOV_2D_model()
{
  setOpenAngles(0,0);
  setRange(0);
  setVelocity(0,0,0,0,0,0);
  setMaxVelocity(1,1,1, 30 * PI/180, 30 * PI/180, 30 * PI/180);
  setSensorPose(0,0,0,0,0,0,0);
}

// constructor with arguments
FOV_2D_model::FOV_2D_model(geometry_msgs::Twist new_vel, geometry_msgs::Twist new_max_vel, geometry_msgs::Pose new_pos, double new_range, double new_angle1, double new_angle2)
{
  setOpenAngles(new_angle1,new_angle2);
  setRange(new_range);
  setVelocity(new_vel);
  setMaxVelocity(new_max_vel);
  setSensorPose(new_pos);
}  

// destructor
FOV_2D_model::~FOV_2D_model(){}

// function to set actual velocity
void FOV_2D_model::setVelocity(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z)
{

  vel_.linear.x = lin_x;
  vel_.linear.y = lin_y;
  vel_.linear.z = lin_z;
  vel_.angular.x = ang_x;
  vel_.angular.y = ang_y;
  vel_.angular.z = ang_z;
  
}

// function to set actual velocity
void FOV_2D_model::setVelocity(geometry_msgs::Twist new_vel)
{
  vel_ = new_vel;
}

// function to set maximal velocity
void FOV_2D_model::setMaxVelocity(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z)
{

  max_vel_.linear.x = lin_x;
  max_vel_.linear.y = lin_y;
  max_vel_.linear.z = lin_z;
  max_vel_.angular.x = ang_x;
  max_vel_.angular.y = ang_y;
  max_vel_.angular.z = ang_z;
  
}

// function to set maximal velocity
void FOV_2D_model::setMaxVelocity(geometry_msgs::Twist new_max_vel)
{
  max_vel_ = new_max_vel;
}

// function to set general sensor pose
void FOV_2D_model::setSensorPose(double x, double y, double z, double quat_x, double quat_y, double quat_z, double quat_w)
{
  sensor_pose_.position.x = x;
  sensor_pose_.position.y = y;
  sensor_pose_.position.z = z;

  sensor_pose_.orientation.x = quat_x;
  sensor_pose_.orientation.y = quat_y;
  sensor_pose_.orientation.z = quat_z;
  sensor_pose_.orientation.w = quat_w;
}

// function to set general sensor pose
void FOV_2D_model::setSensorPose(double x, double y, double z, geometry_msgs::Quaternion orientation)
{
  sensor_pose_.position.x = x;
  sensor_pose_.position.y = y;
  sensor_pose_.position.z = z;

  sensor_pose_.orientation = orientation;
}

// function to set general sensor pose
void FOV_2D_model::setSensorPose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation)
{
  sensor_pose_.position = position;
  sensor_pose_.orientation = orientation;
}

// function to set general sensor pose
void FOV_2D_model::setSensorPose(geometry_msgs::Pose new_pos)
{
  sensor_pose_ = new_pos;
}

// function to set sensor opening angles
void FOV_2D_model::setOpenAngles(double open_ang1, double open_ang2)
{
  if(open_angles_.empty())
  {
    open_angles_.push_back(open_ang1);
    open_angles_.push_back(open_ang2);
  }
  else
  {
    while(!open_angles_.empty())
    {
      open_angles_.pop_back();
    }
    open_angles_.push_back(open_ang1);
    open_angles_.push_back(open_ang2);
  }
}

// function to set sensor range
void FOV_2D_model::setRange(double new_range)
{
  range_ = new_range;
}

// function to get actual velocity
geometry_msgs::Twist FOV_2D_model::getVelocity()
{
  return vel_;
}

// functgion to get the maximal velocity
geometry_msgs::Twist FOV_2D_model::getMaxVelocity()
{
  return max_vel_;
}

geometry_msgs::Pose FOV_2D_model::getSensorPose()
{
  return sensor_pose_;
}

// functions to get sensor opening angles
std::vector<double> FOV_2D_model::getOpenAngles()
{
  return open_angles_;
}

// function to set sensor range
double FOV_2D_model::getRange()
{
  return range_;
}

// function to generate random number in given interval
double FOV_2D_model::randomNumber(double low, double high)
{
  return ((double) rand() / RAND_MAX)*(high - low) + low;
}
