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

#ifndef SENSOR_PLACEMENT_H
#define SENSOR_PLACEMENT_H

// standard includes
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>

// ros includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <ros/console.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>

// external includes
#include <sensor_model.h>
#include <particle.h>

using namespace std;

class sensor_placement_node
{
private:

  // actual 2D-grid-map
  nav_msgs::OccupancyGrid map_;

  // actual area of interest to be covered by the sensor nodes
  geometry_msgs::Polygon area_of_interest_;

  // number of sensors
  int sensor_num_;

  // maximal range of sensors
  double sensor_range_;

  // opening angle for 2D sensors
  double open_angle_2D_;

  // number of particles for PSO
  int particle_num_;

  // maximal number of PSO iterations
  int iter_max_;

  // minimal coverage to stop PSO before reaching maximal number of PSO iterations
  double min_cov_;

  // particle swarm
  vector<seneka_particle::particle> particle_swarm_;

  // vector storing the positions global best solution of the particle swarm
  vector<geometry_msgs::Pose> global_best_;

  // PSO actual best coverage
  double best_cov_;

  // number of targets for PSO
  int target_num_;

  // target array
  int* targets_;

  // PSO parameter constants
  double PSO_param_1_;
  double PSO_param_2_;
  double PSO_param_3_;

public:

  // constructor
  sensor_placement_node();

  // destrcutor
  ~sensor_placement_node();

  // create node handles
  ros::NodeHandle nh_, pnh_;

  // declaration of ros subscribers
  ros::Subscriber map_sub_;
  ros::Subscriber poly_sub_;

  // declaration of ros publishers

  // function to get an array of targets from the map and the area of interest specified as polygon
  void getTargets();

  // function to initialize PSO-Algorithm
  void initializePSO();

  // function for the actual partcile-swarm-optimization
  void PSOptimize();

  // function to get the current global best solution
  void getGlobalBest();

  // function to gerenate random numbers in given interval
  double randomNumber(double low, double high);

  // callback function for the map topic
  void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &map);

  // callback function for the polygon topic
  void polygonCB(const geometry_msgs::Polygon::ConstPtr &poly);

};

#endif
