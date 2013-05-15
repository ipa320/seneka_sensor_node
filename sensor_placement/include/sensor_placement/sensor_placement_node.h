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

#ifndef SENSOR_PLACEMENT_H
#define SENSOR_PLACEMENT_H

// standard includes
#include <iostream>
#include <vector>
#include <math.h>
#include <time.h>
#include <stdlib.h>

// ros includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <ros/console.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetMap.h>

// external includes
#include <sensor_model.h>
#include <particle.h>
#include <seneka_utilities.h>

using namespace std;
using namespace seneka_utilities;

class sensor_placement_node
{
private:

  // actual 2D-grid-map
  nav_msgs::OccupancyGrid map_;

  // bool variable to check if a map was already received
  bool map_received_;

  // bool variable to check if a polygon was already received
  bool poly_received_;

  // bool variable to check if the targets were already taken from the map
  bool targets_saved_;

  // actual area of interest to be covered by the sensor nodes
  geometry_msgs::PolygonStamped area_of_interest_;
  geometry_msgs::PolygonStamped poly_;

  // number of sensors
  int sensor_num_;

  // maximal range of sensors
  double sensor_range_;

  // opening angles for sensors
  std::vector<double> open_angles_;

  // maximal allowed linear velocity for each sensor in particles
  double max_lin_vel_;

  // maximal allowed angular velocity for each sensor in particles
  double max_ang_vel_;

  // number of particles for PSO
  int particle_num_;

  // maximal number of PSO iterations
  int iter_max_;

  // minimal coverage to stop PSO before reaching maximal number of PSO iterations
  double min_cov_;

  // particle swarm
  vector<particle> particle_swarm_;

  // vector storing the positions global best solution of the particle swarm
  particle global_best_;

  // PSO actual best coverage
  double best_cov_;
  int global_best_multiple_coverage_;

  // number of targets for PSO
  int target_num_;

  // target vectors
  vector<geometry_msgs::Point32> targets_;

  // perimeter vectors
  vector<int> perimeter_x_;
  vector<int> perimeter_y_;

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
  ros::Subscriber poly_sub_;

  // declaration of ros publishers
  ros::Publisher poly_pub_;
  ros::Publisher marker_array_pub_; 
  ros::Publisher map_pub_, map_meta_pub_; 

  // declaration of ros service servers
  ros::ServiceServer ss_start_PSO_;
  ros::ServiceServer ss_test_;

  // declaration of ros service clients
  ros::ServiceClient sc_get_map_;

  // *************************** functions ***************************

  // function to get an array of targets from the map and the area of interest specified as polygon
  bool getTargets();

  // function to initialize PSO-Algorithm
  void initializePSO();

  // function for the actual partcile-swarm-optimization
  void PSOptimize();

  // function to get the current global best solution
  void getGlobalBest();

  // callback function for the start PSO service
  bool startPSOCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // callback function for the test service
  bool testServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // callback function for the polygon topic
  void polygonCB(const geometry_msgs::Polygon::ConstPtr &poly);

  void publishPolygon();

};

#endif
