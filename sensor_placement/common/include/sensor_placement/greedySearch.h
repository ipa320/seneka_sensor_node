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
 * Author: Muhammad Bilal Chughtai, email:Muhammad.Chughtai@ipa.fraunhofer.de
 *
 * Date of creation: August 2013
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

#ifndef GREEDYSEARCH_H
#define GREEDYSEARCH_H

// standard includes
#include <iostream>
#include <vector>

// ros includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <ros/console.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

// external includes
#include <sensor_model.h>
#include <seneka_utilities.h>

using namespace seneka_utilities;

class greedySearch
{
private:

  // std-vector for storing the sensors
  std::vector< FOV_2D_model > sensors_;

  // point info vectors
  std::vector<point_info> * pPoint_info_vec_;
  std::vector<GS_point_info> GS_pool_;

  // number of sensors
  int sensor_num_;

  // number of targets
  int target_num_;
  int covered_targets_num_;

  // coverage data for all specified orientations at fixed point (modified in updateGSpointsRaytracing)
  std::vector<int> coverage_vec_;


  // actual coverage
  double coverage_;

  // maximum sensor coverage information
  int max_sensor_cov_;
  int max_sensor_cov_point_id_;
  geometry_msgs::Pose max_sensor_cov_pose_;

  // angle and cell resolution for the Greedy Search
  unsigned int angle_resolution_;
  unsigned int cell_search_resolution_;

  // actual area of interest to be covered by the sensor nodes
  const geometry_msgs::PolygonStamped * pArea_of_interest_;

  // forbidden area for the placement of sensors
  const geometry_msgs::PolygonStamped * pForbidden_poly_;

  // actual map
  const nav_msgs::OccupancyGrid * pMap_;


public:

  // standard constructor
  greedySearch();

  // constructor with arguments
  greedySearch(int num_of_sensors, int num_of_targets, FOV_2D_model sensor_model);

  // destructor
  ~greedySearch();

  // declaration of ros publisher for publishing greedySearch solution
  ros::Publisher marker_array_pub_;

  // ************************ update functions ************************;

  // function for finding maximum coverage position (using Greedy Search Algorithm) and placing sensor at that position
  void greedyPlacement(size_t sensor_index);

  // function for finding maximum coverage position (using Greedy Search Algorithm) and placing sensor at that position
  void newGreedyPlacement(size_t sensor_index);

  // function to update the GS_point_info with raytracing
  void updateGSpointsRaytracing(size_t sensor_index, int point_id, bool update_covered_info);

  //function to get the coverage done by the sensor
  int getCoverageRaytracing(size_t sensor_index);

  // function to calculate coverage achieved
  double calGScoverage();


  // ************************ getter functions ************************

  // function to get angle resolution for Greedy Placement function
  unsigned int getAngleResolution();

  // function to get cell search resolution for Greedy Placement function
  unsigned int getCellSearchResolution();

  // function to get maximum sensor coverage
  int getMaxSensorCov();

  // function to get maximum sensor coverage point ID
  int getMaxSensorCovPointID();

  // function to get maximum sensor coverage pose
  geometry_msgs::Pose getMaxSensorCovPOSE();


  // ************************ setter functions ************************

  // function to set maximum coverage by a sensor
  void setMaxSensorCov(int coverage);

  // function to set max sensor coverage point ID
  void setMaxSensorCovPointID(int point_id);

  // function to set maximum sensor coverage pose
  void setMaxSensorCovPOSE(geometry_msgs::Pose sensor_pose);

  // function to reset maximum coverage information for new sensor placement
  void resetMaxSensorCovInfo();

  // function to set the information for all targets (point_info_vec_)
  void setPointInfoVec(std::vector<point_info> & point_info_vec, int target_num);

  // function to set the information for GS pool
  void setGSpool(const std::vector<GS_point_info> &GS_pool);

  // function to reset the max targets covered information for all points in GS pool
  void resetGSpool();

  // function that sets the map
  void setMap(const nav_msgs::OccupancyGrid & new_map);

  // function that sets the area of interest
  void setAreaOfInterest(const geometry_msgs::PolygonStamped & new_poly);

  // function that sets forbidden area
  void setForbiddenArea(const geometry_msgs::PolygonStamped & new_forbidden_area);

  // function that sets the opening angles for each sensor
  bool setOpenAngles(std::vector<double> new_angles);

  // function that sets the range for each sensor
  void setRange(double new_range);

  // function to create and set a lookup table for raytracing for each sensor
  void setLookupTable(const std::vector< std::vector<geometry_msgs::Point32> > * pLookup_table);

  // function to set angle resolution for Greedy Placement function
  void setAngleResolution(unsigned int angle_resolution);

  // function to set the cell search resolution for Greedy Placement function
  void setCellSearchResolution(unsigned int cell_search_resolution);


  // ************************* help functions *************************

  // returns all visualization markers of the Greedy Search solution
  visualization_msgs::MarkerArray getVisualizationMarkers();


};

#endif // GREEDYSEARCH_H
