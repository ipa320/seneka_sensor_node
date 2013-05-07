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

#ifndef PARTICLE_H
#define PARTICLE_H

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
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h> 
#include <std_msgs/String.h> 
#include <visualization_msgs/MarkerArray.h>

// external includes
#include <sensor_model.h>

#define PI 3.14159265

namespace seneka_particle{

class particle
{
private:

  // std-vector storing the sensors
  std::vector<seneka_sensor_model::FOV_2D_model > sensors_;

  // std-vector storing the personal best solution of the particle
  std::vector<seneka_sensor_model::FOV_2D_model> pers_best_;

  // target vectors
  std::vector<int> targets_x_;
  std::vector<int> targets_y_;

  // perimeter vectors
  std::vector<int> perimeter_x_;
  std::vector<int> perimeter_y_;

  // number of sensors
  int sensor_num_;

  // number of targets
  int target_num_;

  // actual coverage
  double coverage_;

  // personal best coverage
  double pers_best_coverage_;

  // multiple coverage numbers
  int multiple_coverage_;
  int pers_best_multiple_coverage_;

  // coverage matrix in row-major order (sensors in rows, target in columns)
  std::vector<int> coverage_matrix_;

  // the i-th entry holds the number of sensors covering the target with index i
  std::vector<int> num_sensors_cover_target_;

  // actual area of interest to be covered by the sensor nodes
  geometry_msgs::PolygonStamped area_of_interest_;

  // actual map
  nav_msgs::OccupancyGrid map_;

public:

  // standard constructor
  particle();

  // constructor with arguments
  particle(int num_of_sensors, int num_of_targets, seneka_sensor_model::FOV_2D_model sensor_model);

  // destructor
  ~particle();

  // ************************ getter functions ************************

  // function to get personal best solution
  std::vector<seneka_sensor_model::FOV_2D_model> getPersonalBest();

  // function to get actual solution
  std::vector<seneka_sensor_model::FOV_2D_model> getActualSolution();

  // function to get the sensor positions of the actual solution
  std::vector<geometry_msgs::Pose> getSolutionPositions();

  // function to get the sensor positions of the personal best solution
  std::vector<geometry_msgs::Pose> getPersonalBestPositions();

  // function to get personal best coverage
  double getBestCoverage();

  // function to get actual coverage
  double getActualCoverage();

  // function to get multiple coverage index
  int getMultipleCoverageIndex();

  // ************************ setter functions ************************

  // function that sets the member variable sensor_num_ and reserves capacity for vector sensors_
  void setSensorNum(int num_of_sensors);

  // function that sets the member variables targets_x_ and targets_y_
  void setTargets(const std::vector<int> & in_x, const std::vector<int> & in_y);

  // function that sets the member variables perimeter_x_ and perimeter_y_
  void setPerimeter(const std::vector<int> & in_x, const std::vector<int> & in_y);

  // function that sets the map
  void setMap(const nav_msgs::OccupancyGrid & new_map);

  // function that sets the area of interest
  void setAreaOfInterest(const geometry_msgs::PolygonStamped & new_poly);

  // function that sets the opening angles for each sensor in the particle
  bool setOpenAngles(std::vector<double> new_angles);

  // function that sets the range for each sensor in the particle
  void setRange(double new_range);

  // ********************** coordinate functions **********************

  // functions to calculate between map (grid) and world coordinates
  double mapToWorldX(int map_x);
  double mapToWorldY(int map_y);
  int worldToMapX(double world_x);
  int worldToMapY(double world_y);

  // ************************ update functions ************************

  // function to place the sensors randomly on the perimeter
  void placeSensorsRandomlyOnPerimeter();

  // function to initialize the sensors velocities randomly
  void initializeRandomSensorVelocities();

  // function to update particle during PSO
  void updateParticle(std::vector<geometry_msgs::Pose> global_best, double PSO_param_1, double PSO_param_2, double PSO_param_3);

  // function to calculate the actual  and personal best coverage
  void calcCoverage();

  // function to calculate coverage matrix
  void calcCoverageMatrix();

  // function to the multiple coverage number
  void calcMultipleCoverage();

  // function to check coverage of given sensor and target
  bool checkCoverage(seneka_sensor_model::FOV_2D_model sensor, int target_x, int target_y);

  // function to check if the new sensor position is accepted
  bool newPositionAccepted(geometry_msgs::Pose new_pose_candidate);

  // ************************* help functions *************************

  // function to check if a given point is inside (return 1), outside (return -1) 
  // or on an edge (return 0) of a given polygon
  int pointInPolygon(geometry_msgs::Pose2D point, geometry_msgs::Polygon polygon);

  // helper functions to check if a point lies on a 1D-Segment
  // segID = 0 (edge), segID = 1 (beam), segID = 2 (line)
  bool pointOn1DSegementPose(geometry_msgs::Pose2D start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID);
  bool pointOn1DSegementPoint(geometry_msgs::Point32 start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID);

  // helper function to check if the beam of line from start intersects the given plygon edge
  // segID = 0 (beam), segID = 1 (line)
  bool edgeIntersectsBeamOrLine(geometry_msgs::Pose2D start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID);

  // helper function to find an uncovered target far away from a given sensor position
  // the return value is the index of that uncovered target
  int findFarthestUncoveredTarget(size_t sensor_index);

  // functions to calculate the norm of a 2D/3D vector
  double vecNorm(double x, double y, double z = 0);
  double vecNorm(geometry_msgs::Vector3 v);

  // function to calculate the dot product of two vectors
  double vecDotProd(geometry_msgs::Vector3 v, geometry_msgs::Vector3 w);

  // function to generate random number in given interval
  double randomNumber(double low, double high);

  // signum function
  int signum(double x);

  // returns all visualization markers of the particle
  visualization_msgs::MarkerArray getVisualizationMarkers();

};
}
#endif
