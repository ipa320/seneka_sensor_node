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

#include <particle.h>

using namespace seneka_particle;

// standard constructor
particle::particle()
{

  // initialize number of sensors
  sensor_num_ = 0;

  // initialize number of targets
  target_num_ = 0;

  // intialize coverage
  coverage_ = 0;

  // initialize coverage matrix

  // initialize sensor array with as many entries as specified by sensors_num_
  
}

// constructor with arguments
particle::particle(int num_of_sensors, int num_of_targets, seneka_sensor_model::FOV_2D_model sensor_model)
{
  // initialize number of sensors
  sensor_num_ = num_of_sensors;

  // initialze number of targets
  target_num_ = num_of_targets;

  // intialize coverage
  coverage_ = 0;

  // initialize personal best coverage
  pers_best_coverage_ = 0;

  // initialize coverage matrix
  coverage_matrix_.assign(sensor_num_*target_num_, 0);

  // initialize sensor vector with as many entries as specified by sensors_num_
  sensors_.assign(sensor_num_, sensor_model);
}

// destructor
particle::~particle(){}

// function to get personal best solution
std::vector<seneka_sensor_model::FOV_2D_model> particle::getPersonalBest()
{
  return pers_best_;
}

// function to get actual solution
std::vector<seneka_sensor_model::FOV_2D_model> particle::getActualSolution()
{
  return sensors_;
}

// function to get the sensor positions of the actual solution
std::vector<geometry_msgs::Pose> particle::getSolutionPositions()
{
  // initiliaze workspace
  std::vector<geometry_msgs::Pose> result;

  for(size_t i = 0; i < sensors_.size(); i++)
  {
    result.push_back(sensors_[i].getSensorPose());
  }

  return result;
}

// function to get the sensor positions of the personal best solution
std::vector<geometry_msgs::Pose> particle::getPersonalBestPositions()
{
  // initiliaze workspace
  std::vector<geometry_msgs::Pose> result;

  for(size_t i = 0; i < pers_best_.size(); i++)
  {
    result.push_back(pers_best_[i].getSensorPose());
  }

  return result;
}


// function to get personal best coverage
double particle::getBestCoverage()
{
  return pers_best_coverage_;
}

// function to get actual coverage
double particle::getActualCoverage()
{
  return coverage_;
}

// function that sets the member varialbe sensor_um_ and reserves capacity for vector sensors_
void particle::setSensorNum(int num_of_sensors)
{
   // set number of sensors
  sensor_num_ = num_of_sensors;

  // intialize coverage
  coverage_ = 0;

  // initialize sensor vector with as many entries as specified by sensors_num_
  sensors_.reserve(sensor_num_);

}

// function that sets the member variables targets_x_ and targets_y_
void particle::setTargets(const std::vector<int> & in_x, const std::vector<int> & in_y)
{
  targets_x_ = in_x;
  targets_y_ = in_y;
  target_num_ = targets_x_.size();
}

// function that sets the member variables perimeter_x_ and perimeter_y_
void particle::setPerimeter(const std::vector<int> & in_x, const std::vector<int> & in_y)
{
  perimeter_x_ = in_x;
  perimeter_y_ = in_y;
}

// function that set the map
void particle::setMap(const nav_msgs::OccupancyGrid & new_map)
{
  map_ = new_map;
}

// function that sets the area of interest
void particle::setAreaOfInterest(const geometry_msgs::PolygonStamped & new_poly)
{
  area_of_interest_ = new_poly;
}

// function that sets the opening angles for each sensor in the particle
bool particle::setOpenAngles(std::vector<double> new_angles)
{
  bool result = false;
  if(new_angles.empty() || (new_angles.size() != 2) )
  {
    ROS_WARN("wrong input in particle::setOpenAngles!");
    return result;
  }
  else
  {
    for(size_t i = 0; i < sensors_.size(); i++)
    {
      sensors_[i].setOpenAngles(new_angles[0], new_angles[1]);
    }
    result = true;
    return result;
  }
}

// function that sets the range for each sensor in the particle
void particle::setRange(double new_range)
{
  for(size_t i = 0; i < sensors_.size(); i++)
  {
    sensors_[i].setRange(new_range);
  }
}

// functions to calculate between map (grid) and world coordinates
double particle::mapToWorldX(int map_x)
{
  return map_.info.origin.position.x + (map_x * map_.info.resolution);
}

double particle::mapToWorldY(int map_y)
{
  return map_.info.origin.position.y + (map_y * map_.info.resolution);
}

int particle::worldToMapX(double world_x)
{
  return (world_x - map_.info.origin.position.x) / map_.info.resolution;
}

int particle::worldToMapY(double world_y)
{
  return (world_y - map_.info.origin.position.y) / map_.info.resolution;
}

// function to place the sensors randomly on the perimeter
void particle::placeSensorsRandomlyOnPerimeter()
{
  // initialize workspace
  size_t edge_ind = 0;
  size_t successor = 0;
  double t = 0;
  geometry_msgs::Pose randomPose;

  for(size_t i = 0; i < sensors_.size(); i++)
  {
    // get index of a random edge of the area of interest specified by a polygon
    edge_ind = (int) randomNumber(0, area_of_interest_.polygon.points.size() -1);
    successor = 0;

    if(edge_ind < (area_of_interest_.polygon.points.size() - 1))
      successor = edge_ind++;
    
    t = randomNumber(0,1);

    // get random Pose on perimeter of the area of interest specified by a polygon
    randomPose.position.x = area_of_interest_.polygon.points[edge_ind].x
                          + t * (area_of_interest_.polygon.points[successor].x - area_of_interest_.polygon.points[edge_ind].x);
    randomPose.position.y = area_of_interest_.polygon.points[edge_ind].y 
                          + t * (area_of_interest_.polygon.points[successor].y - area_of_interest_.polygon.points[edge_ind].y);
    randomPose.position.z = 0;

    randomPose.orientation = tf::createQuaternionMsgFromYaw(randomNumber(-PI,PI));

    sensors_[i].setSensorPose(randomPose);

  }
}

// function to initialize the sensors velocities randomly
void particle::initializeRandomSensorVelocities()
{
  // initialize workspace
  geometry_msgs::Twist randomVel;
  geometry_msgs::Twist maxVel;

  for(size_t i = 0; i < sensors_.size(); i++)
  {
    maxVel = sensors_[i].getMaxVelocity();
    randomVel.linear.x = -maxVel.linear.x + 2 * maxVel.linear.x * randomNumber(0,1);
    randomVel.linear.y = -maxVel.linear.y + 2 * maxVel.linear.y * randomNumber(0,1);
    randomVel.linear.z = 0;
    randomVel.angular.x = 0;
    randomVel.angular.y = 0;
    randomVel.angular.z = -maxVel.angular.z + 2 * maxVel.angular.z * randomNumber(0,1);

    sensors_[i].setVelocity(randomVel);
  }
}

// function to update particle during PSO
void particle::updateParticle(std::vector<geometry_msgs::Pose> global_best, double PSO_param_1, double PSO_param_2, double PSO_param_3)
{
  // initialize workspace
  double actual_angle = 0;
  double p_best_angle = 0;
  double g_best_angle = 0;
  double new_angle = 0;

  size_t edge_ind = 0;
  size_t successor = 0;
  double t = 0;

  geometry_msgs::Twist initial_vel;
  geometry_msgs::Pose actual_pose;
  geometry_msgs::Pose personal_best_pose;
  geometry_msgs::Pose global_best_pose;
  geometry_msgs::Twist new_vel;
  geometry_msgs::Pose new_pose;
  geometry_msgs::Pose2D new_pos2D;

  for(size_t i = 0; i < sensors_.size(); i++)
  {
    // get initial velocity for actual sensor
    initial_vel = sensors_[i].getVelocity();

    // get current pose for actual sensor
    actual_pose = sensors_[i].getSensorPose();

    // get personal best pose for actual sensor
    personal_best_pose = pers_best_[i].getSensorPose();

    // get global best pose for actual sensor
    global_best_pose = global_best[i];

    // get orientation angle for actual sensor
    actual_angle = tf::getYaw(actual_pose.orientation);

    // get personal best orientation angle for actual sensor
    p_best_angle = tf::getYaw(personal_best_pose.orientation);

    // get global best orientation angle for actual sensor
    g_best_angle = tf::getYaw(global_best_pose.orientation);

    // calculate vector of camera facing direction
    geometry_msgs::Vector3 vec_sensor_dir;
    vec_sensor_dir.x = cos(actual_angle);
    vec_sensor_dir.y = sin(actual_angle);
    vec_sensor_dir.z = 0;

    // here is the actual particle swarm optimization step
    // update the velocities for each sensor

    // update the linear part
    new_vel.linear.x = (PSO_param_1 * initial_vel.linear.x)
                     + (PSO_param_2 * randomNumber(0,1) * (personal_best_pose.position.x - actual_pose.position.x))
                     + (PSO_param_3 * randomNumber(0,1) * (global_best_pose.position.x - actual_pose.position.x));
    new_vel.linear.y = (PSO_param_1 * initial_vel.linear.y)
                     + (PSO_param_2 * randomNumber(0,1) * (personal_best_pose.position.y - actual_pose.position.y))
                     + (PSO_param_3 * randomNumber(0,1) * (global_best_pose.position.y - actual_pose.position.y));
    new_vel.linear.z = 0;

    // update the angular part
    new_vel.angular.x = 0;
    new_vel.angular.y = 0;
    new_vel.angular.z = (PSO_param_1 * initial_vel.angular.z)
                      + (PSO_param_2 * randomNumber(0,1) * (actual_angle - p_best_angle))
                      + (PSO_param_3 * randomNumber(0,1) * (actual_angle - g_best_angle));

    // set new velocity
    sensors_[i].setVelocity(new_vel);

    // update sensor pose
    new_pos2D.x = new_pose.position.x = actual_pose.position.x + new_vel.linear.x;
    new_pos2D.y = new_pose.position.y = actual_pose.position.y + new_vel.linear.y;
    new_pose.position.z = 0;

    new_angle = actual_angle + new_vel.angular.z;

    while(fabs(new_angle) > PI)
      new_angle = new_angle + (-1) * signum(new_angle) * PI;

    new_pos2D.theta = new_angle;

    if(pointInPolygon(new_pos2D, area_of_interest_.polygon) == -1)
    {
      // get index of a random edge of the area of interest specified by a polygon
      edge_ind = (int) randomNumber(0, area_of_interest_.polygon.points.size() -1);
      successor = 0;

      if(edge_ind < (area_of_interest_.polygon.points.size() - 1))
        successor = edge_ind++;
    
      t = randomNumber(0,1);

      // get random Pose on perimeter of the area of interest specified by a polygon
      new_pose.position.x = area_of_interest_.polygon.points[edge_ind].x
                            + t * (area_of_interest_.polygon.points[successor].x - area_of_interest_.polygon.points[edge_ind].x);
      new_pose.position.y = area_of_interest_.polygon.points[edge_ind].y 
                          + t * (area_of_interest_.polygon.points[successor].y - area_of_interest_.polygon.points[edge_ind].y);
      new_pose.position.z = 0;
    }

    new_pose.orientation = tf::createQuaternionMsgFromYaw(signum(new_angle) * std::min(fabs(new_angle), PI));

    // set new sensor pose
    sensors_[i].setSensorPose(new_pose);

    // update the coverage matrix
    calcCoverageMatrix();
    // calculate new coverage
    calcCoverage();                      
 }
}

// function to calculate the actual  and personal best coverage
void particle::calcCoverage()
{
  // initialize workspace
  int num_covered_targets = 0;
  std::vector<bool> target_already_counted;
  target_already_counted.assign(target_num_, false);

  if(coverage_matrix_.empty())
  {
    coverage_matrix_.assign(sensor_num_*target_num_, 0);
  }
  // go through the sensor vector
  for(size_t i = 0; i < sensors_.size(); i++)
  {
    // go through the target vectors
    for(size_t j = 0; j < targets_x_.size(); j++)
    {
      if( (coverage_matrix_[j * sensors_.size() + i] == 1) && (target_already_counted[j] == false) )
      {
        num_covered_targets++;
        target_already_counted[j] = true;
      }
    }
  }
  // calculate coverage percentage
  coverage_ = (double) num_covered_targets / target_num_;
  // check if the actual coverage is a new personal best
  if(coverage_ > pers_best_coverage_)
  {
    pers_best_coverage_ = coverage_;
    pers_best_ = sensors_;    
  }
}

// function to calculate coverage matrix
void particle::calcCoverageMatrix()
{
  if(coverage_matrix_.empty())
  {
    coverage_matrix_.assign(sensor_num_*target_num_, 0);
  }
  // go through the sensor vector
  for(size_t i = 0; i < sensors_.size(); i++)
  {
    // go through the target vectors
    for(size_t j = 0; j < targets_x_.size(); j++)  
    {
      if(checkCoverage(sensors_[i], targets_x_[j], targets_y_[j]))
        coverage_matrix_[ j * sensors_.size() + i] = 1;
      else
        coverage_matrix_[ j * sensors_.size() + i] = 0;
    }
  }
}

bool particle::checkCoverage(seneka_sensor_model::FOV_2D_model sensor, int target_x, int target_y)
{
  // initialize workspace
  bool result = false;
  double sensor_range;
  std::vector<double> sensor_angles;
  double alpha;
  double beta;
  geometry_msgs::Pose sensor_pose;
  geometry_msgs::Pose2D target_world_Coord;
  geometry_msgs::Vector3 vec_sensor_target, vec_sensor_dir;

  // get pose of given sensor
  sensor_pose = sensor.getSensorPose();

  // get range of given sensor
  sensor_range = sensor.getRange();

  // get angles of given sensor
  sensor_angles = sensor.getOpenAngles();

  // calculate world coordinates from map coordinates of given target
  target_world_Coord.x = mapToWorldX(target_x);
  target_world_Coord.y = mapToWorldY(target_y);
  target_world_Coord.theta = 0;

  // calculate vector between sensor and target
  vec_sensor_target.x = target_world_Coord.x - sensor_pose.position.x;
  vec_sensor_target.y = target_world_Coord.y - sensor_pose.position.y;
  vec_sensor_target.z = 0;

  // get angle of sensor facing direction
  alpha = tf::getYaw(sensor_pose.orientation);

  // calculate vector of camera facing direction
  vec_sensor_dir.x = cos(alpha);
  vec_sensor_dir.y = sin(alpha);
  vec_sensor_dir.z = 0;

  // calculate angle between camera facing direction and target
  beta = acos( fabs(vecDotProd(vec_sensor_target, vec_sensor_dir) ) / (vecNorm(vec_sensor_target) * vecNorm(vec_sensor_dir) ));
  // check if given target is visible by given sensor
  if( (beta <= ((double) sensor_angles[0]/2)) && (vecNorm(vec_sensor_target) <= sensor_range) )
    result = true;

  return result;
}

// function to check if a given point is inside (return 1), outside (return -1) 
// or on an edge (return 0) of a given polygon
int particle::pointInPolygon(geometry_msgs::Pose2D point, geometry_msgs::Polygon polygon)
{
  // initialize workspace variables
  int result = -1;
  bool ignore = false;
  size_t start_index = 0;
  bool start_index_set = false;
  int intersect_count = 0;
  geometry_msgs::Point32 poly_point_1;
  geometry_msgs::Point32 poly_point_2;

  // check in the first loopf, if the point lies on any of the polygons edges
  // and also find the start_index for later algorithm steps
  for(size_t i = 0; i < polygon.points.size(); i++)
  {
    if(i + 1 < polygon.points.size())
    {
      poly_point_1 = polygon.points[i];
      poly_point_2 = polygon.points[i+1];
    }
    else
    {
      poly_point_1 = polygon.points[i];
      poly_point_2 = polygon.points[0];
    }

    if(pointOn1DSegementPose(point, poly_point_1, poly_point_2, 0))
    {
      // point lies on the edge of the polygon
      return 0;
    }
    else
    {
      if(start_index_set == false)
      {
        if(poly_point_1.x != point.x)
        {
          start_index = i;
          start_index_set = true; 
        }
      }
    }
  }

  // initialize points defining the beam
  geometry_msgs::Point32 g_1;
  g_1.x = 0;
  g_1.y = point.y;
  g_1.z = 0;

  geometry_msgs::Point32 g_2;
  g_2.x = 1;
  g_2.y = point.y;
  g_2.z = 0;

  // initialize starting point of polygon
  poly_point_1 = polygon.points[start_index];
  // initialize counters
  size_t loop_counter = start_index + 1;
  size_t loop_counter_mod = loop_counter % polygon.points.size();

  bool while_loop_valid = true;
  int start_index_counter = 0;

  while(while_loop_valid)
  {
    if(loop_counter_mod == (start_index) && start_index_counter == 1)
    {
      while_loop_valid = false;
    }

    if(loop_counter_mod == (start_index + 1) && start_index_counter == 0)
    {
      start_index_counter++;
    }

    poly_point_2 = polygon.points[loop_counter_mod];

    if(ignore)
    {
      // check if poly_point_2 is on the complete beam line
      if(pointOn1DSegementPoint(poly_point_2, g_1, g_2, 2))
      {
        ignore = true;
      }
      else
      {
        if(edgeIntersectsBeamOrLine(point, poly_point_1, poly_point_2, 1))
        {
          // the line from starting point intersects the given polygon edge
          intersect_count ++;
        }
        ignore = false;
        poly_point_1 = poly_point_2;
        
      }
    }
    else
    {
      // check if poly_point_2 is on the beam
      if(pointOn1DSegementPoint(poly_point_2, g_1, g_2, 1))
      {
        ignore = true;
      }
      else
      {
        if(edgeIntersectsBeamOrLine(point, poly_point_1, poly_point_2, 0))
        {
          // the beam to the right from starting point intersects the given polygon edge
          intersect_count ++;
        }
        ignore = false;
        poly_point_1 = poly_point_2;
      }
    }
    // increment loop counters
    loop_counter++;
    loop_counter_mod = loop_counter % polygon.points.size();    
  }

  if(intersect_count % 2 == 0)
    result = -1;
  else
    result = 1;

  return result;
}

// helper function to check if a point lies on a 1D-Segment
// segID = 0 (edge), segID = 1 (beam), segID = 2 (line)
bool particle::pointOn1DSegementPose(geometry_msgs::Pose2D start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID)
{
  bool result = false;

  if( (start.x == border_1.x && start.y == border_1.y) || (start.x == border_2.x && start.y == border_2.y))
  {
    // start equals one of the borders
    result = true;
  }
  else
  {
    double t = 0;
    if( (border_2.x - border_1.x) != 0)
    {
      t = (start.x - border_1.x) / (border_2.x - border_1.x);
    }
    else
    {
      if( (border_2.y - border_1.y) != 0)
      {
        t = (start.y - border_1.y) / (border_2.y - border_1.y);
      }
    }
    bool checker = false;
    switch(segID)
    {
      case 0: // edge
        if(t > 0 && t < 1)
          checker = true;
        break;
      case 1: // beam
        if(t > 0)
          checker = true;
        break;
      case 2: // line
        checker = true;
        break;
      default: // wrong input
        checker = false; 
    }
    if(checker)
    {
      if( ( fabs(border_1.x*(1-t) + t*border_2.x - start.x) <= 0.1) && ( fabs(border_1.y*(1-t) + t*border_2.y - start.y) <= 0.1) )
      {
        // start lies on the segment
        result = true;
      }
    }
  }

  return result;
}

bool particle::pointOn1DSegementPoint(geometry_msgs::Point32 start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID)
{
  geometry_msgs::Pose2D help_pose;
  help_pose.x = start.x;
  help_pose.y = start.y;
  help_pose.theta = 0;

  return pointOn1DSegementPose(help_pose, border_1, border_2, segID);
}

// helper function to check if the beam of line from start intersects the given plygon edge
// segID = 0 (beam), segID = 1 (line)
bool particle::edgeIntersectsBeamOrLine(geometry_msgs::Pose2D start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID)
{
  // initialize workspace
  bool result = false;
  if(border_1.y == border_2.y)
  {
    // edge is parallel or coincides with line or beam
    result = false;
  }
  else
  {
    double t = -(start.x - border_1.x) + ( (border_2.x - border_1.x) * (start.y - border_1.y) ) / (border_2.y - border_1.y);
    double s = (start.y - border_1.y) / (border_2.y - border_1.y);

    switch(segID)
    {
      case 0: // beam
        if(t > 0 && s > 0 && s < 1)
          result = true;
        break;
      case 1: // line
        if(s > 0 && s < 1)
          result = true;
        break;
      default: // wrong input
        result = false; 
    }
  }

  return result;

}


// function to calculate the norm of a 2D/3D vector
double vecNorm(double x, double y, double z)
{
  return sqrt( x * x + y * y + z * z);
}

double particle::vecNorm(geometry_msgs::Vector3 v)
{
  return sqrt(vecDotProd(v,v));
}

// function to calculate the dot product of two vectors
double particle::vecDotProd(geometry_msgs::Vector3 v, geometry_msgs::Vector3 w)
{
  return (v.x * w.x + v.y * w.y + v.z * w.z);
}

// function to generate random number in given interval
double particle::randomNumber(double low, double high)
{
  return ((double) rand() / RAND_MAX) * (high - low) + low;
}

// signum function
int particle::signum(double x)
{
  if(x >= 0)
    return 1;
  if(x < 0)
    return -1;
}

// returns all visualization markers of the particle
visualization_msgs::MarkerArray particle::getVisualizationMarkers()
{
  visualization_msgs::MarkerArray array, tmp;
  std::vector<seneka_sensor_model::FOV_2D_model>::iterator it;
  unsigned int id = 0;
  // loop over all sensors
  for ( it = sensors_.begin(); it != sensors_.end(); ++it )
  {
    tmp = it->getVisualizationMarkers(id);
    // copy over all markers
    for (unsigned int i = 0; i < tmp.markers.size(); i++)
      array.markers.push_back(tmp.markers.at(i));
    
    id++;
  }

  return array;
}
