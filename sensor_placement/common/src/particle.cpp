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
void particle::setTargets(std::vector<int> in_x, std::vector<int> in_y)
{
  targets_x_ = in_x;
  targets_y_ = in_y;
}

// function that sets the member variables perimeter_x_ and perimeter_y_
void particle::setPerimeter(std::vector<int> in_x, std::vector<int> in_y)
{
  perimeter_x_ = in_x;
  perimeter_y_ = in_y;
}

// function that set the map
void particle::setMap(nav_msgs::OccupancyGrid new_map)
{
  map_ = new_map;
}

// function that sets the area of interest
void particle::setAreaOfInterest(geometry_msgs::PolygonStamped new_poly)
{
  area_of_interest_ = new_poly;
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
    edge_ind = (int) randomNumber(0, area_of_interest_.polygon.points.size());
    successor = 0;
    geometry_msgs::Pose randomPose;

    if(edge_ind < area_of_interest_.polygon.points.size())
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
  for(size_t i = 0; i < sensors_.size(); i++)
  {

    // get initial velocity for actual sensor
    geometry_msgs::Twist initial_vel = sensors_[i].getVelocity();

    // get current pose for actual sensor
    geometry_msgs::Pose actual_pose = sensors_[i].getSensorPose();

    // get personal best pose for actual sensor
    geometry_msgs::Pose personal_best_pose = pers_best_[i].getSensorPose();

    // get global best pose for actual sensor
    geometry_msgs::Pose global_best_pose = global_best[i];

    // here is the actual particle swarm optimization step
    // update the velocities for each sensor
    geometry_msgs::Twist new_vel;

    // update the linear part
    new_vel.linear.x = (PSO_param_1 * initial_vel.linear.x)
                     + (PSO_param_2 * randomNumber(0,1) * (personal_best_pose.position.x - actual_pose.position.x))
                     + (PSO_param_3 * randomNumber(0,1) * (global_best_pose.position.x - actual_pose.position.x));
    new_vel.linear.y = (PSO_param_1 * initial_vel.linear.y)
                     + (PSO_param_2 * randomNumber(0,1) * (personal_best_pose.position.y - actual_pose.position.y))
                     + (PSO_param_3 * randomNumber(0,1) * (global_best_pose.position.y - actual_pose.position.y));
    new_vel.linear.z = (PSO_param_1 * initial_vel.linear.z)
                     + (PSO_param_2 * randomNumber(0,1) * (personal_best_pose.position.z - actual_pose.position.z))
                     + (PSO_param_3 * randomNumber(0,1) * (global_best_pose.position.z - actual_pose.position.z));

    // upadate the angular part
    /*new_vel.angular.x = (PSO_param_1 * initial_vel.angular.x)
                     + (PSO_param_2 * randomNumber(0,1) * (personal_best_pose.position.x - actual_pose.position.x))
                     + (PSO_param_3 * randomNumber(0,1) * (global_best_pose.position.x - actual_pose.position.x));
    new_vel.angular.y = (PSO_param_1 * initial_vel.angular.y)
                     + (PSO_param_2 * randomNumber(0,1) * (personal_best_pose.position.y - actual_pose.position.y))
                     + (PSO_param_3 * randomNumber(0,1) * (global_best_pose.position.y - actual_pose.position.y));
    new_vel.angular.z = (PSO_param_1 * initial_vel.angular.z)
                     + (PSO_param_2 * randomNumber(0,1) * (personal_best_pose.position.z - actual_pose.position.z))
                     + (PSO_param_3 * randomNumber(0,1) * (global_best_pose.position.z - actual_pose.position.z));*/
 }
}

// function to calculate the actual  and personal best coverage
void particle::calcCoverage()
{
  // initialize workspace
  int num_covered_objects = 0;
  std::vector<bool> target_already_counted;
  target_already_counted.assign(targets_x_.size(), false);

  if(!coverage_matrix_.empty())
  {
    // go through the sensor vector
    for(size_t i = 0; i < sensors_.size(); i++)
    {
      // go through the target vectors
      for(size_t j = 0; j < targets_x_.size(); j++)
      {
        if( (coverage_matrix_[j * sensors_.size() + i] == 1) && (target_already_counted[j] == false) )
        {
          num_covered_objects++;
          target_already_counted[j] = true;
        }
      }
    }
    // calculate coverage percentage
    coverage_ = num_covered_objects / targets_x_.size();
    // check if the actual coverage is a new personal best
    if(coverage_ > pers_best_coverage_)
    {
      pers_best_coverage_ = coverage_;
      pers_best_ = sensors_;
    }
  }
}

// function to calculate coverage matrix
void particle::calcCoverageMatrix()
{
  if(!coverage_matrix_.empty())
  {
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
  target_world_Coord.y = mapToWorldX(target_y);
  target_world_Coord.theta = 0;

  // calculate vector between sensor and target
  vec_sensor_target.x = target_world_Coord.x - sensor_pose.position.x;
  vec_sensor_target.y = target_world_Coord.y - sensor_pose.position.y;
  vec_sensor_target.z = 0;

  // get angle of sensor facing direction
  alpha = tf::getYaw(sensor_pose.orientation);

  // calculate vector of camera facving direction
  vec_sensor_dir.x = cos(alpha);
  vec_sensor_dir.y = sin(alpha);
  vec_sensor_dir.z = 0;

  // calculate angle between camera facing direction and target
  beta = acos( (vecDotProd(vec_sensor_target, vec_sensor_dir) ) / (vecNorm(vec_sensor_target) * vecNorm(vec_sensor_dir) ));

  // check if given target is visible by given sensor
  if( (beta <= sensor_angles[0]/2) && vecNorm(vec_sensor_target) <= sensor_range)
    result = true;

  return result;
}

// function to calculate the norm of a 2D/3D vector
double vecNorm(double x, double y, double z)
{
  return sqrt( x * x + y * y + z * z);
}

double particle::vecNorm(geometry_msgs::Vector3 v)
{
  return vecDotProd(v,v);
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

// visualize the sensors of the particle
visualization_msgs::MarkerArray particle::visualize()
{
  visualization_msgs::MarkerArray array, tmp;
  std::vector<seneka_sensor_model::FOV_2D_model>::iterator it;
  unsigned int id = 0;
  // loop over all sensors
  for ( it = sensors_.begin(); it != sensors_.end(); ++it )
  {
    tmp = it->visualize(id);
    // copy over all markers
    for (unsigned int i = 0; i < tmp.markers.size(); i++)
      array.markers.push_back(tmp.markers.at(i));
    
    id++;
  }

  return array;
}