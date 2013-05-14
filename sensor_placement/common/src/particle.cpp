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

#include <particle.h>

// standard constructor
particle::particle()
{

  // initialize number of sensors
  sensor_num_ = 0;

  // initialize number of targets
  target_num_ = 0;

  // intialize coverage
  coverage_ = 0;

  // initialize personal best coverage
  pers_best_coverage_ = 0;

  //initialize multiple coverage indices
  multiple_coverage_ = 0;
  pers_best_multiple_coverage_ = 0;

  // initialize coverage matrix

  // initialize sensor array with as many entries as specified by sensors_num_
  
}

// constructor with arguments
particle::particle(int num_of_sensors, int num_of_targets, FOV_2D_model sensor_model)
{
  // initialize number of sensors
  sensor_num_ = num_of_sensors;

  // initialze number of targets
  target_num_ = num_of_targets;

  // intialize coverage
  coverage_ = 0;

  // initialize personal best coverage
  pers_best_coverage_ = 0;

  //initialize multiple coverage indices
  multiple_coverage_ = 0;
  pers_best_multiple_coverage_ = 0;

  // initialize coverage matrix
  coverage_matrix_.assign(sensor_num_*target_num_, 0);

  // initialize sensor vector with as many entries as specified by sensors_num_
  sensors_.assign(sensor_num_, sensor_model);
}

// destructor
particle::~particle(){}

// function to get personal best solution
std::vector<FOV_2D_model> particle::getPersonalBest()
{
  return pers_best_;
}

// function to get actual solution
std::vector<FOV_2D_model> particle::getActualSolution()
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

// function to get multiple coverage index
int particle::getMultipleCoverageIndex()
{
  return multiple_coverage_;
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

// function that sets the member variable targets_
void particle::setTargets(const std::vector<geometry_msgs::Point32> & targets)
{
  targets_ = targets;
  target_num_ = targets_.size();
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

  int target_ind = -1;

  geometry_msgs::Twist initial_vel;
  geometry_msgs::Pose actual_pose;
  geometry_msgs::Pose personal_best_pose;
  geometry_msgs::Pose global_best_pose;
  geometry_msgs::Twist new_vel;
  geometry_msgs::Pose new_pose;

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
    new_pose.position.x = actual_pose.position.x + new_vel.linear.x;
    new_pose.position.y = actual_pose.position.y + new_vel.linear.y;
    new_pose.position.z = 0;

    new_angle = actual_angle + new_vel.angular.z;

    // readjust the new angle in the correct interval [-PI,PI]
    while(fabs(new_angle) > PI)
      new_angle = new_angle + (-1) * signum(new_angle) * PI;

    new_pose.orientation = tf::createQuaternionMsgFromYaw(signum(new_angle) * std::min(fabs(new_angle), PI));

    if(!newPositionAccepted(new_pose))
    {
      // find uncovered target far away from initial sensor position
      target_ind = findFarthestUncoveredTarget(i);

      // update sensor position
      new_pose.position.x = targets_[target_ind].x;
      new_pose.position.y = targets_[target_ind].y;
      new_pose.position.z = 0;

      // invert sensor direction
      new_angle = new_angle + (-1) * signum(new_angle) * PI;
      new_pose.orientation = tf::createQuaternionMsgFromYaw(signum(new_angle) * std::min(fabs(new_angle), PI));
    }

    while(!newOrientationAccepted(i, new_pose))
    {
      // try next angle in 10/180*PI steps
      new_angle = new_angle + 0.1745;
      if(new_angle > PI)
        new_angle = new_angle - 2 * PI;
      new_pose.orientation = tf::createQuaternionMsgFromYaw(signum(new_angle) * std::min(fabs(new_angle), PI));
    }

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
  num_sensors_cover_target_.assign(target_num_, 0);

  if(coverage_matrix_.empty())
  {
    coverage_matrix_.assign(sensor_num_*target_num_, 0);
  }
  // go through the sensor vector
  for(size_t i = 0; i < sensors_.size(); i++)
  {
    // go through the target vectors
    for(size_t j = 0; j < targets_.size(); j++)
    {
      if( coverage_matrix_[j * sensors_.size() + i] == 1)
      {
        if(target_already_counted[j] == false)
        {
          num_covered_targets++;
          target_already_counted[j] = true;
          num_sensors_cover_target_[j]++;
        }
        else
        {
          num_sensors_cover_target_[j]++;
        }
      }
    }
  }
  // calculate coverage percentage
  coverage_ = (double) num_covered_targets / target_num_;

  calcMultipleCoverage();

  // check if the actual coverage is a new personal best
  if(coverage_ > pers_best_coverage_)
  {
    pers_best_coverage_ = coverage_;
    pers_best_multiple_coverage_ = multiple_coverage_;
    pers_best_ = sensors_;    
  }
  else
  {
    if( (coverage_ == pers_best_coverage_) && (multiple_coverage_ < pers_best_multiple_coverage_) )
    {
      pers_best_coverage_ = coverage_;
      pers_best_multiple_coverage_ = multiple_coverage_;
      pers_best_ = sensors_;
    }
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
    for(size_t j = 0; j < targets_.size(); j++)  
    {
      if(checkCoverage(sensors_[i], targets_[j]))
        coverage_matrix_[ j * sensors_.size() + i] = 1;
      else
        coverage_matrix_[ j * sensors_.size() + i] = 0;
    }
  }
}

// function to the multiple coverage number
void particle::calcMultipleCoverage()
{
  multiple_coverage_ = 0;
  for(size_t i = 0; i < num_sensors_cover_target_.size(); i++)
  {
    if(num_sensors_cover_target_[i] > 0)
      multiple_coverage_++;
  }
}

bool particle::checkCoverage(FOV_2D_model sensor, geometry_msgs::Point32 target)
{
  // initialize workspace
  bool result = false;
  double sensor_range;
  std::vector<double> sensor_angles;
  double alpha;
  double beta;
  geometry_msgs::Pose sensor_pose;
  geometry_msgs::Vector3 vec_sensor_target, vec_sensor_dir;

  // get pose of given sensor
  sensor_pose = sensor.getSensorPose();

  // get range of given sensor
  sensor_range = sensor.getRange();

  // get angles of given sensor
  sensor_angles = sensor.getOpenAngles();

  // calculate vector between sensor and target
  vec_sensor_target.x = target.x - sensor_pose.position.x;
  vec_sensor_target.y = target.y - sensor_pose.position.y;
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

// function to check if the new sensor position is accepted
bool particle::newPositionAccepted(geometry_msgs::Pose new_pose_candidate)
{
  bool result = false;
  geometry_msgs::Pose2D dummy_pose2D;

  dummy_pose2D.x = new_pose_candidate.position.x;
  dummy_pose2D.y = new_pose_candidate.position.y;
  dummy_pose2D.theta = tf::getYaw(new_pose_candidate.orientation);

  if(pointInPolygon(dummy_pose2D, area_of_interest_.polygon) == -1)
    result = false;
  else
    result = true;

  return result;
}

// function to check if the new sensor orientation is accepted
bool particle::newOrientationAccepted(size_t sensor_index, geometry_msgs::Pose new_pose_candidate)
{
  // initialize workspace
  bool result = false;

  if(!sensorBeamIntersectsPerimeter(sensor_index, new_pose_candidate))
    result = true;

  return result;
}

// helper function to find an uncovered target far away from a given sensor position
// the return value is the index of that uncovered target
int particle::findFarthestUncoveredTarget(size_t sensor_index)
{
  // initialize workspace
  int result = -1;
  double max_distance = 0;
  double actual_distance = 0;
  geometry_msgs::Pose sensor_pose = sensors_[sensor_index].getSensorPose();
  geometry_msgs::Vector3 vec_sensor_target, vec_sensor_dir;

  for(size_t i = 0; i < targets_.size(); i++)
  {
    if(coverage_matrix_[i * sensors_.size() + sensor_index] == 0)
    {
      // we found an uncovered target, check if this is further away from the sensor than the current maximum

      // calculate vector between sensor and target
      vec_sensor_target.x = targets_[i].x - sensor_pose.position.x;
      vec_sensor_target.y = targets_[i].y - sensor_pose.position.y;
      vec_sensor_target.z = 0;

      actual_distance = vecNorm(vec_sensor_target);

      if(actual_distance > max_distance)
      {
        max_distance = actual_distance;
        result = i;
      }
    }
  }
  return result;
}

// helper function to check, if the sensor is facing outside the area of interest
bool particle::sensorBeamIntersectsPerimeter(size_t sensor_index, geometry_msgs::Pose new_pose_candidate)
{
  // intialize workspace
  bool result = false;
  size_t poly_size = area_of_interest_.polygon.points.size();
  size_t loop_index = 0;

  geometry_msgs::Point32 poly_point_1 = geometry_msgs::Point32();
  geometry_msgs::Point32 poly_point_2 = geometry_msgs::Point32();

  double alpha = tf::getYaw(new_pose_candidate.orientation);
  double sensor_range = sensors_[sensor_index].getRange();

  double v1, v2, t;

  // calculate vector (with length sensor_range) of camera facing direction
  v1 = sensor_range * cos(alpha);
  v2 = sensor_range * sin(alpha);

  // go through the points of the polygon
  // if the first intersection was found, exit the loop
  while(loop_index < poly_size && result == false)
  {
    // check each edge of the perimeter
    if(loop_index + 1 < poly_size)
    {
      poly_point_1 = area_of_interest_.polygon.points[loop_index];
      poly_point_2 = area_of_interest_.polygon.points[loop_index+1];
    }
    else
    {
      poly_point_1 = area_of_interest_.polygon.points[loop_index];
      poly_point_2 = area_of_interest_.polygon.points[0];
    }
    if(v1 == 0)
      t = intersectionCalculation(v2,v1,poly_point_2.y - poly_point_1.y, poly_point_2.x - poly_point_1.x, poly_point_1.y - new_pose_candidate.position.y, poly_point_1.x - new_pose_candidate.position.x);
    else
      t = intersectionCalculation(v1,v2,poly_point_2.x - poly_point_1.x, poly_point_2.y - poly_point_1.y, poly_point_1.x - new_pose_candidate.position.x, poly_point_1.y - new_pose_candidate.position.y);

    if(t > 0 && t < 1)
      result = true;

    loop_index++;
  }

  return result;
}

// helper function for the actual calculation step in sensorBeamIntersectsPerimeter function
double particle::intersectionCalculation(double v1, double v2, double x1, double x2, double y1, double y2)
{
  // initialize workspace
  double result = 0;
  // without loss of generality we assume v1 to be nonzero 
  if(floor(x2*v1 - x1*v2) > 0.01)
  {
    result = (y1 / v1) - (x1 / v1)*( (y2 - y1 * (v2 / v1) ) / (x2 - x1 * (v2 / v1)) );
  }

  return result;
}

// returns all visualization markers of the particle
visualization_msgs::MarkerArray particle::getVisualizationMarkers()
{
  visualization_msgs::MarkerArray array, tmp;
  std::vector<FOV_2D_model>::iterator it;
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
