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
    result.push_back(sensors_.at(i).getSensorPose());
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
    result.push_back(pers_best_.at(i).getSensorPose());
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

// function that sets the member variable targets_with_info_
void particle::setTargetsWithInfo(const std::vector<target_info> &targets_with_info, int target_num)
{
  targets_with_info_ = targets_with_info;
  target_num_ = target_num;
  covered_targets_num_ = 0;
  multiple_coverage_ = 0;
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
      sensors_.at(i).setOpenAngles(new_angles.at(0), new_angles.at(1));
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
    sensors_.at(i).setRange(new_range);
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
    randomPose.position.x = area_of_interest_.polygon.points.at(edge_ind).x
                          + t * (area_of_interest_.polygon.points.at(successor).x - area_of_interest_.polygon.points.at(edge_ind).x);
    randomPose.position.y = area_of_interest_.polygon.points.at(edge_ind).y 
                          + t * (area_of_interest_.polygon.points.at(successor).y - area_of_interest_.polygon.points.at(edge_ind).y);
    randomPose.position.z = 0;

    randomPose.orientation = tf::createQuaternionMsgFromYaw(randomNumber(-PI,PI));

    sensors_.at(i).setSensorPose(randomPose);

    // update the target information
    updateTargetsInfo(i);
  }
  // calculate new coverage
  calcCoverage();
  if(coverage_ == 0)
  {
    pers_best_coverage_ = coverage_;
    pers_best_multiple_coverage_ = multiple_coverage_;
    pers_best_ = sensors_;
  }
}

// function to place all sensors at a given pose
void particle::placeSensorsAtPos(geometry_msgs::Pose new_pose)
{
  for(size_t i = 0; i < sensors_.size(); i++)
  {
    sensors_.at(i).setSensorPose(new_pose);
    updateTargetsInfo(i);
  }
  calcCoverage();
}

// function to initialize the sensors velocities randomly
void particle::initializeRandomSensorVelocities()
{
  // initialize workspace
  geometry_msgs::Twist randomVel;
  geometry_msgs::Twist maxVel;

  for(size_t i = 0; i < sensors_.size(); i++)
  {
    maxVel = sensors_.at(i).getMaxVelocity();
    randomVel.linear.x = -maxVel.linear.x + 2 * maxVel.linear.x * randomNumber(0,1);
    randomVel.linear.y = -maxVel.linear.y + 2 * maxVel.linear.y * randomNumber(0,1);
    randomVel.linear.z = 0;
    randomVel.angular.x = 0;
    randomVel.angular.y = 0;
    randomVel.angular.z = -maxVel.angular.z + 2 * maxVel.angular.z * randomNumber(0,1);

    sensors_.at(i).setVelocity(randomVel);
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
    initial_vel = sensors_.at(i).getVelocity();

    // get current pose for actual sensor
    actual_pose = sensors_.at(i).getSensorPose();

    // get personal best pose for actual sensor
    personal_best_pose = pers_best_.at(i).getSensorPose();

    // get global best pose for actual sensor
    global_best_pose = global_best.at(i);

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
    sensors_.at(i).setVelocity(new_vel);

    // update sensor pose
    new_pose.position.x = actual_pose.position.x + new_vel.linear.x;
    new_pose.position.y = actual_pose.position.y + new_vel.linear.y;
    new_pose.position.z = 0;

    new_angle = actual_angle + new_vel.angular.z;
    // readjust the new angle in the correct interval [-PI,PI]
    while(fabs(new_angle) > PI)
      new_angle = new_angle + (-2) * signum(new_angle) * PI;

    new_pose.orientation = tf::createQuaternionMsgFromYaw(signum(new_angle) * std::min(fabs(new_angle), PI));
    if(!newPositionAccepted(new_pose))
    {
      // find uncovered target far away from initial sensor position
      target_ind = findFarthestUncoveredTarget(i);

      // update sensor position
      new_pose.position.x = targets_with_info_.at(target_ind).world_pos.x;
      new_pose.position.y = targets_with_info_.at(target_ind).world_pos.y;
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
    sensors_.at(i).setSensorPose(new_pose);

    // update the target information
    updateTargetsInfo(i);
  }
  // calculate new coverage
  calcCoverage(); 
}

// function to update the targets_with_info variable
void particle::updateTargetsInfo(size_t sensor_index)
{
  // initialize workspace
  geometry_msgs::Pose sensor_pose = sensors_.at(sensor_index).getSensorPose();

  double sensor_range = sensors_.at(sensor_index).getRange();

  std::vector<double> open_ang = sensors_.at(sensor_index).getOpenAngles();

  double delta = open_ang.front();

  double alpha = tf::getYaw(sensor_pose.orientation);

  double help_angle = 0;

  double x_min = mapToWorldX(0, map_);
  double x_max = mapToWorldX(map_.info.width, map_);
  double y_min = mapToWorldY(0, map_);
  double y_max = mapToWorldY(map_.info.height, map_);

  // initialize index variables
  uint32_t top_index; 
  uint32_t left_index; 
  uint32_t bottom_index; 
  uint32_t right_index; 

  // initialize sensor_kite to approximate the sensors' FOV
  geometry_msgs::Polygon sensor_kite;

  // initialize new point for sensor_kite
  geometry_msgs::Point32 p;

  p.z = 0;

  p.x = std::max(x_min, std::min(x_max, sensor_pose.position.x));
  p.y = std::max(y_min, std::min(y_max, sensor_pose.position.y));

  sensor_kite.points.push_back(p);

  help_angle = alpha - std::min(PI/2, 0.5 * delta);

  if(fabs(help_angle) > PI)
    help_angle = help_angle + (-2) * signum(help_angle) * PI;

  p.x = std::max(x_min, std::min(x_max, sensor_pose.position.x + cos(help_angle)*sensor_range));
  p.y = std::max(y_min, std::min(y_max, sensor_pose.position.y + sin(help_angle)*sensor_range));

  sensor_kite.points.push_back(p);

  help_angle = alpha;

  if(fabs(help_angle) > PI)
    help_angle = help_angle + (-2) * signum(help_angle) * PI;

  p.x = std::max(x_min, std::min(x_max, sensor_pose.position.x + cos(help_angle)*sensor_range));
  p.y = std::max(y_min, std::min(y_max, sensor_pose.position.y + sin(help_angle)*sensor_range));

  sensor_kite.points.push_back(p);

  help_angle = alpha + std::min(PI/2, 0.5 * delta);

  if(fabs(help_angle) > PI)
    help_angle = help_angle + (-2) * signum(help_angle) * PI;

  p.x = std::max(x_min, std::min(x_max, sensor_pose.position.x + cos(help_angle)*sensor_range));
  p.y = std::max(y_min, std::min(y_max, sensor_pose.position.y + sin(help_angle)*sensor_range));

  sensor_kite.points.push_back(p);

  // get bounding box around the sensors' FOV
  geometry_msgs::Polygon bounding_box = getBoundingBox2D(sensor_kite, map_);

  // go through bounding box and update only the targets_with_info within 
    
  // first point of polygon contains x_min and y_min, 3rd contains x_max and y_max
  worldToMap2D(bounding_box.points.at(0), map_, left_index, top_index);
  worldToMap2D(bounding_box.points.at(2), map_, right_index, bottom_index);

  for(uint32_t y = top_index; y < bottom_index; y++ )
  { 
    for (uint32_t x = left_index; x < right_index; x++)
    { 

      // now check every potential target in the sensors' bounding box
      if(targets_with_info_.at(y * map_.info.width + x).potential_target == 1)
      {
        // now we found a target
        if(!targets_with_info_.at(y * map_.info.width + x).occupied)
        {
          // now we found a non-occupied target, so check the coverage
          if(checkCoverage(sensors_.at(sensor_index), targets_with_info_.at(y * map_.info.width + x).world_pos))
          {
            // now we found a non-occupied target covered by the given sensor
            targets_with_info_.at(y * map_.info.width + x).covered_by_sensor.at(sensor_index) = true;

            if(!targets_with_info_.at(y * map_.info.width + x).covered)
            {
              // now the given target is covered by at least one sensor
              targets_with_info_.at(y * map_.info.width + x).covered = true;
              // increment the covered targets counter only if the given target is not covered by another sensor yet
              covered_targets_num_++;

            }
            else
            {
              if(!targets_with_info_.at(y * map_.info.width + x).multiple_covered)
                multiple_coverage_++;

              // now the given target is covered by multiple sensors
              targets_with_info_.at(y * map_.info.width + x).multiple_covered = true;
            }
          }
        }
      }
    } 
  } 
}

// function to calculate the actual  and personal best coverage
void particle::calcCoverage()
{

  // calculate coverage percentage
  coverage_ = (double) covered_targets_num_ / target_num_;

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
  beta = acos( vecDotProd(vec_sensor_target, vec_sensor_dir) / (vecNorm(vec_sensor_target) * vecNorm(vec_sensor_dir) ));
  // check if given target is visible by given sensor
  if( (beta <= ((double) sensor_angles.front()/2)) && (vecNorm(vec_sensor_target) <= sensor_range) )
    result = true;

  return result;
}

// function to check if the new sensor position is accepted
bool particle::newPositionAccepted(geometry_msgs::Pose new_pose_candidate)
{
  // initialize workspace
  bool result = false;
  geometry_msgs::Pose2D dummy_pose2D;
  unsigned int pose_x_index, pose_y_index = 0;

  dummy_pose2D.x = new_pose_candidate.position.x;
  dummy_pose2D.y = new_pose_candidate.position.y;
  dummy_pose2D.theta = tf::getYaw(new_pose_candidate.orientation);
  
  if(pointInPolygon(dummy_pose2D, area_of_interest_.polygon) == -1)
  {
    // the pose candidate is not within the area of interest
    result = false;
  }
  else
  {
    pose_x_index = worldToMapX(new_pose_candidate.position.x, map_);
    pose_y_index = worldToMapY(new_pose_candidate.position.y, map_);

    if(pose_y_index * map_.info.width + pose_x_index >= targets_with_info_.size())
    {
      // indices are out of bounds
      result = false;
    }
    else
    {
      if(targets_with_info_.at(pose_y_index * map_.info.width + pose_x_index).occupied)
      {
        // the pose candidate is within the area of interest 
        // but the desired position is occupied
        result = false;
      }
      else
      {
        // the pose candidate was accepted
        result = true;
      }
    }

  }

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
  geometry_msgs::Pose sensor_pose = sensors_.at(sensor_index).getSensorPose();
  geometry_msgs::Vector3 vec_sensor_target, vec_sensor_dir;

  for(size_t i = 0; i < targets_with_info_.size(); i++)
  {
    if( (!targets_with_info_.at(i).covered) && (targets_with_info_.at(i).potential_target == 1) )
    {
      // we found an uncovered target, check if this is further away from the sensor than the current maximum

      // calculate vector between sensor and target
      vec_sensor_target.x = targets_with_info_.at(i).world_pos.x - sensor_pose.position.x;
      vec_sensor_target.y = targets_with_info_.at(i).world_pos.y - sensor_pose.position.y;
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
  double sensor_range = sensors_.at(sensor_index).getRange();

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
      poly_point_1 = area_of_interest_.polygon.points.at(loop_index);
      poly_point_2 = area_of_interest_.polygon.points.at(loop_index+1);
    }
    else
    {
      poly_point_1 = area_of_interest_.polygon.points.at(loop_index);
      poly_point_2 = area_of_interest_.polygon.points.at(0);
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
