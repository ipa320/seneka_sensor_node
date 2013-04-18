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

  // initialize targets
  targets_ = NULL;

  // initialize number of sensors
  sensor_num_ = 0;

  // initialize number of targets
  target_num_ = 0;

  // intialize coverage
  coverage_ = 0;

  // initialize coverage matrix
  coverage_matrix_ = NULL;

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

  // initialize coverage matrix
  coverage_matrix_ = new int[sensor_num_*target_num_];

  // initialize sensor vector with as many entries as specified by sensors_num_
  sensors_.assign(sensor_num_, sensor_model);
}

// destructor
particle::~particle()
{
  delete[] coverage_matrix_;
}

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
  std::vector<geometry_msgs::Pose> result;

  for(size_t i = 0; i < sensors_.size(); i++)
  {
    result.push_back(sensors_[i].getSensorPose());
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

double particle::randomNumber(double low, double high)
{
  return ((double) rand() / RAND_MAX) * (high - low) + low;
}
