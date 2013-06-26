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

#include <sensor_placement_node.h>

// constructor
sensor_placement_node::sensor_placement_node()
{
  // create node handles
  nh_ = ros::NodeHandle();
  pnh_ = ros::NodeHandle("~");

  // ros subscribers

  // ros publishers
  forbidden_poly_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("forbidden_area", 1,true);
  poly_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("out_poly",1,true);
  nav_path_pub_ = nh_.advertise<nav_msgs::Path>("out_path",1,true);
  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("out_marker_array",1,true);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("out_cropped_map",1,true);
  map_meta_pub_ = nh_.advertise<nav_msgs::MapMetaData>("out_cropped_map_metadata",1,true);

  // ros service servers
  ss_start_PSO_ = nh_.advertiseService("StartPSO", &sensor_placement_node::startPSOCallback, this);
  ss_test_ = nh_.advertiseService("TestService", &sensor_placement_node::testServiceCallback, this);

  // ros service clients
  sc_get_map_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");

  // get parameters from parameter server if possible
  getParams();

  // initialize best coverage
  best_cov_ = 0;

  // initialize global best multiple coverage
  global_best_multiple_coverage_ = 0;

  // initialize number of targets
  target_num_ = 0;

  // initialize best particle index
  best_particle_index_ = 0;

  // initialize other variables
  map_received_ = false;
  poly_received_ = true;
  targets_saved_ = false;

  // forbidden area initialization
  if(forbidden_poly_.polygon.points.empty())
  {
    geometry_msgs::Point32 p2_test;
    p2_test.x = 115;
    p2_test.y = 145;
    p2_test.z = 0;

    forbidden_poly_.polygon.points.push_back(p2_test);

    p2_test.x = 123;
    p2_test.y = 145;
    p2_test.z = 0;

    forbidden_poly_.polygon.points.push_back(p2_test);

    p2_test.x = 123;
    p2_test.y = 153;
    p2_test.z = 0;

    forbidden_poly_.polygon.points.push_back(p2_test);

    p2_test.x = 115;
    p2_test.y = 153;
    p2_test.z = 0;

    forbidden_poly_.polygon.points.push_back(p2_test);

    forbidden_poly_.header.frame_id = "/map";
  }

  if(poly_.polygon.points.empty())
  {
    geometry_msgs::Point32 p_test;
    p_test.x = 90;
    p_test.y = 120;
    p_test.z = 0;

    poly_.polygon.points.push_back(p_test);

    p_test.x = 210;
    p_test.y = 120;
    p_test.z = 0;

    poly_.polygon.points.push_back(p_test);

    p_test.x = 210;
    p_test.y = 200;
    p_test.z = 0;

    poly_.polygon.points.push_back(p_test);

    p_test.x = 90;
    p_test.y = 200;
    p_test.z = 0;

    poly_.polygon.points.push_back(p_test);

    poly_.header.frame_id = "/map";
  }

  area_of_interest_ = poly_;

}

// destructor
sensor_placement_node::~sensor_placement_node(){}

// function to get the ROS parameters from yaml-file
void sensor_placement_node::getParams()
{
  if(!pnh_.hasParam("number_of_sensors"))
  {
    ROS_WARN("No parameter number_of_sensors on parameter server. Using default [5]");
  }
  pnh_.param("number_of_sensors",sensor_num_,5);

  if(!pnh_.hasParam("max_sensor_range"))
  {
    ROS_WARN("No parameter max_sensor_range on parameter server. Using default [5.0 in m]");
  }
  pnh_.param("max_sensor_range",sensor_range_,5.0);
  
  double open_angle_1, open_angle_2;

  if(!pnh_.hasParam("open_angle_1"))
  {
    ROS_WARN("No parameter open_angle_1 on parameter server. Using default [1.5708 in rad]");
  }
  pnh_.param("open_angle_1",open_angle_1,1.5708);

  open_angles_.push_back(open_angle_1);

  if(!pnh_.hasParam("open_angle_2"))
  {
    ROS_WARN("No parameter open_angle_2 on parameter server. Using default [0.0 in rad]");
  }
  pnh_.param("open_angle_2",open_angle_2,0.0);

  open_angles_.push_back(open_angle_2);

  if(!pnh_.hasParam("max_linear_sensor_velocity"))
  {
    ROS_WARN("No parameter max_linear_sensor_velocity on parameter server. Using default [1.0]");
  }
  pnh_.param("max_linear_sensor_velocity",max_lin_vel_,1.0);

  if(!pnh_.hasParam("max_angular_sensor_velocity"))
  {
    ROS_WARN("No parameter max_angular_sensor_velocity on parameter server. Using default [0.5236]");
  }
  pnh_.param("max_angular_sensor_velocity",max_ang_vel_,0.5236);

  if(!pnh_.hasParam("number_of_particles"))
  {
    ROS_WARN("No parameter number_of_particles on parameter server. Using default [20]");
  }
  pnh_.param("number_of_particles",particle_num_,20);

  if(!pnh_.hasParam("max_num_iterations"))
  {
    ROS_WARN("No parameter max_num_iterations on parameter server. Using default [400]");
  }
  pnh_.param("max_num_iterations",iter_max_,400);

  if(!pnh_.hasParam("min_coverage_to_stop"))
  {
    ROS_WARN("No parameter min_coverage_to_stop on parameter server. Using default [0.95]");
  }
  pnh_.param("min_coverage_to_stop",min_cov_,0.95);

  // get PSO configuration parameters
  if(!pnh_.hasParam("c1"))
  {
    ROS_WARN("No parameter c1 on parameter server. Using default [0.729]");
  }
  pnh_.param("c1",PSO_param_1_,0.729);

  if(!pnh_.hasParam("c2"))
  {
    ROS_WARN("No parameter c2 on parameter server. Using default [1.49445]");
  }
  pnh_.param("c2",PSO_param_2_,1.49445);

  if(!pnh_.hasParam("c3"))
  {
    ROS_WARN("No parameter c3 on parameter server. Using default [1.49445]");
  }
  pnh_.param("c3",PSO_param_3_,1.49445);
}

bool sensor_placement_node::getTargets()
{
  // initialize result
  bool result = false;

  if(map_received_ == true)
  {
    // only if we received a map, we can get targets

    // initialize a dummy target_info object
    target_info dummy_target_info;
    targets_with_info_.assign(map_.info.width * map_.info.height, dummy_target_info);
    dummy_target_info.covered_by_sensor.assign(sensor_num_, false);

    if(poly_received_ == false)
    {
      // if no polygon was specified, we consider the non-occupied grid cells as targets
      for(unsigned int i = 0; i < map_.info.width; i++)
      {
        for(unsigned int j = 0; j < map_.info.height; j++)
        {
          // calculate world coordinates from map coordinates of given target
          geometry_msgs::Pose2D world_Coord;
          world_Coord.x = mapToWorldX(i, map_);
          world_Coord.y = mapToWorldY(j, map_);
          world_Coord.theta = 0;

          dummy_target_info.world_pos.x = mapToWorldX(i, map_);
          dummy_target_info.world_pos.y = mapToWorldY(j, map_);
          dummy_target_info.world_pos.z = 0;

          dummy_target_info.forbidden = false;    //all targets are allowed unless found in forbidden area
          dummy_target_info.occupied = true;
          dummy_target_info.covered = false;
          dummy_target_info.multiple_covered = false;
          dummy_target_info.potential_target = -1;

          if(map_.data.at( j * map_.info.width + i) == 0)
          {
            if(pointInPolygon(world_Coord, forbidden_poly_.polygon) == 1 || pointInPolygon(world_Coord, forbidden_poly_.polygon) == 0)
            {
              // the given position is on the forbidden area
              dummy_target_info.forbidden = true;
            }
            dummy_target_info.occupied = false;
            dummy_target_info.potential_target = 1;
            target_num_++;
          }
        }
      }
      result = true;
    }
    else
    {
      // if area of interest polygon was specified, we consider the non-occupied grid cells within the polygon as targets
      for(unsigned int i = 0; i < map_.info.width; i++)
      {
        for(unsigned int j = 0; j < map_.info.height; j++)
        {
          // calculate world coordinates from map coordinates of given target
          geometry_msgs::Pose2D world_Coord;
          world_Coord.x = mapToWorldX(i, map_);
          world_Coord.y = mapToWorldY(j, map_);
          world_Coord.theta = 0;

          dummy_target_info.world_pos.x = mapToWorldX(i, map_);
          dummy_target_info.world_pos.y = mapToWorldY(j, map_);
          dummy_target_info.world_pos.z = 0;

          dummy_target_info.forbidden = false;    //all targets are allowed unless found in forbidden area
          dummy_target_info.occupied = true;
          dummy_target_info.covered = false;
          dummy_target_info.multiple_covered = false;
          dummy_target_info.potential_target = -1;

          // the given position lies withhin the polygon
          if(pointInPolygon(world_Coord, area_of_interest_.polygon) == 1)
          {
            if(pointInPolygon(world_Coord, forbidden_poly_.polygon) == 1 || pointInPolygon(world_Coord, forbidden_poly_.polygon) == 0)
            {
              // the given position is on the forbidden area
              dummy_target_info.forbidden = true;
            }
            dummy_target_info.potential_target = 1;

            if(map_.data.at( j * map_.info.width + i) == 0)
            {
              target_num_++;
              dummy_target_info.occupied = false;
            }
          }
          // the given position lies on the perimeter
          if( pointInPolygon(world_Coord, area_of_interest_.polygon) == 0)
          {
            if(pointInPolygon(world_Coord, forbidden_poly_.polygon) == 1 || pointInPolygon(world_Coord, forbidden_poly_.polygon) == 0)
            {
              // the given position is on the forbidden area
              dummy_target_info.forbidden = true;
            }
            dummy_target_info.potential_target = 0;

            if(map_.data.at( j * map_.info.width + i) == 0)
            {
              dummy_target_info.occupied = false;
            }
          }
          // save the target information
          targets_with_info_.at(j * map_.info.width + i) = dummy_target_info;
        }
      }
      result = true;
    }
  }
  return result;
}

// function to initialize PSO-Algorithm
void sensor_placement_node::initializePSO()
{
  // initialize pointer to dummy sensor_model
  FOV_2D_model dummy_2D_model;
  dummy_2D_model.setMaxVelocity(max_lin_vel_, max_lin_vel_, max_lin_vel_, max_ang_vel_, max_ang_vel_, max_ang_vel_);

  // initialize dummy particle
  particle dummy_particle = particle(sensor_num_, target_num_, dummy_2D_model);

  dummy_particle.setMap(map_);
  dummy_particle.setAreaOfInterest(area_of_interest_);
  dummy_particle.setForbiddenArea(forbidden_poly_);
  dummy_particle.setOpenAngles(open_angles_);
  dummy_particle.setRange(sensor_range_);
  dummy_particle.setTargetsWithInfo(targets_with_info_, target_num_);

  ROS_INFO_STREAM("creating lookup tables for dummy particle..");
  dummy_particle.setLookupTable(sensor_range_);
  ROS_INFO_STREAM("lookup tables created.");

  // initialize particle swarm with given number of particles containing given number of sensors
  particle_swarm_.assign(particle_num_,dummy_particle);

  // initialize the global best solution
  global_best_ = dummy_particle;

  double actual_coverage = 0;

  // initialize sensors randomly on perimeter for each particle with random velocities
  if(poly_received_)
  {
    for(size_t i = 0; i < particle_swarm_.size(); i++)
    {     
      // initialize sensor poses randomly on perimeter
      //particle_swarm_.at(i).placeSensorsRandomlyOnPerimeter();
      particle_swarm_.at(i).initializeSensorsOnPerimeter();
      // initialize sensor velocities randomly
      particle_swarm_.at(i).initializeRandomSensorVelocities();
      // get calculated coverage
      actual_coverage = particle_swarm_.at(i).getActualCoverage();
      // check if the actual coverage is a new global best
      if(actual_coverage > best_cov_)
      {
        best_cov_ = actual_coverage;
        global_best_ = particle_swarm_.at(i);
      }
    }
    // after the initialization step we're looking for a new global best solution 
    getGlobalBest();

    // publish the actual global best visualization
    marker_array_pub_.publish(global_best_.getVisualizationMarkers());
  }
}

// function for the actual particle-swarm-optimization
void sensor_placement_node::PSOptimize()
{
  // PSO-iterator
  int iter = 0;
  std::vector<geometry_msgs::Pose> global_pose;

  // iteration step
  // continue calculation as long as there are iteration steps left and actual best coverage is
  // lower than mininmal coverage to stop
  while(iter < iter_max_ && best_cov_ < min_cov_)
  {
    global_pose = global_best_.getSolutionPositions();
    // update each particle in vector
    for(size_t i=0; i < particle_swarm_.size(); i++)
    {
      // reset targets_with_info_ for each particle before the update step
      particle_swarm_.at(i).setTargetsWithInfo(targets_with_info_, target_num_);
      // now we're ready to update the particle
      particle_swarm_.at(i).updateParticle(global_pose, PSO_param_1_, PSO_param_2_, PSO_param_3_);
    }
    // after the update step we're looking for a new global best solution
    getGlobalBest();

    // publish the actual global best visualization
    marker_array_pub_.publish(global_best_.getVisualizationMarkers());

    ROS_INFO_STREAM("iteration: " << iter << " with coverage: " << best_cov_);

    // increment PSO-iterator
    iter++;
  }

}

void sensor_placement_node::getGlobalBest()
{
  // a new global best solution is accepted if
  // (1) the coverage is higher than the old best coverage or
  // (2) the coverage is equal to the old best coverage but there are less targets covered by multiple sensors
  for(size_t i=0; i < particle_swarm_.size(); i++)
  {
    if(particle_swarm_.at(i).getActualCoverage() > best_cov_)
    {
      best_cov_ = particle_swarm_.at(i).getActualCoverage();
      global_best_ = particle_swarm_.at(i);
      global_best_multiple_coverage_ = particle_swarm_.at(i).getMultipleCoverageIndex();
      best_particle_index_ = i;
    }
    else
    {
      if( (particle_swarm_.at(i).getActualCoverage() == best_cov_) && (particle_swarm_.at(i).getMultipleCoverageIndex() < global_best_multiple_coverage_ ))
      {
        best_cov_ = particle_swarm_.at(i).getActualCoverage();
        global_best_ = particle_swarm_.at(i);
        global_best_multiple_coverage_ = particle_swarm_.at(i).getMultipleCoverageIndex();
        best_particle_index_ = i;
      }
    }
  }
}

// callback function for the start PSO service
bool sensor_placement_node::startPSOCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{

  // call static_map-service from map_server to get the actual map  
  sc_get_map_.waitForExistence();

  nav_msgs::GetMap srv_map;

  if(sc_get_map_.call(srv_map))
  {
    ROS_INFO("Map service called successfully");

    if(poly_received_)
    {
      // get bounding box of area of interest
      geometry_msgs::Polygon bound_box = getBoundingBox2D(area_of_interest_.polygon, srv_map.response.map);
      // cropMap to boundingBox
      cropMap(bound_box, srv_map.response.map, map_);
      // publish cropped map
      map_.header.stamp = ros::Time::now();
      map_pub_.publish(map_);
      map_meta_pub_.publish(map_.info);
      map_received_ = true;
    }
    else
    {
      map_ = srv_map.response.map;
      map_pub_.publish(map_);
      map_meta_pub_.publish(map_.info);
      map_received_ = true;
    }
  }
  else
  {
    ROS_INFO("Failed to call map service");
  }

  if(map_received_)
  ROS_INFO("Received a map");

  ROS_INFO("getting targets from specified map and area of interest!");

  targets_saved_ = getTargets();
  if(targets_saved_)
  {
    ROS_INFO_STREAM("Saved " << target_num_ << " targets in std-vector");
    ROS_INFO_STREAM("Saved " << targets_with_info_.size() << " targets with info in std-vector");
  }


  ROS_INFO("Initializing particle swarm");
  initializePSO();

  // publish global best visualization
  marker_array_pub_.publish(global_best_.getVisualizationMarkers());

  ROS_INFO("Particle swarm Optimization step");
  PSOptimize();

  // get the PSO result as nav_msgs::Path in UTM coordinates and publish it
  PSO_result_ = particle_swarm_.at(best_particle_index_).particle::getSolutionPositionsAsPath();

  nav_path_pub_.publish(PSO_result_);

  ROS_INFO_STREAM("Print the best solution as Path: " << PSO_result_);

  // publish global best visualization
  marker_array_pub_.publish(global_best_.getVisualizationMarkers());

  ROS_INFO("Clean up everything");
  particle_swarm_.clear();
  targets_with_info_.clear();
  target_num_ = 0;
  best_cov_ = 0;
  best_particle_index_ = 0;

  ROS_INFO("PSO terminated successfully");

  return true;

}

// callback function for the test service
bool sensor_placement_node::testServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  // call static_map-service from map_server to get the actual map  
  sc_get_map_.waitForExistence();

  nav_msgs::GetMap srv_map;

  if(sc_get_map_.call(srv_map))
  {
    ROS_INFO("Map service called successfully");

    if(poly_received_)
    {
      // get bounding box of area of interest
      geometry_msgs::Polygon bound_box = getBoundingBox2D(area_of_interest_.polygon, srv_map.response.map);
      // cropMap to boundingBox
      cropMap(bound_box, srv_map.response.map, map_);
      // publish cropped map
      map_.header.stamp = ros::Time::now();
      map_pub_.publish(map_);
      map_meta_pub_.publish(map_.info);
      map_received_ = true;
    }
    else
    {
      map_ = srv_map.response.map;
      map_pub_.publish(map_);
      map_meta_pub_.publish(map_.info);
      map_received_ = true;
    }
  }
  else
  {
    ROS_INFO("Failed to call map service");
  }

  if(map_received_)
  ROS_INFO("Received a map");

  ROS_INFO("getting targets from specified map and area of interest!");

  targets_saved_ = getTargets();
  if(targets_saved_)
  {
    ROS_INFO_STREAM("Saved " << target_num_ << " targets in std-vector");
    ROS_INFO_STREAM("Saved " << targets_with_info_.size() << " targets with info in std-vector");
  }
  // initialize pointer to dummy sensor_model
  FOV_2D_model dummy_2D_model;
  dummy_2D_model.setMaxVelocity(max_lin_vel_, max_lin_vel_, max_lin_vel_, max_ang_vel_, max_ang_vel_, max_ang_vel_);

  particle_num_ = 1;
  sensor_num_ = 1;
  open_angles_.at(0) = 1.5 * PI;

  // initialize dummy particle
  particle dummy_particle = particle(sensor_num_, target_num_, dummy_2D_model);
  // initialize particle swarm with given number of particles containing given number of sensors

  dummy_particle.setMap(map_);
  dummy_particle.setAreaOfInterest(area_of_interest_);
  dummy_particle..setForbiddenArea(forbidden_poly_);
  dummy_particle.setOpenAngles(open_angles_);
  dummy_particle.setRange(5);
  dummy_particle.setTargetsWithInfo(targets_with_info_, target_num_);

  ROS_INFO_STREAM("creating lookup tables for dummy particle..");
  dummy_particle.setLookupTable(5);
  ROS_INFO_STREAM("lookup tables created.");



  particle_swarm_.assign(particle_num_,dummy_particle);

  // initialze the global best solution
  global_best_ = dummy_particle;

  double actual_coverage = 0;

  // initialize sensors randomly on perimeter for each particle with random velocities
  if(poly_received_)
  {
    for(size_t i = 0; i < particle_swarm_.size(); i++)
    {
      geometry_msgs::Pose test_pos = geometry_msgs::Pose();
      test_pos.position.x = area_of_interest_.polygon.points.at(0).x+5;
      test_pos.position.y = area_of_interest_.polygon.points.at(0).y+5;
      test_pos.orientation = tf::createQuaternionMsgFromYaw(PI/4);

      particle_swarm_.at(i).placeSensorsAtPos(test_pos);

      global_best_ = particle_swarm_.at(i);

      actual_coverage = global_best_.getActualCoverage();

      ROS_INFO_STREAM("coverage: " << actual_coverage);

      ROS_INFO_STREAM("orientation: " << test_pos.orientation);
      ROS_INFO_STREAM("orientation-map: " << tf::getYaw(map_.info.origin.orientation));
    }
  }
  // publish global best visualization
  marker_array_pub_.publish(global_best_.getVisualizationMarkers());

  return true;
}

void sensor_placement_node::publishPolygon()
{

  poly_pub_.publish(poly_);
  forbidden_poly_pub_.publish(forbidden_poly_);

}

int main(int argc, char **argv)
{
  // initialize ros and specify node name
  ros::init(argc, argv, "sensor_placement_node");

  // create Node Class
  sensor_placement_node my_placement_node;

  // initialize random seed for all underlying classes
  srand(time(NULL));

  // initialize loop rate
  ros::Rate loop_rate(10);

  while(my_placement_node.nh_.ok())
  {
     //can add this later to the funtion below
    my_placement_node.publishPolygon();
    ros::spinOnce();

    loop_rate.sleep();
  }

}