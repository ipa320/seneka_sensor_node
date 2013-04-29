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

#include <sensor_placement_node.h>

// constructor
sensor_placement_node::sensor_placement_node()
{
  // create node handles
  nh_ = ros::NodeHandle();
  pnh_ = ros::NodeHandle("~");

  // ros subscribers

  // ros publishers
  poly_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("out_poly",1);
  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("out_marker_array",1);

  // ros service servers
  ss_start_PSO_ = nh_.advertiseService("StartPSO", &sensor_placement_node::startPSOCallback, this);

  // ros service clients
  sc_get_map_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");

  // get parameters from parameter server if possible
  if(!pnh_.hasParam("number_of_sensors"))
  {
    ROS_WARN("No parameter number_of_sensors on parameter server. Using default [5]");
  }
  pnh_.param("number_of_sensors",sensor_num_,5);

  if(!pnh_.hasParam("max_sensor_range"))
  {
    ROS_WARN("No parameter max_sensor_range on parameter server. Using default [5.0 in m]");
  }
  pnh_.param("max_sensor_range_",sensor_range_,5.0);

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

  // initialize best coverage
  best_cov_ = 0;

  // initialize number of targets
  target_num_ = 0;

  // initialize other variables
  map_received_ = false;
  poly_received_ = true;
  targets_saved_ = false;

  if(poly_.polygon.points.empty())
  {
    geometry_msgs::Point32 p_test;
    p_test.x = -10;
    p_test.y = 0;
    p_test.z = 0;

    poly_.polygon.points.push_back(p_test);

    p_test.x = 10;
    p_test.y = 0;
    p_test.z = 0;

    poly_.polygon.points.push_back(p_test);

    p_test.x = 10;
    p_test.y = 20;
    p_test.z = 0;

    poly_.polygon.points.push_back(p_test);

    p_test.x = -10;
    p_test.y = 20;
    p_test.z = 0;

    poly_.polygon.points.push_back(p_test);

    poly_.header.frame_id = "/map";
  }

  area_of_interest_ = poly_;

}

// destructor
sensor_placement_node::~sensor_placement_node(){}

bool sensor_placement_node::getTargets()
{
  // initialize result
  bool result = false;

  if(map_received_ == true)
  { 
    // only if we received a map, we can get targets
    if(poly_received_ == false)
    {
      // if no polygon was specified, we consider the non-occupied grid cells as targets
      for(unsigned int i = 0; i < map_.info.width; i++)
      {
        for(unsigned int j = 0; j < map_.info.height; j++)
        {
          if(map_.data[ j * map_.info.width + i] == 0)
          {
            targets_x_.push_back(i);
            targets_y_.push_back(j);
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
          world_Coord.x = mapToWorldX(i);
          world_Coord.y = mapToWorldX(j);
          world_Coord.theta = 0;

          if(pointInPolygon(world_Coord, area_of_interest_.polygon) == 1 || 
             pointInPolygon(world_Coord, area_of_interest_.polygon) == 0 )
          {
            if(map_.data[ j * map_.info.width + i] == 0)
            {
              targets_x_.push_back(i);
              targets_y_.push_back(j);
              target_num_++;

              if( pointInPolygon(world_Coord, area_of_interest_.polygon) == 0 )
              {
                perimeter_x_.push_back(i);
                perimeter_y_.push_back(j);
              }
            }
          }
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
  seneka_sensor_model::FOV_2D_model dummy_2D_model;

  // initialize dummy particle
  seneka_particle::particle dummy_particle = seneka_particle::particle(sensor_num_, target_num_, dummy_2D_model);
  // initialize particle swarm with given number of particles containing given number of sensors
  particle_swarm_.assign(particle_num_,dummy_particle);

  // initialze the global best solution
  global_best_ = dummy_particle;

  double actual_coverage = 0;

  // initialize sensors randomly on perimeter for each particle with random velocities
  if(poly_received_)
  {
    for(size_t i = 0; i < particle_swarm_.size(); i++)
    {
      // set map, area of interest, targets and open angles for each particle
      particle_swarm_[i].setMap(map_);
      particle_swarm_[i].setAreaOfInterest(area_of_interest_);
      particle_swarm_[i].setOpenAngles(open_angles_);
      particle_swarm_[i].setRange(sensor_range_);
      particle_swarm_[i].setTargets(targets_x_, targets_y_);
      // initiliaze sensor poses randomly on perimeter
      particle_swarm_[i].placeSensorsRandomlyOnPerimeter();
      // initialze sensor velocities randomly
      particle_swarm_[i].initializeRandomSensorVelocities();
      // calculate the initial coverage matrix
      particle_swarm_[i].calcCoverageMatrix();
      // calculate the initial coverage
      particle_swarm_[i].calcCoverage();
      // get calculated coverage
      actual_coverage = particle_swarm_[i].getActualCoverage();
      ROS_INFO_STREAM("actual coverage: " << actual_coverage << " from particle: " << i);
      // check if the actual coverage is a new global best
      if(actual_coverage > best_cov_)
      {
        best_cov_ = actual_coverage;
        ROS_INFO_STREAM("new best coverage: "<< best_cov_<<" gained from particle " << i);
        global_best_ = particle_swarm_[i];
      }
    }
  }
}

// function for the actual particle-swarm-optimization
void sensor_placement_node::PSOptimize()
{
  // PSO-iterator
  int iter = 0;
 
  // iteration step
  // continue calculation as long as there are iteration steps left and actual best coverage is 
  // lower than mininmal coverage to stop
  while(iter < iter_max_ && best_cov_ < min_cov_)
  {
    // update each particle in vector
    for(size_t i=0; i < particle_swarm_.size(); i++)
    {
      particle_swarm_[i].updateParticle(global_best_.getSolutionPositions(), PSO_param_1_, PSO_param_2_, PSO_param_3_);
      
    }

    // after update step look for new global best solution 
    getGlobalBest();

    // increment PSO-iterator
    iter++;
  }

}

void sensor_placement_node::getGlobalBest()
{
  for(size_t i=0; i < particle_swarm_.size(); i++)
  {
    if(particle_swarm_[i].getActualCoverage() > best_cov_)
    {
      best_cov_ = particle_swarm_[i].getActualCoverage();
      global_best_ = particle_swarm_[i];
    }
  }
}

// functions to calculate between map (grid) and world coordinates
double sensor_placement_node::mapToWorldX(int map_x)
{
  if(map_received_)
    return map_.info.origin.position.x + (map_x * map_.info.resolution);
  else
    return 0;
}

double sensor_placement_node::mapToWorldY(int map_y)
{
  if(map_received_)
    return map_.info.origin.position.y + (map_y * map_.info.resolution);
  else
    return 0;
}

int sensor_placement_node::worldToMapX(double world_x)
{
  if(map_received_)
    return (world_x - map_.info.origin.position.x) / map_.info.resolution;
  else 
    return 0;
}

int sensor_placement_node::worldToMapY(double world_y)
{
  if(map_received_)
    return (world_y - map_.info.origin.position.y) / map_.info.resolution;
  else 
    return 0;
}

// function to gerenate random numbers in given interval
double sensor_placement_node::randomNumber(double low, double high)
{
  return ((double) rand()/ RAND_MAX) * (high - low) + low;
}

// function to check if a given point is inside (return 1), outside (return -1) 
// or on an edge (return 0) of a given polygon
int sensor_placement_node::pointInPolygon(geometry_msgs::Pose2D point, geometry_msgs::Polygon polygon)
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
bool sensor_placement_node::pointOn1DSegementPose(geometry_msgs::Pose2D start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID)
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
      if( (border_1.x*(1-t) + t*border_2.x == start.x) && ((border_1.y*(1-t) + t*border_2.y == start.y)) )
      {
        // start lies on the segment
        result = true;
      }
    }
  }

  return result;
}

bool sensor_placement_node::pointOn1DSegementPoint(geometry_msgs::Point32 start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID)
{
  geometry_msgs::Pose2D help_pose;
  help_pose.x = start.x;
  help_pose.y = start.y;
  help_pose.theta = 0;

  return pointOn1DSegementPose(help_pose, border_1, border_2, segID);
}

// helper function to check if the beam of line from start intersects the given plygon edge
// segID = 0 (beam), segID = 1 (line)
bool sensor_placement_node::edgeIntersectsBeamOrLine(geometry_msgs::Pose2D start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID)
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

// callback function for the start PSO service
bool sensor_placement_node::startPSOCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{

  // ******************* test stuff *******************
  geometry_msgs::Pose2D test_pose;

  test_pose.x = -40;
  test_pose.y = 1;
  test_pose.theta = 0;

  //cout << "Test point in Polygon: " << pointInPolygon(test_pose, poly_.polygon) << endl;

  // ******************* test stuff end *******************

  // call static_map-service from map_server to get the actual map  
  sc_get_map_.waitForExistence();

  nav_msgs::GetMap srv_map;

  if(sc_get_map_.call(srv_map))
  {
    ROS_INFO("Map service called successfully");
    const nav_msgs::OccupancyGrid& new_map (srv_map.response.map);
    map_ = new_map;
    map_received_ = true;
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
    ROS_INFO_STREAM("Saved " << target_num_ << " targets in std-vector");

  ROS_INFO("Initializing particle swarm");
  initializePSO();

  ROS_INFO("Particle swarm Optimization step");
  //PSOptimize();

  // publish global best visualization
  marker_array_pub_.publish(global_best_.getVisualizationMarkers());

  ROS_INFO("Clean up everything");
  particle_swarm_.erase(particle_swarm_.begin(), particle_swarm_.end());
  targets_x_.erase(targets_x_.begin(),targets_x_.end());
  targets_y_.erase(targets_y_.begin(),targets_y_.end());
  target_num_ = 0;
  best_cov_ = 0;

  ROS_INFO("PSO terminated successfully");

  return true;

}

// callback function for the map topic
/*void sensor_placement_node::mapCB(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
  map_ = *map;

  map_received_ = true;
}*/

void sensor_placement_node::publishPolygon()
{  

  poly_pub_.publish(poly_);

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
    my_placement_node.publishPolygon();
    ros::spinOnce();

    loop_rate.sleep();
  }

}
