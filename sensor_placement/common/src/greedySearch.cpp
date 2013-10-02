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

#include <greedySearch.h>

// standard constructor
greedySearch::greedySearch()
{
  // initialize number of sensors
  sensor_num_ = 0;

  // initialize number of targets
  target_num_ = 0;

  // intialize coverage
  coverage_ = 0;

  // intialize covered targets number
  covered_targets_num_ = 0;

  // initialize information related to maximum coverage by a sensor
  max_sensor_cov_ = 0;
  max_sensor_cov_point_id_ = 0;
}


// constructor with arguments
greedySearch::greedySearch(int num_of_sensors, int num_of_targets, FOV_2D_model sensor_model)
{
  // initialize number of sensors
  sensor_num_ = num_of_sensors;

  // initialze number of targets
  target_num_ = num_of_targets;

  // intialize coverage
  coverage_ = 0;

  // intialize covered targets number
  covered_targets_num_ = 0;

  // initialize information related to maximum coverage by a sensor
  max_sensor_cov_ = 0;
  max_sensor_cov_point_id_ = 0;

  // initialize sensor vector with as many entries as specified by sensors_num_
  sensors_.assign(sensor_num_, sensor_model);

}

greedySearch::~greedySearch(){}


// function for finding maximum coverage position (using Greedy Search Algorithm) and placing sensor at that position
void greedySearch::greedyPlacement(size_t sensor_index)
{
  unsigned int angle_resolution;
  unsigned int cell_search_resolution;
  double num_of_steps;
  bool update_covered_info;
  int placement_point_id;
  geometry_msgs::Pose new_pose;
  geometry_msgs::Pose placement_pose;

  //initializations
  angle_resolution = getAngleResolution();
  cell_search_resolution = getCellSearchResolution();
  num_of_steps = ceil(360/angle_resolution);

  //while searching, restrict updating of 'covered' info in updateGSpointsRaytracing
  update_covered_info=false;
  //reset previous max coverage information before searching for new position
  resetMaxSensorCovInfo();
  //reset max targets covered information
  resetGSpool();

  //place the current sensor on all points in GS pool one by one and calculate coverge
  for (size_t point_id=0; point_id<GS_pool_.size(); point_id=point_id+cell_search_resolution)
  {
    //first calculate world position of current point id and place sensor at that position
    new_pose.position.x = mapToWorldX(GS_pool_[point_id].p.x, *pMap_);
    new_pose.position.y = mapToWorldY(GS_pool_[point_id].p.y, *pMap_);
    new_pose.position.z = 0;

    //check all orientations
    for (int alpha=0; alpha<angle_resolution*num_of_steps; alpha=alpha+angle_resolution)
    {
      //look around at all angles and save only the max coverage angle
      new_pose.orientation = tf::createQuaternionMsgFromYaw(alpha*(PI/180));

      sensors_.at(sensor_index).setSensorPose(new_pose);

      //now update targets/points covered and save the maximum coverage information
      updateGSpointsRaytracing(sensor_index, point_id, update_covered_info);
    }
  }
  //Get maximum coverage pose
  placement_pose = getMaxSensorCovPOSE();
  //Get maximum coverage point ID
  placement_point_id = getMaxSensorCovPointID();
  //place the sensor at max coverage point
  sensors_.at(sensor_index).setSensorPose(placement_pose);

  //now update the 'covered' info
  update_covered_info = true;
  updateGSpointsRaytracing(sensor_index, placement_point_id, update_covered_info);

}



// function for finding maximum coverage position (using Greedy Search Algorithm) and placing sensor at that position
void greedySearch::newGreedyPlacement(size_t sensor_index)
{
  unsigned int angle_resolution;
  unsigned int cell_search_resolution;
  unsigned int index;
  unsigned int new_sum, max_sum, max_cov_ind;
  unsigned int num_of_sectors;
  double num_of_steps;
  double max_cov_orientation;
  int placement_point_id;
  int coverage;
  int angle_inc;
  bool update_covered_info;
  geometry_msgs::Pose new_pose;
  geometry_msgs::Pose placement_pose;
  std::vector<double> orig_ang_r;
  std::vector<double> gs_sector_r(2,0);
  std::vector<double> new_open_angles(2,0);

  //initilizations
  gs_sector_r.at(0) = 0.5236; //  make this a parameter similar to openangles (a vector) -b-
  gs_sector_r.at(1) = 0.0;

  angle_inc = (int) (gs_sector_r.at(0)*(180/PI));

  orig_ang_r = sensors_.at(sensor_index).getOpenAngles();
  cell_search_resolution = getCellSearchResolution();
  num_of_sectors = (orig_ang_r.at(0)-orig_ang_r.at(1))/(gs_sector_r.at(0)-gs_sector_r.at(1));
  if ((num_of_sectors>720) || (num_of_sectors==0))
    ROS_WARN("num_of_sectors is too high or invalid");

  //change the sensor open angles for search
  setOpenAngles(gs_sector_r);
  //while searching, restrict updating of 'covered' info in updateGSpointsRaytracing
  update_covered_info=false;
  //reset previous max coverage information before searching for new position
  resetMaxSensorCovInfo();
  //reset max targets covered information
  resetGSpool();

  //place the current sensor on all points in GS pool one by one and calculate coverge
  for (size_t point_id=0; point_id<GS_pool_.size(); point_id=point_id+cell_search_resolution)
  {
    //clear data
    coverage_vec_.clear();

    //calculate world position of current point id and place sensor at that position
    new_pose.position.x = mapToWorldX(GS_pool_[point_id].p.x, *pMap_);
    new_pose.position.y = mapToWorldY(GS_pool_[point_id].p.y, *pMap_);
    new_pose.position.z = 0;

    for (int alpha=0; alpha<360; alpha=alpha+angle_inc)
    {
      //look around at all N non-overlapping sectors, N = num_of_sectors
      new_pose.orientation = tf::createQuaternionMsgFromYaw(alpha*(PI/180));
      sensors_.at(sensor_index).setSensorPose(new_pose);
      coverage = getCoverageRaytracing(sensor_index);         //get coverage at new_pose
      coverage_vec_.push_back(coverage);                      //save in coverage in coverage_vec_
    }

    //coverage array at given point is computed for all non-overlapping sectors, now find N consecutive sectors that give maximum coverage. (N = num_of_sectors)
    max_cov_ind=0;
    max_sum=0;

    for (size_t cov_vec_ind=0; cov_vec_ind<coverage_vec_.size(); cov_vec_ind++)
    {
      //compute consecutive sum of N elements from coverage_vec_[start] to coverage_vec_[end]
      new_sum=0;

      for (size_t k = 0; k<num_of_sectors; k++)
      {
        new_sum = new_sum + coverage_vec_.at( (cov_vec_ind+k) % (coverage_vec_.size()) );
      }

      //save the cov_vec_ind of highest sum in max_cov_ind
      if (new_sum>max_sum)
      {
        max_sum = new_sum;
        max_cov_ind = cov_vec_ind;
      }
    }


    //evaluate orientation for maximum coverage at current point and set new_pose
    max_cov_orientation = max_cov_ind*gs_sector_r.at(0);    // -b- check if angle1 or angle2 should be used
    new_pose.orientation = tf::createQuaternionMsgFromYaw(max_cov_orientation);
    sensors_.at(sensor_index).setSensorPose(new_pose);

    //now update max_targets_covered. And if this pose is better than than prev, then also update max_coverage_pose
    updateGSpointsRaytracing(sensor_index, point_id, update_covered_info);
  }

  //search complete, so reset the sensor open angles back to original values
  new_open_angles.at(0) = orig_ang_r.at(0);
  new_open_angles.at(1) = orig_ang_r.at(1);
  setOpenAngles(new_open_angles);

  //Get maximum coverage pose
  placement_pose = getMaxSensorCovPOSE();
  //Get maximum coverage point ID
  placement_point_id = getMaxSensorCovPointID();
  //place the sensor at max coverage point
  sensors_.at(sensor_index).setSensorPose(placement_pose);

  //now update the 'covered' info
  update_covered_info = true;
  updateGSpointsRaytracing(sensor_index, placement_point_id, update_covered_info);

}


//function to get the coverage done by the sensor
int greedySearch::getCoverageRaytracing(size_t sensor_index)
{
  //clear vector of ray end points
  sensors_.at(sensor_index).clearRayEndPoints();

  unsigned int max_number_of_rays = sensors_.at(sensor_index).getLookupTable()->size();
  geometry_msgs::Pose sensor_pose = sensors_.at(sensor_index).getSensorPose();

  std::vector<double> open_ang = sensors_.at(sensor_index).getOpenAngles();
  double orientation = tf::getYaw(sensor_pose.orientation);

  //get angles of sensor and keep them between 0 and 2*PI
  double angle1 = orientation - (open_ang.front() / 2.0);
  if(angle1 >= 2.0*PI)
    angle1 -= 2.0*PI;
  else if(angle1 < 0)
    angle1 += 2.0*PI;

  double angle2 = orientation + (open_ang.front() / 2.0);
  if(angle2 >= 2.0*PI)
    angle2 -= 2.0*PI;
  else if(angle2 < 0)
    angle2 += 2.0*PI;

  unsigned int ray_start = sensors_.at(sensor_index).rayOfAngle(angle1);
  unsigned int ray_end = sensors_.at(sensor_index).rayOfAngle(angle2);

  unsigned int number_of_rays_to_check;

  //are the rays in between the beginning and end of the lookup table?
  if(ray_end >= ray_start)
    number_of_rays_to_check = ray_end - ray_start + 1;
  else
    number_of_rays_to_check = max_number_of_rays - ray_start + ray_end + 1;

  unsigned int rays_checked = 0;
  unsigned int ray = ray_start;

  // initialize coverage by old and new orientaion of the sensor on the current position
  int coverage_by_new_orientation = 0;

  //go through all rays
  while(rays_checked < number_of_rays_to_check)
  {
    geometry_msgs::Point ray_end_point;
    ray_end_point.x = 0;
    ray_end_point.y = 0;

    int x, y;
    int cell;
    int lookup_table_x, lookup_table_y;

    //go through ray
    for(cell=0; cell < sensors_.at(sensor_index).getLookupTable()->at(ray).size(); cell++)
    {
      lookup_table_x = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell).x;
      lookup_table_y = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell).y;

      //absolute x and y map coordinates of the current cell
      x = worldToMapX(sensor_pose.position.x, *pMap_) + lookup_table_x;
      y = worldToMapY(sensor_pose.position.y, *pMap_) + lookup_table_y;

      int cell_in_vector_coordinates = y * pMap_->info.width + x;


      //cell coordinates are valid (not outside of the area of interest)
      if(((y >= 0) && (x >= 0) && (y < pMap_->info.height) && (x < pMap_->info.width)) && (cell_in_vector_coordinates < pPoint_info_vec_->size()))
      {
        //cell not on the perimeter
        if(pPoint_info_vec_->at(cell_in_vector_coordinates).potential_target != 0)
        {
          //cell a potential target and not occupied
          if((pPoint_info_vec_->at(cell_in_vector_coordinates).potential_target == 1) &&
             (pPoint_info_vec_->at(cell_in_vector_coordinates).occupied == false))
          {
            //cell not already covered
            if(pPoint_info_vec_->at(cell_in_vector_coordinates).covered == false)
            {
              coverage_by_new_orientation++;
            }
          }
          //cell not a potential target or occupied -> skip rest of this ray
          else
          {
            break;
          }
        }
        else
        {
          //cell on perimeter and not occupied -> continue with the next cell on the ray (no coverage)
          if(pPoint_info_vec_->at(cell_in_vector_coordinates).occupied == false)
          {
            //continue with next cell without (no coverage)
            continue;
          }
          else
          //cell on perimeter and occupied -> skip rest of this ray
          {
            break;
          }
        }
      }
      else
      //cell coordinates not valid (outside the area of interest) -> skip rest of this ray
      {
        break;
      }
    }


    //skipped some part of the ray -> get coordinates of the last non-occupied cell
    if((cell != sensors_.at(sensor_index).getLookupTable()->at(ray).size()-1) && (cell != 0))
    {
      lookup_table_x = sensors_.at(sensor_index).getLookupTable()->at(ray).at(std::max(0,cell-1)).x;
      lookup_table_y = sensors_.at(sensor_index).getLookupTable()->at(ray).at(std::max(0,cell-1)).y;
    }

    //absolute x and y map coordinates of the last non-occupied cell
    x = worldToMapX(sensor_pose.position.x, *pMap_) + lookup_table_x;
    y = worldToMapY(sensor_pose.position.y, *pMap_) + lookup_table_y;

    //update endpoint
    if(lookup_table_x <= 0)
      //point is left of sensor
    {
      ray_end_point.x = mapToWorldX(x, *pMap_) - sensor_pose.position.x;
    }
    else
      //cell is right of sensor
    {
      ray_end_point.x = mapToWorldX(x, *pMap_) - sensor_pose.position.x + pMap_->info.resolution; //add one cell for visualization
    }

    if(lookup_table_y <= 0)
      //cell is below sensor
    {
      ray_end_point.y = mapToWorldY(y, *pMap_) - sensor_pose.position.y;
    }
    else
      //cell is over sensor
    {
      ray_end_point.y = mapToWorldY(y, *pMap_) - sensor_pose.position.y + pMap_->info.resolution; //add one cell for visualization
    }

    //add endpoint to the vector of endpoints
    sensors_.at(sensor_index).addRayEndPoint(ray_end_point);

    //increase counter
    rays_checked++;

    //reached end of circle -> set ray to 0
    if(ray == (max_number_of_rays -1))
    {
      ray = 0;
    }
    else
    {
      ray++;
    }
  }
  //all rays checked

  return coverage_by_new_orientation;
}


//function to update the GS_point_info with raytracing
void greedySearch::updateGSpointsRaytracing(size_t sensor_index, int point_id, bool update_covered_info)
{
  //clear vector of ray end points
  sensors_.at(sensor_index).clearRayEndPoints();

  unsigned int max_number_of_rays = sensors_.at(sensor_index).getLookupTable()->size();
  geometry_msgs::Pose sensor_pose = sensors_.at(sensor_index).getSensorPose();

  std::vector<double> open_ang = sensors_.at(sensor_index).getOpenAngles();
  double orientation = tf::getYaw(sensor_pose.orientation);

  //get angles of sensor and keep them between 0 and 2*PI
  double angle1 = orientation - (open_ang.front() / 2.0);
  if(angle1 >= 2.0*PI)
    angle1 -= 2.0*PI;
  else if(angle1 < 0)
    angle1 += 2.0*PI;

  double angle2 = orientation + (open_ang.front() / 2.0);
  if(angle2 >= 2.0*PI)
    angle2 -= 2.0*PI;
  else if(angle2 < 0)
    angle2 += 2.0*PI;

  unsigned int ray_start = sensors_.at(sensor_index).rayOfAngle(angle1);
  unsigned int ray_end = sensors_.at(sensor_index).rayOfAngle(angle2);

  unsigned int number_of_rays_to_check;

  //are the rays in between the beginning and end of the lookup table?
  if(ray_end >= ray_start)
    number_of_rays_to_check = ray_end - ray_start + 1;
  else
    number_of_rays_to_check = max_number_of_rays - ray_start + ray_end + 1;

  unsigned int rays_checked = 0;
  unsigned int ray = ray_start;

  // initialize coverage by old and new orientaion of the sensor on the current position
  int coverage_by_old_orientation = GS_pool_[point_id].max_targets_covered;
  int coverage_by_new_orientation = 0;

  //go through all rays
  while(rays_checked < number_of_rays_to_check)
  {
    geometry_msgs::Point ray_end_point;
    ray_end_point.x = 0;
    ray_end_point.y = 0;

    int x, y;
    int cell;
    int lookup_table_x, lookup_table_y;

    //go through ray
    for(cell=0; cell < sensors_.at(sensor_index).getLookupTable()->at(ray).size(); cell++)
    {
      lookup_table_x = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell).x;
      lookup_table_y = sensors_.at(sensor_index).getLookupTable()->at(ray).at(cell).y;

      //absolute x and y map coordinates of the current cell
      x = worldToMapX(sensor_pose.position.x, *pMap_) + lookup_table_x;
      y = worldToMapY(sensor_pose.position.y, *pMap_) + lookup_table_y;

      int cell_in_vector_coordinates = y * pMap_->info.width + x;


      //cell coordinates are valid (not outside of the area of interest)
      if(((y >= 0) && (x >= 0) && (y < pMap_->info.height) && (x < pMap_->info.width)) && (cell_in_vector_coordinates < pPoint_info_vec_->size()))
      {
        //cell not on the perimeter
        if(pPoint_info_vec_->at(cell_in_vector_coordinates).potential_target != 0)
        {
          //cell a potential target and not occupied
          if((pPoint_info_vec_->at(cell_in_vector_coordinates).potential_target == 1) &&
             (pPoint_info_vec_->at(cell_in_vector_coordinates).occupied == false))
          {
            //cell not already covered
            if(pPoint_info_vec_->at(cell_in_vector_coordinates).covered == false)
            {
              coverage_by_new_orientation++;
              //mark this position as covered only if the current pose is the final selected position for sensor placement
              if(update_covered_info == true)
              {
                pPoint_info_vec_->at(cell_in_vector_coordinates).covered = true;
                covered_targets_num_++;
              }
            }
          }
          //cell not a potential target or occupied -> skip rest of this ray
          else
          {
            break;
          }
        }
        else
        {
          //cell on perimeter and not occupied -> continue with the next cell on the ray (no coverage)
          if(pPoint_info_vec_->at(cell_in_vector_coordinates).occupied == false)
          {
            //continue with next cell without (no coverage)
            continue;
          }
          else
          //cell on perimeter and occupied -> skip rest of this ray
          {
            break;
          }
        }
      }
      else
      //cell coordinates not valid (outside the area of interest) -> skip rest of this ray
      {
        break;
      }
    }


    //skipped some part of the ray -> get coordinates of the last non-occupied cell
    if((cell != sensors_.at(sensor_index).getLookupTable()->at(ray).size()-1) && (cell != 0))
    {
      lookup_table_x = sensors_.at(sensor_index).getLookupTable()->at(ray).at(std::max(0,cell-1)).x;
      lookup_table_y = sensors_.at(sensor_index).getLookupTable()->at(ray).at(std::max(0,cell-1)).y;
    }

    //absolute x and y map coordinates of the last non-occupied cell
    x = worldToMapX(sensor_pose.position.x, *pMap_) + lookup_table_x;
    y = worldToMapY(sensor_pose.position.y, *pMap_) + lookup_table_y;

    //update endpoint
    if(lookup_table_x <= 0)
      //point is left of sensor
    {
      ray_end_point.x = mapToWorldX(x, *pMap_) - sensor_pose.position.x;
    }
    else
      //cell is right of sensor
    {
      ray_end_point.x = mapToWorldX(x, *pMap_) - sensor_pose.position.x + pMap_->info.resolution; //add one cell for visualization
    }

    if(lookup_table_y <= 0)
      //cell is below sensor
    {
      ray_end_point.y = mapToWorldY(y, *pMap_) - sensor_pose.position.y;
    }
    else
      //cell is over sensor
    {
      ray_end_point.y = mapToWorldY(y, *pMap_) - sensor_pose.position.y + pMap_->info.resolution; //add one cell for visualization
    }

    //add endpoint to the vector of endpoints
    sensors_.at(sensor_index).addRayEndPoint(ray_end_point);

    //increase counter
    rays_checked++;

    //reached end of circle -> set ray to 0
    if(ray == (max_number_of_rays -1))
    {
      ray = 0;
    }
    else
    {
      ray++;
    }
  }
  //all rays checked

  //update coverage of this position if it exceeds the coverage noted with old orientation at this position
  if(coverage_by_new_orientation>coverage_by_old_orientation)
  {
    GS_pool_[point_id].max_targets_covered = coverage_by_new_orientation;
    //check if this position gives better coverage than of all previously checked positions
    if(coverage_by_new_orientation>getMaxSensorCov())
    {
      //save the max sensor coverage for comparison with other positions
      setMaxSensorCov(coverage_by_new_orientation);
      //save the max sensor coverage pose
      setMaxSensorCovPOSE(sensor_pose);
      //save the max sensor coverage point id
      setMaxSensorCovPointID(point_id);
    }
  }
}


// function to calculate coverage achieved
double greedySearch::calGScoverage()
{
  // calculate coverage percentage
  coverage_ = (double) covered_targets_num_ / target_num_;

  return coverage_;
}

// function to get maximum sensor coverage
int greedySearch::getMaxSensorCov()
{
  return max_sensor_cov_;
}

// function to get maximum sensor coverage point ID
int greedySearch::getMaxSensorCovPointID()
{
  return max_sensor_cov_point_id_;
}

// function to get maximum sensor coverage pose
geometry_msgs::Pose greedySearch::getMaxSensorCovPOSE()
{
  return max_sensor_cov_pose_;
}

// function to get angle resolution for Greedy Placement function
unsigned int greedySearch::getAngleResolution()
{
  return angle_resolution_;
}

// function to get cell search resolution for Greedy Placement function
unsigned int greedySearch::getCellSearchResolution()
{
  return cell_search_resolution_;
}

// function to set maximum coverage by a sensor
void greedySearch::setMaxSensorCov(int coverage)
{
  max_sensor_cov_ = coverage;
}

// function to set maximum sensor coverage point ID
void greedySearch::setMaxSensorCovPointID(int point_id)
{
  max_sensor_cov_point_id_ = point_id;
}

// function to set maximum sensor coverage pose
void greedySearch::setMaxSensorCovPOSE(geometry_msgs::Pose sensor_pose)
{
  max_sensor_cov_pose_ = sensor_pose;
}

// function to set the information for all targets (point_info_vec_)
void greedySearch::setPointInfoVec(std::vector<point_info> & point_info_vec, int target_num)
{
  pPoint_info_vec_ = &point_info_vec;
  target_num_ = target_num;
  if (pPoint_info_vec_ == NULL)
    ROS_ERROR("point_info_vec not set correctly");
}

// function to set the information for GS pool
void greedySearch::setGSpool(const std::vector<GS_point_info> &GS_pool)
{
  GS_pool_ = GS_pool;
  covered_targets_num_ = 0;
}

// function that set the map
void greedySearch::setMap(const nav_msgs::OccupancyGrid & new_map)
{
  pMap_ = &new_map;
  if (pMap_ == NULL)
    ROS_ERROR("Map was not set correctly.");
}

// function that sets the area of interest
void greedySearch::setAreaOfInterest(const geometry_msgs::PolygonStamped & new_poly)
{
  pArea_of_interest_ = & new_poly;
  if (pArea_of_interest_ == NULL)
    ROS_ERROR("AoI was not set correctly.");

}

// function that sets forbidden area
void greedySearch::setForbiddenArea(const geometry_msgs::PolygonStamped & new_forbidden_area)
{
  pForbidden_poly_ = & new_forbidden_area;
  if (pForbidden_poly_ == NULL)
    ROS_ERROR("Forbidden Area was not set correctly.");

}

// function that sets the opening angles for each sensor
bool greedySearch::setOpenAngles(std::vector<double> new_angles)
{
  bool result = false;
  if(new_angles.empty() || (new_angles.size() != 2) )
  {
    ROS_WARN("wrong input in greedySearch::setOpenAngles!");
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

// function that sets the range for each sensor
void greedySearch::setRange(double new_range)
{
  for(size_t i = 0; i < sensors_.size(); i++)
  {
    sensors_.at(i).setRange(new_range);
  }
}

// function to create and set a lookup table for raytracing for each sensor in the greedySearch solution
void greedySearch::setLookupTable(const std::vector< std::vector<geometry_msgs::Point32> > * pLookup_table)
{
  if (pLookup_table != NULL)
  {
    for(size_t i = 0; i < sensors_.size(); i++)
    {
      sensors_.at(i).setLookupTable(pLookup_table);
    }
  }
  else
    ROS_ERROR("LookupTable not set correctly");
}

// function to set angle resolution while doing Greedy Search
void greedySearch::setAngleResolution(unsigned int angle_resolution_param)
{
  if ((angle_resolution_param > 0) && (angle_resolution_param < 360))
  {
    angle_resolution_=angle_resolution_param;
  }
  else
    ROS_ERROR("Wrong input in angle_resolution parameter for Greedy Search");
}

// function to set the cell search resolution while doing Greedy Search
void greedySearch::setCellSearchResolution(unsigned int cell_search_resolution_param)
{
  cell_search_resolution_=cell_search_resolution_param;
}

// function to reset maximum coverage information for new sensor placement
void greedySearch::resetMaxSensorCovInfo()
{
  geometry_msgs::Pose reset_pose;
  reset_pose.position.x = 0;
  reset_pose.position.y = 0;
  reset_pose.position.z = 0;
  reset_pose.orientation = tf::createQuaternionMsgFromYaw(0);

  setMaxSensorCovPOSE(reset_pose);
  setMaxSensorCov(0);
  setMaxSensorCovPointID(0);
}

// function to reset the max targets covered information for all points in GS pool
void greedySearch::resetGSpool()
{
  for(int i=0; i<GS_pool_.size(); i++)
  {
    GS_pool_.at(i).reset();
  }
}

// returns all visualization markers of the greedySearch solution
visualization_msgs::MarkerArray greedySearch::getVisualizationMarkers()
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




