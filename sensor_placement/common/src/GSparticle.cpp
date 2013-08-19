#include <GSparticle.h>

// standard constructor
GSparticle::GSparticle()
{
  // global coverage information (i.e. information related to all GS target points)
  global_max_coverage_ = 0;
  global_max_coverage_id_ = 0;
}


// constructor with arguments
GSparticle::GSparticle(int num_of_sensors, int num_of_targets, FOV_2D_model sensor_model)
{
  // initialize number of sensors
  sensor_num_ = num_of_sensors;

  // initialze number of targets
  target_num_ = num_of_targets;

  // intialize coverage
//  coverage_ = 0;    -b-

  // global coverage information (i.e. information related to all GS target points)
  global_max_coverage_ = 0;
  global_max_coverage_id_ = 0;

/*
  // initialize personal best coverage
  pers_best_coverage_ = 0;

  //initialize multiple coverage indices
  multiple_coverage_ = 0;
  pers_best_multiple_coverage_ = 0;
*/
  // initialize sensor vector with as many entries as specified by sensors_num_
  sensors_.assign(sensor_num_, sensor_model);

}



GSparticle::~GSparticle(){}


//get targets (GS_point_info for all points of interest for Greedy Search)
bool GSparticle::getGSTargets()
{
  // initialize result
  bool result = false;

  if(map_received_ == true)
  {
    // only if we received a map, we can get targets
    point_info dummy_point_info;
    point_info_vec_.assign(map_.info.width * map_.info.height, dummy_point_info);

    // create dummy GS_point_info to save information in the pool
    GS_point_info dummy_GS_point_info;

    if(AoI_received_ == false)
    {
      for(unsigned int i = 0; i < map_.info.width; i++)
      {
        for(unsigned int j = 0; j < map_.info.height; j++)
        {
          // calculate world coordinates from map coordinates of given target
          geometry_msgs::Pose2D world_Coord;
          world_Coord.x = mapToWorldX(i, map_);
          world_Coord.y = mapToWorldY(j, map_);
          world_Coord.theta = 0;

          dummy_point_info.p.x = i;
          dummy_point_info.p.y = j;
          dummy_point_info.potential_target = 1;    //check -b-


          if(pointInPolygon(world_Coord, forbidden_area_.polygon) == 0)
          {
            // points outside the forbidden area are of interest for greedy search
            dummy_GS_point_info.p.x=i;
            dummy_GS_point_info.p.y=j;
            dummy_GS_point_info.max_targets_covered=0;
          }

          // saving information
          point_info_vec_.at(j * map_.info.width + i) = dummy_point_info;
          GS_pool_.push_back(dummy_GS_point_info);
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

          dummy_point_info.p.x = i;
          dummy_point_info.p.y = j;
          dummy_point_info.potential_target = -1;

          // the given position lies withhin the polygon
          if(pointInPolygon(world_Coord, area_of_interest_.polygon) == 2)
          {
            dummy_point_info.potential_target = 1; //mark the point as potential target

            if(pointInPolygon(world_Coord, forbidden_area_.polygon) == 0)
            {
              // points outside the forbidden area are of interest for greedy search
              dummy_GS_point_info.p.x=i;
              dummy_GS_point_info.p.y=j;
              dummy_GS_point_info.max_targets_covered=0;
            }

          }
          // the given position lies on the perimeter
          else if(pointInPolygon(world_Coord, area_of_interest_.polygon) == 1)
          {

            dummy_point_info.potential_target = 0; //mark the point as not a potential target -b-

            if(pointInPolygon(world_Coord, forbidden_area_.polygon) == 0)
            {
              // points outside the forbidden area are of interest for greedy search
              dummy_GS_point_info.p.x=i;
              dummy_GS_point_info.p.y=j;
              dummy_GS_point_info.max_targets_covered=0;
            }
          }
          // save the point information
          point_info_vec_.at(j * map_.info.width + i) = dummy_point_info;
          GS_pool_.push_back(dummy_GS_point_info);

        }
      }
      result = true;
    }
  }
  else
  {
    ROS_WARN("No map received! Not able to propose sensor positions.");
  }
  ROS_INFO_STREAM("GS total points saved: " << gs_obj_.getPoolCount());
  return result;
}


//function to update the GS_point_info with raytracing (lookup table)
void GSparticle::updateGSpointsRaytracing(size_t sensor_index, int point_id)
{
  //clear vector of ray end points
  sensors_.at(sensor_index).clearRayEndPoints();

  unsigned int max_number_of_rays = sensors_.at(sensor_index).getLookupTable()->size();
  geometry_msgs::Pose sensor_pose = sensors_.at(sensor_index).getSensorPose();

  std::vector<double> open_ang = sensors_.at(sensor_index).getOpenAngles();
  double orientation = tf::getYaw(sensor_pose.orientation);

  //get angles of sensor and keep them between 0 and 2*PI     //MODIFY here for efficient search of max coverage angle
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

  // initialize old and new coverage of the sensor position indicated by point_id // -b-
  int old_coverage = GS_pool_[point_id].max_targets_coverage;
  int new_coverage = 0;

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
//    if((y >= 0) && (x >= 0) && (y < pMap_->info.height) && (x < pMap_->info.width) && (cell_in_vector_coordinates < pTargets_with_info_fix_->size()))
      if((y >= 0) && (x >= 0) && (y < pMap_->info.height) && (x < pMap_->info.width))
      {
        if(inGSpool(cell_in_vector_coordinates))
        {
          new_coverage++;

          if (new_coverage>old_coverage)
          {
            //update coverage of this sensor placement point only if it exceeds the old coverage
            GS_pool_[point_id].max_targets_coverage = new_coverage;

            if (new_coverage>getGlobalMaxCoverage())
            {
              //save the new global max coverage for comparison in next iterations
              setGlobalMaxCoverage(new_coverage);
              //save the global coverage pose for sensor placement
              setGlobalMaxCoveragePOSE(sensor_pose);
            }
          }
        }
        else
        {
          //the given point is outside the GS pool so skip rest of the ray    //-b- skip or continue ???
          break;
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
}


//function to update the GS_point_info with raytracing (lookup table)
void GSparticle::removeGSpointsRaytracing(size_t sensor_index, int point_id)
{
  //clear vector of ray end points
  sensors_.at(sensor_index).clearRayEndPoints();

  unsigned int max_number_of_rays = sensors_.at(sensor_index).getLookupTable()->size();
  geometry_msgs::Pose sensor_pose = sensors_.at(sensor_index).getSensorPose();

  std::vector<double> open_ang = sensors_.at(sensor_index).getOpenAngles();
  double orientation = tf::getYaw(sensor_pose.orientation);

  //get angles of sensor and keep them between 0 and 2*PI     //MODIFY here for efficient search of max coverage angle
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

  // initialize old and new coverage of the sensor position indicated by point_id // -b-
  int old_coverage = GS_pool_[point_id].max_targets_coverage;
  int new_coverage = 0;

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
//    if((y >= 0) && (x >= 0) && (y < pMap_->info.height) && (x < pMap_->info.width) && (cell_in_vector_coordinates < pTargets_with_info_fix_->size()))
      if((y >= 0) && (x >= 0) && (y < pMap_->info.height) && (x < pMap_->info.width))
      {
        if(inGSpool(cell_in_vector_coordinates))
        {
          new_coverage++;

          if (new_coverage>old_coverage)
          {
            //update coverage of this sensor placement point only if it exceeds the old coverage
            GS_pool_[point_id].max_targets_coverage = new_coverage;

            if (new_coverage>getGlobalMaxCoverage())
            {
              //save the new global max coverage for comparison in next iterations
              setGlobalMaxCoverage(new_coverage);
              //save the global coverage pose for sensor placement
              setGlobalMaxCoveragePOSE(sensor_pose);
            }
          }
        }
        else
        {
          //the given point is outside the GS pool so skip rest of the ray    //-b- skip or continue ???
          break;
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
}




// function to get global_max_coverage_
int getGlobalMaxCoverage()
{
  return global_max_coverage_;
}

// function to get global_max_coverage_id_
geometry_msgs::Pose getGlobalMaxCoveragePOSE()
{
  return global_max_coverage_pose_;
}


/*

not needed with the new implementation

// deletes the point at given index from GS_pool
// NOTE: modifies the arrangement of points - can not rely on points being in a certain order if this function is used
void GSparticle::deleteGSpoint(int point_index)
{
  int last_index = GS_pool_.size();
  temp GS_point_info;

  //swap given point with the last point in the pool
  temp = GS_pool_[last_index];
  GS_pool_[last_index] = GS_pool_[point_index];
  GS_pool_[point_index] = temp;

  //decrease pool_count_ so that it is not accessed in the next step of GS Sensor placement
  pool_count_ = pool_count_-1;
}
*/


// function to see if the given point is in GSpool or not; returns the pool_index on success and -1 if the point wasn't found
// NOTE: the bounds are not verified -b-
// also check mismatch due to rounding off of cell coordinates value
bool GSparticle::inGSpool (int cell_in_vector_coordinates)
{
  int max_count=pool_count_;
  //checking whether or not cell_in_vector_coordinates is present in Greedy Search pool
  for (int i=0; i<max_count; i++)
    {
      current_coordinates = GS_pool_[i].p.y * map_ + GS_pool_[i].p.x;

      if (current_coordinates==cell_in_vector_coordinates)
        {
          //match found
          return true;
        }


    }
  //no match found
  return false;
}


// function that set the map
void GSparticle::setMap(const nav_msgs::OccupancyGrid & new_map)
{
  pMap_ = &new_map;
  if (pMap_ == NULL)
    ROS_ERROR("Map was not set correctly.");
}

// function that sets the area of interest
void GSparticle::setAreaOfInterest(const geometry_msgs::PolygonStamped & new_poly)
{
  pArea_of_interest_ = & new_poly;
  if (pArea_of_interest_ == NULL)
    ROS_ERROR("AoI was not set correctly.");

}

// function that sets forbidden area
void GSparticle::setForbiddenArea(const geometry_msgs::PolygonStamped & new_forbidden_area)
{
  pForbidden_poly_ = & new_forbidden_area;
  if (pForbidden_poly_ == NULL)
    ROS_ERROR("Forbidden Area was not set correctly.");

}

// function that sets the opening angles for each sensor in the GSparticle
bool GSparticle::setOpenAngles(std::vector<double> new_angles)
{
  bool result = false;
  if(new_angles.empty() || (new_angles.size() != 2) )
  {
    ROS_WARN("wrong input in GSparticle::setOpenAngles!");
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

// function that sets the range for each sensor in the GSparticle
void GSparticle::setRange(double new_range)
{
  for(size_t i = 0; i < sensors_.size(); i++)
  {
    sensors_.at(i).setRange(new_range);
  }
}

// function to create and set a lookup table for raytracing for each sensor in the GSparticle
void GSparticle::setLookupTable(const std::vector< std::vector<geometry_msgs::Point32> > * pLookup_table)
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




// function to set global_max_coverage_
void setGlobalMaxCoverage(int coverage)
{
  global_max_coverage_ = coverage;
}

/*
// function to set global_max_coverage_id_ variable
void setGlobalMaxCoverageID(int point_id)
{
  global_max_coverage_id_ = point_id;
}
*/

// function to set global_max_coverage_pose_
void getGlobalMaxCoveragePOSE(geometry_msgs::Pose sensor_pose)
{
  global_max_coverage_pose_ = sensor_pose;
}


// function to place all sensors at a given pose
void GSparticle::placeSensorsAtPos(geometry_msgs::Pose new_pose)
{
  for(size_t i = 0; i < sensors_.size(); i++)
  {
    sensors_.at(i).setSensorPose(new_pose);

    updateTargetsInfoRaytracing(i);
  }
  calcCoverage();
}

// function to initialize the sensors velocities randomly
void GSparticle::initializeRandomSensorVelocities()
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



// returns pool_index of point which covers maximum targets. returns -1 on failure
int GSparticle::getMaxCoverageGSpoint()
{
  int current_max=0;
  int pool_index=-1;
  int max_count=getPoolCount();

  for (int i=0; i<max_count i++)
  {
    if GS_pool_[i].max_targets_covered>current_max
    {
      //update current_max
      current_max=GS_pool_[i].max_targets_covered;
      pool_index=i;
    }
  }
  return pool_index;
}


// places each sensor at point where maximum coverage is acheived
void GSparticle::sensorPlacementGS()
{
  int max_point_index=0;

  geometry_msgs::Point32 newPose;

  for(size_t i = 0; i < sensors_.size(); i++)
  {

  max_point_index = getMaxCoverageGSpoint();

  newPose.x = mapToWorldX(GS_pool_[max_point_index].p.x, map_);
  newPose.y = mapToWorldY(GS_pool_[max_point_index].p.y, map_);
  newPose.z = 0;

  sensors_.at(i).setSensorPose(newPose);

  deleteGSpoint(max_point_index);

  }

}

// returns all visualization markers of the GSparticle
visualization_msgs::MarkerArray GSparticle::getVisualizationMarkers()
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




