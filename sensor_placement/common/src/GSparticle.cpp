#include <GSparticle.h>

// standard constructor
GSparticle::GSparticle()
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
GSparticle::GSparticle(int num_of_sensors, int num_of_targets, FOV_2D_model sensor_model)
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



GSparticle::~GSparticle(){}


// Greedy Search for maximum coverage position
void GSparticle::gSearch(size_t sensor_index)
{
  double angle_resolution;
  double num_of_steps;
  bool update_covered_info;
  int placement_point_id;
  geometry_msgs::Pose new_pose;
  geometry_msgs::Pose placement_pose;
  double rad;   // -b-
  int gs_size;

  num_of_steps = ceil(360/angle_resolution);

  //while searching, restrict updating of 'covered' info in updateGSpointsRaytracing
  update_covered_info=false;

  //place the current sensor on all points in GS pool one by one and calculate coverge
  for (size_t point_id=0; point_id<GS_pool_.size(); point_id=point_id+50)
  {

    //first calculate world position of current point id and place sensor at that position
    new_pose.position.x = mapToWorldX(GS_pool_[point_id].p.x, *pMap_);
    new_pose.position.y = mapToWorldY(GS_pool_[point_id].p.y, *pMap_);
    new_pose.position.z = 0;

    //check all orientations
//  for (int alpha=0; alpha<angle_resolution*num_of_steps; alpha=alpha+angle_resolution)    // -b- test the accuracy of implementation of steps
    for (int alpha=0; alpha<360; alpha=alpha+45)
    {
      rad=alpha*(PI/180); //-b-
      //look around at all angles and save only the max coverage angle
      new_pose.orientation = tf::createQuaternionMsgFromYaw(PI/4);

      sensors_.at(sensor_index).setSensorPose(new_pose);

      //update function also updates the max coverage position
      updateGSpointsRaytracing(sensor_index, point_id, update_covered_info);
    }
  }
  //Get maximum coverage pose
  placement_pose = getMaxSensorCovPOSE();
  //Get maximum coverage point ID
  placement_point_id = getMaxSensorCovPointID();

  ROS_INFO_STREAM("placement_point_id: " << placement_point_id);

  //place the sensor at max coverage point
  sensors_.at(sensor_index).setSensorPose(placement_pose);

  //now update the 'covered' info
  update_covered_info = true;
  updateGSpointsRaytracing(sensor_index, placement_point_id, update_covered_info);   //-b- !!!!


  GS_pool_.erase(GS_pool_.begin()+placement_point_id);

  gs_size = GS_pool_.size();

  ROS_INFO_STREAM("new pool size: " << gs_size);


}




//function to update the GS_point_info with raytracing (lookup table)
void GSparticle::updateGSpointsRaytracing(size_t sensor_index, int point_id, bool update_covered_info)
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
//    if((y >= 0) && (x >= 0) && (y < pMap_->info.height) && (x < pMap_->info.width) && (cell_in_vector_coordinates < pTargets_with_info_fix_->size()))
      if(((y >= 0) && (x >= 0) && (y < pMap_->info.height) && (x < pMap_->info.width)) && (cell_in_vector_coordinates < pPoint_info_vec_->size()))  // -b-
      {
        if(pPoint_info_vec_->at(cell_in_vector_coordinates).potential_target != 0)
        {
          //cell not on the perimeter
          if(pPoint_info_vec_->at(cell_in_vector_coordinates).potential_target == 1)
          {
            //cell a potential target
            if(pPoint_info_vec_->at(cell_in_vector_coordinates).covered == false)
            {
              //cell not already covered
              coverage_by_new_orientation++;

              //mark this position as covered only if the current pose is the final selected position for sensor placement
              if(update_covered_info=true)
              {
                pPoint_info_vec_->at(cell_in_vector_coordinates).covered = true;
                covered_targets_num_++;   // -b-
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
/*          //cell on perimeter and not occupied -> continue with the next cell on the ray (no coverage)
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
*/
          continue;   // -b- occupied info?
        }
      }
      else
      //cell coordinates not valid (outside the area of interest) -> skip rest of this ray
      {
        break;
      }

      //update coverage of this position if it exceeds the coverage noted with old orientation
      if (coverage_by_new_orientation>coverage_by_old_orientation)
      {
        GS_pool_[point_id].max_targets_covered = coverage_by_new_orientation;
        //check if this position gives better coverage than of all previously checked positions
        if (coverage_by_new_orientation>getMaxSensorCov())
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


// function to place all sensors at a given pose
void GSparticle::placeSensorsAtPos(geometry_msgs::Pose new_pose)
{
  for(size_t i = 0; i < sensors_.size(); i++)
  {
    sensors_.at(i).setSensorPose(new_pose);
  }
}


// function to calculate coverage achieved
double GSparticle::calGScoverage()
{
  // calculate coverage percentage
  coverage_ = (double) covered_targets_num_ / target_num_;

  return coverage_;
}

// function to get maximum sensor coverage
int GSparticle::getMaxSensorCov()
{
  return max_sensor_cov_;
}

// function to get maximum sensor coverage point ID
int GSparticle::getMaxSensorCovPointID()
{
  return max_sensor_cov_point_id_;
}

// function to get maximum sensor coverage pose
geometry_msgs::Pose GSparticle::getMaxSensorCovPOSE()
{
  return max_sensor_cov_pose_;
}


// function to set maximum coverage by a sensor
void GSparticle::setMaxSensorCov(int coverage)
{
  max_sensor_cov_ = coverage;
}

// function to set maximum sensor coverage point ID
void GSparticle::setMaxSensorCovPointID(int point_id)
{
  max_sensor_cov_point_id_ = point_id;
}

// function to set maximum sensor coverage pose
void GSparticle::setMaxSensorCovPOSE(geometry_msgs::Pose sensor_pose)
{
  max_sensor_cov_pose_ = sensor_pose;
}


// function to set the information for all targets (point_info_vec_)
void GSparticle::setPointInfoVec(std::vector<point_info> & point_info_vec, int target_num)
{
  pPoint_info_vec_ = &point_info_vec;
  target_num_ = target_num;
  if (pPoint_info_vec_ == NULL)
    ROS_ERROR("point_info_vec not set correctly");
}

// function to set the information for GS pool
void GSparticle::setGSpool(const std::vector<GS_point_info> &GS_pool)
{
  GS_pool_ = GS_pool;
  covered_targets_num_ = 0;   //-b-
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




