#ifndef GREEDYSEARCH_H
#define GREEDYSEARCH_H

// standard includes
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>

// ros includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <ros/console.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

// external includes
#include <sensor_model.h>
#include <seneka_utilities.h>
//#include <sensor_placement_node.h>

using namespace seneka_utilities;

class greedySearch
{
private:


  // std-vector storing the sensors
  std::vector< FOV_2D_model > sensors_;

  // point info vectors
  std::vector<point_info> * pPoint_info_vec_;
  std::vector<GS_point_info> GS_pool_;

  // number of sensors
  int sensor_num_;

  // number of targets
  int target_num_;
  int covered_targets_num_;

  // actual coverage
  double coverage_;

  // maximum sensor coverage information
  int max_sensor_cov_;
  int max_sensor_cov_point_id_;
  geometry_msgs::Pose max_sensor_cov_pose_;

  // actual area of interest to be covered by the sensor nodes
  const geometry_msgs::PolygonStamped * pArea_of_interest_;

  // forbidden area for the placement of sensors
  const geometry_msgs::PolygonStamped * pForbidden_poly_;

  // actual map
  const nav_msgs::OccupancyGrid * pMap_;


public:
  // standard constructor
  greedySearch();

  // constructor with arguments
  greedySearch(int num_of_sensors, int num_of_targets, FOV_2D_model sensor_model);

  // destructor
  ~greedySearch();


  // declaration of ros publisher for greedySearch solution
  ros::Publisher marker_array_pub_;

  // ************************ update functions ************************;

  // Greedy Search for maximum coverage position
  void greedyPlacement(size_t sensor_index);

  // function to place all sensors at a given pose
  void placeSensorsAtPos(geometry_msgs::Pose new_pose);

  //function to update the GS_point_info with raytracing (lookup table)
  void updateGSpointsRaytracing(size_t sensor_index, int point_id, bool update_covered_info);

  // function to calculate coverage achieved
  double calGScoverage();


  // ************************ getter functions ************************

  // function to get maximum sensor coverage
  int getMaxSensorCov();

  // function to get maximum sensor coverage point ID
  int getMaxSensorCovPointID();

  // function to get maximum sensor coverage pose
  geometry_msgs::Pose getMaxSensorCovPOSE();


  // ************************ setter functions ************************

  // function to set maximum coverage by a sensor
  void setMaxSensorCov(int coverage);

  // function to set max sensor coverage point ID
  void setMaxSensorCovPointID(int point_id);

  // function to set maximum sensor coverage pose
  void setMaxSensorCovPOSE(geometry_msgs::Pose sensor_pose);

  //function to reset maximum coverage information for new sensor placement
  void resetMaxSensorCovInfo();

    // function to set the information for all targets (point_info_vec_)
  void setPointInfoVec(std::vector<point_info> & point_info_vec, int target_num);

  // function to set the information for GS pool
  void setGSpool(const std::vector<GS_point_info> &GS_pool);

  // function to reset the max targets covered information for all points in GS pool
  void resetGSpool();


  // function that sets the map
  void setMap(const nav_msgs::OccupancyGrid & new_map);

  // function that sets the area of interest
  void setAreaOfInterest(const geometry_msgs::PolygonStamped & new_poly);

  // function that sets forbidden area
  void setForbiddenArea(const geometry_msgs::PolygonStamped & new_forbidden_area);

  // function that sets the opening angles for each sensor in the particle
  bool setOpenAngles(std::vector<double> new_angles);

  // function that sets the range for each sensor in the particle
  void setRange(double new_range);

  //function to create and set a lookup table for raytracing for each sensor in the particle
  void setLookupTable(const std::vector< std::vector<geometry_msgs::Point32> > * pLookup_table);


  // ************************* help functions *************************

  // returns all visualization markers of the particle
  visualization_msgs::MarkerArray getVisualizationMarkers();


};

#endif // GREEDYSEARCH_H
