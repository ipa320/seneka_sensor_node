#ifndef GSPARTICLE_H
#define GSPARTICLE_H

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
#include </home/mig-mc/git/groovy_workspace/src/seneka/sensor_placement/include/sensor_placement/sensor_placement_node.h>

using namespace seneka_utilities;

class GSparticle
{
private:

/*
  // std-vector storing the sensors
  std::vector< FOV_2D_model > sensors_;

  // std-vector storing the personal best solution of the particle
  std::vector< FOV_2D_model > pers_best_;
*/

  // a dummy sensor to get coverage on Greedy Search pool of points
  FOV_2D_model dummy_sensor;    //-b-

  // actual area of interest to be covered by the sensor nodes
  const geometry_msgs::PolygonStamped * pArea_of_interest_;

  // forbidden area for the placement of sensors
  const geometry_msgs::PolygonStamped * pForbidden_poly_;

  // actual map
  const nav_msgs::OccupancyGrid * pMap_;

  // global coverage information (i.e. information related to all GS target points)
  int global_max_coverage_;
  geometry_msgs::Pose global_max_coverage_pose_;




public:
  // standard constructor
  GSparticle();

  // constructor with arguments
  GSparticle(int num_of_sensors, int num_of_targets, FOV_2D_model sensor_model);

  // destructor
  ~GSparticle();

  // ************************ getter functions ************************

/*  // function to get personal best solution
  std::vector< FOV_2D_model > getPersonalBest();

  // function to get actual solution
  std::vector< FOV_2D_model > getActualSolution();

  // function to get the sensor positions of the actual solution
  std::vector<geometry_msgs::Pose> getSolutionPositions();

  // function to get the sensor positions of the actual solution as nav_msgs::Path
  nav_msgs::Path getSolutionPositionsAsPath();

  // function to get the sensor positions of the personal best solution
  std::vector<geometry_msgs::Pose> getPersonalBestPositions();

  // function to get personal best coverage
  double getBestCoverage();

  // function to get actual coverage
  double getActualCoverage();

  // function to get multiple coverage index
  int getMultipleCoverageIndex();
*/

  //get targets (GS_point_info for all points of interest for Greedy Search)
  bool getGSTargets();

  // function to get global_max_coverage_
  int getGlobalMaxCoverage();

  // function to get global_max_coverage_pose_
  geometry_msgs::Pose getGlobalMaxCoveragePOSE();


  // ************************ setter functions ************************

  // function that sets the member variable sensor_num_ and reserves capacity for vector sensors_
/*  void setSensorNum(int num_of_sensors);

  // function to set the fix information for all targets
  void setTargetsWithInfoFix(const std::vector<target_info_fix> &targets_with_info_fix, int target_num);

  // function to set the variable information for all targets
  void setTargetsWithInfoVar(const std::vector<target_info_var> &targets_with_info_var);

  // function to reset the variable information for all targets
  void resetTargetsWithInfoVar();
*/
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

  // function to set global_max_coverage_
  void setGlobalMaxCoverage(int coverage);

  // function to set global_max_coverage_id_
  void setGlobalMaxCoveragePOSE(geometry_msgs::Pose sensor_pose);


  // ************************ update functions ************************
/*
  // function to place the sensors randomly on the perimeter
  void placeSensorsRandomlyOnPerimeter();

  // function to initialize the sensors on the perimeter
  void initializeSensorsOnPerimeter();

  // function to initialize the sensors velocities randomly
  void initializeRandomSensorVelocities();

  // function to update particle during PSO
  void updateParticle(std::vector<geometry_msgs::Pose> global_best, double PSO_param_1, double PSO_param_2, double PSO_param_3);

  // function to update the targets_with_info variable
  void updateTargetsInfo(size_t sensor_index);

  //function to update the targets_with_info variable with raytracing (lookup table)
  void updateTargetsInfoRaytracing(size_t sensor_index);

*/
  // function to place all sensors at a given pose
  void placeSensorsAtPos(geometry_msgs::Pose new_pose);

  // function to calculate the actual  and personal best coverage
  void calcCoverage();

  // function to check coverage of given sensor and target
  bool checkCoverage(FOV_2D_model sensor, geometry_msgs::Point32 target);

  // function to check if the new sensor position is accepted
  bool newPositionAccepted(geometry_msgs::Pose new_pose_candidate);

  // function to check if the new sensor orientation is accepted
  bool newOrientationAccepted(size_t sensor_index, geometry_msgs::Pose new_pose_candidate);

  // ************************* help functions *************************
/*
  // helper function to find an uncovered target far away from a given sensor position
  // the return value is the index of that uncovered target
  unsigned int findFarthestUncoveredTarget(size_t sensor_index);

  // helper function to find a random uncovered and non occupied target not forbidden
  // the return value is the index of that uncovered target
  unsigned int randomFreeTarget();

  // helper function to check, if the sensor is facing outside the area of interest
  bool sensorBeamIntersectsPerimeter(size_t sensor_index, geometry_msgs::Pose new_pose_candidate);

  // helper function for the actual calculation step in sensorBeamIntersectsPerimeter function
  double intersectionCalculation(double v1, double v2, double x1, double x2, double y1, double y2);
*/

  // function to see if the given point is in GSpool or not; returns the poolCount on success and -1 if the point wasn't found
  int inGSpool (int cell_in_vector_coordinates);

  // returns pool_index of point which covers maximum targets. returns -1 on failure
  int getMaxCoverageGSpoint();

  // deletes the point at given index from GS_pool
  // NOTE: modifies the arrangement of points - can not rely on points being in a certain order if this function is used
  void deleteGSpoint(int point_index);

  // returns all visualization markers of the particle
  visualization_msgs::MarkerArray getVisualizationMarkers();


};

#endif // GSPARTICLE_H
