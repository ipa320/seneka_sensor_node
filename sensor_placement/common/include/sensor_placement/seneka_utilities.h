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
 * Author: Matthias Gruhler, email:Matthias.Gruhler@ipa.fhg.de
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

#ifndef SENEKA_UTILITIES_H
#define SENEKA_UTILITIES_H

// standard includes
// ----

// ros message includes
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>

namespace seneka_utilities
{
  /* ----------------------------------- */
  /* ------------- structs ------------- */
  /* ----------------------------------- */

  //information about the target which do not change
  struct target_info_fix
  {
    // holds the position of the target/cell in world coordinates
    geometry_msgs::Point32 world_pos;
    // holds the info if the given cell is occupied
    bool occupied;
    // holds the info if the given cell is occupied, free or unknown
    int8_t map_data;
    // holds the info if the given cell is a potential target, i.e. the position is inside
    // the area of interest (-1 == outside, 0 == on perimeter, 1 == inside)
    int8_t potential_target;
    // holds if the target lies within the forbidden area --
    bool forbidden;
  };

  //information about the target which change and need to be reset
  struct target_info_var
  {
    // holds the coverage info for the given target by sensors
    std::vector<bool> covered_by_sensor;
    // holds if the target is covered by at least one sensor
    bool covered;
    // holds if the target is covered by multiple sensors
    bool multiple_covered;

    // function to reset the changeable information about the target
    void reset();
  };

  //set of points for Greedy Search
  struct GS_point
  {
    // x coordinate of the point
    unsigned int x;
    // y coordinate of the point
    unsigned int y;
  };

  struct GS_point_info
  {
    //co-ordinates of this point
    GS_point p;
    //maximum number of targets covered
    unsigned int max_targets_covered;
    // function to reset the GS point information
    void reset();

  };


  struct point_info
  {
    // holds the info if the given cell is a potential target, i.e. the position is inside
    // the area of interest (-1 == outside, 0 == on perimeter, 1 == inside)
    int8_t potential_target;
    // holds the info if the given cell is occupied
    bool occupied;
    // holds if the target is covered
    bool covered;

  };




  /* ----------------------------------- */
  /* -------------- MATH --------------- */
  /* ----------------------------------- */
#define PI 3.14159265
#define EPSILON 0.0001
#define UTM_OFFSET_X 5429177.716900000
#define UTM_OFFSET_Y 457937.6288770000


  // function to gerenate random numbers in given interval
  // make sure to have set the seed in the main functions
  double randomNumber(double low, double high);

  // functions to calculate the norm of a 2D/3D vector
  double vecNorm(double x, double y, double z = 0);
  double vecNorm(geometry_msgs::Vector3 v);

  // function to calculate the dot product of two vectors
  double vecDotProd(geometry_msgs::Vector3 v, geometry_msgs::Vector3 w);

  // signum function
  int signum(double x);

  /* ----------------------------------- */
  /* --------- map conversion ---------- */
  /* ----------------------------------- */
  // functions to calculate between map (grid) and world coordinates
  double mapToWorldX(int map_x, const nav_msgs::OccupancyGrid & map);
  double mapToWorldY(int map_y, const nav_msgs::OccupancyGrid & map);
  unsigned int worldToMapX(double world_x, const nav_msgs::OccupancyGrid & map);
  unsigned int worldToMapY(double world_y, const nav_msgs::OccupancyGrid & map);
  unsigned int worldToMapUnboundedX(double world_x, const nav_msgs::OccupancyGrid & map);
  unsigned int worldToMapUnboundedY(double world_y, const nav_msgs::OccupancyGrid & map);
  geometry_msgs::Point32 mapToWorld2D(unsigned int map_x, unsigned int map_y,
                                      const nav_msgs::OccupancyGrid & map);
  void worldToMap2D(const geometry_msgs::Point32 &p,
                    const nav_msgs::OccupancyGrid &map,
                    unsigned int &map_x, unsigned int &map_y);

  // crops a map to the given bounding_box
  // taken from https://kforge.ros.org/navigation/trac/attachment/ticket/5/map-server-crop-map.patch
  void cropMap(const geometry_msgs::Polygon &bounding_box,
               const nav_msgs::OccupancyGrid &map,
               nav_msgs::OccupancyGrid &croppedMap);

  /* ----------------------------------- */
  /* --------- geometric stuff --------- */
  /* ----------------------------------- */
  // function to check if a given point is inside (return 1), outside (return -1)
  // or on an edge (return 0) of a given polygon
  int pointInPolygon(geometry_msgs::Pose2D point, geometry_msgs::Polygon polygon);

  // helper functions to check if a point lies on a 1D-Segment
  // segID = 0 (edge), segID = 1 (beam), segID = 2 (line)
  bool pointOn1DSegementPose(geometry_msgs::Pose2D start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID);
  bool pointOn1DSegementPoint(geometry_msgs::Point32 start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID);

  // helper function to check if the beam of line from start intersects the given plygon edge
  // segID = 0 (beam), segID = 1 (line)
  bool edgeIntersectsBeamOrLine(geometry_msgs::Pose2D start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID);

  // get 2D bounding box of polygon
  // (assuming z=0 for all points, otherwise, a down-projection occurs)
  // returns bounding polygon consisting of 4 points
  geometry_msgs::Polygon getBoundingBox2D(const geometry_msgs::Polygon &polygon, const nav_msgs::OccupancyGrid &map);

  // get 3D bounding box of polygon
  // returns 8 points, first four are lower plane
  geometry_msgs::Polygon getBoundingBox3D(const geometry_msgs::Polygon &polygon, const nav_msgs::OccupancyGrid &map);

  /* ----------------------------------- */
  /* --------- raytracing -------------- */
  /* ----------------------------------- */

  //function to raytrace between two cells/positions
  //returns a vector of the relative positions of cells the ray passes through
  std::vector<geometry_msgs::Point32> raytraceLine(int end_cell_x, int end_cell_y, int start_cell_x = 0, int start_cell_y = 0);


  //function to raytrace a circle
  //returns a vector of the relative positions of cells the ray passes through
  std::vector<geometry_msgs::Point32> raytraceCircle(unsigned int radius_in_cells);


  //helper function for raytraceCircle to add cell postion and its 7 mirrors to the vectors
  void addCircleCells(std::vector< std::vector<geometry_msgs::Point32> >& octants, int x, int y);


  //function to create a lookup table of all cells inside a circle
  //returns a 2D vector with all rays necessary for all cells inside a circle
  //each ray is a vector of cells
  //lookuptable[2][3] is the relativ position of the third cell in the second ray
  std::vector< std::vector<geometry_msgs::Point32> > createLookupTableCircle(unsigned int radius_in_cells);

}; // end namespace

#endif
