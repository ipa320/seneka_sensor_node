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

#include <seneka_utilities.h>

namespace seneka_utilities
{
  /* ----------------------------------- */
  /* ------ STRUCT FUNCTIONS ----------- */
  /* ----------------------------------- */

  // function to reset the changeable information about the target
  void target_info_var::reset()
  {
    covered = false;
    multiple_covered = false;

    std::fill(covered_by_sensor.begin(), covered_by_sensor.end(), false);
  }

  // function to reset the targets covered info
  void GS_point_info::reset()
  {
    max_targets_covered = 0;
  }

  /* ----------------------------------- */
  /* -------------- MATH --------------- */
  /* ----------------------------------- */
  // function to gerenate random numbers in given interval
  double randomNumber(double low, double high)
  {
    return ((double) rand()/ RAND_MAX) * (high - low) + low;
  }

  // function to calculate the norm of a 2D/3D vector
  double vecNorm(double x, double y, double z)
  {
    return sqrt( x * x + y * y + z * z);
  }

  double vecNorm(geometry_msgs::Vector3 v)
  {
    return sqrt(vecDotProd(v,v));
  }

  // function to calculate the dot product of two vectors
  double vecDotProd(geometry_msgs::Vector3 v, geometry_msgs::Vector3 w)
  {
    return (v.x * w.x + v.y * w.y + v.z * w.z);
  }

  // signum function
  int signum(double x)
  {
    if (x > 0)
      return 1;
    else if (x < 0)
      return -1;
    else
      return 0;
  }

  /* ----------------------------------- */
  /* --------- map conversion ---------- */
  /* ----------------------------------- */
  // functions to calculate between map (grid) and world coordinates
  double mapToWorldX(int map_x, const nav_msgs::OccupancyGrid & map)
  {
    return map.info.origin.position.x + (map_x * map.info.resolution);
  }

  double mapToWorldY(int map_y, const nav_msgs::OccupancyGrid & map)
  {
    return map.info.origin.position.y + (map_y * map.info.resolution);
  }

  unsigned int worldToMapX(double world_x, const nav_msgs::OccupancyGrid & map)
  {
    unsigned int result = std::max((unsigned int) 0 , std::min(map.info.width, (unsigned int) floor((world_x - map.info.origin.position.x + EPSILON) / map.info.resolution)));

    if(fabs(mapToWorldX(result, map) - (map.info.origin.position.x + map.info.width * map.info.resolution)) < EPSILON)
    {
      return result-1;
    }
    else
    {
      return result;
    }
  }

  unsigned int worldToMapY(double world_y, const nav_msgs::OccupancyGrid & map)
  {
    unsigned int result = std::max((unsigned int) 0 , std::min(map.info.height, (unsigned int) floor((world_y - map.info.origin.position.y + EPSILON) / map.info.resolution)));

    if(fabs(mapToWorldY(result, map) - (map.info.origin.position.y + map.info.height * map.info.resolution)) < EPSILON)
    {
      return result-1;
    }
    else
    {
      return result;
    }
  }

  unsigned int worldToMapUnboundedX(double world_x, const nav_msgs::OccupancyGrid & map)
  {
    return floor((world_x - map.info.origin.position.x + EPSILON) / map.info.resolution);
  }

  unsigned int worldToMapUnboundedY(double world_y, const nav_msgs::OccupancyGrid & map)
  {
    return floor((world_y - map.info.origin.position.y + EPSILON) / map.info.resolution);
  }

  geometry_msgs::Point32 mapToWorld2D(unsigned int map_x, unsigned int map_y,
                                      const nav_msgs::OccupancyGrid & map)
  {
    geometry_msgs::Point32 p;
    p.x = mapToWorldX(map_x,map);
    p.y = mapToWorldY(map_y,map);
    p.z = 0.0;
    return p;
  }

  void worldToMap2D(const geometry_msgs::Point32 &p,
                    const nav_msgs::OccupancyGrid &map,
                    unsigned int &map_x, unsigned int &map_y)
  {
    map_x = worldToMapX(p.x, map);
    map_y = worldToMapY(p.y, map);
  }

  // crops a map to the given bounding_box
  // taken from https://kforge.ros.org/navigation/trac/attachment/ticket/5/map-server-crop-map.patch
  void cropMap(const geometry_msgs::Polygon &bounding_box,
               const nav_msgs::OccupancyGrid &map,
               nav_msgs::OccupancyGrid &croppedMap)
  {
    uint32_t top_index;
    uint32_t left_index;
    uint32_t bottom_index;
    uint32_t right_index;

    // first point of polygon contains x_min and y_min, 3rd contains x_max and y_max
    worldToMap2D(bounding_box.points.at(0), map, left_index, top_index);
    worldToMap2D(bounding_box.points.at(2), map, right_index, bottom_index);

    croppedMap.info = map.info;
    croppedMap.info.width = right_index - left_index;
    croppedMap.info.height = bottom_index - top_index;
    croppedMap.info.origin.position.x =
       map.info.origin.position.x + left_index * map.info.resolution;
    croppedMap.info.origin.position.y =
       map.info.origin.position.y + top_index * map.info.resolution;
    croppedMap.data.resize(croppedMap.info.width * croppedMap.info.height);

    uint32_t i = 0;
    for(uint32_t y = top_index; y < bottom_index; y++ ) {
      for (uint32_t x = left_index; x < right_index; x++, i++ ) {
        croppedMap.data[i] = map.data[y * map.info.width + x];
      }
    }
  }

  /* ----------------------------------- */
  /* --------- geometric stuff --------- */
  /* ----------------------------------- */
  // function to check if a given point is inside (return 2), outside (return 0)
  // or on an edge (return 1) of a given polygon
  // If no polygon is given, it returns -1
  int pointInPolygon(geometry_msgs::Pose2D point, geometry_msgs::Polygon polygon)
  {
    // check if we received a polygon
    if (polygon.points.size() == 0)
      return -1;

    // initialize workspace variables
    int result = 0;
    bool ignore = false;
    size_t start_index = 0;
    bool start_index_set = false;
    int intersect_count = 0;
    geometry_msgs::Point32 poly_point_1;
    geometry_msgs::Point32 poly_point_2;

    // check in the first loop, if the point lies on any of the polygons edges
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
        return 1;
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
      result = 0;
    else
      result = 2;

    return result;
  }

  // helper function to check if a point lies on a 1D-Segment
  // segID = 0 (edge), segID = 1 (beam), segID = 2 (line)
  bool pointOn1DSegementPose(geometry_msgs::Pose2D start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID)
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
        if( ( fabs(border_1.x*(1-t) + t*border_2.x - start.x) <= 0.01) && ( fabs(border_1.y*(1-t) + t*border_2.y - start.y) <= 0.01) )
        {
          // start lies on the segment
          result = true;
        }
      }
    }

    return result;
  }

  bool pointOn1DSegementPoint(geometry_msgs::Point32 start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID)
  {
    geometry_msgs::Pose2D help_pose;
    help_pose.x = start.x;
    help_pose.y = start.y;
    help_pose.theta = 0;

    return pointOn1DSegementPose(help_pose, border_1, border_2, segID);
  }

  // helper function to check if the beam of line from start intersects the given plygon edge
  // segID = 0 (beam), segID = 1 (line)
  bool edgeIntersectsBeamOrLine(geometry_msgs::Pose2D start, geometry_msgs::Point32 border_1, geometry_msgs::Point32 border_2, int segID)
  {
    // initialize workspace
    bool result = false;
    double t = 0;
    double s = 0;
    if(border_1.y == border_2.y)
    {
      // edge is parallel or coincides with line or beam
      result = false;
    }
    else
    {
      t = -(start.x - border_1.x) + ( (border_2.x - border_1.x) * (start.y - border_1.y) ) / (border_2.y - border_1.y);
      s = (start.y - border_1.y) / (border_2.y - border_1.y);

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

  // get 2D bounding box of polygon
  // (assuming z=0 for all points, otherwise, a down-projection occurs)
  // returns bounding polygon consisting of 4 points
  geometry_msgs::Polygon getBoundingBox2D(const geometry_msgs::Polygon & polygon, const nav_msgs::OccupancyGrid &map)
  {
    geometry_msgs::Polygon out_poly;
    double x_min = mapToWorldX(map.info.width, map);
    double x_max = mapToWorldX(0, map);
    double y_min = mapToWorldY(map.info.height, map);
    double y_max = mapToWorldY(0, map);

    for (unsigned int i = 0; i < polygon.points.size(); i++)
    {
      if ( polygon.points[i].x > x_max )
        x_max = polygon.points[i].x;
      if ( polygon.points[i].y > y_max )
        y_max = polygon.points[i].y;
      if ( polygon.points[i].x < x_min )
        x_min = polygon.points[i].x;
      if ( polygon.points[i].y < y_min )
        y_min = polygon.points[i].y;
    }
    geometry_msgs::Point32 p;
    p.x = x_min; p.y = y_min;
    out_poly.points.push_back(p);
    p.x = x_max; p.y = y_min;
    out_poly.points.push_back(p);
    p.x = x_max; p.y = y_max;
    out_poly.points.push_back(p);
    p.x = x_min; p.y = y_max;
    out_poly.points.push_back(p);

    return out_poly;
  }

  // get 3D bounding box of polygon
  // returns 8 points, first four are lower plane
  geometry_msgs::Polygon getBoundingBox3D(const geometry_msgs::Polygon & polygon, const nav_msgs::OccupancyGrid &map)
  {
    // TO DO: remove zero-init and initialize variables with min and max values of the map
    geometry_msgs::Polygon out_poly;
    double x_min = 0.0, x_max = 0.0;
    double y_min = 0.0, y_max = 0.0;
    double z_min = 0.0, z_max = 0.0;
    for (unsigned int i = 0; i < polygon.points.size(); i++)
    {
      if ( polygon.points[i].x > x_max )
        x_max = polygon.points[i].x;
      if ( polygon.points[i].y > y_max )
        y_max = polygon.points[i].y;
      if ( polygon.points[i].z > z_max )
        z_max = polygon.points[i].z;
      if ( polygon.points[i].x < x_min )
        x_min = polygon.points[i].x;
      if ( polygon.points[i].y < y_min )
        y_min = polygon.points[i].y;
      if ( polygon.points[i].z < z_min )
        z_min = polygon.points[i].z;
    }
    geometry_msgs::Point32 p;
    p.x = x_min; p.y = y_min; p.z = z_min;
    out_poly.points.push_back(p);
    p.x = x_max; p.y = y_min; p.z = z_min;
    out_poly.points.push_back(p);
    p.x = x_max; p.y = y_max; p.z = z_min;
    out_poly.points.push_back(p);
    p.x = x_min; p.y = y_max; p.z = z_min;
    out_poly.points.push_back(p);
    p.x = x_min; p.y = y_max; p.z = z_max;
    out_poly.points.push_back(p);
    p.x = x_min; p.y = y_min; p.z = z_max;
    out_poly.points.push_back(p);
    p.x = x_max; p.y = y_min; p.z = z_max;
    out_poly.points.push_back(p);
    p.x = x_max; p.y = y_max; p.z = z_max;
    out_poly.points.push_back(p);

    return out_poly;
  }

  /* ----------------------------------- */
  /* --------- raytracing -------------- */
  /* ----------------------------------- */

  //function to raytrace between two cells/positions
  //returns a vector of the relative positions of cells the ray passes through
  std::vector<geometry_msgs::Point32> raytraceLine(int end_cell_x, int end_cell_y, int start_cell_x, int start_cell_y)
  {
    //absolute distances
    unsigned int dx = std::abs(end_cell_x - start_cell_x);
    unsigned int dy = std::abs(end_cell_y - start_cell_y);

    //number of cells in the ray is fix due to 4-connectivity
    unsigned int number_of_points = dx+dy;

    //all cells in the ray
    std::vector<geometry_msgs::Point32> ray_points;

    //increase or decrease x,y?
    int x_inc, y_inc;

    if(start_cell_x < end_cell_x)
      x_inc = 1;
    else
      x_inc = -1;

    if(start_cell_y < end_cell_y)
      y_inc = 1;
    else
      y_inc = -1;

    //current cell
    int x = start_cell_x;
    int y = start_cell_y;

    //bresenham errors
    int error = 0;
    int e_x, e_y;

    //add first cell
    geometry_msgs::Point32 first_cell;
    first_cell.x = x;
    first_cell.y = y;
    first_cell.z = 0;

    ray_points.push_back(first_cell);

    //make 1 step in either x or y direction
    for(unsigned int i=0; i<number_of_points; i++)
    {
      e_x = error - dx;
      e_y = error + dy;

      if(std::abs(e_y) < std::abs(e_x))
        //error is smaller if moving in x direction
      {
        x += x_inc;
        error = e_y;
      }
      else
        //error is smaller moving in y direction
      {
        y += y_inc;
        error = e_x;
      }

      //add current cell
      geometry_msgs::Point32 current_cell;
      current_cell.x = x;
      current_cell.y = y;
      current_cell.z = 0;

      ray_points.push_back(current_cell);
    }

    return ray_points;
  }

  //function to raytrace a circle
  //returns a vector of the relative positions of cells the ray passes through
  std::vector<geometry_msgs::Point32> raytraceCircle(unsigned int radius_in_cells)
  {
    //vector storing the circle cells divided in eight parts of the circle due to symmetry
    std::vector< std::vector<geometry_msgs::Point32> > octants;
    octants.resize(8);

    int x,y;
    int error;

    x = radius_in_cells;
    y = 0;
    error = 1 - radius_in_cells;

    while(x >= y)
    {
      //add current cell and mirrors to the vectors
      addCircleCells(octants,x,y);

      //go one cell higher
      y++;

      if(error < 0)
      {
        error += y * 2 + 1;
      }
      else
      {
        x--;
        error += 2 * (y - x + 1);

        //if x is decreased, also add the cell next to it to ensure 4-connectivity
        addCircleCells(octants,x+1,y);
      }
    }

    //final vector for all cells of the circle in correct order
    std::vector<geometry_msgs::Point32> circle_cells;

    //for every octant
    for(unsigned int i=0; i<8; i++)
    {
      if(i%2 != 0)
      {
        //flip every second octant vector
        std::reverse(octants[i].begin(),octants[i].end());
      }

      //delete last element and add to final vector
      octants[i].pop_back();
      circle_cells.insert(circle_cells.end(), octants[i].begin(), octants[i].end());
    }

    return circle_cells;
  }

  //NEW NEW NEW
  //helper function for raytraceCircle to add cell postion and its 7 mirrors to the vectors
  void addCircleCells(std::vector< std::vector<geometry_msgs::Point32> >& octants, int x, int y)
  {
    //one cell in each octant
    geometry_msgs::Point32 cells[8];

    cells[0].x = x;
    cells[0].y = y;

    cells[1].x = y;
    cells[1].y = x;

    cells[2].x = -y;
    cells[2].y = x;

    cells[3].x = -x;
    cells[3].y = y;

    cells[4].x = -x;
    cells[4].y = -y;

    cells[5].x = -y;
    cells[5].y = -x;

    cells[6].x = y;
    cells[6].y = -x;

    cells[7].x = x;
    cells[7].y = -y;

    for(unsigned int i=0; i<8; i++)
    {
      cells[i].z = 0;
      octants[i].push_back(cells[i]);
    }
  }

  //function to create a lookup table of all cells inside a circle
  //returns a 2D vector with all rays necessary for all cells inside a circle
  //each ray is a vector of cells
  //lookuptable[2][3] is the relativ position of the third cell in the second ray
  std::vector< std::vector< geometry_msgs::Point32> > createLookupTableCircle(unsigned int radius_in_cells)
  {
    //get cells on the circle
    std::vector<geometry_msgs::Point32> circle = raytraceCircle(radius_in_cells);
    unsigned int number_of_rays = circle.size();

    //final lookup-table containing all rays
    std::vector< std::vector<geometry_msgs::Point32> > lookup_table_complete_circle;
    lookup_table_complete_circle.resize(number_of_rays);

    //raytrace to every cell on the circle
    for(unsigned int i=0; i<number_of_rays; i++)
    {
      lookup_table_complete_circle[i] = raytraceLine(circle[i].x,circle[i].y);
    }

    return lookup_table_complete_circle;
  }

  /* ----------------------------------- */
  /* --------- other functions --------- */
  /* ----------------------------------- */

  // function to convert a value from int to double
  double intToDouble (int in_value)
  {
    double temp;

    temp = (double (in_value)) / 10000;

    return temp;
  }


/*
  // function to convert a value from double to int. and keep value only upto map resolution
  int doubleToIntWithRes (double in_value)
  {
    int temp;

    temp = (int) ( round (in_value / pMap_->info.resolution) * 10000 );

    return temp;
  }
*/

  // function to convert a value from double to int
  int doubleToInt (double in_value)
  {
    int temp;

    temp = (int) ( in_value * 10000 );

    return temp;
  }




} // end namespace
