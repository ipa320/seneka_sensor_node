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
 *
 * Date of creation: July 2013
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

// standard includes
#include <vector>

// ros includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <ros/console.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <std_srvs/Empty.h>


//####################
//#### node class ####
class NodeClass
{
public:
  // Handle
  ros::NodeHandle nh_;

  // Publishers
  ros::Publisher AoI_pub_;
  ros::Publisher forbidden_area_pub_;
  ros::Publisher PoI_pub_;

  // Data Types for Publishers
  geometry_msgs::PolygonStamped AoI_poly_;
  geometry_msgs::PolygonStamped forbidden_area_poly_;
  geometry_msgs::Point32 PoI_;

  // Services to trigger Publishing
  ros::ServiceServer ss_AoI_;
  ros::ServiceServer ss_forbidden_area_;
  ros::ServiceServer ss_PoI_;

  // Constructor
  NodeClass()
  {
    // intialize NodeHandle
    nh_ = ros::NodeHandle();

    // initialize Publishers
    AoI_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("out_AoI_polygon", 1);
    forbidden_area_pub_ = 
      nh_.advertise<geometry_msgs::PolygonStamped>("out_forbidden_area_polygon", 1);
    PoI_pub_ = nh_.advertise<geometry_msgs::Point32>("out_PoI", 1);

    // initialize Datatypes
    if(forbidden_area_poly_.polygon.points.empty())
    {
      geometry_msgs::Point32 p2_test;
      p2_test.x = 115;
      p2_test.y = 145;
      p2_test.z = 0;

      forbidden_area_poly_.polygon.points.push_back(p2_test);

      p2_test.x = 123;
      p2_test.y = 145;
      p2_test.z = 0;

      forbidden_area_poly_.polygon.points.push_back(p2_test);

      p2_test.x = 123;
      p2_test.y = 153;
      p2_test.z = 0;

      forbidden_area_poly_.polygon.points.push_back(p2_test);

      p2_test.x = 115;
      p2_test.y = 153;
      p2_test.z = 0;

      forbidden_area_poly_.polygon.points.push_back(p2_test);

      forbidden_area_poly_.header.frame_id = "/map";
    }

    if(AoI_poly_.polygon.points.empty())
    {
      geometry_msgs::Point32 p_test;
      p_test.x = 90;
      p_test.y = 120;
      p_test.z = 0;

      AoI_poly_.polygon.points.push_back(p_test);

      p_test.x = 210;
      p_test.y = 120;
      p_test.z = 0;

      AoI_poly_.polygon.points.push_back(p_test);

      p_test.x = 210;
      p_test.y = 200;
      p_test.z = 0;

      AoI_poly_.polygon.points.push_back(p_test);

      p_test.x = 90;
      p_test.y = 200;
      p_test.z = 0;

      AoI_poly_.polygon.points.push_back(p_test);

      AoI_poly_.header.frame_id = "/map";
    }

    PoI_.x = 100;
    PoI_.y = 100;
    PoI_.z = 0;

    // Service Initializations
    ss_AoI_ = nh_.advertiseService("publish_AoI", &NodeClass::srvCB_AoI, this);
    ss_forbidden_area_ = nh_.advertiseService("publish_forbidden_area", 
                                              &NodeClass::srvCB_forbidden_area, this);
    ss_PoI_ = nh_.advertiseService("publish_PoI", &NodeClass::srvCB_PoI, this);
  }

  // Destructor
  ~NodeClass(){};

  // service callback functions
  // publishes the respective topics
  bool srvCB_AoI(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res)
  {
    if(!AoI_poly_.polygon.points.empty())
    {
      AoI_pub_.publish(AoI_poly_);
      return true;
    }
    else
    {
      ROS_WARN("No AoI specified!");
      return false;
    }
  }

  bool srvCB_forbidden_area(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res)
  {
    if(!forbidden_area_poly_.polygon.points.empty())
    {
      forbidden_area_pub_.publish(forbidden_area_poly_);
      return true;
    }
    else
    {
      ROS_WARN("No forbidden area specified!");
      return false;
    }
  }

  bool srvCB_PoI(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res)
  {
    PoI_pub_.publish(PoI_);
    return true;
  }
}; //NodeClass

//######################
//#### main program ####
int main(int argc, char **argv)
{
  // initialize ros and specify node name
  ros::init(argc, argv, "sensor_placement_test_publisher");

  NodeClass nodeclass;

  // initialize loop rate
  ros::Rate loop_rate(1);

  while(nodeclass.nh_.ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
  }

}