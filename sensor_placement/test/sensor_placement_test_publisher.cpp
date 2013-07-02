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
#include <XmlRpc.h>

// ros includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <ros/console.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_srvs/Empty.h>

// boost includes
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

//####################
//#### node class ####
class NodeClass
{
public:
  // Handle
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

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
    pnh_ = ros::NodeHandle("~");

    // initialize Publishers
    AoI_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("out_AoI_polygon", 1);
    forbidden_area_pub_ = 
      nh_.advertise<geometry_msgs::PolygonStamped>("out_forbidden_area_polygon", 1);
    PoI_pub_ = nh_.advertise<geometry_msgs::Point32>("out_PoI", 1);

    // initialize Datatypes
    AoI_poly_.polygon = loadPolygon("area_of_interest");
    AoI_poly_.header.frame_id = "/map";

    forbidden_area_poly_.polygon = loadPolygon("forbidden_area");
    forbidden_area_poly_.header.frame_id = "/map";

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

  // grabs a list of lists from the parameter server and returns a polygon
  geometry_msgs::Polygon loadPolygon(std::string polygon_param)
  {
    geometry_msgs::Polygon polygon;
    geometry_msgs::Point32 pt;
    
    // grab the polygon from the parameter server if possible
    XmlRpc::XmlRpcValue point_list;
    std::string polygon_param_name, point_list_string;
    std::vector<std::string> point_string_vector;
    if(pnh_.searchParam(polygon_param, polygon_param_name))
    {
      pnh_.getParam(polygon_param_name, point_list);
      
      // parse XmlRpc Value to std::string
      if(point_list.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        point_list_string = std::string(point_list);

        // if there's just an empty footprint up there, return
        if(point_list_string == "[]" || point_list_string == "")
          return polygon;

        // prepare the string for seperation
        boost::erase_all(point_list_string, " ");
        boost::char_separator<char> sep("[]");
        // seperate the string and put the single elements in a vector
        boost::tokenizer<boost::char_separator<char> > tokens(point_list_string, sep);
        point_string_vector = std::vector<std::string>(tokens.begin(), tokens.end());
      }

      // make sure we have a list of lists
      if( ! (point_list.getType() == XmlRpc::XmlRpcValue::TypeArray && 
             point_list.size() > 2) && 
          ! (point_list.getType() == XmlRpc::XmlRpcValue::TypeString && 
             point_string_vector.size() > 5) )
      {
        ROS_FATAL("The polygon %s must be specified as a list of lists on the parameter server, but it was specified as %s",
                  polygon_param_name.c_str(), std::string(point_list).c_str());
        throw std::runtime_error("The polygon must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
      }

      // check again first condition of if loop above.
      if(point_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for(int i = 0; i < point_list.size(); ++i)
        {
          // make sure we have a list of lists of size 2
          XmlRpc::XmlRpcValue point = point_list[i];
          if( ! (point.getType() == XmlRpc::XmlRpcValue::TypeArray &&
                 point.size() == 2))
          {
            ROS_FATAL("The polygon must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
            throw std::runtime_error("The polygon must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
          }

          // make sure that the value we're looking at is either a double or an int
          if( ! (point[0].getType() == XmlRpc::XmlRpcValue::TypeInt ||
                 point[0].getType() == XmlRpc::XmlRpcValue::TypeDouble) )
          {
            ROS_FATAL("Values in the polygon specification must be numbers");
            throw std::runtime_error("Values in the polygon specification must be numbers");
          }
          pt.x = point[0].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[0]) : (double)(point[0]);

          // make sure that the value we're looking at is either a double or an int
          if(!(point[1].getType() == XmlRpc::XmlRpcValue::TypeInt || point[1].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
            ROS_FATAL("Values in the polygon specification must be numbers");
            throw std::runtime_error("Values in the polygon specification must be numbers");
          }
          pt.y = point[1].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[1]) : (double)(point[1]);

          // push the last value to the polygon
          polygon.points.push_back(pt);

          // remove that funny representation and push it in correct form
          nh_.deleteParam(polygon_param_name);
          std::ostringstream oss;
          bool first = true;

    
          BOOST_FOREACH(geometry_msgs::Point32 p, polygon.points)
          {
            if(first)
            {
              oss << "[[" << p.x << "," << p.y << "]";
              first = false;
            }
            else
            {
              oss << ",[" << p.x << "," << p.y << "]";
            }
          }
          oss << "]";
          nh_.setParam(polygon_param_name, oss.str().c_str());
          nh_.setParam(polygon_param, oss.str().c_str());
        }
      }
      // now also check the second condition
      else if(point_list.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        geometry_msgs::Polygon polygon_spec;
        bool valid_polygon = true;
        BOOST_FOREACH(std::string t, point_string_vector)
        {
          if( t != "," )
          {
            boost::erase_all(t, " ");
            boost::char_separator<char> pt_sep(",");
            boost::tokenizer<boost::char_separator<char> > pt_tokens(t, pt_sep);
            std::vector<std::string> point(pt_tokens.begin(), pt_tokens.end());

            if(point.size() != 2)
            {
              ROS_WARN("Each point must have exactly 2 coordinates");
              valid_polygon = false;
              break;
            }

            std::vector<double> tmp_pt;
            BOOST_FOREACH(std::string p, point)
            {
              std::istringstream iss(p);
              double temp;
              if(iss >> temp)
              {
                tmp_pt.push_back(temp);
              }
              else
              {
                ROS_WARN("Each coordinate must convert to a double.");
                valid_polygon = false;
                break;
              }
            }

            if(!valid_polygon)
              break;

            geometry_msgs::Point32 pt;
            pt.x = tmp_pt[0];
            pt.y = tmp_pt[1];

            polygon_spec.points.push_back(pt);
          }
        }

        if (valid_polygon)
        {
          polygon = polygon_spec;
          nh_.setParam(polygon_param_name, point_list_string);
        }
        else
        {
          ROS_FATAL("This polygon is not valid! It must be specified as a list of lists with at least 3 points, you specified %s", point_list_string.c_str());
          throw std::runtime_error("This polygon must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
        }
      }
    }
    else
      ROS_ERROR("Did not find the parameter \"%s\" on the parameter server. Could not load Polygon!",
                polygon_param.c_str());

    return polygon;
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