/*!
*****************************************************************
* seneka_control_interface.cpp
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_control_interface
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
* 
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
* Modified xx/20xx: 
*
* Description:
* The seneka_control_interface package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
* This package might work with other hardware and can be used for other purposes, 
* however the development has been specifically for this project and the deployed sensors.
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/

#include <ros/ros.h>
#include <seneka_socketcan/SocketCAN.h>
#include <seneka_laser_scan/SenekaLaserScan.h>
#include <seneka_leg/SenekaLeg.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <seneka_srv/JointTrajectory.h>

#include <iostream>
#include <boost/thread/mutex.hpp>

using namespace std;

/*********************************************/
/*************** main function ***************/
/*********************************************/

class ControlNode {
	ros::NodeHandle nh_;
	ros::Publisher pub_joints_, pub_diag_;
	std::vector<boost::shared_ptr<ros::Publisher> > pub_btns_;
	ros::Subscriber sub_joint_path_command_;///< subscriber for a trajectory
	ros::ServiceServer srv_joint_path_command_;
	sensor_msgs::JointState joint_state_;
	
	std::vector<SenekaGeneralCANDevice*> devices_;
	std::vector<bool> btns_;
	
	double max_tolerance_;
	boost::mutex lock_;
	
	/// reorder a vector with a given list of indices, if not possible returns false
	template< class T >
	bool _reorder(std::vector<T> &v, std::vector<size_t> const &order )  {
		std::vector<T> cp(order.size());	//not efficient but we have only small vectors...
		for(size_t i=0; i<order.size(); i++)
			if(order[i]<v.size()) cp[i] = v[order[i]];
			else return false;
		v = cp;
		return true;
	}
	
	bool reorderMsg(trajectory_msgs::JointTrajectory &traj) {
		//build map
		std::vector<size_t> int2ext(joint_state_.name.size());
		size_t found=0;
		for(size_t j=0; j<joint_state_.name.size(); j++) {
			for(size_t i=0; i<traj.joint_names.size(); i++)
				if(joint_state_.name[j]==traj.joint_names[i]) {
					int2ext[j] = i;
					++found;
					break;
				}
			if(found!=j+1) ROS_INFO("missing joint name: %s", joint_state_.name[j].c_str());
		}
				
		if(found!=joint_state_.name.size()) {
			ROS_ERROR("missing joint names in trajectory");
			return false;
		}
		
		bool r = true;
		for(size_t i=0; i<traj.points.size(); i++) {
			r &= _reorder(traj.points[i].positions, int2ext);
			_reorder(traj.points[i].velocities, int2ext);
			_reorder(traj.points[i].accelerations, int2ext);
			
			if(traj.points[i].positions.size()!=found)
				r = false;
		}
		
		return r;
	}
public:
	
	ControlNode() {
		pub_joints_  = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
		pub_diag_ = nh_.advertise<diagnostic_msgs::DiagnosticArray> ("/diagnostics", 1);
		sub_joint_path_command_ = nh_.subscribe("joint_path_command", 1, &ControlNode::cb_joint_path_command, this);
		srv_joint_path_command_ = nh_.advertiseService("joint_path_command", &ControlNode::cb_joint_path_srv, this);
		
		ros::NodeHandle pnh("~");	//parameter lookup in local namespace
		pnh.param<double>("max_tolerance", max_tolerance_, 0.01);
	}
	
	void add(SenekaGeneralCANDevice &dev, const std::string &name) {
		ros::NodeHandle pnh("~");	//parameter lookup in local namespace
		
		int can_id;
		if(!pnh.getParam(name+"/can_id", can_id)) {
			ROS_ERROR("could not read parameter %s/can_id\nwill NOT connect device!", name.c_str() );
			return;
		}
		
		lock_.lock();
		devices_.push_back(&dev);
		dev.init(can_id);
		dev.setUpdateCallback((int)joint_state_.position.size(), boost::bind(&ControlNode::update_joint, this, _1, _2) );
		pub_btns_.push_back( boost::shared_ptr<ros::Publisher>() );
		btns_.push_back( false );
		
		std::string btn;
		if(pnh.getParam(name+"/button", btn)) {
			dev.setUpdateCallbackButton(boost::bind(&ControlNode::update_button, this, _1, _2) );
			pub_btns_.back().reset( new ros::Publisher(nh_.advertise<std_msgs::Bool>(btn, 10)) );
		}
		
		for(size_t i=0; i<dev.getNumJoints(); i++) {
			const std::string fullname = name+(dev.getNumJoints()>1?boost::lexical_cast<std::string>(i):"");
			joint_state_.position.push_back(0.);
			joint_state_.name.push_back(fullname);
			
			double off, fact;
			pnh.param<double>(fullname+"/offset", off, 0.);
			pnh.param<double>(fullname+"/factor", fact, 1.);
			dev.setCorrection(i, off, fact);
		}
		lock_.unlock();
	}
	
	bool cb_joint_path_srv(seneka_srv::JointTrajectory::Request  &req, seneka_srv::JointTrajectory::Response &res)
	{
		res.success = _cb_joint_path_command(req.traj, req.check_switch);
		return true;
	}
	
	void cb_joint_path_command(const trajectory_msgs::JointTrajectory &jt)
	{
		_cb_joint_path_command(jt, seneka_srv::JointTrajectory::Request::_check_switch_type());
	}
	
	bool _cb_joint_path_command(const trajectory_msgs::JointTrajectory &jt, const seneka_srv::JointTrajectory::Request::_check_switch_type &check_switch)
	{
		for(size_t p=0; p<jt.points.size(); p++) {
			if(jt.joint_names.size()!=jt.points[p].positions.size()) {
				ROS_ERROR("mismatching size");
				return false;
			}
			
			//set target
			for(size_t i=0; i<jt.points[p].positions.size(); i++) {
				size_t j = std::distance(joint_state_.name.begin(), std::find_if(joint_state_.name.begin(), joint_state_.name.end(), std::bind2nd(std::equal_to<std::string>(), jt.joint_names[i])));
				if(j>=joint_state_.name.size()) continue;
				
				for(size_t d=0; d<devices_.size(); d++) {
					if(j<devices_[d]->getNumJoints()) {
						devices_[d]->setTarget(j, jt.points[p].positions[i]);
						break;
					}
					j-=devices_[d]->getNumJoints();
				}
			}
			
			//wait until reached or timeout
			bool ok = false, check_was_ok = true;
			ros::Rate rate(30);
			ros::Time end = ros::Time::now()+ros::Duration(jt.points[p].time_from_start);
			while( jt.points[p].time_from_start<ros::Duration(0) || ros::Time::now()<end ) {
				bool out = false;
				for(size_t i=0; i<jt.points[p].positions.size(); i++) {
					const size_t j = std::distance(joint_state_.name.begin(), std::find_if(joint_state_.name.begin(), joint_state_.name.end(), std::bind2nd(std::equal_to<std::string>(), jt.joint_names[i])));
					if(j>=joint_state_.name.size()) continue;
					
					bool check = true;
					boost::mutex::scoped_lock lock(lock_);						
					if(p<check_switch.size() && check_switch[p]) {
						size_t jj=j;
						for(size_t d=0; d<devices_.size(); d++) {
							if(jj<devices_[d]->getNumJoints()) {
								check_was_ok = false;
								if(btns_[d]) {
									devices_[d]->setTarget(jj, joint_state_.position[j]);
									check = false;
									check_was_ok = true;
								}
								break;
							}
							jj-=devices_[d]->getNumJoints();
						}
					}
					
					if(check && std::abs(joint_state_.position[j]-jt.points[p].positions[i])>max_tolerance_)
						out = true;
				}
				
				if(!out) {
					ok = true;
					break;
				}
				rate.sleep();
			}
			
			if(!ok) {
				ROS_ERROR("timeout");
				return false;
			}
			if(!check_was_ok) {
				ROS_ERROR("check did not succeed");
				return false;
			}
		}
		
		return true;
	}
	
	void publishState() {
		// publishing diagnotic messages
		diagnostic_msgs::DiagnosticArray diagnostics;
		diagnostics.status.resize(joint_state_.name.size());

		// set data to diagnostics
		size_t j=0;
		for(size_t i=0; i<devices_.size(); i++) {
			for(size_t k=0; k<devices_[i]->getNumJoints(); k++) {
				diagnostics.status[j].level = devices_[i]->error()?2:1;
				diagnostics.status[j].name = joint_state_.name[j];
				diagnostics.status[j].message = devices_[i]->error()?"error":"ok";
				++j;
			}
		}
		
		// publish diagnostic message
		pub_diag_.publish(diagnostics);
	}
	
	void update_joint(const int id, const double val) {
		if(id<0 || id>=(int)joint_state_.position.size())
			return;
			
		boost::mutex::scoped_lock lock(lock_);
		joint_state_.position[id] = val;
		if(pub_joints_.getNumSubscribers()>0) pub_joints_.publish(joint_state_);
	}
	
	void update_button(const int id, const bool val) {
		if(id<0 || id>=(int)pub_btns_.size() || !pub_btns_[id])
			return;
			
		if(pub_btns_[id]->getNumSubscribers()>0) {
			std_msgs::Bool msg;
			msg.data = val;
			pub_btns_[id]->publish(msg);
		}
		boost::mutex::scoped_lock lock(lock_);
		btns_[id] = val;
	}
};

int main(int argc, char *argv[]) {

  // ROS initialization; apply "seneka_control_interface" as node name;
  ros::init(argc, argv, "seneka_control_interface");
  ros::NodeHandle nh;

  SenekaLaserScan laser_scan;
  SenekaLeg    turret;
  SenekaLeg      tilt;
  SenekaLeg       leg1;
  SenekaLeg       leg2;
  SenekaLeg       leg3;
  
  ControlNode node;
  
  node.add(turret, "turret");
  node.add(tilt, "tilt");
  node.add(leg1, "leg1");
  node.add(leg2, "leg2");
  node.add(leg3, "leg3");
  
  ros::Rate rate(20);
  while(ros::ok()) {
	  node.publishState();
	  ros::spinOnce();
	  rate.sleep();
  }

  return 0;

}

/********************************************/
/********************************************/
/********************************************/
