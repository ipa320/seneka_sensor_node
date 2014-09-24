/*!
*****************************************************************
* SenekaTilt.h
*
* Copyright (c) 2013
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
* Repository name: seneka_sensor_node
*
* ROS package name: seneka_tilt
*
* Author: Thorsten Kannacher, E-Mail: Thorsten.Andreas.Kannacher@ipa.fraunhofer.de
* 
* Supervised by: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
*
* Date of creation: Jun 2014
* Modified xx/20xx: 
*
* Description:
* The seneka_tilt package is part of the seneka_sensor_node metapackage, developed for the SeNeKa project at Fraunhofer IPA.
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

#pragma once

#include <seneka_socketcan/SocketCAN.h>
#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
//#include <boost/atomic.hpp>

class SenekaGeneralCANDevice {
	SenekaGeneralCANDevice(const SenekaGeneralCANDevice&);
  protected:
	typedef boost::function<void(const struct can_frame&)> callback;

    SenekaGeneralCANDevice(const size_t num_joints=1, const std::string &can_interface = "can0") :
    error_(false), joint_id_(-1), can_id_(-1), num_joints_(num_joints), corr_(num_joints) {
	  // open socket for CAN communication; respect optionally given differing CAN interface;
	  if(!SocketCAN::openRAW(socket_, can_interface))
		error_ = true;
	  
	  thread_.reset(new boost::thread(boost::ref(*this)));
	}
    
    void addListener(const int can_id, const callback &cb) {
		struct can_filter f;
		f.can_id = can_id_ = can_id;
		f.can_mask = CAN_SFF_MASK;
		
		lock_.lock();
		filters_.push_back(f);
		cbs_.push_back(cb);
		lock_.unlock();
		
		SocketCAN::setFilter(socket_, &filters_[0], (int)filters_.size());
	}

    struct can_frame readFrame() const {
	  struct can_frame frame;
	  SocketCAN::readRAW(socket_, frame);
	  return frame;
	}
	
    struct can_frame fillFrame(const int can_id, const int can_dlc=8) const {
	  struct can_frame frame = {};

	  frame.can_id  = can_id;
	  frame.can_dlc = can_dlc; // count of data bytes;
	  
	  return frame;
	}
	
	bool sendFrame(const struct can_frame & frame) const {
		const bool r = SocketCAN::writeRAW(socket_, frame);
		if(!r) error_ = true;
		return r;
	}
	
	void updated(double val, const size_t joint=0) {
		if(joint>=num_joints_) return;
		
		val = (val+corr_[joint].corr_offset_) * corr_[joint].corr_factor_;
		
		lock_.lock();
		const int id = joint_id_+(int)joint;
		update_callback tmp = update_cb_;
		lock_.unlock();
		
		tmp(id, val);
	}
	
	void updated(bool val) {
		lock_.lock();
		const int id = joint_id_;
		update_callback tmp = update_cb_btn_;
		lock_.unlock();
		
		tmp(id, val);
	}
	
	virtual void _setTarget(const int joint, const double val) = 0;
	
  public:
  
	typedef boost::function<void(const int, const double)> update_callback;
	typedef boost::function<void(const int, const bool)> update_callback_btn;
	
	void setUpdateCallback(const int joint_id, const update_callback &cb) {
		lock_.lock();
		joint_id_ = joint_id;
		update_cb_ = cb;
		lock_.unlock();
	}
	
	void setUpdateCallbackButton(const update_callback_btn &cb) {
		lock_.lock();
		update_cb_btn_ = cb;
		lock_.unlock();
	}
  
    virtual ~SenekaGeneralCANDevice() {
		running_ = false;
		if(thread_) thread_->join();
	}
	
	void setCorrection(const size_t joint, const double off, const double fact) {
		 if(joint>=num_joints_) return;
		
		 corr_[joint].corr_offset_ = off;
		 corr_[joint].corr_factor_ = fact;
	}
	
	void operator()() {
		struct can_frame frame;
		running_ = true;
		
		while(running_) {
			SocketCAN::readRAW(socket_, frame);
			
			lock_.lock();
			for(size_t i=0; i<filters_.size(); i++) {
				if(filters_[i].can_id == frame.can_id) {
					callback tmp = cbs_[i];
					lock_.unlock();
					tmp(frame);
					lock_.lock();
				}
			}
			lock_.unlock();
		}
	}
	
	virtual void init(const int can_id) = 0;
	virtual void setTarget(const int joint, double val) {
		if(joint>=num_joints_) return;
		
		val = (val/corr_[joint].corr_factor_)-corr_[joint].corr_offset_;		
		_setTarget(joint, val);
	}
		
	int getNumJoints() const {return num_joints_;}
	int getCanID() const {return can_id_;}
	bool error() const {return error_/*.load()*/;}

  private:

    int socket_; // socket for CAN communication;
    //mutable boost::atomic<bool> error_;
    mutable bool error_;
    std::vector<struct can_filter> filters_;
    std::vector<callback> cbs_;
    
    boost::mutex lock_;
    boost::shared_ptr<boost::thread> thread_;
    bool running_;
    
    //comm funcs
    int joint_id_, can_id_;
    size_t num_joints_;
    update_callback update_cb_;
    update_callback_btn update_cb_btn_;
    
    struct SCorrection {
		double corr_offset_, corr_factor_;
		SCorrection(): corr_offset_(0.), corr_factor_(1.) {}
	};
	std::vector<SCorrection> corr_;
};
