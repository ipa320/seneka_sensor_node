/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: SENEKA
 * \note
 *   ROS stack name: SENEKA
 * \note
 *   ROS package name: termoVideoManager
 *
 * \author
 *   Author: Johannes Goth (cmm-jg)
 * \author
 *   Supervised by: Christophe Maufroy (cmm)
 *
 * \date Date of creation: 21.02.2013
 *
 * \brief
 *   frameManager.h
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
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

#ifndef FRAMEMANAGER_H_
#define FRAMEMANAGER_H_


// project files
#include "frameManager.h"
#include "videoRecorder.h"
#include "videoRecorder.cpp"
// libraries
#include <boost/thread.hpp>
#include <vector>
// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "ImageBuilder.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class FrameManager {
public:
	// public member functions
	FrameManager();
	FrameManager(ros::NodeHandle &nHandler);
	virtual ~FrameManager();
	void processFrame(const sensor_msgs::Image& img);
	int getVideo();
	bool isSnapShotRunning(){return snapshotRunning;};
	void startSnapshots(int interval);
	void stopSnapshots();
	bool isLiveStreamRunning(){return liveStreamRunning;};
	void startLiveStream();
	void stopLiveStream();

private:

	// private member functions
	void cacheFrame(sensor_msgs::Image frame);
	void verifyCacheSize();
	void storeCache(std::vector<sensor_msgs::Image>* cache, bool* threadActive);
	int createVideo();
	std::vector<sensor_msgs::Image>* getCurrentCache();
	std::vector<cv::Mat>* getCurrentLiveStreamCache();
	void storeFrame(sensor_msgs::Image frame);
	void displayFrame(cv::Mat* mat);
	cv::Mat convertTemperatureValuesToRGB(sensor_msgs::Image* frame, unsigned int* frameCount);
	void createSnapshots(int interval);

	// state machine
	enum states {ON_DEMAND, LIVE_STREAM};
	int stateMachine;

	// cache attributes and references (memory buffers)
	u_int fpv;			// frames per video
	u_int fpc; 			// frames per cache
	u_int fpb; 			// frames per binary
	bool fullVideoAvailable;
	bool createVideoActive;
	bool usingCacheA;
	bool usingCacheB;
	std::vector<sensor_msgs::Image>* cacheA;
	std::vector<sensor_msgs::Image>* cacheB;

	// live stream specific
	std::vector<cv::Mat>* cacheC;
	std::vector<cv::Mat>* cacheD;
	bool usingCacheC;
	bool usingCacheD;
	bool storingCacheC;
	bool storingCacheD;
	boost::thread storingThreadC;
	boost::thread storingThreadD;
	unsigned int liveStreamFrameCount;

	// file storage parameters
	std::string binaryFilePath;
	std::string videoFilePath;
	std::string outputFolder;
	u_int binaryFileIndex;
	std::vector<boost::mutex*> binaryFileMutexes;

	// termo-to-rgb converter
	optris::ImageBuilder iBuilder;
	bool showFrame;

	// output video parameters
	u_int vfr;			// video frame rate
	int videoCodec; 	// Codec for video coding eg. CV_FOURCC('D','I','V','X')

	// threads parameters
	boost::thread storingThreadA;
	boost::thread storingThreadB;
	bool storingCacheA;
	bool storingCacheB;
	boost::thread creatingVideoThread;
	boost::thread cachingThread;
	boost::thread snapshotThread;
	bool snapshotRunning;
	bool liveStreamRunning;
};

#endif /* FRAMEMANAGERH_ */
