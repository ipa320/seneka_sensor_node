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
 *   ROS package name: seneka_video_manager
 *
 * \author
 *   Author: Johannes Goth (cmm-jg)
 * \author
 *   Supervised by: Christophe Maufroy (cmm)
 *
 * \date Date of creation: 21.02.2013
 *
 * \brief
 *   frameManager.cpp
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

// std and own includes
#include "frameManager.h"
#include "boost_serialization_cvMat.h"
#include <vector>
#include <fstream>

// built boost includes
#include <boost/bind.hpp>

// separately to built boost includes
#include <boost/thread.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>

// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <ros/ros.h>
// openCV includes
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace io = boost::iostreams;

// public member functions
FrameManager::FrameManager() {

	// initialize parameters
	outputFolder = "/tmp/";
	binaryFilePath = outputFolder + "container";
	videoFilePath = outputFolder + "videoOnDemand.avi";
	fpv = 400;	 	// example: frame per video -> 40 sec * 10 frames = 400 frames
	fpc = 100;		// example: frames per cache -> 10 sec * 10 frames = 100 frames
	fpb = fpc;		// frames per binary
	vfr = 15;
	videoCodec = CV_FOURCC('D','I','V','X');
	binaryFileIndex = 0;
	fullVideoAvailable = false;
	createVideoActive = false;
	usingCacheA = true;
	usingCacheB = false;
	storingCacheA = false;
	storingCacheB = false;
	cacheA = new std::vector<cv::Mat>;
	cacheB = new std::vector<cv::Mat>;
	showFrame = false;
	snapshotRunning = false;
	liveStreamRunning = false;
	stateMachine = ON_DEMAND;

	for(int i=0; i < (int)(fpv/fpb)+1; i++){
		binaryFileMutexes.push_back(new boost::mutex());
	}
}

FrameManager::FrameManager(ros::NodeHandle &pnHandle) {

	// initialize configurable parameters
	int tmp_fpv, tmp_fpc, tmp_fpb, tmp_vfr;

	pnHandle.getParam("framesPerVideo", tmp_fpv);
	pnHandle.getParam("framesPerCache", tmp_fpc);
	pnHandle.getParam("framesPerBinary", tmp_fpb);
	pnHandle.getParam("videoFrameRate", tmp_vfr);

	if(!pnHandle.hasParam("framesPerVideo") || tmp_fpv<=0){
		ROS_WARN("Used default parameter for framesPerVideo [200]");
		fpv = 200;
	}
	else
		fpv=(u_int)tmp_fpv;

	if(!pnHandle.hasParam("framesPerCache") || tmp_fpc<=0){
		ROS_WARN("Used default parameter for framesPerCache [100]");
		fpc = 100;
	}
	else
		fpc=(u_int)tmp_fpc;


	if(!pnHandle.hasParam("framesPerBinary") || tmp_fpb<=0){
		ROS_WARN("Used default parameter for framesPerBinary [100]");
		fpb = 100;
	}
	else
		fpb=(u_int)tmp_fpb;

	if(!pnHandle.hasParam("videoFrameRate") || tmp_vfr<=0){
		ROS_WARN("Used default parameter for videoFrameRate [10]");
		vfr = 15;
	}
	else
		vfr=(u_int)tmp_vfr;

	if(!pnHandle.hasParam("outputFolder")){
		ROS_WARN("Used default parameter for outputFolder [/tmp]");
		outputFolder = "/tmp/";
		binaryFilePath = outputFolder + "container";
		videoFilePath = outputFolder + "videoOnDemand.avi";
	}
	else{
		pnHandle.getParam("outputFolder", outputFolder);
		binaryFilePath = outputFolder + "container";
		videoFilePath = outputFolder + "videoOnDemand.avi";
	}

	if(!pnHandle.hasParam("showFrame")){
		ROS_WARN("Used default parameter for showFrame [true]");
		showFrame = true;
	}
	else
		pnHandle.getParam("showFrame", showFrame);

	// initialize fixed parameters
	videoCodec = CV_FOURCC('D','I','V','X');
	binaryFileIndex = 0;
	fullVideoAvailable = false;
	createVideoActive = false;
	usingCacheA = true;
	usingCacheB = false;
	storingCacheA = false;
	storingCacheB = false;
	cacheA = new std::vector<cv::Mat>;
	cacheB = new std::vector<cv::Mat>;
	snapshotRunning = false;
	liveStreamRunning = false;
	stateMachine = ON_DEMAND;

	for(int i=0; i < (int)(fpv/fpb)+1; i++){
		binaryFileMutexes.push_back(new boost::mutex());
	}
}

FrameManager::~FrameManager() {
	delete cacheA;
	delete cacheB;
}

void FrameManager::processFrame(const sensor_msgs::Image& img){
	//ROS_INFO("processFrame ... ");

	cv_bridge::CvImageConstPtr cvptrS;
	try
	{
		// convert sensor_msgs::Image to cv_bridge::CvImageConstPtr
		cvptrS = cv_bridge::toCvShare(img, cvptrS, sensor_msgs::image_encodings::BGR8);
		// caching current frame into memory
		cacheFrame(cvptrS->image);

		if(stateMachine == LIVE_STREAM){
			// display current frame
			if(showFrame)
				displayFrame((cv::Mat*)&(cvptrS->image));
		}

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void FrameManager::cacheFrame(cv::Mat frame){
	//ROS_INFO("cacheFrame ... ");

	std::vector<cv::Mat>* currentCache = getCurrentCache();
	currentCache->push_back(frame);
}

std::vector<cv::Mat>* FrameManager::getCurrentCache(){
	//ROS_INFO("getCurrentCache ... ");

	// verifying the sizes of memory buffers
	// and if necessary it will schedule further tasks
	verifyCacheSize();

	if(usingCacheA == true && usingCacheB == false){
		return cacheA;
	}
	else if (usingCacheB == true && usingCacheA == false){
		return cacheB;
	}
	else{
		std::cerr << "Could not get current cache ..." << std::endl;
		return NULL;
	}
}

void FrameManager::verifyCacheSize(){
	//ROS_INFO("verifyCacheSize ... ");

	if(cacheA->size() == fpc && storingCacheA == false){
		// cacheA is full -> store frames into binary file

		storingCacheA = true;
		usingCacheA = false;
		usingCacheB = true;

		ROS_INFO("Waiting for storingCacheB");
		storingThreadB.join();
		storingThreadA = boost::thread(boost::bind(&FrameManager::storeCache, this, cacheA, &storingCacheA));
	}
	else if (cacheB->size() == fpc && storingCacheB == false){
		// cacheB is full -> store frames into binary file

		storingCacheB = true;
		usingCacheB = false;
		usingCacheA = true;

		ROS_INFO("Waiting for storingCacheA");
		storingThreadA.join();
		storingThreadB = boost::thread(boost::bind(&FrameManager::storeCache, this, cacheB, &storingCacheB));
	}

}

void FrameManager::storeCache(std::vector<cv::Mat>* cache, bool* threadActive){

	ROS_INFO("storeCache into binary file...");
	// define fileStorage-filename
	std::stringstream fileName;
	fileName << binaryFilePath << binaryFileIndex << ".bin";

	// lock current binary file as output
	binaryFileMutexes[binaryFileIndex]->lock();
	// open outputFile
	std::ofstream ofs(fileName.str().c_str(), std::ios::out | std::ios::binary);

	// scope is required to ensure archive and filtering stream buffer go out of scope
	// before stream
	{
		//		 	 //compressing frame size
		//		      io::filtering_streambuf<io::output> out;
		//		      out.push(io::zlib_compressor(io::zlib::best_speed));
		//		      out.push(ofs);

		boost::archive::binary_oarchive oa(ofs);

		// writes each frame which is stored in cache into binary file
		for (std::vector<cv::Mat>::iterator it = cache->begin() ; it != cache->end(); it++){
			// writes frame per frame into binary file, using cv::Mat serialization
			oa << *it;
		}
	}
	// close file
	ofs.close();
	// unlock current binary file
	binaryFileMutexes[binaryFileIndex]->unlock();

	if(binaryFileIndex < (fpv/fpb)-1)
		binaryFileIndex++;
	else{
		if(fullVideoAvailable == false)
			fullVideoAvailable = true;
		binaryFileIndex = 0;
	}
	// clean cache
	cache->clear();
	*threadActive = false;
	ROS_INFO("finished storingThread");
}

int FrameManager::getVideo(){

	if(stateMachine == ON_DEMAND){
		// a full video is available if the minimal count of frames is reached (minimal count = frame per video)
		if(fullVideoAvailable == true){
			// it is only possible to create one video at a time
			if(createVideoActive == false){
				// starts creating the video in a separate thread
				creatingVideoThread = boost::thread(boost::bind(&FrameManager::createVideo, this));
				return 1;
			}
			else
				return -1;	// return -1 if video creation is still active
		}
		else
			return -2;	// return -2 if no full video is available
	}
	else if(stateMachine == LIVE_STREAM){
		// videoOnDemand isn't available in LIVE_STREAM state
		ROS_WARN("This function is in LIVE_STREAM state not available!");
		return -3;
	}
	else{
		ROS_ERROR("Unknown state for the state machine");
		return 0;
	}


}

int FrameManager::createVideo(){

	ROS_INFO("createVideo ...");

	createVideoActive = true;

	// create a videoRecorder instance
	VideoRecorder* vRecoder = new VideoRecorder(vfr, videoCodec);
	bool firstFrame = true;

	/* start binary for video creation
	 * selecting binaryFileIndex + 1 to get the oldest binary file,
	 * which includes the first frame for the video creation
	 */
	int currentIndex = binaryFileIndex + 1;
	int numBinaries = fpv/fpb;

	// add all binaries which are required (numBinaries) into a video
	for(int i=0; i < numBinaries; i++){
		std::stringstream inputFileName;
		int mutexID;

		if(currentIndex < numBinaries){
			// load currentIndex - numBinaries-1
			inputFileName << binaryFilePath << (currentIndex) << ".bin";
			mutexID = currentIndex;
			currentIndex++;

		}
		else{
			// load currentIndex - numBinaries-1
			inputFileName << binaryFilePath << (currentIndex-numBinaries) << ".bin";
			mutexID = currentIndex-numBinaries;
			currentIndex++;
		}

		// lock current binary file as input
		binaryFileMutexes[mutexID]->lock();
		// open inputFile
		std::ifstream ifs(inputFileName.str().c_str(), std::ios::in | std::ios::binary);

		// scope is required to ensure archive and filtering stream buffer go out of scope
		// before stream
		{
			//			// decompressing frame size
			//			io::filtering_streambuf<io::input> in;
			//			in.push(io::zlib_decompressor());
			//			in.push(ifs);

			boost::archive::binary_iarchive ia(ifs);

			bool hasContent = true;
			while (hasContent)
			{
				cv::Mat loadedFrame;
				if(firstFrame){
					// try to read image from binary file
					hasContent = boost::serialization::try_stream_next(ia, ifs, loadedFrame);
					if (hasContent == true){
						vRecoder->createVideo(videoFilePath, loadedFrame.cols, loadedFrame.rows);
						// add frame to video
						vRecoder->addFrame(loadedFrame);
						firstFrame = false;

						// display current frame
						if(showFrame)
							displayFrame(&loadedFrame);
					}
				}
				else{
					// try to read image from binary file
					hasContent = boost::serialization::try_stream_next(ia, ifs, loadedFrame);
					// add frame to video
					if (hasContent == true){
						vRecoder->addFrame(loadedFrame);

						// display current frame
						if(showFrame)
							displayFrame(&loadedFrame);
					}
				}
			}
			ifs.close();
			// unlock current binary file
			binaryFileMutexes[mutexID]->unlock();
		}
	}
	// release video
	vRecoder->releaseVideo();
	createVideoActive = false;
	ROS_INFO("finished createVideo ...");

	return -1;
}

void FrameManager::displayFrame(cv::Mat* mat){
	cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
	cv::imshow( "Display window", *mat );
	cv::waitKey(1);
}

void FrameManager::startSnapshots(int interval){

	if(!isSnapShotRunning()){
		snapshotThread = boost::thread(boost::bind(&FrameManager::createSnapshots, this, interval));
		snapshotRunning = true;
	}
}

void FrameManager::stopSnapshots(){
	// call interrupt point of snapshotThread
	snapshotThread.interrupt();
}

void FrameManager::createSnapshots(int interval){
	ROS_INFO("Starting creating snapshots ...");
	while(true){

		std::stringstream imgFile;
		std::vector<cv::Mat>* currentCache = getCurrentCache();

		if(currentCache->size() > 0){
			cv::Mat img = currentCache->at(currentCache->size()-1);

			// file name and path to the image files
			// file name is the current system time stamp
			imgFile << outputFolder << ros::Time::now() << ".jpg";

			// create JPEG image
			cv::imwrite(imgFile.str(), img);

			// display current frame
			if(showFrame && stateMachine != LIVE_STREAM)
				displayFrame(&img);

			// try to set thread to sleep or if an interrupt occurred stop thread
			try{
				// set thread sleeping for the chosen interval
				boost::this_thread::sleep(boost::posix_time::milliseconds(interval*1000));
			}
			catch(boost::thread_interrupted const& )
			{
				// defined actions if an interrupt occurred
				snapshotRunning = false;
				ROS_INFO("Stopped creating snapshots ...");
				break;
			}
		}
	}
}

void FrameManager::startLiveStream(){
	ROS_INFO("Starting live stream mode ...");

	if(createVideoActive){
		ROS_WARN("State change not possible ... creating video at the moment !!");
	}
	else{
		// initialize the LIVE_STREAM state
		stateMachine = LIVE_STREAM;
		liveStreamRunning = true;

		// TODO: Impl. video coding for live stream
	}
}

void FrameManager::stopLiveStream(){
	ROS_INFO("Stopped live stream mode ...");

	stateMachine = ON_DEMAND;
	liveStreamRunning = false;
}


