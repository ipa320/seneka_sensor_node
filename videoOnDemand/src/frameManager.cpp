/*
 * frameManager.cpp
 *
 *  Created on: 21.02.2013
 *      Author: Johannes Goth (cmm-jg)
 */

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

//TODO: delete the following methods after develope phase

void frameManager::threadTestFkt(){
	// TODO: empty - has to be deleted after testing
	ROS_INFO("testing ... threadTestFkt ...!!");
}

void frameManager::testFrameManager(){
	ROS_INFO("testing ... frameManager ...!!");
}

// public methods
frameManager::frameManager() {

	// initialize parameters
	path = "/home/cmm-jg/Bilder/store";
	fileNodeLable = "cvMat";
	maxCacheSize = 200; // 25fps * 30sec = 750frames
	maxRecTime = 2;
	binaryFileIndex = 0;
	usingCacheA = true;
	usingCacheB = false;
	storingCacheA = false;
	storingCacheB = false;
	cacheA = new std::vector<cv::Mat>;
	cacheB = new std::vector<cv::Mat>;
	binaryFileConst = 4;

	for(int i=0; i < (binaryFileConst); i++){
		binaryFileMutexes.push_back(new boost::mutex());
	}
}

frameManager::~frameManager() {
	delete cacheA;
	delete cacheB;
}

void frameManager::processFrame(const sensor_msgs::Image& img){

	//ROS_INFO("processFrame ... ");

	cv_bridge::CvImageConstPtr cvptrS;
	try
	{
		// convert the sensor_msgs::Image to cv_bridge::CvImageConstPtr
		cvptrS = cv_bridge::toCvShare(img, cvptrS, sensor_msgs::image_encodings::BGR8);

		// TODO: thread concept for calling cacheFrame()-method
		cacheFrame(cvptrS->image);
		//storeFrame(cvptrS->image);

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void frameManager::cacheFrame(cv::Mat frame){

	//ROS_INFO("cacheFrame ... ");

	std::vector<cv::Mat>* currentCache = getCurrentCache();

	//tCaching.join();
	//tCaching = boost::thread(boost::bind(&std::vector<cv::Mat>::push_back, currentCache, frame));
	currentCache->push_back(frame);
}

std::vector<cv::Mat>* frameManager::getCurrentCache(){

	//ROS_INFO("getCurrentCache ... ");

	// checks the sizes of memory buffers and if necessary it will schedule further tasks
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

void frameManager::verifyCacheSize(){

	//ROS_INFO("verifyCacheSize ... ");

	if(cacheA->size() == maxCacheSize && storingCacheA == false){// && cacheB->size() < maxCacheSize){
		// cacheA is full -> store frame to fileStorages

		storingCacheA = true;
		usingCacheA = false;
		usingCacheB = true;

		ROS_INFO(">-> waiting for storingCacheB");
		storingThreadB.join();
		storingThreadA = boost::thread(boost::bind(&frameManager::storeCache, this, cacheA, &storingCacheA));

//		storeCache(cacheA);
	}
	else if (cacheB->size() == maxCacheSize && storingCacheB == false){// && cacheA->size() < maxCacheSize){
		// cacheB is full -> store frame to fileStorage

		storingCacheB = true;
		usingCacheB = false;
		usingCacheA = true;

		ROS_INFO(">-> waiting for storingCacheA");
		storingThreadA.join();

		// TODO: 	testweiser Aufruf der createVideo() methode nach 3 Durchläufen
		if(binaryFileIndex > 2)
			creatingVideoThread = boost::thread(boost::bind(&frameManager::createVideo, this));

		storingThreadB = boost::thread(boost::bind(&frameManager::storeCache, this, cacheB, &storingCacheB));

//		storeCache(cacheB);
	}
}

void frameManager::storeCache(std::vector<cv::Mat>* cache, bool* threadActive){

	ROS_INFO(">->-> storeCache into binary file...");

	// define fileStorage-filename
	std::stringstream fileName;
	fileName << path << binaryFileIndex << ".bin";

    std::ofstream ofs(fileName.str().c_str(), std::ios::out | std::ios::binary);

    // scope is required to ensure archive and filtering stream buffer go out of scope
    // before stream
    {
    	// compressing frame size
//      io::filtering_streambuf<io::output> out;
//      out.push(io::zlib_compressor(io::zlib::best_speed));
//      out.push(ofs);

        boost::archive::binary_oarchive oa(ofs);

        // writes each frame which is stored in cache into binary file
    	for (std::vector<cv::Mat>::iterator it = cache->begin() ; it != cache->end(); it++){

    		// write frame into binary file, using cv:Mat serialization
    		oa << *it;
    	}
    }

    // close file
    ofs.close();

    // current const. allocation -> has to be adapted dynamic
    if(binaryFileIndex < binaryFileConst)
    	binaryFileIndex++;
    else
    	binaryFileIndex = 0;

	// clean cache
	cache->clear();
	*threadActive = false;
	ROS_INFO(">->->-> finished thread");
}

int frameManager::getVideo(){


	createVideo();

	return -1;
}

int frameManager::createVideo(){

	ROS_INFO("createVideo ...");

	// TODO: 	video kann erzeugt werden, momentan aus einem input file
	// 			muss erweitert werden auf bsp 4 input files, je nach gewünschter video länge

	std::stringstream inputFileName;
	inputFileName << path << (binaryFileIndex-1) << ".bin";
	std::stringstream outputFileName;
	outputFileName << "/home/cmm-jg/Bilder/videoOnDemand.avi";

	// open inputFile

	cv::Mat loadedFrame;
    std::ifstream ifs(inputFileName.str().c_str(), std::ios::in | std::ios::binary);

    // scope is required to ensure archive and filtering stream buffer go out of scope
    // before stream
    {
    	// decompressing frame size
//    	io::filtering_streambuf<io::input> in;
//    	in.push(io::zlib_decompressor());
//    	in.push(ifs);

    	boost::archive::binary_iarchive ia(ifs);

    	// create a videoRecorder instance
    	videoRecorder* vRecoder = new videoRecorder();

    	bool cont = true, firstFrame = true, showFrame = false;
    	while (cont)
    	{
    		//std::cout << "loading image from binary file ... "<< std::endl;
    		if(cont && firstFrame){
    			cont = boost::serialization::try_stream_next(ia, ifs, loadedFrame);
    			// define the video parameters
    			vRecoder->createVideo(outputFileName.str(), loadedFrame.cols, loadedFrame.rows);
    			firstFrame = false;
    		}
    		else if(cont){
    			cont = boost::serialization::try_stream_next(ia, ifs, loadedFrame);
    			vRecoder->addFrame(loadedFrame);
    		}

    		if(cont && showFrame){
    			std::cout << "loading image from binary file ... "<< std::endl;
        	    cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
        	    cv::imshow( "Display window", loadedFrame );
        	    cv::waitKey(0);
    		}
    	}
    	vRecoder->releaseVideo();
    	ROS_INFO("finished createVideo ...");
    }
    ifs.close();


	return -1;
}
