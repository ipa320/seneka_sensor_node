/*
 * frameManager.cpp
 *
 *  Created on: 21.02.2013
 *      Author: Johannes Goth (cmm-jg)
 */


// own stuff
#include "frameManager.h"
// libraries
#include <boost/thread.hpp>
//#include <boost/thread/mutex.hpp>
#include <vector>
//#include <thread>
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


//TODO: delete the following methods after develope phase

void threadTestFkt(cv::Mat frame){
	// TODO: empty - has to be deleted after testing
}

void frameManager::testFrameManager(){
	ROS_INFO("testing ... frameManager ...!!");
}

// public methods
frameManager::frameManager() {

	// initialize variables
	maxCacheSize = 100; // 25fps * 30sec = 750frames
	maxRecTime = 2;
	currentFileStorage = 0;
	currentFileStorageSize = 0;
	useCacheA = true;
	useCacheB = false;
	cacheA = new std::vector<cv::Mat>;
	cacheB = new std::vector<cv::Mat>;
//	mxCacheA.unlock();
//	mxCacheB.unlock();

	// create fileStorages at hard disk drive
	for(int i=0; i< (maxRecTime/0.5); i++){

		std::stringstream fileName;
		fileName << "/home/cmm-jg/Bilder/store" << i << ".yml";

		cv::FileStorage* tmp = new cv::FileStorage();
		tmp->open(fileName.str(), cv::FileStorage::WRITE);

		fileStorages.push_back(tmp);
	}

//	access to a file storage is possible via a file node
//	cv::FileNode* n = new cv::FileNode();
//	n->size();

}

frameManager::~frameManager() {
	delete cacheA;
	delete cacheB;
}

void frameManager::processFrame(const sensor_msgs::Image& img){

	ROS_INFO("processFrame ... ");

	cv_bridge::CvImageConstPtr cvptrS;
	try
	{
		// convert the sensor_msgs::Image to cv_bridge::CvImageConstPtr
		cvptrS = cv_bridge::toCvShare(img, cvptrS, sensor_msgs::image_encodings::BGR8);

//		caching frame in separate thread
//		boost::thread tCacheFrame(this->cacheFrame(cvptrS->image);
//		boost::thread* thr = new boost::thread(boost::bind(&frameManager::cacheFrame, this), cvptrS->image);

		cacheFrame(cvptrS->image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void frameManager::cacheFrame(cv::Mat frame){

	ROS_INFO("cacheFrame ... ");

	std::vector<cv::Mat>* currentCache = getCurrentCache();
	currentCache->push_back(frame);
}

std::vector<cv::Mat>* frameManager::getCurrentCache(){

	ROS_INFO("getCurrentCache ... ");

	// checks the sizes of memory buffers and if necessary it will schedule further tasks
	verifyCacheSize();

	if(useCacheA == true && useCacheB == false){
		return cacheA;
	}
	else if (useCacheB == true && useCacheA == false){
		return cacheB;
	}
	else{
		std::cerr << "Could not get current cache ..." << std::endl;
		return NULL;
	}
}
/*
boost::mutex* frameManager::getMutexToCache(bool usedCache){
	if(usedCache){
		if(useCacheA == true && useCacheB == false){
			return &mxCacheA;
		}
		else if (useCacheB == true && useCacheA == false){
			return &mxCacheB;
		}
	}else{
		if(useCacheA == false && useCacheB == true){
			return &mxCacheA;
		}
		else if (useCacheB == false && useCacheA == true){
			return &mxCacheB;
		}
	}
	std::cerr << "ERROR in getMutexToCache()" << std::endl;
	return NULL;
}
*/

void frameManager::verifyCacheSize(){
	ROS_INFO("verifyCacheSize ... ");

	if(cacheA->size() == maxCacheSize){// && cacheB->size() < maxCacheSize){
		useCacheA = false;
		useCacheB = true;

		// cacheA is full -> store frame to fileStorage
		// TODO: should be done by a separate thread
		storeCache(cacheA);
	}
	else if (cacheB->size() == maxCacheSize){// && cacheA->size() < maxCacheSize){
		useCacheB = false;
		useCacheA = true;

		// cacheB is full -> store frame to fileStorage
		// TODO: should be done by a separate thread
		storeCache(cacheB);
	}

}

void frameManager::storeCache(std::vector<cv::Mat>* cache){

	ROS_INFO("storeCache ... ");

	while(currentFileStorageSize != cache->size()){
		std::stringstream elementName;
		elementName << "cvMat" << currentFileStorageSize;
		// store frame to the current file storage
		*fileStorages[currentFileStorage] << elementName.str() << cache->back();
		cache->pop_back();

		currentFileStorageSize++;
	}
	// reset currentFileStorageSize
	currentFileStorageSize = 0;

	// check if max. record time is reached
	if(currentFileStorage <= maxRecTime/0.5)
		currentFileStorage++;
	else
		currentFileStorage = 0;

	// muss der cache (ringbuffer) geleert (delete) werden bzw. neu initialisiert werden
	delete cache;
	cache = new std::vector<cv::Mat>;
}

int frameManager::getVideo(){
	return -1;
}

int frameManager::createVideo(){
	return -1;
}
