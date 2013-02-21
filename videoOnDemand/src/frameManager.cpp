/*
 * frameManager.cpp
 *
 *  Created on: 21.02.2013
 *      Author: cmm-jg
 */

#include "frameManager.h"
#include <vector>

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

frameManager::frameManager() {

	// initialize variables
	maxCacheSize = 750; // 25fps * 30sec = 750frames
	useCacheA = true;
	useCacheB = false;
	cacheA = new std::vector<cv::Mat>;
	cacheB = new std::vector<cv::Mat>;

}

frameManager::~frameManager() {
	delete cacheA;
	delete cacheB;
}

void frameManager::startCaching(){
	// TODO: can be deleted
}

void frameManager::cacheFrame(cv::Mat frame){

	std::vector<cv::Mat>* currentCache = getCurrentCache();
	currentCache->push_back(frame);
}

std::vector<cv::Mat>* frameManager::getCurrentCache(){

	// checks the sizes of memory buffers and if necessary it will schedule further tasks
	verifyCacheSize();

	if(useCacheA == true && useCacheB == false){
		return cacheA;
	}
	else if (useCacheB == true && useCacheA == false){
		return cacheB;
	}
	return NULL;
}

void frameManager::verifyCacheSize(){
	if(cacheA->size() >= maxCacheSize && cacheB->size() < maxCacheSize){
		useCacheA = false;
		useCacheB = true;

		// call storage method for cacheA
	}
	else if (cacheB->size() >= maxCacheSize && cacheA->size() < maxCacheSize){
		useCacheB = false;
		useCacheA = true;

		// call storage method for cacheB
	}

}

void frameManager::storeCache(std::vector<cv::Mat>* cache){

}


