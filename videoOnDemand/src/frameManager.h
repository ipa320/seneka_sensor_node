/*
 * frameManager.h
 *
 *  Created on: 21.02.2013
 *      Author: Johannes Goth (cmm-jg)
 */

#ifndef FRAMEMANAGER_H_
#define FRAMEMANAGER_H_


// own stuff
#include "frameManager.h"
// libraries
//#include <boost/thread.hpp>
//#include <boost/thread/mutex.hpp>
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

class frameManager {
public:
	frameManager();
	virtual ~frameManager();
	void processFrame(const sensor_msgs::Image& img);
	int getVideo();
	void testFrameManager();

private:

	// methods
	void cacheFrame(cv::Mat frame);
	void verifyCacheSize();
	void storeCache(std::vector<cv::Mat>* cache);
	int createVideo();
	std::vector<cv::Mat>* getCurrentCache();
	//boost::mutex* getMutexToCache(bool usedCache);



	// object attributes
	u_int maxCacheSize;	// in frames
	u_int maxRecTime; 	// in minutes
	bool useCacheA;
	bool useCacheB;
	std::vector<cv::Mat>* cacheA;
	std::vector<cv::Mat>* cacheB;

//	boost::mutex mut;
//	boost::mutex mxCacheA;
//	boost::mutex mxCacheB;

	u_int currentFileStorageSize;

	u_int currentFileStorage;
	std::vector<cv::FileStorage*> fileStorages;


};

#endif /* FRAMEMANAGERH_ */
