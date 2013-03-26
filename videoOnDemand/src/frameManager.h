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
#include "videoRecorder.h"
#include "videoRecorder.cpp"
// libraries
#include <boost/thread.hpp>
#include <vector>
// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
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
	static void threadTestFkt();

private:

	// private memeber functions
	void cacheFrame(cv::Mat frame);
	void verifyCacheSize();
	void storeCache(std::vector<cv::Mat>* cache, bool* threadActive);
	int createVideo();
	std::vector<cv::Mat>* getCurrentCache();
	void storeFrame(cv::Mat frame);



	// cache attributes and references (memory buffers)
	u_int fpv;			// frames per video
	u_int fpc; 			// frames per cache
	u_int fpb; 			// frames per binary
	bool fullVideoAvailable;
	bool usingCacheA;
	bool usingCacheB;
	std::vector<cv::Mat>* cacheA;
	std::vector<cv::Mat>* cacheB;

	// file storage parameters
	std::string path;
	u_int binaryFileIndex;
	std::vector<boost::mutex*> binaryFileMutexes;


	// threads
	boost::thread storingThreadA;
	boost::thread storingThreadB;
	boost::thread tCaching;
	bool storingCacheA;
	bool storingCacheB;
	boost::thread creatingVideoThread;
	boost::thread cachingThread;

	// mutex stuff (currently not in use)
	//	boost::mutex mut;

};

#endif /* FRAMEMANAGERH_ */
