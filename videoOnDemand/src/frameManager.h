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




class FrameManager {
public:
	FrameManager();
	FrameManager(ros::NodeHandle &nHandler);
	virtual ~FrameManager();
	void processFrame(const sensor_msgs::Image& img);
	int getVideo();

private:

	// private memeber functions
	void cacheFrame(cv::Mat frame);
	void verifyCacheSize();
	void storeCache(std::vector<cv::Mat>* cache, bool* threadActive);
	int createVideo();
	std::vector<cv::Mat>* getCurrentCache();
	void storeFrame(cv::Mat frame);
	void displayFrame(cv::Mat* mat);

	// cache attributes and references (memory buffers)
	u_int fpv;			// frames per video
	u_int fpc; 			// frames per cache
	u_int fpb; 			// frames per binary
	bool fullVideoAvailable;
	bool createVideoActive;
	bool usingCacheA;
	bool usingCacheB;
	std::vector<cv::Mat>* cacheA;
	std::vector<cv::Mat>* cacheB;

	// file storage parameters
	std::string binaryFilePath;
	std::string videoFilePath;
	u_int binaryFileIndex;
	std::vector<boost::mutex*> binaryFileMutexes;

	// output video parameters
	u_int vfr;			// video frame rate
	int videoCodec; 	// Codec for video coding eg. CV_FOURCC('D','I','V','X')
	bool showFrame;

	// threads parameters
	boost::thread storingThreadA;
	boost::thread storingThreadB;
	boost::thread tCaching;
	bool storingCacheA;
	bool storingCacheB;
	boost::thread creatingVideoThread;
	boost::thread cachingThread;
};

#endif /* FRAMEMANAGERH_ */
