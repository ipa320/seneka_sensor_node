/*
 * videoRecorder.h
 *
 *  Created on: 21.02.2013
 *      Author: Johannes Goth (cmm-jg)
 */

#ifndef VIDEORECORDER_H_
#define VIDEORECORDER_H_

// ROS includes
#include "ros/ros.h"

class VideoRecorder {
public:

	// public member functions
	VideoRecorder();
	virtual ~VideoRecorder();
	void createVideo(std::string vf, int frameWidth, int frameHeight);
	void addFrame(cv::Mat frame);
	void releaseVideo();

private:

	// private member functions
	void openVideo();

	// private attributes and references
	int codec;
	int fps;					// frame per second
	int frameWidth;
	int frameHeight;
	cv::VideoWriter videoWriter;
	std::string videoFileName;
	CvSize videoSize;
};

#endif /* VIDEORECORDERH_ */
