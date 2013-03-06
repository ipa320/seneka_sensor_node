/*
 * videoRecorder.cpp
 *
 *  Created on: 21.02.2013
 *      Author: Johannes Goth (cmm-jg)
 */

#include "videoRecorder.h"

// public member functions

videoRecorder::videoRecorder(){
	videocreated = false;
	codec = CV_FOURCC('D','I','V','X');
	fps = 25;
	frameWidth = 0;
	frameHeight = 0;
}

videoRecorder::~videoRecorder(){
	// TODO: delete open pointers
}

void videoRecorder::createVideo(std::string vf, int frameWidth, int frameHeight){
	videoFileName = vf;
	videoSize.height = frameHeight;		// cv::mat obj.rows
	videoSize.width = frameWidth;		// cv::mat obj.cols
	videocreated = true;				// to provide only on active video recorder (better would be a singleton pattern)
	openVideo();
}

void videoRecorder::addFrame(cv::Mat frame){
	if(videoWriter.isOpened() == true)
		videoWriter.write(frame);
	else
		ROS_ERROR("Video file is closed ...");
}

bool videoRecorder::isVideoRecorderActive(){
	return videocreated;
}

void videoRecorder::releaseVideo(){
	videoWriter.release();
}

// private member functions

void videoRecorder::openVideo(){
	videoWriter.open(videoFileName, codec, fps, videoSize, 1);
}
