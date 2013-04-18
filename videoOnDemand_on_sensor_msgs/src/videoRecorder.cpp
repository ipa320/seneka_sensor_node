/*
 * videoRecorder.cpp
 *
 *  Created on: 21.02.2013
 *      Author: Johannes Goth (cmm-jg)
 */

#include "videoRecorder.h"

// public member functions

VideoRecorder::VideoRecorder(){
	codec = CV_FOURCC('D','I','V','X');
	//codec = CV_FOURCC('3','I','V,'2');
	fps = 10;
	frameWidth = 0;
	frameHeight = 0;
}

VideoRecorder::VideoRecorder(u_int fps, int codec){
	this->codec = codec;
	this->fps = fps;
	frameWidth = 0;
	frameHeight = 0;
}

VideoRecorder::~VideoRecorder(){}

void VideoRecorder::createVideo(std::string vf, int frameWidth, int frameHeight){
	videoFileName = vf;
	videoSize.height = frameHeight;		// cv::mat obj.rows
	videoSize.width = frameWidth;		// cv::mat obj.cols
	openVideo();
}

void VideoRecorder::addFrame(cv::Mat frame){
	if(videoWriter.isOpened() == true)
		videoWriter.write(frame);
	else
		ROS_ERROR("Video file is closed ...");
}

void VideoRecorder::releaseVideo(){
	videoWriter.release();
}

// private member functions

void VideoRecorder::openVideo(){
	if(!videoWriter.isOpened()){
		videoWriter.open(videoFileName, codec, fps, videoSize, 1);
	}
	else{
		ROS_ERROR("Video is still open ...!");
	}

}
