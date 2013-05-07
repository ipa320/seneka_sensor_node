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
 *   ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *   Author: Johannes Goth (cmm-jg)
 * \author
 *   Supervised by: Christophe Maufroy
 *
 * \date Date of creation: 21.02.2013
 *
 * \brief
 *   TODO FILL IN BRIEF DESCRIPTION HERE
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

#include "videoRecorder.h"

VideoRecorder::VideoRecorder(){
	codec = CV_FOURCC('D','I','V','X');
	//codec = CV_FOURCC('X','2','6','4');
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

void VideoRecorder::openVideo(){
	if(!videoWriter.isOpened()){
		videoWriter.open(videoFileName, codec, fps, videoSize, 1);
	}
	else{
		ROS_ERROR("Video is still open ...!");
	}
}

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
