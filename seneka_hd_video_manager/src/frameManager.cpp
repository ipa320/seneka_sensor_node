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
 *   ROS package name: seneka_video_manager
 *
 * \author
 *   Author: Johannes Goth (cmm-jg)
 * \author
 *   Supervised by: Christophe Maufroy (cmm)
 *
 * \date Date of creation: 21.02.2013
 *
 * \brief
 *   frameManager.cpp
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

// std and own includes
#include "frameManager.h"
#include "boost_serialization_avpacket.h"
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

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/mathematics.h>
}

// public member functions
FrameManager::FrameManager() {

	// initialize parameters
	outputFolder = "/tmp/";
	binaryFilePath = outputFolder + "container";
	videoFilePath = outputFolder + "videoOnDemand.avi";
	fpv = 400;	 	// frame per video -> 40 sec * 10 frames = 400 frames
	fpc = 100;		// frames per cache -> 10 sec * 10 frames = 100 frames
	fpb = fpc;		// frames per binary
	vfr = 10;
	videoCodec = CV_FOURCC('D','I','V','X');
	binaryFileIndex = 0;
	fullVideoAvailable = false;
	createVideoActive = false;
	usingCacheA = true;
	usingCacheB = false;
	storingCacheA = false;
	storingCacheB = false;
	cacheA = new std::vector<AVPacket>;
	cacheB = new std::vector<AVPacket>;
	showFrame = false;
	snapshotRunning = false;
	liveStreamRunning = false;
	stateMachine = ON_DEMAND;

	for(int i=0; i < (int)(fpv/fpb)+1; i++){
		binaryFileMutexes.push_back(new boost::mutex());
	}

	ausloeser = 0;
}

FrameManager::FrameManager(ros::NodeHandle &nHandler) {

	// initialize configurable parameters
	int tmp_fpv, tmp_fpc, tmp_fpb, tmp_vfr;

	nHandler.getParam("framesPerVideo", tmp_fpv);
	nHandler.getParam("framesPerCache", tmp_fpc);
	nHandler.getParam("framesPerBinary", tmp_fpb);
	nHandler.getParam("videoFrameRate", tmp_vfr);

	if(!nHandler.hasParam("framesPerVideo") || tmp_fpv<=0){
		ROS_WARN("Used default parameter for framesPerVideo [200]");
		fpv = 200;
	}
	else
		fpv=(u_int)tmp_fpv;

	if(!nHandler.hasParam("framesPerCache") || tmp_fpc<=0){
		ROS_WARN("Used default parameter for framesPerCache [100]");
		fpc = 100;
	}
	else
		fpc=(u_int)tmp_fpc;


	if(!nHandler.hasParam("framesPerBinary") || tmp_fpb<=0){
		ROS_WARN("Used default parameter for framesPerBinary [100]");
		fpb = 100;
	}
	else
		fpb=(u_int)tmp_fpb;

	if(!nHandler.hasParam("videoFrameRate") || tmp_vfr<=0){
		ROS_WARN("Used default parameter for videoFrameRate [10]");
		vfr = 10;
	}
	else
		vfr=(u_int)tmp_vfr;

	if(!nHandler.hasParam("outputFolder")){
		ROS_WARN("Used default parameter for outputFolder [/tmp]");
		outputFolder = "/tmp/";
		binaryFilePath = outputFolder + "container";
		videoFilePath = outputFolder + "videoOnDemand.avi";
	}
	else{
		nHandler.getParam("outputFolder", outputFolder);
		binaryFilePath = outputFolder + "container";
		videoFilePath = outputFolder + "videoOnDemand.avi";
	}

	if(!nHandler.hasParam("showFrame")){
		ROS_WARN("Used default parameter for showFrame [true]");
		showFrame = true;
	}
	else
		nHandler.getParam("showFrame", showFrame);

	// initialize fixed parameters
	videoCodec = CV_FOURCC('D','I','V','X');
	binaryFileIndex = 0;
	fullVideoAvailable = false;
	createVideoActive = false;
	usingCacheA = true;
	usingCacheB = false;
	storingCacheA = false;
	storingCacheB = false;
	cacheA = new std::vector<AVPacket>;
	cacheB = new std::vector<AVPacket>;
	snapshotRunning = false;
	liveStreamRunning = false;
	stateMachine = ON_DEMAND;

	for(int i=0; i < (int)(fpv/fpb)+1; i++){
		binaryFileMutexes.push_back(new boost::mutex());
	}
}

FrameManager::~FrameManager() {
	delete cacheA;
	delete cacheB;
}

void FrameManager::cacheFrame(AVPacket* packet){

	std::vector<AVPacket>* currentCache = getCurrentCache();
	currentCache->push_back(*packet);

	// manual tigger, just for testing issues
	ausloeser++;
	if(ausloeser == 500)
		createVideo();
}

std::vector<AVPacket>* FrameManager::getCurrentCache(){

	// checks the sizes of memory buffers
	// and if necessary it will schedule further tasks
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

void FrameManager::verifyCacheSize(){
	//ROS_INFO("verifyCacheSize ... ");

	if(cacheA->size() == fpc && storingCacheA == false){
		// cacheA is full -> store frames into binary file

		storingCacheA = true;
		usingCacheA = false;
		usingCacheB = true;

		ROS_INFO("Waiting for storingCacheB");
		storingThreadB.join();
		storingThreadA = boost::thread(boost::bind(&FrameManager::storeCache, this, cacheA, &storingCacheA));
	}
	else if (cacheB->size() == fpc && storingCacheB == false){
		// cacheB is full -> store frames into binary file

		storingCacheB = true;
		usingCacheB = false;
		usingCacheA = true;

		ROS_INFO("Waiting for storingCacheA");
		storingThreadA.join();
		storingThreadB = boost::thread(boost::bind(&FrameManager::storeCache, this, cacheB, &storingCacheB));
	}
}

void FrameManager::storeCache(std::vector<AVPacket>* cache, bool* threadActive){

	ROS_INFO("storeCache into binary file...");
	// define fileStorage-filename
	std::stringstream fileName;
	fileName << binaryFilePath << binaryFileIndex << ".bin";

	// lock current binary file as output
	binaryFileMutexes[binaryFileIndex]->lock();
	// open outputFile
	std::ofstream ofs(fileName.str().c_str(), std::ios::out | std::ios::binary);

	// scope is required to ensure archive and filtering stream buffer go out of scope
	// before stream
	{

		boost::archive::binary_oarchive oa(ofs);

		// writes each frame which is stored in cache into binary file
		for (std::vector<AVPacket>::iterator it = cache->begin() ; it != cache->end(); it++){
			// writes frame per frame into binary file, using AVPacket serialization
			oa << *it;
		}
	}
	// close file
	ofs.close();
	// unlock current binary file
	binaryFileMutexes[binaryFileIndex]->unlock();

	if(binaryFileIndex < (fpv/fpb)-1)
		binaryFileIndex++;
	else{
		if(fullVideoAvailable == false)
			fullVideoAvailable = true;
		binaryFileIndex = 0;
	}
	// clean cache
	cache->clear();
	*threadActive = false;
	ROS_INFO("finished storingThread");
}

int FrameManager::getVideo(){
	// TODO: could be implemented like in ThermoVideoManger or general VideoManager
	ROS_ERROR("Unknown state for the state machine");
	return 0;
}

int FrameManager::createVideo(){

	ROS_INFO("createVideo ...");

	createVideoActive = true;
	bool firstFrame = true;
	/* start binary for video creation
	 * selecting binaryFileIndex + 1 to get the oldest binary file,
	 * which includes the first frame for the video creation
	 */
	int currentIndex = binaryFileIndex + 1;
	int numBinaries = fpv/fpb;

	// ffmpeg - libav output parameters
	char *outputFilename = "output.mp4";
	AVOutputFormat *fmt;
	AVFormatContext *oc;
	AVStream *video_st;
	double video_pts;

	//ffmpeg - libav output initialization
	fmt = av_guess_format(NULL, outputFilename, NULL);
	if (!fmt) {
		fprintf(stderr, "Could not find suitable output format\n");
		return 1;
	}

	/* allocate the output media context */
	oc = avformat_alloc_context();
	if (!oc) {
		fprintf(stderr, "Memory error\n");
		return 1;
	}

	oc->oformat = fmt;
	snprintf(oc->filename, sizeof(oc->filename), "%s", outputFilename);

	video_st = NULL;

	if (fmt->video_codec != CODEC_ID_NONE) {
		video_st = avformat_new_stream(oc, 0);

		AVCodec *codec;

		if (!video_st) {
			fprintf(stderr, "Could not alloc stream\n");
			exit(1);
		}

		//video_st->codec = avcodec_alloc_context();

		AVCodecContext *c = video_st->codec;
		c = avcodec_alloc_context();
		c->codec_type = pCodecCtx->codec_type; //AVMEDIA_TYPE_VIDEO;
		c->codec_id = pCodecCtx->codec_id; //fmt->video_codec;
		c->bit_rate = pCodecCtx->bit_rate;
		c->width = pCodecCtx->width;
		c->height = pCodecCtx->height;
		c->time_base.den = pCodecCtx->time_base.den;
		c->time_base.num = pCodecCtx->time_base.num;
		c->gop_size = pCodecCtx->gop_size;
		c->pix_fmt = pCodecCtx->pix_fmt;


		c->bit_rate_tolerance = pCodecCtx->bit_rate_tolerance;
		c->rc_max_rate = pCodecCtx->rc_max_rate;
		c->rc_buffer_size = pCodecCtx->rc_buffer_size;
		c->max_b_frames = 100;//pCodecCtx->max_b_frames;
		c->b_frame_strategy = pCodecCtx->b_frame_strategy;
		c->coder_type = pCodecCtx->coder_type;
		c->me_cmp = pCodecCtx->me_cmp;
		c->me_range = pCodecCtx->me_range;
		c->qmin = 10;
		c->qmax = 51;
		c->scenechange_threshold = 40;
		c->profile = pCodecCtx->profile;//FF_PROFILE_H264_BASELINE;
		c->flags = CODEC_FLAG_GLOBAL_HEADER;
		c->me_method = ME_HEX;
		c->me_subpel_quality = 5;
		c->i_quant_factor = 0.71;
		c->qcompress = 0.6;
		c->max_qdiff = 4;
		c->directpred = 1;
		c->flags2 |= CODEC_FLAG2_FASTPSKIP;


		std::cout << "here" << std::endl;
//		c->max_b_frames = pCodecCtx->max_b_frames;
//		c->mb_decision = pCodecCtx->mb_decision;
//
//		if (c->codec_id == CODEC_ID_MPEG2VIDEO) {
//			/* just for testing, we also add B frames */
//			c->max_b_frames = 2;
//		}
//		if (c->codec_id == CODEC_ID_MPEG1VIDEO){
//			/* Needed to avoid using macroblocks in which some coeffs overflow.
//					This does not happen with normal video, it just happens here as
//					the motion of the chroma plane does not match the luma plane. */
//			c->mb_decision=2;
//		}
//		// some formats want stream headers to be separate
//		if(oc->oformat->flags & AVFMT_GLOBALHEADER){
//			c->flags |= CODEC_FLAG_GLOBAL_HEADER;
//		}
//
//		/* find the video encoder */
//		codec = avcodec_find_encoder(c->codec_id);
//		if (!codec) {
//			fprintf(stderr, "codec not found\n");
//			exit(1);
//		}

		/* open the codec */
		if (avcodec_open2(c, codec, NULL) < 0) {
			fprintf(stderr, "could not open codec\n");
			exit(1);
		}
	}

	/* set the output parameters (must be done even if no parameters). */
	if (av_set_parameters(oc, NULL) < 0) {
		fprintf(stderr, "Invalid output format parameters\n");
		return 1;
	}

	av_dump_format(oc, 0, outputFilename, 1);

	/* open the output file, if needed */
	if (!(fmt->flags & AVFMT_NOFILE)) {
		if (avio_open(&oc->pb, outputFilename, AVIO_FLAG_WRITE) < 0) {
			fprintf(stderr, "Could not open '%s'\n", outputFilename	);
			return 1;
		}
	}

	/* write the stream header, if any */
	av_write_header(oc);

	// add all binaries which are required (numBinaries) into a video
	for(int i=0; i < numBinaries; i++){
		std::stringstream inputFileName;
		int mutexID;

		if(currentIndex < numBinaries){
			// load currentIndex - numBinaries-1
			inputFileName << binaryFilePath << (currentIndex) << ".bin";
			mutexID = currentIndex;
			currentIndex++;
		}
		else{
			// load currentIndex - numBinaries-1
			inputFileName << binaryFilePath << (currentIndex-numBinaries) << ".bin";
			mutexID = currentIndex-numBinaries;
			currentIndex++;
		}

		// lock current binary file as input
		binaryFileMutexes[mutexID]->lock();

		// open inputFile (binary)
		std::ifstream ifs(inputFileName.str().c_str(), std::ios::in | std::ios::binary);

		// scope is required to ensure archive and filtering stream buffer go out of scope
		// before stream
		{
			boost::archive::binary_iarchive ia(ifs);
			bool hasContent = true;
			while (hasContent)
			{
				int ret;
				AVPacket loadedPacket;
				AVPacket refactoredPacked;
				av_init_packet(&loadedPacket);
				av_init_packet(&refactoredPacked);

				if(firstFrame){
					// try to read image from binary file
					hasContent = boost::serialization::try_stream_next(ia, ifs, loadedPacket);

					// for video creation it is necessary that the first frame is a key frame
					if (hasContent == true && loadedPacket.flags == 1 ){

						// its the first frame
						firstFrame = false;

						if (loadedPacket.pts != AV_NOPTS_VALUE){
							//packet.pts= av_rescale_q(video_st->codec->coded_frame->pts, video_st->codec->time_base, video_st->time_base);
							refactoredPacked.pts = av_rescale_q(loadedPacket.pts, pCodecCtx->time_base, video_st->time_base);
							refactoredPacked.dts = av_rescale_q(loadedPacket.dts, pCodecCtx->time_base, video_st->time_base);
						}
						if(loadedPacket.flags == 1)
							refactoredPacked.flags |= AV_PKT_FLAG_KEY;

						refactoredPacked.stream_index= video_st->index;
						refactoredPacked.data= loadedPacket.data;
						refactoredPacked.size= loadedPacket.size;


						// Add loaddedFrame to videoContainer or cache it in a vector
						ret = av_interleaved_write_frame(oc, &refactoredPacked);
//						ret = av_write_frame(oc, &refactoredPacked);oc

						std::cout << "add first frame, pts: " << loadedPacket.pts << " dts: " << loadedPacket.dts << " duration: " << loadedPacket.duration << " returnValue: " << ret << std::endl;

					}
				}
				else{
					// try to read image from binary file
					hasContent = boost::serialization::try_stream_next(ia, ifs, loadedPacket);

					// add frame to video
					if (hasContent == true){

						if (loadedPacket.pts != AV_NOPTS_VALUE){
							//packet.pts= av_rescale_q(video_st->codec->coded_frame->pts, video_st->codec->time_base, video_st->time_base);
							refactoredPacked.pts = av_rescale_q(loadedPacket.pts, pCodecCtx->time_base, video_st->time_base);
							refactoredPacked.dts = av_rescale_q(loadedPacket.dts, pCodecCtx->time_base, video_st->time_base);
						}
						if(loadedPacket.flags == 1)
							refactoredPacked.flags |= AV_PKT_FLAG_KEY;

						refactoredPacked.stream_index= video_st->index;
						refactoredPacked.data= loadedPacket.data;
						refactoredPacked.size= loadedPacket.size;

						// Add loaddedFrame to videoContainer or cache it in a vector
						ret = av_interleaved_write_frame(oc, &refactoredPacked);
//						ret = av_write_frame(oc, &refactoredPacked);oc
						std::cout << "add further frame, pts: " << loadedPacket.pts << " dts: " << loadedPacket.dts << " duration: " << loadedPacket.duration << " returnValue: " << ret << std::endl;
					}

				}
			}
			ifs.close();
			// unlock current binary file
			binaryFileMutexes[mutexID]->unlock();
		}
	}
	 av_write_trailer(oc);
    /* close the output file */
    avio_close(oc->pb);

	createVideoActive = false;
	ROS_INFO("finished createVideo ...");

	return -1;
}

void FrameManager::displayFrame(AVPacket* mat){
	// TODO: could be implemented like in ThermoVideoManger or general VideoManager
}

void FrameManager::startSnapshots(int interval){
	// TODO: could be implemented like in ThermoVideoManger or general VideoManager
}

void FrameManager::stopSnapshots(){
	// TODO: could be implemented like in ThermoVideoManger or general VideoManager
}

void FrameManager::createSnapshots(int interval){
	// TODO: could be implemented like in ThermoVideoManger or general VideoManager
}

void FrameManager::startLiveStream(){
	// TODO: could be implemented like in ThermoVideoManger or general VideoManager
}

void FrameManager::stopLiveStream(){
	// TODO: could be implemented like in ThermoVideoManger or general VideoManager
}

void FrameManager::saveAVFormatContext(AVCodecContext fc){
	this->pCodecCtx = &fc;
}
