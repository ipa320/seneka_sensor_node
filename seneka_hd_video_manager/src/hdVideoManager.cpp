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
 *   videoManager.cpp
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

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <ros/ros.h>
#include <iostream>
#include "frameManager.h"
#include "frameManager.cpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <stdio.h>

#include <SDL/SDL.h>
#include <SDL/SDL_thread.h>
}

#ifdef __MINGW32__
#undef main /* Prevents SDL from overriding main() */
#endif

// global attributes
FrameManager* fManager;

void SaveFrame(AVFrame *pFrame, int width, int height, int iFrame) {
	FILE *pFile;
	char szFilename[32];
	int  y;

	// Open file
	sprintf(szFilename,"frame%d.ppm", iFrame);
	pFile=fopen(szFilename, "wb");
	if(pFile==NULL)
		return;

	// Write header
	fprintf(pFile, "P6\n%d %d\n255\n", width, height);

	// Write pixel data
	for(y=0; y<height; y++)
		fwrite(pFrame->data[0]+y*pFrame->linesize[0], 1, width*3, pFile);

	// Close file
	fclose(pFile);
}

int connectToVideoStream(int argc, char **argv){

	// TODO: devel. params
	bool cachingPacket = true;
	bool createImgFile = false;
	bool showFrame = true;

	AVFormatContext *pFormatCtx = NULL;
	int             i, videoStream;
	AVCodecContext  *pCodecCtx = NULL;
	AVCodec         *pCodec = NULL;
	AVFrame         *pFrame = NULL;
	AVFrame         *pFrameRGB = NULL;
	AVPacket        packet;
	int             frameFinished;
	int             numBytes;
	uint8_t         *buffer = NULL;

	AVDictionary    *optionsDict = NULL;
	struct SwsContext      *sws_ctx_rgb = NULL;
	struct SwsContext      *sws_ctx_yuv = NULL;


	// SDL variables
	SDL_Overlay     *bmp = NULL;
	SDL_Surface     *screen = NULL;
	SDL_Rect        rect;
	SDL_Event       event;



	av_log_set_level(AV_LOG_DEBUG);

	if(argc < 2) {
		printf("Please provide a movie file\n");
		return -1;
	}
	// Register all formats and codecs
	av_register_all();

	// SDL init
	if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER)) {
		fprintf(stderr, "Could not initialize SDL - %s\n", SDL_GetError());
		exit(1);
	}

	// Open video file
	if(avformat_open_input(&pFormatCtx, argv[1], NULL, NULL)!=0)
		return -1; // Couldn't open file

	// Retrieve stream information
	if(avformat_find_stream_info(pFormatCtx, NULL)<0)
		return -1; // Couldn't find stream information

	// Dump information about file onto standard error
	av_dump_format(pFormatCtx, 0, argv[1], 0);

	// Find the first video stream
	videoStream=-1;
	ROS_INFO("streams incl. in source: %d", pFormatCtx->nb_streams);
	for(i=0; i<pFormatCtx->nb_streams; i++){
		if(pFormatCtx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO) {
			videoStream=i;
			break;
		}
	}

	if(videoStream==-1)
		return -1; // Didn't find a video stream

	// Get a pointer to the codec context for the video stream
	pCodecCtx=pFormatCtx->streams[videoStream]->codec;

	// Find the decoder for the video stream
	pCodec=avcodec_find_decoder(pCodecCtx->codec_id);
	if(pCodec==NULL) {
		fprintf(stderr, "Unsupported codec!\n");
		return -1; // Codec not found
	}
	// Open codec
	if(avcodec_open2(pCodecCtx, pCodec, &optionsDict)<0)
		return -1; // Could not open codec

	// Allocate an AVFrame structure -> encoded video frames
	pFrame=avcodec_alloc_frame();
	// Allocate an AVFrame structure -> converted/decoded pFrame
	pFrameRGB=avcodec_alloc_frame();
	if(pFrameRGB==NULL)
		return -1;

	// Make a screen to put our video
#ifndef __DARWIN__
	screen = SDL_SetVideoMode(pCodecCtx->width, pCodecCtx->height, 0, 0);
#else
	screen = SDL_SetVideoMode(pCodecCtx->width, pCodecCtx->height, 24, 0);
#endif
	if(!screen) {
		fprintf(stderr, "SDL: could not set video mode - exiting\n");
		exit(1);
	}

	// SDL allocate a place to put our YUV image on that screen
	bmp = SDL_CreateYUVOverlay(pCodecCtx->width, pCodecCtx->height, SDL_YV12_OVERLAY, screen);

	// Determine required buffer size and allocate buffer
	numBytes=avpicture_get_size(PIX_FMT_RGB24, pCodecCtx->width,
			pCodecCtx->height);
	buffer=(uint8_t *)av_malloc(numBytes*sizeof(uint8_t));

	sws_ctx_rgb = sws_getContext
			(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, pCodecCtx->width,
					pCodecCtx->height, PIX_FMT_RGB24, SWS_BILINEAR,	NULL, NULL,	NULL);
	sws_ctx_yuv = sws_getContext
			(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, pCodecCtx->width,
					pCodecCtx->height, PIX_FMT_YUV420P, SWS_BILINEAR,	NULL, NULL,	NULL);

	// Assign appropriate parts of buffer to image planes in pFrameRGB
	// Note that pFrameRGB is an AVFrame, but AVFrame is a superset
	// of AVPicture
	avpicture_fill((AVPicture *)pFrameRGB, buffer, PIX_FMT_RGB24,
			pCodecCtx->width, pCodecCtx->height);

	// Read frames and save first five frames to disk
	i=0;

	while(av_read_frame(pFormatCtx, &packet)>=0) {

		// Is this a packet from the video stream?
		if(packet.stream_index==videoStream) {

			if(cachingPacket){
				fManager->cacheFrame(&packet);
				fManager->saveAVFormatContext(*pCodecCtx);

//				std::cout << "add further frame, pts: " << packet.pts << " dts: " << packet.dts << " duration: " << packet.duration << " returnValue: " << ret << std::endl;
			}

			// Decode video frame
			avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, &packet);

			// Did we get a video frame?
			if(frameFinished) {
				// RGB OUTPUT
				if(createImgFile){
					// Convert the image from its native format to RGB
					sws_scale(sws_ctx_rgb, (uint8_t const * const *)pFrame->data, pFrame->linesize,
							0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);

					// Save the frame to disk
					if(++i<=5)
						SaveFrame(pFrameRGB, pCodecCtx->width, pCodecCtx->height, i);
				}

				// YUV OUTPU
				if(showFrame){
					SDL_LockYUVOverlay(bmp);

					AVPicture pict;
					pict.data[0] = bmp->pixels[0];
					pict.data[1] = bmp->pixels[2];
					pict.data[2] = bmp->pixels[1];

					pict.linesize[0] = bmp->pitches[0];
					pict.linesize[1] = bmp->pitches[2];
					pict.linesize[2] = bmp->pitches[1];

					// Convert the image into YUV format that SDL uses
					sws_scale
					(
							sws_ctx_yuv,
							(uint8_t const * const *)pFrame->data,
							pFrame->linesize,
							0,
							pCodecCtx->height,
							pict.data,
							pict.linesize
					);

					SDL_UnlockYUVOverlay(bmp);

					rect.x = 0;
					rect.y = 0;
					rect.w = pCodecCtx->width;
					rect.h = pCodecCtx->height;
					SDL_DisplayYUVOverlay(bmp, &rect);
				}
			}
		}

		// Free the packet that was allocated by av_read_frame
		av_free_packet(&packet);
		SDL_PollEvent(&event);
		switch(event.type) {
		case SDL_QUIT:
			SDL_Quit();
			exit(0);
			break;
		default:
			break;
		}
	}

	// Free the RGB image
	av_free(buffer);
	av_free(pFrameRGB);

	// Free the YUV frame
	av_free(pFrame);

	// Close the codec
	avcodec_close(pCodecCtx);

	// Close the video file
	avformat_close_input(&pFormatCtx);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "HD_VIDEO_MANAGER");
	// attributes
	ros::NodeHandle nHandler("HD_VIDEO_MANAGER");
	fManager = new FrameManager();

	// connect to source and collect encoded frames
	while(true){

		ROS_INFO("Connecting to source ... ");
		int statusInfo = connectToVideoStream(argc, argv);

		// if input file is missing
		if(statusInfo == -1)
			return -1;
	}

	return 0;
}
