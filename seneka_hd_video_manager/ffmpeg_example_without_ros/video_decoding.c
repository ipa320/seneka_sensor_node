// tutorial01.c
//
// This tutorial was written by Stephen Dranger (dranger@gmail.com) and updated
// for ffmpeg version N-42806-gf4451d2 by Michael Penkov 
// (misha.penkov@gmail.com). 
//
// Code based on a tutorial by Martin Bohme (boehme@inb.uni-luebeckREMOVETHIS.de)
// Tested on Gentoo, CVS version 5/01/07 compiled with GCC 4.1.1

// A small sample program that shows how to use libavformat and libavcodec to
// read video from a file.
//
// Use the Makefile to build all examples.
//
// Run using
//
// tutorial01 myvideofile.mpg
//
// to write the first five frames from "myvideofile.mpg" to disk in PPM
// format.

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>

#define STREAM_DURATION   5.0
#define STREAM_FRAME_RATE 25 /* 25 images/s */
#define STREAM_NB_FRAMES  ((int)(STREAM_DURATION * STREAM_FRAME_RATE))
#define STREAM_PIX_FMT PIX_FMT_YUV420P /* default pix_fmt */	

#include <stdio.h>


void SaveFrameToFile(AVFrame* pFrame, int width, int height, const char* format, int iFrame) {
    AVCodec* encoderCodec = avcodec_find_encoder_by_name(format);
    AVCodecContext* encoderCtx = avcodec_alloc_context();
    avcodec_open2(encoderCtx, encoderCodec, NULL);
    char szFilename[32];
    FILE *pFile;

    encoderCtx->width = width;
    encoderCtx->height = height;
    encoderCtx->pix_fmt = PIX_FMT_RGB24; // Is this line really needed? 

    int bufferSize = avpicture_get_size(encoderCtx->pix_fmt, width, height);
    uint8_t* buffer = (uint8_t*) av_malloc(bufferSize);
    int newSize = avcodec_encode_video(encoderCtx, buffer, bufferSize, pFrame);

    sprintf(szFilename,"output/frame%d.png", iFrame);
    pFile=fopen(szFilename, "wb");
    fwrite(buffer, 1, newSize, pFile);
    fclose(pFile);
}


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

int main(int argc, char *argv[]) {
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
	int		saveAsImg = 1;

	// output parameters
	char *outputFilename = "output.mp4";
	AVOutputFormat *fmt;
	AVFormatContext *oc;
	AVStream *video_st;
	double video_pts;	

	AVDictionary    *optionsDict = NULL;
	struct SwsContext      *sws_ctx = NULL;

	if(argc < 2) {
		printf("Please provide a movie file\n");
		return -1;
	}

		av_log_set_level(AV_LOG_DEBUG);

	// Register all formats and codecs

	av_register_all();
	avformat_network_init();	

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


	for(i=0; i<pFormatCtx->nb_streams; i++)
		if(pFormatCtx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO) {
			videoStream=i;
			break;
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

	// Allocate video frame
	pFrame=avcodec_alloc_frame();

	// Allocate an AVFrame structure
	pFrameRGB=avcodec_alloc_frame();
	if(pFrameRGB==NULL)
		return -1;

	// Determine required buffer size and allocate buffer
	numBytes=avpicture_get_size(PIX_FMT_RGB24, pCodecCtx->width,
			pCodecCtx->height);
	buffer=(uint8_t *)av_malloc(numBytes*sizeof(uint8_t));

	sws_ctx = sws_getContext
			(
					pCodecCtx->width,
					pCodecCtx->height,
					pCodecCtx->pix_fmt,
					pCodecCtx->width,
					pCodecCtx->height,
					PIX_FMT_RGB24,
					SWS_BILINEAR,
					NULL,
					NULL,
					NULL
			);

	// Assign appropriate parts of buffer to image planes in pFrameRGB
	// Note that pFrameRGB is an AVFrame, but AVFrame is a superset
	// of AVPicture
	avpicture_fill((AVPicture *)pFrameRGB, buffer, PIX_FMT_RGB24,
			pCodecCtx->width, pCodecCtx->height);


	//output configuraion
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
	
//	s->oformat = oformat;
//	if (oc->oformat->priv_data_size > 0) {
//		oc->priv_data = av_mallocz(oc->oformat->priv_data_size);
//		if (!oc->priv_data)
//			av_log(oc, AV_LOG_ERROR, "Out of memory\n");
//		if (oc->oformat->priv_class) {
//			*(const AVClass**)oc->priv_data= oc->oformat->priv_class;
//			av_opt_set_defaults(oc->priv_data);
//		}
//	} else
//		oc		->priv_data = NULL;


	snprintf(oc->filename, sizeof(oc->filename), "%s", outputFilename);

	video_st = NULL;
    	if (fmt->video_codec != CODEC_ID_NONE) {
		video_st = avformat_new_stream(oc, 0);

		AVCodec *codec;
		

		if (!video_st) {
			fprintf(stderr, "Could not alloc stream\n");
			exit(1);
		}
		
		video_st->codec = avcodec_alloc_context();

		AVCodecContext *c = video_st->codec;

		c->codec_type = AVMEDIA_TYPE_VIDEO; //pCodecCtx->codec_type;
		c->codec_id = fmt->video_codec; //pCodecCtx->codec_id;
		c->bit_rate = pCodecCtx->bit_rate;
		c->width = pCodecCtx->width;
		c->height = pCodecCtx->height;
		c->time_base.den = pCodecCtx->time_base.den;
		c->time_base.num = pCodecCtx->time_base.num;
		c->gop_size = pCodecCtx->gop_size;
		c->pix_fmt = pCodecCtx->pix_fmt;

		
//		c->bit_rate_tolerance = pCodecCtx->bit_rate_tolerance;
//		c->rc_max_rate = pCodecCtx->rc_max_rate;
//		c->rc_buffer_size = pCodecCtx->rc_buffer_size;
//		c->max_b_frames = 100;//pCodecCtx->max_b_frames;
//		c->b_frame_strategy = pCodecCtx->b_frame_strategy;
//		c->coder_type = pCodecCtx->coder_type;
//		c->me_cmp = pCodecCtx->me_cmp;
//		c->me_range = pCodecCtx->me_range;
//		c->qmin = 10;
//		c->qmax = 51;
//		c->scenechange_threshold = 40;
//		c->profile = pCodecCtx->profile;//FF_PROFILE_H264_BASELINE;
		c->flags = CODEC_FLAG_GLOBAL_HEADER;
//		c->me_method = ME_HEX;
//		c->me_subpel_quality = 5;
//		c->i_quant_factor = 0.71;
//		c->qcompress = 0.6;
//		c->max_qdiff = 4;
//		c->directpred = 1;
//		c->flags2 |= CODEC_FLAG2_FASTPSKIP;



		
//		c->max_b_frames = pCodecCtx->max_b_frames;
//		c->mb_decision = pCodecCtx->mb_decision;
		
//		if (c->codec_id == CODEC_ID_MPEG2VIDEO) {
//			/* just for testing, we also add B frames */
//			c->max_b_frames = 2;
//		}
//		if (c->codec_id == CODEC_ID_MPEG1VIDEO){
//			/* Needed to avoid using macroblocks in which some coeffs overflow.
//			This does not happen with normal video, it just happens here as
//			the motion of the chroma plane does not match the luma plane. */
//			c->mb_decision=2;
//		}
//		// some formats want stream headers to be separate
//		if(oc->oformat->flags & AVFMT_GLOBALHEADER){
//			c->flags |= CODEC_FLAG_GLOBAL_HEADER;
//		}

		/* find the video encoder */
		codec = avcodec_find_encoder(c->codec_id);
		if (!codec) {
		        fprintf(stderr, "codec not found\n");
		        exit(1);
		}

		/* open the codec */
		if (avcodec_open(c, codec) < 0) {
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

//	/* now that all the parameters are set, we can open the audio and video codecs and allocate the necessary encode buffers */
//	if (video_st)
//        	open_video(oc, video_st);

	/* open the output file, if needed */
	if (!(fmt->flags & AVFMT_NOFILE)) {
        	if (avio_open(&oc->pb, outputFilename, AVIO_FLAG_WRITE) < 0) {
        	fprintf(stderr, "Could not open '%s'\n", outputFilename	);
        	return 1;
        	}
    	}

    	/* write the stream header, if any */
    	av_write_header(oc);

	// Read frames and save first five frames to disk
	i=0;


//	if (video_st)
//		video_pts = (double)video_st->pts.val * video_st->time_base.num / video_st->time_base.den;
//	else
//		video_pts = 0.0;

	while(av_read_frame(pFormatCtx, &packet)>=0) {
		// Is this a packet from the video stream?
		if(packet.stream_index==videoStream) {

			AVPacket pkt;
			av_init_packet(&pkt);
			

			// TODO: FOKUS THIS PROBLEM ->time_base sourcen
			if (pkt.pts != AV_NOPTS_VALUE){
				//packet.pts= av_rescale_q(video_st->codec->coded_frame->pts, video_st->codec->time_base, video_st->time_base);
				pkt.pts = av_rescale_q(packet.pts, pCodecCtx->time_base, video_st->time_base);
				pkt.dts = av_rescale_q(packet.dts, pCodecCtx->time_base, video_st->time_base);
			}
			if(pkt.flags == 1)
				pkt.flags |= AV_PKT_FLAG_KEY;
	
			pkt.stream_index= video_st->index;
			pkt.data= packet.data;
			pkt.size= packet.size;

//			pkt.flags |= AV_PKT_FLAG_KEY;
//			pkt.stream_index= video_st->index;
//			pkt.data= packet.data;
//			pkt.size= packet.size;

			printf("pkt: %d, %d, %d, %d\n", (int)pkt.pts, (int)pkt.dts, (int)pkt.flags, (int)pkt.duration);
			printf("packet: %d, %d, %d, %d\n", (int)packet.pts, (int)packet.dts, (int)packet.flags, (int)packet.duration);
			/* write the compressed frame in the media file */
			int ret = -1;
            		ret = av_write_frame(oc, &pkt);

			if(saveAsImg == 1){
				// Decode video frame
				avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, 
						&packet);

				// Did we get a video frame?
				if(frameFinished) {
					// Convert the image from its native format to RGB
					sws_scale
					(
							sws_ctx,
							(uint8_t const * const *)pFrame->data,
							pFrame->linesize,
							0,
							pCodecCtx->height,
							pFrameRGB->data,
							pFrameRGB->linesize
					);

					// Save the frame to disk
					//if(++i<=10)
					i++;
					if (i%5 == 0){
						//SaveFrameToFile(pFrameRGB, pCodecCtx->width, pCodecCtx->height, "mjpeg", i);
						SaveFrameToFile(pFrameRGB, pCodecCtx->width, pCodecCtx->height, "png", i);
					}
					//SaveFrame(pFrameRGB, pCodecCtx->width, pCodecCtx->height, i);
				}
			}
		}

		// Free the packet that was allocated by av_read_frame
		av_free_packet(&packet);
	}

	// closing output
	av_write_trailer(oc);
	/* free the streams */
	for(i = 0; i < oc->nb_streams; i++) {
		av_freep(&oc->streams[i]->codec);
		av_freep(&oc->streams[i]);
	}

	if (!(fmt->flags & AVFMT_NOFILE)) {
		/* close the output file */
        	avio_close(oc->pb);
	}

	/* free the stream */
	av_free(oc);

	// Free the RGB image
	av_free(buffer);
	av_free(pFrameRGB);

	// Free the YUV frame
	av_free(pFrame);

	// Close the codec
	avcodec_close(pCodecCtx);

	// Close the video file
	avformat_close_input(&pFormatCtx);

	return 0;
}
