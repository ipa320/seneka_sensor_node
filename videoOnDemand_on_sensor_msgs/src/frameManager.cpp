/*
 * frameManager.cpp
 *
 *  Created on: 21.02.2013
 *      Author: Johannes Goth (cmm-jg)
 */

// std and own includes
#include "frameManager.h"
//#include "boost_serialization_cvMat.h"
#include "boost_serialization_sensor_msgs_image.h"
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
//#include <boost/Image.h>

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

// public member functions
FrameManager::FrameManager() {

	// initialize parameters
	binaryFilePath = "/home/cmm-jg/Bilder/container";
	videoFilePath = "/home/cmm-jg/Bilder/videoOnDemand.avi";
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
	cacheA = new std::vector<sensor_msgs::Image>;
	cacheB = new std::vector<sensor_msgs::Image>;
	iBuilder.setPaletteScalingMethod(optris::eMinMax);
	iBuilder.setPalette(optris::eIron);
	iBuilder.setManualTemperatureRange((float)20, (float)40);
	showFrame = false;

	for(int i=0; i < (int)(fpv/fpb)+1; i++){
		binaryFileMutexes.push_back(new boost::mutex());
	}
}

FrameManager::FrameManager(ros::NodeHandle &nHandler) {

	// initialize configurable parameters
	int tmp_fpv, tmp_fpc, tmp_fpb, tmp_vfr;

	nHandler.getParam("framesPerVideo", tmp_fpv);
	nHandler.getParam("framesPerCache", tmp_fpc);
	nHandler.getParam("framesPerBinary", tmp_fpb);
	nHandler.getParam("videoFrameRate", tmp_vfr);
	nHandler.getParam("binaryFilePath", binaryFilePath);
	nHandler.getParam("videoFilePath", videoFilePath);

	if(tmp_fpv>0) fpv=(u_int)tmp_fpv;
	if(tmp_fpc>0) fpc=(u_int)tmp_fpc;
	if(tmp_fpb>0) fpb=(u_int)tmp_fpb;
	if(tmp_vfr>0) vfr=(u_int)tmp_vfr;

	// initialize fixed parameters
	videoCodec = CV_FOURCC('D','I','V','X');
	binaryFileIndex = 0;
	fullVideoAvailable = false;
	createVideoActive = false;
	usingCacheA = true;
	usingCacheB = false;
	storingCacheA = false;
	storingCacheB = false;
	cacheA = new std::vector<sensor_msgs::Image>;
	cacheB = new std::vector<sensor_msgs::Image>;

	// TODO: make as configurable params
	iBuilder.setPaletteScalingMethod(optris::eMinMax);
	iBuilder.setPalette(optris::eIron);
	iBuilder.setManualTemperatureRange((float)20, (float)40);
	showFrame = true;

	for(int i=0; i < (int)(fpv/fpb)+1; i++){
		binaryFileMutexes.push_back(new boost::mutex());
	}
}

FrameManager::~FrameManager() {
	delete cacheA;
	delete cacheB;
}

void FrameManager::processFrame(const sensor_msgs::Image& img){
	//ROS_INFO("processFrame ... ");
	cacheFrame(img);
}

void FrameManager::cacheFrame(sensor_msgs::Image frame){
	//ROS_INFO("cacheFrame ... ");
	std::vector<sensor_msgs::Image>* currentCache = getCurrentCache();
	currentCache->push_back(frame);
}

std::vector<sensor_msgs::Image>* FrameManager::getCurrentCache(){
	//ROS_INFO("getCurrentCache ... ");

	// proofs the sizes of memory buffers and if necessary it will schedule further tasks
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
		// cacheA is full -> store frame to fileStorages

		storingCacheA = true;
		usingCacheA = false;
		usingCacheB = true;

		ROS_INFO("Waiting for storingCacheB");
		storingThreadB.join();
		storingThreadA = boost::thread(boost::bind(&FrameManager::storeCache, this, cacheA, &storingCacheA));
	}
	else if (cacheB->size() == fpc && storingCacheB == false){
		// cacheB is full -> store frame to fileStorage

		storingCacheB = true;
		usingCacheB = false;
		usingCacheA = true;

		//		std::cout << "thread ID: "<< boost::this_thread::get_id() << std::endl;
		ROS_INFO("Waiting for storingCacheA");
		storingThreadA.join();
		storingThreadB = boost::thread(boost::bind(&FrameManager::storeCache, this, cacheB, &storingCacheB));
	}
}

void FrameManager::storeCache(std::vector<sensor_msgs::Image>* cache, bool* threadActive){

	ROS_INFO("storeCache into binary file...");
	//std::cout << "thread ID: "<< boost::this_thread::get_id() << std::endl;
	// define fileStorage-filename
	std::stringstream fileName;
	fileName << binaryFilePath << binaryFileIndex << ".bin";

	// lock current binary file as output
	binaryFileMutexes[binaryFileIndex]->lock();
	std::ofstream ofs(fileName.str().c_str(), std::ios::out | std::ios::binary);

	// scope is required to ensure archive and filtering stream buffer go out of scope
	// before stream
	{
		// compressing frame size
		//      io::filtering_streambuf<io::output> out;
		//      out.push(io::zlib_compressor(io::zlib::best_speed));
		//      out.push(ofs);

		boost::archive::binary_oarchive oa(ofs);

		// writes each frame which is stored in cache into binary file
		for (std::vector<sensor_msgs::Image>::iterator it = cache->begin() ; it != cache->end(); it++){
			// write frame into binary file, using cv:Mat serialization
			oa << *it;
		}
	}

	// close file
	ofs.close();
	// unlock current binary file
	binaryFileMutexes[binaryFileIndex]->unlock();

	// current const. allocation -> has to be adapted dynamic
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

	if(fullVideoAvailable == true){
		if(createVideoActive == false){
			creatingVideoThread = boost::thread(boost::bind(&FrameManager::createVideo, this));
			return 1;
		}
		else
			return -1;
	}
	else
		return -2;
}

int FrameManager::createVideo(){

	ROS_INFO("createVideo ...");

	createVideoActive = true;

	// create a videoRecorder instance
	VideoRecorder* vRecoder = new VideoRecorder(vfr, videoCodec);
	bool firstFrame = true;
	unsigned int frameCount = 0;

	/* start point binary for video
	 * selecting binaryFileIndex + 2 to get a buffer for storing new frames to a binary
	 */
	int currentIndex = binaryFileIndex + 2;
	int numBinaries = fpv/fpb;

	// add all binaries which are required (numBinaries) into a video
	for(int i=0; i < numBinaries; i++){
		std::stringstream inputFileName;
		int mutexID;

		if(currentIndex < numBinaries-1){
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
		// open inputFile
		std::ifstream ifs(inputFileName.str().c_str(), std::ios::in | std::ios::binary);

		// scope is required to ensure archive and filtering stream buffer go out of scope
		// before stream
		{
			// decompressing frame size
			//    	io::filtering_streambuf<io::input> in;
			//    	in.push(io::zlib_decompressor());
			//    	in.push(ifs);

			boost::archive::binary_iarchive ia(ifs);

			bool cont = true;
			while (cont)
			{
				sensor_msgs::Image loadedFrame;
				//std::cout << "loading image from binary file ... "<< std::endl;
				if(firstFrame){
					cont = boost::serialization::try_stream_next(ia, ifs, loadedFrame);
					if (cont == true){
						cv::Mat mat = converter(&loadedFrame, &frameCount);
						// define the video parameters
						vRecoder->createVideo(videoFilePath, mat.cols, mat.rows);
						vRecoder->addFrame(mat);
						firstFrame = false;
					}

				}
				else{
					cont = boost::serialization::try_stream_next(ia, ifs, loadedFrame);
					if (cont == true)
						vRecoder->addFrame(converter(&loadedFrame, &frameCount));
				}
			}
			ifs.close();
			// unlock current binary file
			binaryFileMutexes[mutexID]->unlock();
		}
	}
	// release video
	vRecoder->releaseVideo();
	createVideoActive = false;
	ROS_INFO("finished createVideo ...");

	return -1;
}

cv::Mat FrameManager::converter(sensor_msgs::Image* frame, unsigned int* frameCount){

	unsigned char* buffer = NULL;

	buffer = new unsigned char[frame->width * frame->height * 3];

	iBuilder.setSize(frame->width, frame->height, false);

	const unsigned char* data = &(*frame).data[0];
	iBuilder.convertTemperatureToPaletteImage((unsigned short*)data, buffer);

	sensor_msgs::Image rgb_img;
	rgb_img.header.frame_id = "thermal_image_view";
	rgb_img.height 	        = frame->height;
	rgb_img.width 	        = frame->width;
	rgb_img.encoding        = "rgb8";
	rgb_img.step		    = frame->width*3;
	rgb_img.data.resize(rgb_img.height*rgb_img.step);

	rgb_img.header.seq      = *frameCount++;
	rgb_img.header.stamp    = ros::Time::now();

	for(unsigned int i=0; i< frame->width* frame->height*3; i++)
	{
		rgb_img.data[i] = buffer[i];
	}

	cv_bridge::CvImageConstPtr cvptrS;
	try
	{
		// convert the sensor_msgs::Image to cv_bridge::CvImageConstPtr
		cvptrS = cv_bridge::toCvShare(rgb_img, cvptrS, sensor_msgs::image_encodings::BGR8);
		cv::Mat mat = cvptrS->image;
		if(showFrame)
			displayFrame(&mat);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	return cvptrS->image;
}

void FrameManager::displayFrame(cv::Mat* mat){
	cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
	cv::imshow( "Display window", *mat );
	cv::waitKey(1);
}
