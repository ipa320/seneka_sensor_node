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
	snapshotRunning = false;
	liveStreamRunning = false;
	stateMachine = ON_DEMAND;

	// live stream specific
	cacheC = new std::vector<cv::Mat>;
	cacheD = new std::vector<cv::Mat>;
	usingCacheC = true;
	usingCacheD = false;
	storingCacheC = false;
	storingCacheD = false;
	liveStreamFrameCount = 0;

	for(int i=0; i < (int)(fpv/fpb)+1; i++){
		binaryFileMutexes.push_back(new boost::mutex());
	}
}

FrameManager::FrameManager(ros::NodeHandle &nHandler) {

	// initialize configurable parameters
	int tmp_fpv, tmp_fpc, tmp_fpb, tmp_vfr, tmp_PaletteScalingMethod, tmp_Palette, minTemperature, maxTemperature;

	nHandler.getParam("framesPerVideo", tmp_fpv);
	nHandler.getParam("framesPerCache", tmp_fpc);
	nHandler.getParam("framesPerBinary", tmp_fpb);
	nHandler.getParam("videoFrameRate", tmp_vfr);
	nHandler.getParam("binaryFilePath", binaryFilePath);
	nHandler.getParam("videoFilePath", videoFilePath);
	nHandler.getParam("showFrame", showFrame);
	nHandler.getParam("minTemperature", minTemperature);
	nHandler.getParam("maxTemperature", maxTemperature);
	nHandler.getParam("PaletteScalingMethod", tmp_PaletteScalingMethod);
	nHandler.getParam("Palette", tmp_Palette);


	if(tmp_fpv>0) fpv=(u_int)tmp_fpv;
	if(tmp_fpc>0) fpc=(u_int)tmp_fpc;
	if(tmp_fpb>0) fpb=(u_int)tmp_fpb;
	if(tmp_vfr>0) vfr=(u_int)tmp_vfr;

	iBuilder.setPaletteScalingMethod((optris::EnumOptrisPaletteScalingMethod)tmp_PaletteScalingMethod);
	iBuilder.setPalette((optris::EnumOptrisColoringPalette)tmp_Palette);
	iBuilder.setManualTemperatureRange((float)minTemperature, (float)maxTemperature);

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
	snapshotRunning = false;
	liveStreamRunning = false;
	stateMachine = ON_DEMAND;

	// live stream specific
	cacheC = new std::vector<cv::Mat>;
	cacheD = new std::vector<cv::Mat>;
	usingCacheC = true;
	usingCacheD = false;
	storingCacheC = false;
	storingCacheD = false;
	liveStreamFrameCount = 0;

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
	// caching current frame in memory
	cacheFrame(img);
}

void FrameManager::cacheFrame(sensor_msgs::Image frame){
	//ROS_INFO("cacheFrame ... ");

	if(stateMachine == ON_DEMAND){
		std::vector<sensor_msgs::Image>* currentCache = getCurrentCache();
		currentCache->push_back(frame);
	}
	else if(stateMachine == LIVE_STREAM){
		// acccess the current live stream cache
		std::vector<cv::Mat>* currentCache = getCurrentLiveStreamCache();
		// converting and caching of current frame
		currentCache->push_back(convertTemperatureValuesToRGB(&frame, &liveStreamFrameCount));
	}
	else{
		ROS_ERROR("Unknown state for the state machine");
	}


}
std::vector<sensor_msgs::Image>* FrameManager::getCurrentCache(){
	//ROS_INFO("getCurrentCache ... ");

	// verifying the sizes of memory buffers
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

std::vector<cv::Mat>* FrameManager::getCurrentLiveStreamCache(){
	//ROS_INFO("getCurrentCache ... ");

	// verifying the sizes of memory buffers
	// and if necessary it will schedule further tasks
	verifyCacheSize();

	if(usingCacheC == true && usingCacheD == false){
		return cacheC;
	}
	else if (usingCacheD == true && usingCacheC == false){
		return cacheD;
	}
	else{
		std::cerr << "Could not get current cache ..." << std::endl;
		return NULL;
	}
}

void FrameManager::verifyCacheSize(){
	//ROS_INFO("verifyCacheSize ... ");

	if(stateMachine == ON_DEMAND){
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
	else if(stateMachine == LIVE_STREAM){
		if(cacheC->size() == fpc && storingCacheC == false){
			// if this cache is full, change the current to use cache and clear is cache
			// at the moment der isn't a method required to store a cache to the hard disk drive
			usingCacheC = false;
			usingCacheD = true;
			cacheC->clear();
		}
		else if (cacheB->size() == fpc && storingCacheB == false){
			// if this cache is full, change the current to use cache and clear is cache
			// at the moment der isn't a method required to store a cache to the hard disk drive

			usingCacheD = false;
			usingCacheC = true;
			cacheD->clear();
		}
	}
	else{
		ROS_ERROR("Unknown state for the state machine");
	}

}

void FrameManager::storeCache(std::vector<sensor_msgs::Image>* cache, bool* threadActive){

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
		//		 	 //compressing frame size
		//		      io::filtering_streambuf<io::output> out;
		//		      out.push(io::zlib_compressor(io::zlib::best_speed));
		//		      out.push(ofs);

		boost::archive::binary_oarchive oa(ofs);

		// writes each frame which is stored in cache into binary file
		for (std::vector<sensor_msgs::Image>::iterator it = cache->begin() ; it != cache->end(); it++){
			// writes frame per frame into binary file, using sensor_msgs::Image serialization
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

	if(stateMachine == ON_DEMAND){
		// a full video is available if the minimal count of frames is reached (minimal count = frame per video)
		if(fullVideoAvailable == true){
			// it is only possible to create one video at a time
			if(createVideoActive == false){
				// starts creating the video in a separate thread
				creatingVideoThread = boost::thread(boost::bind(&FrameManager::createVideo, this));
				return 1;
			}
			else
				return -1;
		}
		else
			return -2;
	}
	else if(stateMachine == LIVE_STREAM){
		ROS_ERROR("This function is in LIVE_STREAM state not available!");
		return -2;
	}
	else{
		ROS_ERROR("Unknown state for the state machine");
		return -2;
	}


}

int FrameManager::createVideo(){

	ROS_INFO("createVideo ...");

	createVideoActive = true;

	// create a videoRecorder instance
	VideoRecorder* vRecoder = new VideoRecorder(vfr, videoCodec);
	bool firstFrame = true;
	unsigned int frameCount = 0;

	/* start binary for video creation
	 * selecting binaryFileIndex + 1 to get the oldest binary file,
	 * which includes the first frame for the video creation
	 */
	int currentIndex = binaryFileIndex + 1;
	int numBinaries = fpv/fpb;

	// add all binaries which are required (numBinaries) into a video
	for(int i=0; i < numBinaries; i++){
		std::stringstream inputFileName;
		int mutexID;

		if(currentIndex < numBinaries){
			// load currentIndex - numBinaries-1
			inputFileName << binaryFilePath << (currentIndex) << ".bin";
			mutexID = currentIndex;
			std::cout << "currentIndex: " << currentIndex << " mutexID: " << mutexID << std::endl;
			currentIndex++;

		}
		else{
			// load currentIndex - numBinaries-1
			inputFileName << binaryFilePath << (currentIndex-numBinaries) << ".bin";
			mutexID = currentIndex-numBinaries;
			std::cout << "currentIndex: " << currentIndex << " mutexID: " << mutexID << std::endl;
			currentIndex++;
		}

		// lock current binary file as input
		binaryFileMutexes[mutexID]->lock();
		// open inputFile
		std::ifstream ifs(inputFileName.str().c_str(), std::ios::in | std::ios::binary);

		// scope is required to ensure archive and filtering stream buffer go out of scope
		// before stream
		{
			//			// decompressing frame size
			//			io::filtering_streambuf<io::input> in;
			//			in.push(io::zlib_decompressor());
			//			in.push(ifs);

			boost::archive::binary_iarchive ia(ifs);

			bool hasContent = true;
			while (hasContent)
			{
				sensor_msgs::Image loadedFrame;
				if(firstFrame){
					// try to read a temperature image from binary file
					hasContent = boost::serialization::try_stream_next(ia, ifs, loadedFrame);
					if (hasContent == true){
						// convert temperature image (sensor_msgs::Image) to RGB image (cv::Mat)
						cv::Mat mat = convertTemperatureValuesToRGB(&loadedFrame, &frameCount);
						// define video parameters
						vRecoder->createVideo(videoFilePath, mat.cols, mat.rows);
						// add frame to video
						vRecoder->addFrame(mat);
						firstFrame = false;
					}

				}
				else{
					// try to read a temperature image from binary file
					hasContent = boost::serialization::try_stream_next(ia, ifs, loadedFrame);
					// convert and add frame to video
					if (hasContent == true)
						vRecoder->addFrame(convertTemperatureValuesToRGB(&loadedFrame, &frameCount));
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

cv::Mat FrameManager::convertTemperatureValuesToRGB(sensor_msgs::Image* frame, unsigned int* frameCount){

	unsigned char* buffer = NULL;
	buffer = new unsigned char[frame->width * frame->height * 3];

	iBuilder.setSize(frame->width, frame->height, false);
	const unsigned char* data = &(*frame).data[0];
	iBuilder.convertTemperatureToPaletteImage((unsigned short*)data, buffer);

	// create RGB sensor_msgs::image
	sensor_msgs::Image rgb_img;
	rgb_img.header.frame_id = "thermal_image_view";
	rgb_img.height 	        = frame->height;
	rgb_img.width 	        = frame->width;
	rgb_img.encoding        = "rgb8";
	rgb_img.step		    = frame->width*3;
	rgb_img.data.resize(rgb_img.height*rgb_img.step);

	*frameCount = *frameCount + 1;
	rgb_img.header.seq      = *frameCount;
	rgb_img.header.stamp    = ros::Time::now();

	for(unsigned int i=0; i< frame->width* frame->height*3; i++)
	{
		rgb_img.data[i] = buffer[i];
	}

	cv_bridge::CvImageConstPtr cvptrS;
	try
	{
		// convert the sensor_msgs::Image to cv_bridge::CvImageConstPtr (cv::Mat)
		cvptrS = cv_bridge::toCvShare(rgb_img, cvptrS, sensor_msgs::image_encodings::BGR8);
		cv::Mat mat = cvptrS->image;

		// show frame, if configured in the launch file
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

void FrameManager::startSnapshots(int interval){

	if(!isSnapShotRunning()){
		snapshotThread = boost::thread(boost::bind(&FrameManager::createSnapshots, this, interval));
		snapshotRunning = true;
	}
}

void FrameManager::stopSnapshots(){
	// call interrupt point of snapshotThread
	snapshotThread.interrupt();
}

void FrameManager::createSnapshots(int interval){
	ROS_INFO("Starting creating snapshots ...");
	unsigned int imageCounter = 0;
	while(true){
		if(stateMachine == ON_DEMAND){
			std::stringstream imgFile;
			std::vector<sensor_msgs::Image>* currentCache = getCurrentCache();

			if(currentCache->size() > 0){
				sensor_msgs::Image img = currentCache->at(currentCache->size()-1);

				// file name and path to the image files
				// file name is the current system time stamp
				imgFile << "/home/cmm-jg/Bilder/snapshots/" << ros::Time::now() << ".jpg";

				// create JPEG image
				cv::Mat m = convertTemperatureValuesToRGB(&img, &imageCounter);
				cv::imwrite(imgFile.str(), m);

				// try to set thread to sleep or if an interrupt occurred stop thread
				try{
					// set thread sleeping for the chosen interval
					boost::this_thread::sleep(boost::posix_time::milliseconds(interval*1000));
				}
				catch(boost::thread_interrupted const& )
				{
					// defined actions if an interrupt occurred
					snapshotRunning = false;
					ROS_INFO("Stopped creating snapshots ...");
					break;
				}
			}
		}
		else if(stateMachine == LIVE_STREAM){
			std::stringstream imgFile;
			std::vector<cv::Mat>* currentCache = getCurrentLiveStreamCache();

			if(currentCache->size() > 0){
				cv::Mat img = currentCache->at(currentCache->size()-1);

				// file name and path to the image files
				// file name is the current system time stamp
				imgFile << "/home/cmm-jg/Bilder/snapshots/" << ros::Time::now() << ".jpg";

				// create JPEG image
				cv::imwrite(imgFile.str(), img);

				// try to set thread to sleep or if an interrupt occurred stop thread
				try{
					// set thread sleeping for the chosen interval
					boost::this_thread::sleep(boost::posix_time::milliseconds(interval*1000));
				}
				catch(boost::thread_interrupted const& )
				{
					// defined actions if an interrupt occurred
					snapshotRunning = false;
					ROS_INFO("Stopped creating snapshots ...");
					break;
				}
			}
		}
		else{
			ROS_ERROR("Unknown state for the state machine");
		}
	}
}

void FrameManager::startLiveStream(){
	ROS_INFO("Starting live stream mode ...");

	if(createVideoActive){
		ROS_ERROR("State change not possible ... creating video at the moment !!");
	}
	else{
		// initializie the LIVE_STREAM state
		stateMachine = LIVE_STREAM;
		cacheA->clear();
		cacheB->clear();
		usingCacheC = true;
		usingCacheD = false;
		liveStreamFrameCount = 0;
		liveStreamRunning = true;

		// at the moment the function for the live stream creation is missing
	}
}

void FrameManager::stopLiveStream(){
	ROS_INFO("Stopped live stream mode ...");

	stateMachine = ON_DEMAND;
	cacheC->clear();
	cacheD->clear();
	liveStreamRunning = false;
	// initialize ON_DEMAND state
	usingCacheA = true;
	usingCacheB = false;
	storingCacheA = false;
	storingCacheB = false;
	fullVideoAvailable = false;
}


