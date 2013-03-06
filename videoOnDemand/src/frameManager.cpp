/*
 * frameManager.cpp
 *
 *  Created on: 21.02.2013
 *      Author: Johannes Goth (cmm-jg)
 */


// own stuff
#include "frameManager.h"
// libraries
#include <boost/thread.hpp>
#include <boost/bind.hpp>
//#include <boost/thread/mutex.hpp>
#include <vector>
//#include <thread>
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


//TODO: delete the following methods after develope phase

void frameManager::threadTestFkt(){
	// TODO: empty - has to be deleted after testing
	ROS_INFO("testing ... threadTestFkt ...!!");
}

void frameManager::testFrameManager(){
	ROS_INFO("testing ... frameManager ...!!");
}

// public methods
frameManager::frameManager() {

	// initialize parameters
	path = "/home/cmm-jg/Bilder/store";
	fileNodeLable = "cvMat";
	maxCacheSize = 200; // 25fps * 30sec = 750frames
	maxRecTime = 2;
	currentFileStorage = 0;
	currentFileStorageSize = 0;
	useCacheA = true;
	useCacheB = false;
	storageThreadActiveA = false;
	storageThreadActiveB = false;
	cacheA = new std::vector<cv::Mat>;
	cacheB = new std::vector<cv::Mat>;
//	mxCacheA.unlock();
//	mxCacheB.unlock();

	// creates a fileStorages file for each 30 seconds at hard disk drive
	// and thereby defines the size of the fileStorages-vector
	for(int i=0; i < (maxRecTime/0.5); i++){
		cv::FileStorage* tmp = new cv::FileStorage();
		fileStorages.push_back(tmp);
	}
}

frameManager::~frameManager() {
	delete cacheA;
	delete cacheB;
}

void frameManager::processFrame(const sensor_msgs::Image& img){

	ROS_INFO("processFrame ... ");

	cv_bridge::CvImageConstPtr cvptrS;
	try
	{
		// convert the sensor_msgs::Image to cv_bridge::CvImageConstPtr
		cvptrS = cv_bridge::toCvShare(img, cvptrS, sensor_msgs::image_encodings::BGR8);

		// TODO: thread concept for calling cacheFrame()-method
		cacheFrame(cvptrS->image);

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void frameManager::cacheFrame(cv::Mat frame){

	ROS_INFO("cacheFrame ... ");

	std::vector<cv::Mat>* currentCache = getCurrentCache();
	currentCache->push_back(frame);
}

std::vector<cv::Mat>* frameManager::getCurrentCache(){

	ROS_INFO("getCurrentCache ... ");

	// checks the sizes of memory buffers and if necessary it will schedule further tasks
	verifyCacheSize();

	if(useCacheA == true && useCacheB == false){
		return cacheA;
	}
	else if (useCacheB == true && useCacheA == false){
		return cacheB;
	}
	else{
		std::cerr << "Could not get current cache ..." << std::endl;
		return NULL;
	}
}
/*
boost::mutex* frameManager::getMutexToCache(bool usedCache){
	if(usedCache){
		if(useCacheA == true && useCacheB == false){
			return &mxCacheA;
		}
		else if (useCacheB == true && useCacheA == false){
			return &mxCacheB;
		}
	}else{
		if(useCacheA == false && useCacheB == true){
			return &mxCacheA;
		}
		else if (useCacheB == false && useCacheA == true){
			return &mxCacheB;
		}
	}
	std::cerr << "ERROR in getMutexToCache()" << std::endl;
	return NULL;
}
*/

void frameManager::verifyCacheSize(){

	ROS_INFO("verifyCacheSize ... ");

	if(cacheA->size() == maxCacheSize && storageThreadActiveA == false){// && cacheB->size() < maxCacheSize){
		// cacheA is full -> store frame to fileStorages

		storageThreadActiveA = true;
		useCacheA = false;
		useCacheB = true;

		ROS_INFO(">-> waiting for storageThreadActiveB");
		storageThreadB.join();
		storageThreadA = boost::thread(boost::bind(&frameManager::storeCache, this, cacheA, &storageThreadActiveA));

//		storeCache(cacheA);
	}
	else if (cacheB->size() == maxCacheSize && storageThreadActiveB == false){// && cacheA->size() < maxCacheSize){
		// cacheB is full -> store frame to fileStorage

		storageThreadActiveB = true;
		useCacheB = false;
		useCacheA = true;

		ROS_INFO(">-> waiting for storageThreadActiveA");
		storageThreadA.join();

		// TODO: 	testweiser Aufruf der createVideo() methode nach 3 Durchläufen
		if(currentFileStorage > 2)
			createVideo();
		storageThreadB = boost::thread(boost::bind(&frameManager::storeCache, this, cacheB, &storageThreadActiveB));

//		storeCache(cacheB);
	}
}

void frameManager::storeCache(std::vector<cv::Mat>* cache, bool* threadActive){

	ROS_INFO(">->-> storeCache ...");

	// define fileStorage-filename
	std::stringstream fileName;
	fileName << path << currentFileStorage << ".yml";

	fileStorages[currentFileStorage] = new cv::FileStorage();
	(*fileStorages[currentFileStorage]).open(fileName.str(), cv::FileStorage::WRITE);

	if((*fileStorages[currentFileStorage]).isOpened()){
		for (std::vector<cv::Mat>::iterator it = cache->begin() ; it != cache->end(); it++){

			std::stringstream elementName;
			elementName << fileNodeLable << currentFileStorageSize;

			// store frame to the current file storage
			*fileStorages[currentFileStorage] << elementName.str() << *it;
			currentFileStorageSize++;
		}

		// close fileStorage-file
		(*fileStorages[currentFileStorage]).release();

//		TODO: reset currentFileStorageSize just if 750 frames reached
//		problem: filesStorages können in der momentanen implementierung größer werden wie 750 frames ... abhängig von der chache size

		// reset currentFileStorageSize
		currentFileStorageSize = 0;

		// check if max. record time is reached
		if(currentFileStorage < maxRecTime/0.5)
			currentFileStorage++;
		else
			currentFileStorage = 0;

		// clean cache
		cache->clear();
		*threadActive = false;
		ROS_INFO(">->->-> finished thread");
	}
	else{
		std::cerr << "ERROR: Not able to open " <<  fileName << std::endl;
		return;
	}
}

int frameManager::getVideo(){
	// calls create video method

	createVideo();

	return -1;
}

int frameManager::createVideo(){

	ROS_INFO("createVideo ...");

	// TODO: 	video kann erzeugt werden, momentan aus einnem input file
	// 			muss erweitert werden auf bsp 4 input files, je nach gewünschter video länge

	std::stringstream inputFileName;
	inputFileName << path << (currentFileStorage-1) << ".yml";
	std::stringstream outputFileName;
	outputFileName << "/home/cmm-jg/Bilder/videoOnDemand.avi";

	// open inputFile
	cv::FileStorage* inputFile = new cv::FileStorage();
	(*inputFile).open(inputFileName.str(), cv::FileStorage::READ);
	cv::Mat matInst;
	std::stringstream matrix;
	matrix << fileNodeLable << 0;
	(*inputFile)[matrix.str()] >> matInst;


	// create a videoRecorder instance
	videoRecorder* vRecoder = new videoRecorder();
	// define the video parameters
	vRecoder->createVideo(outputFileName.str(), matInst.cols, matInst.rows);

	for(u_int i=0; i<maxCacheSize; i++){
		std::stringstream matrix;
		matrix << fileNodeLable << i;
		(*inputFile)[matrix.str()] >> matInst;
		vRecoder->addFrame(matInst);
	}

	// close and release video file
	vRecoder->releaseVideo();


	//	access to a file storage is possible via a file node
	//	cv::FileNode* n = new cv::FileNode();
	//	n->size();


	return -1;
}
