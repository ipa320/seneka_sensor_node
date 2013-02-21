/*
 * author: Johannes Goth (cmm-jg)
 *
 */

#include "ros/ros.h"
#include <queue>
#include "sensor_msgs/Image.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>

namespace enc = sensor_msgs::image_encodings;

int matCounter = 0;

class VideoRecorder {
public:

	VideoRecorder(){
		videocreated = false;
		codec = CV_FOURCC('D','I','V','X');
		fps = 25;
		frameWidth = 0;
		frameHeight = 0;
	}

	void createVideo(std::string vf, int frameWidth, int frameHeight){
		videoFile = vf;
		videoSize.height = frameHeight;
		videoSize.width = frameWidth;
		videocreated = true;
		openVideo();
	}

	void addFrame(cv::Mat frame){
		if(videoWriter.isOpened() == true){
			videoWriter.write(frame);
		}
		else{
			ROS_INFO("Video file is closed ...");
		}
	}

	bool existsVideoRecorder(){
		return videocreated;
	}

	void closeVideo(){
		videoWriter.release();
	}


private:
	int codec;
	int fps;
	int frameWidth;
	int frameHeight;
	bool videocreated;
	cv::VideoWriter videoWriter;
	std::string videoFile;
	CvSize videoSize;

	void openVideo(){
		videoWriter.open(videoFile, codec, fps, videoSize, 1);
	}

};

class FrameRingBuffer {
public:
	FrameRingBuffer(){
		maxRingBufferSize = 1000;
		releases = 1;
		vRecoder = new VideoRecorder();

		// mod: fileStorage
		fs.open("/home/cmm-jg/Bilder/test.yml", cv::FileStorage::WRITE);
		fs << "name" << "johannes";
	}
	~FrameRingBuffer(){
		fs.release();
	}
	void addFrame(cv::Mat frame){

		fs << "matCounter" << frame;
		matCounter++;

		if(ringbuffer.size() != maxRingBufferSize)
			ringbuffer.push(frame);
		else{
			ringbuffer.pop();
			ringbuffer.push(frame);
		}
	}
	void getVideo(){
		ROS_INFO("creating video on demand ...");

		videoFile << "/home/cmm-jg/Bilder/videoOnDemand" << releases << ".avi";

		std::queue<cv::Mat> tmpRingBuffer = ringbuffer;
		vRecoder->createVideo(videoFile.str(), tmpRingBuffer.front().cols, tmpRingBuffer.front().rows);

		if(vRecoder->existsVideoRecorder() == true){
			while(tmpRingBuffer.empty() != true){
				vRecoder->addFrame(tmpRingBuffer.front());
				tmpRingBuffer.pop();
			}
			vRecoder->closeVideo();
			releases++;
		}
	}

private:
	std::queue<cv::Mat> ringbuffer;
	u_int maxRingBufferSize;
	u_int releases;

	VideoRecorder* vRecoder;
	std::stringstream videoFile;

	cv::FileStorage fs;
};

void transmitImage(){
	// TODO: creating a socket connection to remote or local host
}

// global attributes of ros node
const float SNAPSHOT_INTERVAL = 25;
int imageCounter = 0;
VideoRecorder* videoRecorder = new VideoRecorder();
FrameRingBuffer* fRingBuffer;


void videoCallback(const sensor_msgs::Image& img)
{
	//ROS_INFO("Video callback method ...");

	// openCV image pointer
	cv_bridge::CvImageConstPtr cvptrS;

	std::stringstream imgFile;
	std::string videoFile = "/home/cmm-jg/Bilder/video.avi";

	if(videoRecorder->existsVideoRecorder() == false){
		videoRecorder->createVideo(videoFile, img.width, img.height);
	}

	try
	{
		// convert the sensor_msgs::Image to cv_bridge::CvImageConstPtr
		cvptrS = cv_bridge::toCvShare(img, cvptrS, enc::BGR8);

/*
		// adding image to videoRecorder as frame
		if(videoRecorder->existsVideoRecorder() == true)
			videoRecorder->addFrame(cvptrS->image);
*/
		fRingBuffer->addFrame(cvptrS->image);
		imageCounter++;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "videoCreator");

	// main access point to communications with the ROS system
	ros::NodeHandle n;
	fRingBuffer = new FrameRingBuffer();

	ROS_INFO("subscribing for thermal_image_view ...");
	ros::Subscriber sub = n.subscribe("/optris/thermal_image_view", 2, videoCallback);

	// frequency in Hz
	ros::Rate loop_rate(SNAPSHOT_INTERVAL);

	// trigger for video on demand
	int tmpCounter = 0;

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();

		if(tmpCounter==1000){
			fRingBuffer->getVideo();
		}
		else if (tmpCounter==500) {
			fRingBuffer->getVideo();
		}

		tmpCounter++;
	}

	if(videoRecorder->existsVideoRecorder() == true)
		videoRecorder->closeVideo();

	return 0;
}
