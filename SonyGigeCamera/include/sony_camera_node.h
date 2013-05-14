#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
//#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "stdio.h"
#include <opencv2/opencv.hpp>

#include <PvSampleUtils.h>
#include <PvDeviceFinderWnd.h>
#include <PvDevice.h>
#include <PvBuffer.h>
#include <PvStream.h>
#include <PvStreamRaw.h>
#include <PvBufferConverter.h>
#include <PvBufferWriter.h>


#ifndef SONY_CAMERA_NODE_H
#define SONY_CAMERA_NODE_H

class Sony_Camera_Node
{

public:
    Sony_Camera_Node();

    ~Sony_Camera_Node();

    void connectCamera();
    void startStreaming();
    void stopStreaming();
    void publishImage();
    void disconnectCamera();

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher publish_rgb_image;
    cv_bridge::CvImage out_msg;

private:
    PvInt64 lSize;
    PvBuffer *lBuffers;
    PvDevice lDevice;
    PvResult lResult;
    PvStream lStream;
    PvDeviceInfo *device_info_;
    PvGenParameterArray *lDeviceParams;
    PvGenInteger *lPayloadSize;
    PvGenCommand *lStart ;
    PvGenCommand *lStop;
    //char lDoodle[];
    //int lDoodleIndex;

    PvInt64 width_, height_ ;


};

#endif // SONY_CAMERA_NODE_H
