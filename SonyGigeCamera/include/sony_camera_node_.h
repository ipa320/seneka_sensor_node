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
    void disconnectCamera();
    void startStreaming();
    void stopStreaming();
    //void publishImage(image_transport::Publisher pub_rgb_mage);
    void publishImage();

      ros::NodeHandle nh;
      image_transport::ImageTransport it;
      image_transport::Publisher rgb_image;
    cv_bridge::CvImage out_msg;

private:
    //PvString address;
    PvBuffer *lBuffers;
    PvBuffer *lBuffer;
    PvResult lOperationResult;
    PvImage *Image;
    PvDevice lDevice;
    PvResult lResult;
    PvStream lStream;
    PvDeviceInfo *device_info_;
    //PvGenParameterArray *device_params_;
    PvGenParameterArray *lDeviceParams;
    PvGenInteger *lPayloadSize;
    PvGenCommand *lStart ;
    PvGenCommand *lStop;

    PvInt64 width_, height_;

};

#endif // SONY_CAMERA_NODE_H
