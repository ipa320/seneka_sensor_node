/*!
*****************************************************************
*
* sony_camera_node.h
*
*
* Copyright (c) 2012
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
*
* Project name: SENEKA
*
* ROS stack name: seneka
*
* ROS package name: SonyGigeCamera
*
*
* Author: Rajib Banik
*
* Supervised by: Matthias Gruhler

* email: Matthias.Gruhler@ipa.fraunhofer.de

* Date of creation: 26.05.2013
*
*
*
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "stdio.h"
#include <opencv2/opencv.hpp>
#include <seneka_srv/zoom.h>
#include <seneka_srv/focus.h>
#include <seneka_srv/videoMode.h>



#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvBuffer.h>
#include <PvStream.h>
#include <PvStreamRaw.h>



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

    bool zoom_in_out(seneka_srv::zoom::Request &req,
                seneka_srv::zoom::Response &res);

    bool focusControl(seneka_srv::focus::Request &req,
                seneka_srv::focus::Response &res);
    bool videoModeNext(seneka_srv::videoMode::Request &req,
                seneka_srv::videoMode::Response &res);

    ros::NodeHandle nh;
    ros::ServiceServer zoom_service_, focus_service_,videoModeNext_service_;
    image_transport::ImageTransport it;
    image_transport::Publisher publish_rgb_image;
    cv_bridge::CvImage out_msg;




private:
    PvInt64 lSize;
    PvBuffer *lBuffers;
    PvDevice lDevice;
    PvResult lResult;
    PvStream lStream;
    PvGenParameterArray *lDeviceParams;
    PvGenInteger *lPayloadSize;
    PvInt64 width_, height_ ;


};

#endif // SONY_CAMERA_NODE_H
