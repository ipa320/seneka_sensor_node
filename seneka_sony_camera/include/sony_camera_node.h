/****************************************************************
*
* Copyright (c) 2014
*
* Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Project name: SeNeKa
* ROS metapackage: seneka_sensor_node
* ROS package: seneka_sony_camera
* GitHub repository: https://github.com/ipa320/seneka_sensor_node
* 
* Package description: The seneka_sony_camera package is part of the
* seneka_sensor_node metapackage, developed for the SeNeKa project at
* Fraunhofer IPA. It implements a ROS driver for the Sony Block Camera
* FCB EH 6300. This package might work with other hardware and can be used
* for other purposes, however the development has been specifically for this
* project and the deployed sensors.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Supervisor: Matthias Gruhler, E-Mail: Matthias.Gruhler@ipa.fraunhofer.de
* Author: Rajib Banik
*
* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Date of creation: 26.05.2013
*
* Modified by: Thorsten Andreas Kannacher, E-Mail: Thorsten.Andreas.Kannacher@fraunhofer.de
* Date of modification: 17.03.2014
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
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

// standart headers
#include "stdio.h"

// ros headers
#include <ros/ros.h>

// ros service headers
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

// seneka service headers
#include <seneka_srv/zoom.h>
#include <seneka_srv/focusAuto.h>
#include <seneka_srv/focus.h>
#include <seneka_srv/videoMode.h>
#include <seneka_srv/focusNearLimit.h>
#include <seneka_srv/infraredCutFilter.h>
#include <seneka_srv/infraredCutFilterAuto.h>
#include <seneka_srv/pictureEffect.h>
#include <seneka_srv/noiseReduction.h>
#include <seneka_srv/backLightCompensation.h>
#include <seneka_srv/statusDisplay.h>
#include <seneka_srv/titleDisplay.h>
#include <seneka_srv/titleText.h>
#include <seneka_srv/streaming.h>
#include <seneka_srv/debugScreen.h>

// pleora ebus sdk headers
#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvBuffer.h>
#include <PvStream.h>
#include <PvStreamRaw.h>

// miscellaneous headers
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv2/opencv.hpp>

#ifndef SONY_CAMERA_NODE_H
#define SONY_CAMERA_NODE_H

class Sony_Camera_Node
{
    private:

        // function prototypes
        void connectCamera();
        void configCamera();
        void startStreaming();
        void stopStreaming();
        //void publishImage();
        void disconnectCamera();
        void zoom_ratio_optical(int ratio);
        void zoom_ratio_digital(int ratio);
        void zoom_in_out(int ratio_optical, int ratio_digital);
        void autoFocusControl(int autoFocus);
        void videoModeNext(int videoMode);
        void focusPosition(int val);
        bool focusNearLimit(int focusLimit);
        void titleText(std::string str);
        void titleDisplay(int val);
        void statusDisplay(int val);
        void backLightCompensation(int val);
        bool noiseReduction(int val);
        void pictureEffect(int val);
        void infraredCutFilter(int val);
        void infraredCutFilterAuto(int val);
        void streaming(bool decider);
        void debugScreen(bool decider);

        // service callback function prototypes
        bool zoom_in_outService(            seneka_srv::zoom::Request &req,
                                            seneka_srv::zoom::Response &res);
    
        bool focusControlService(           seneka_srv::focusAuto::Request &req,
                                            seneka_srv::focusAuto::Response &res);
    
        bool videoModeNextService(          seneka_srv::videoMode::Request &req,
                                            seneka_srv::videoMode::Response &res);

        bool focusPositionService(          seneka_srv::focus::Request &req,
                                            seneka_srv::focus::Response &res);

        bool focusNearLimitService(         seneka_srv::focusNearLimit::Request &req,
                                            seneka_srv::focusNearLimit::Response &res);
    
        bool infraredCutFilterAutoService(  seneka_srv::infraredCutFilterAuto::Request &req,
                                            seneka_srv::infraredCutFilterAuto::Response &res);
    
        bool infraredCutFilterService(      seneka_srv::infraredCutFilter::Request &req,
                                            seneka_srv::infraredCutFilter::Response &res);
    
        bool pictureEffectService(          seneka_srv::pictureEffect::Request &req,
                                            seneka_srv::pictureEffect::Response &res);
    
        bool noiseReductionService(         seneka_srv::noiseReduction::Request &req,
                                            seneka_srv::noiseReduction::Response &res);
    
        bool backLightCompensationService(  seneka_srv::backLightCompensation::Request &req,
                                            seneka_srv::backLightCompensation::Response &res);
    
        bool statusDisplayService(          seneka_srv::statusDisplay::Request &req,
                                            seneka_srv::statusDisplay::Response &res);
    
        bool titleDisplayService(           seneka_srv::titleDisplay::Request &req,
                                            seneka_srv::titleDisplay::Response &res);
    
        bool titleTextService(              seneka_srv::titleText::Request &req,
                                            seneka_srv::titleText::Response &res);

        bool streamingService(              seneka_srv::streaming::Request &req,
                                            seneka_srv::streaming::Response &res);

        bool debugScreenService(            seneka_srv::debugScreen::Request &req,
                                            seneka_srv::debugScreen::Response &res);

        std::string camera_ip_address_param;
        std::string titleText_param;
        int zoom_ratio_optical_param;
        int zoom_ratio_digital_param;
        int videoModeNext_param;
        int focusPosition_param;
        int autoFocus_param;
        int focusLimit_param;
        int titleDisplay_param;
        int statusDisplay_param;
        int backLightCompensation_param;
        int noiseReduction_param;
        int pictureEffect_param;
        int infraredCutFilter_param;
        int infraredCutFilterAuto_param;
        bool streaming_param;
        bool debug_screen_param;

        ros::ServiceServer  zoom_service_,
                            focus_service_,
                            videoModeNext_service_,
                            focusPosition_servie_,
                            focusNearLimit_service_,
                            infraredCutFilterAuto_service,
                            infraredCutFilter_service,
                            pictureEffect_service,
                            noiseReduction_service,
                            backLightCompensation_service,
                            statusDisplay_service,
                            titleDisplay_service,
                            titleText_service,
                            streaming_service,
                            debugScreen_service;

        image_transport::ImageTransport it;
        image_transport::Publisher publish_rgb_image;
        cv_bridge::CvImage out_msg;

        PvInt64 lSize;
        PvInt64 focus_pos;
        PvInt64 focus_auto;
        bool flag;
        PvBuffer *lBuffers;
        PvDevice lDevice;
        PvResult lResult;
        PvStream lStream;
        PvGenParameterArray *lDeviceParams;
        PvGenInteger *lPayloadSize;
        PvInt64 width_, height_ ;

        public:
    
        Sony_Camera_Node();
        ~Sony_Camera_Node();

        ros::NodeHandle nh,  pnh_;

        void publishImage();
};

#endif // SONY_CAMERA_NODE_H
