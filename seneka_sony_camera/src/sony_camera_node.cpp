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

#include "sony_camera_node.h"

PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT ( 1 )

using namespace std;

#define clip(x) (unsigned char)( (x) < 0 ? 0 : ( (x) > 255 ? 255 : (x) ) )

// constructor
Sony_Camera_Node::Sony_Camera_Node():it(nh)
{
    lSize = 0;
    width_ = 0;
    height_ = 0;
    pnh_ = ros::NodeHandle("~");

    // get device parameters need to control streaming
    lDeviceParams = lDevice.GetGenParameters();

    // publish images
    publish_rgb_image               =   it.advertise(       "SonyGigCam_rgb_image", 10);

    // advertise zooming service
    zoom_service_                   =   nh.advertiseService("set_zoomin",               &Sony_Camera_Node::zoom_in_outService, this);

    // advertise autofocus service
    focus_service_                  =   nh.advertiseService("set_autofocus",            &Sony_Camera_Node::focusControlService, this);

    // advertise videomodenext service
    videoModeNext_service_          =   nh.advertiseService("set_videomodenext",        &Sony_Camera_Node::videoModeNextService, this);

    // advertise focus position service
    focusPosition_servie_           =   nh.advertiseService("set_focusPosition",        &Sony_Camera_Node::focusPositionService, this);

    // advertise focus infraredCutFilterAuto service
    focusNearLimit_service_         =   nh.advertiseService("set_focusNearLimit",       &Sony_Camera_Node::focusNearLimitService, this);

    // advertise focus focusNearLimit service
    infraredCutFilterAuto_service   =   nh.advertiseService("set_infraredCutFilterAuto",&Sony_Camera_Node::infraredCutFilterAutoService, this);

    // advertise focus infraredCutFilter service
    infraredCutFilter_service       =   nh.advertiseService("set_infraredCutFilter",    &Sony_Camera_Node::infraredCutFilterService, this);

    // advertise pictureEffect service
    pictureEffect_service           =   nh.advertiseService("set_pictureEffect",        &Sony_Camera_Node::pictureEffectService, this);

    // advertise noiseReduction service
    noiseReduction_service          =   nh.advertiseService("set_noiseReduction",       &Sony_Camera_Node::noiseReductionService, this);

    // advertise backLightCompensation service
    backLightCompensation_service   =   nh.advertiseService("set_backLightCompensation",&Sony_Camera_Node::backLightCompensationService, this);

    // advertise  statusDisplay service
    statusDisplay_service           =   nh.advertiseService("set_statusDisplay",        &Sony_Camera_Node::statusDisplayService, this);

    // advertise  titleDisplay service
    titleDisplay_service            =   nh.advertiseService("set_titleDisplay",         &Sony_Camera_Node::titleDisplayService, this);

    // advertise  titleText service
    titleText_service               =   nh.advertiseService("set_titleText",            &Sony_Camera_Node::titleTextService, this);

    // advertise streaming service
    streaming_service               =   nh.advertiseService("set_streaming",            &Sony_Camera_Node::streamingService, this);



    // read parameter: camera_ip_address
    if(!pnh_.hasParam("camera_ip_address"))
        ROS_WARN("Using default value for initial ip_address parameter [\"192.168.0.1\"].");
    pnh_.param("camera_ip_address", camera_ip_address_param, std::string("192.168.0.1"));

    // read parameter: zoom_ratio_optical
    if(!pnh_.hasParam("zoom_ratio_optical"))
        ROS_WARN("Using default value for initial zoom_ratio_optical parameter [1].");
    pnh_.param("zoom_ratio_optical", zoom_ratio_optical_param, 1);

    // read parameter: zoom_ratio_digital
    if(!pnh_.hasParam("zoom_ratio_digital"))
        ROS_WARN("Using default value for initial zoom_ratio_digital parameter [1].");
    pnh_.param("zoom_ratio_digital", zoom_ratio_digital_param, 1);

    // read parameter: autoFocus
    if(!pnh_.hasParam("autoFocus"))
        ROS_WARN("Using default value for initial autoFocus parameter [3].");
    pnh_.param("autoFocus", autoFocus_param, 3);

    // read parameter: videoModeNext
    if(!pnh_.hasParam("videoModeNext"))
        ROS_WARN("Using default value for initial videoModeNext parameter [11].");
    pnh_.param("videoModeNext", videoModeNext_param, 11);

    // read parameter: focusPosition
    if(!pnh_.hasParam("focusPosition"))
        ROS_WARN("Using default value for initial focusPosition parameter [53248].");
    pnh_.param("focusPosition", focusPosition_param, 53248);

    // read parameter: focusNearLimit
    if(!pnh_.hasParam("focusNearLimit"))
        ROS_WARN("Using default value for initial focusNearLimit parameter [0].");
    pnh_.param("focusNearLimit", focusLimit_param, 0);

    // read parameter: titleText
    if(!pnh_.hasParam("titleText"))
        ROS_WARN("Using default value for initial titleText parameter [""].");
    pnh_.param("titleText", titleText_param, std::string(""));

    // read parameter: titleDisplay
    if(!pnh_.hasParam("titleDisplay"))
        ROS_WARN("Using default value for initial titleDisplay parameter [0].");
    pnh_.param("titleDisplay", titleDisplay_param, 0);

    // read parameter: statusDisplay
    if(!pnh_.hasParam("statusDisplay"))
        ROS_WARN("Using default value for initial statusDisplay parameter [0].");
    pnh_.param("statusDisplay", statusDisplay_param, 0);

    // read parameter: backLightCompensation
    if(!pnh_.hasParam("backLightCompensation"))
        ROS_WARN("Using default value for initial backLightCompensation parameter [0].");
    pnh_.param("backLightCompensation", backLightCompensation_param, 0);

    // read parameter: noiseReduction
    if(!pnh_.hasParam("noiseReduction"))
        ROS_WARN("Using default value for initial noiseReduction parameter [3].");
    pnh_.param("noiseReduction", noiseReduction_param, 3);

    // read parameter: pictureEffect
    if(!pnh_.hasParam("pictureEffect"))
        ROS_WARN("Using default value for initial pictureEffect parameter [0].");
    pnh_.param("pictureEffect", pictureEffect_param, 0);

    // read parameter: infraredCutFilter
    if(!pnh_.hasParam("infraredCutFilter"))
        ROS_WARN("Using default value for initial infraredCutFilter parameter [0].");
    pnh_.param("infraredCutFilter", infraredCutFilter_param, 0);

    // read parameter: infraredCutFilterAuto
    if(!pnh_.hasParam("infraredCutFilterAuto"))
        ROS_WARN("Using default value for initial infraredCutFilterAuto parameter [0].");
    pnh_.param("infraredCutFilterAuto",infraredCutFilterAuto_param, 0);

    // read parameter from parameter server
    // if parameter not found, use default value
    if(!pnh_.hasParam("streaming"))
        ROS_WARN("Using default value for initial streaming parameter [false].");
    pnh_.param("streaming",streaming_param, false);

    // read parameter from parameter server
    // if parameter not found, use default value
    if(!pnh_.hasParam("debug_screen"))
        ROS_WARN("Using default value for initial debug_screen parameter [false].");
    pnh_.param("debug_screen",debug_screen_param, false);



    //connect with the camera device
    connectCamera();

    //config camera by setting parameter in the sony_camera.yaml file
    configCamera();

    //start streaming of the camera device
    if(streaming_param)
        startStreaming();
}

//Destructor
Sony_Camera_Node::~Sony_Camera_Node()
{
    //stop streaming of the camera device
    stopStreaming();

    //disconnect the camera device
    disconnectCamera();
}

//Connect camera to the device
void Sony_Camera_Node::connectCamera()
{
    //PvString address("192.168.0.1");
    PvString address = camera_ip_address_param.c_str();

    ROS_INFO("1. Connecting to the device SonyHD Camera...");
    PvResult lResult1 = lDevice.Connect(address,PvAccessControl);
    while(lResult1 != 0)
    {
        ROS_ERROR("Failed to connect to device!");
        ROS_ERROR("Waiting for device to come up...");
        ros::Duration(2.0).sleep();
        lResult1 = lDevice.Connect( address,PvAccessControl);
    }
    
    ROS_INFO("2. Sucessfully connected to %s.", address.GetAscii());
}

//Default configuration by setting parameter to the sony_camera.yaml file
void Sony_Camera_Node::configCamera()
{
    this->zoom_in_out(zoom_ratio_optical_param, zoom_ratio_digital_param);
    this->autoFocusControl(autoFocus_param);
    this->videoModeNext(videoModeNext_param);
    this->titleText(titleText_param);
    this->titleDisplay(titleDisplay_param);
    this->statusDisplay(statusDisplay_param);
    this->backLightCompensation(backLightCompensation_param);
    this->noiseReduction(noiseReduction_param);
    this->pictureEffect(pictureEffect_param);
    this->infraredCutFilter(infraredCutFilter_param);
    this->infraredCutFilterAuto(infraredCutFilterAuto_param);

    //focusPosition can be set only while the camera's FocusAuto is set to other than Continuos(=3), basically set to off(=0)
    //lDeviceParams->GetEnumValue("FocusAuto",focus_auto);
    if(autoFocus_param == 0)
    {
         this->focusPosition(focusPosition_param);
    }
    //focusNearLimit can be set only while the camera's FocusAuto is set to Continuos(=3)
    if(autoFocus_param == 3)
    {
        this->focusNearLimit(focusLimit_param);
    }
}

bool Sony_Camera_Node::streamingService(seneka_srv::streaming::Request &req,
                                        seneka_srv::streaming::Response &res)
{
    streaming(req.streaming);

    return true;
}

void Sony_Camera_Node::streaming(bool decider)
{
    if(decider)
    {
        streaming_param = true;
        startStreaming();
    }
    else if (!decider)
    {
        streaming_param = false;
        stopStreaming();
    }
}

void Sony_Camera_Node::startStreaming()
{
    //Camera's ip address
    PvString address = camera_ip_address_param.c_str();

    //Negotiate streaming packet size
    lDevice.NegotiatePacketSize();

    //Open stream - have the PvDevice do it for us
    ROS_INFO("3. Opening stream to device.");
    lStream.Open( address );

    //Reading payload size from device
    lPayloadSize = dynamic_cast<PvGenInteger *>( lDeviceParams->Get( "PayloadSize" ) );
    lPayloadSize->GetValue( lSize );

    // Use min of BUFFER_COUNT and how many buffers can be queued in PvStream
    PvUInt32 lBufferCount = ( lStream.GetQueuedBufferMaximum() < BUFFER_COUNT ) ?
                lStream.GetQueuedBufferMaximum() :
                BUFFER_COUNT;

    // Create, alloc buffers
    lBuffers = new PvBuffer[ lBufferCount ];
    for ( PvUInt32 i = 0; i < lBufferCount; i++ )
    {
        lBuffers[ i ].Alloc( static_cast<PvUInt32>( lSize ) );
    }

    // Have to set the Device IP destination to the Stream
    lDevice.SetStreamDestination( lStream.GetLocalIPAddress(), lStream.GetLocalPort() );

    // Queue all buffers in the stream
    for ( PvUInt32 i = 0; i < lBufferCount; i++ )
    {
        lStream.QueueBuffer( lBuffers + i );
    }

    ROS_INFO("Resetting timestamp counter.");
    PvGenCommand *lResetTimestamp = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "GevTimestampControlReset" ) );
    lResetTimestamp->Execute();

    // The buffers are queued in the stream, we just have to tell the device
    // to start sending us images
    ROS_INFO("4. Sending StartAcquisition command to device.");
    PvGenCommand *lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
    lResult = lStart->Execute();

}

// Image publisher
void Sony_Camera_Node::publishImage()
{
    PvBuffer *lBuffer = NULL;
    PvResult lOperationResult;
    PvImage *Image = NULL;

    // Retrieve next buffer
    lResult = lStream.RetrieveBuffer( &lBuffer, &lOperationResult, 10000 );
    if ( lResult.IsOK() )
    {
        if(lOperationResult.IsOK())
        {
            // If the buffer contains an image, display width and height
            if ( lBuffer->GetPayloadType() == PvPayloadTypeImage )
            {
                // Get image specific buffer interface
                Image = lBuffer->GetImage();

                // Read width, height
                width_ = (int) Image->GetWidth();
                height_ = (int) Image->GetHeight();
            }

            //Converting YUV image formate to RGB
            int i = 0,j = 0, r1 = 0, g1 = 0, b1 = 0, r2 = 0, g2 = 0, b2 = 0;
            IplImage* m_RGB = cvCreateImage(cvSize(width_, height_), IPL_DEPTH_8U, 3);
            unsigned char* pData = (unsigned char *) Image->GetDataPointer();

            for(i = 0, j=0; i < width_ * height_*3 ; i+=6, j+=4)
            {
                unsigned char u = pData[j];
                unsigned char y1 = pData[j+1];
                unsigned char v = pData[j+2];
                unsigned char y2 = pData[j+3];

                b1 = clip(1.0*y1 + 8 + 1.402*(v-128));
                g1 = clip(1.0*y1 - 0.34413*(u-128) - 0.71414*(v-128));
                r1 = clip(1.0*y1 + 1.772*(u-128));
                b2 = clip(1.0*y2 + 8 + 1.402*(v-128));
                g2 = clip(1.0*y2 - 0.34413*(u-128) - 0.71414*(v-128));
                r2 = clip(1.0*y2 + 1.772*(u-128));

                m_RGB->imageData[i] = r1;
                m_RGB->imageData[i+1] = g1;
                m_RGB->imageData[i+2] = b1;
                m_RGB->imageData[i+3] = r2;
                m_RGB->imageData[i+4] = g2;
                m_RGB->imageData[i+5] = b2;
            }

            //IplImage image to cv::Mat
            cv::Mat image(m_RGB);

            if (debug_screen_param)
                cv::namedWindow("Sony",cv::WINDOW_AUTOSIZE);

            cv_bridge::CvImage out_msg;
            out_msg.header.stamp   = ros::Time::now();
            out_msg.encoding = sensor_msgs::image_encodings::RGB8;
            out_msg.image    = image;

            publish_rgb_image.publish(out_msg.toImageMsg());

            if (debug_screen_param)
                cv::imshow("Current Image",image);

            cv::waitKey(30);
        }
        // re-queue the buffer in the stream object

        lStream.QueueBuffer( lBuffer );
    }
    else
    {
        // Timeout
        ROS_WARN("Timeout.");
    }
}

void Sony_Camera_Node::stopStreaming()
{
    // Clean-up
    // Tell the device to stop sending images
    ROS_INFO("Sending AcquisitionStop command to the device.");
    PvGenCommand* lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );
    lStop->Execute();

    // Abort all buffers from the stream, unqueue
    ROS_INFO("Aborting buffers still in stream.");
    lStream.AbortQueuedBuffers();
    while ( lStream.GetQueuedBufferCount() > 0 )
    {
        PvBuffer *lBuffer = NULL;
        PvResult lOperationResult;

        lStream.RetrieveBuffer( &lBuffer, &lOperationResult );

        ROS_INFO("Post-abort retrieved buffer: %s", lOperationResult.GetCodeString().GetAscii());
    }

    // Release buffers
    ROS_INFO("Releasing buffers.");
    delete []lBuffers;

    // Now close the stream. Also optionnal but nice to have
    ROS_INFO("Closing stream.");
    lStream.Close();
}

void Sony_Camera_Node::disconnectCamera()
{
    // Finally disconnect the device. Optional, still nice to have
    ROS_INFO("Disconnecting device.");
    lDevice.Disconnect();
}


//Service: zooming functionalities- Optical Zoom/Digital Zoom
bool Sony_Camera_Node::zoom_in_outService(seneka_srv::zoom::Request &req,
                                          seneka_srv::zoom::Response &res)
{
    if(req.zoom_ratio_digital==1){
        zoom_ratio_optical(req.zoom_ratio_optical);
        res.success = true;
    }

    else if (req.zoom_ratio_optical==20 && req.zoom_ratio_digital>=2) {
        zoom_ratio_digital(req.zoom_ratio_digital);
        res.success = true;
    }
    else{
        res.success = false;
    }
    return true;
}

//Parametric: zooming functionalities- Optical Zoom/Digital Zoom
void Sony_Camera_Node::zoom_in_out(int ratio_optical, int ratio_digital)
{
    if(ratio_digital==1){
        zoom_ratio_optical(ratio_optical);
    }
    else if (ratio_optical==20 && ratio_digital>=2) {
        zoom_ratio_digital(ratio_digital);
    }
    else{
        ROS_WARN("Warning!: Zoom ratio should be given according to the camera standard. Default is set to optical 1.");
    }
}

void Sony_Camera_Node::zoom_ratio_optical(int ratio)
{
    // Optical zoom: x_x_= x{1-20}x{1}.
    // x in first position = req.zoom_ratio_optical
    // x in 2nd position = req.zoom_ratio_digital

    switch(ratio)
    {
    //zoomRatio: x1x1
    case 1:
        lDeviceParams->SetEnumValue("ZoomRatio",0);
        break;

        //zoomRatio: x2x1
    case 2:
        lDeviceParams->SetEnumValue("ZoomRatio",4);
        break;

        //zoomRatio: x3x1
    case 3:
        lDeviceParams->SetEnumValue("ZoomRatio",6);
        break;

        //zoomRatio: x4x1
    case 4:
        lDeviceParams->SetEnumValue("ZoomRatio",8);
        break;

        //zoomRatio: x5x1
    case 5:
        lDeviceParams->SetEnumValue("ZoomRatio",10);
        break;

        //zoomRatio: x6x1
    case 6:
        lDeviceParams->SetEnumValue("ZoomRatio",11);
        break;

        //zoomRatio: x7x1
    case 7:
        lDeviceParams->SetEnumValue("ZoomRatio",13);
        break;

        //zoomRatio: x8x1
    case 8:
        lDeviceParams->SetEnumValue("ZoomRatio",14);
        break;

        //zoomRatio: x9x1
    case 9:
        lDeviceParams->SetEnumValue("ZoomRatio",15);
        break;

        //zoomRatio: x10x1
    case 10:
        lDeviceParams->SetEnumValue("ZoomRatio",16);
        break;

        //zoomRatio: x11x1
    case 11:
        lDeviceParams->SetEnumValue("ZoomRatio",23);
        break;

        //zoomRatio: x12x1
    case 12:
        lDeviceParams->SetEnumValue("ZoomRatio",24);
        break;

        //zoomRatio: x13x1
    case 13:
        lDeviceParams->SetEnumValue("ZoomRatio",25);
        break;

        //zoomRatio: x14x1
    case 14:
        lDeviceParams->SetEnumValue("ZoomRatio",26);
        break;

        //zoomRatio: x15x1
    case 15:
        lDeviceParams->SetEnumValue("ZoomRatio",27);
        break;

        //zoomRatio: x16x1
    case 16:
        lDeviceParams->SetEnumValue("ZoomRatio",28);
        break;

        //zoomRatio: x17x1
    case 17:
        lDeviceParams->SetEnumValue("ZoomRatio",29);
        break;

        //zoomRatio: x18x1
    case 18:
        lDeviceParams->SetEnumValue("ZoomRatio",31);
        break;

        //zoomRatio: x19x1
    case 19:
        lDeviceParams->SetEnumValue("ZoomRatio",43);
        break;

        //zoomRatio: x20x1
    case 20:
        lDeviceParams->SetEnumValue("ZoomRatio",44);
        break;

        //zoomRatio: x1x1(default value)
    default:
        ROS_WARN("Warning!: Zoom ratio should be given according to the camera standard. Default is set to 1.");
        lDeviceParams->SetEnumValue("ZoomRatio",0);
        break;
    }

    lDeviceParams->ExecuteCommand("CAM_Zoom");
}

void Sony_Camera_Node::zoom_ratio_digital(int ratio)
{
    //Digital zoom: x_x_= x{20}x{2-12}.
    //x in frist position = req.zoom_ratio_optical and
    //x in 2nd position = req.zoom_ratio_digital

    switch(ratio)
    {

        //zoomRatio: x20x2
        case 2:
            lDeviceParams->SetEnumValue("ZoomRatio",83);
            break;

        //zoomRatio: x20x3
        case 3:
            lDeviceParams->SetEnumValue("ZoomRatio",84);
            break;

        //zoomRatio: x20x4
        case 4:
            lDeviceParams->SetEnumValue("ZoomRatio",85);
            break;

        //zoomRatio: x20x5
        case 5:
            lDeviceParams->SetEnumValue("ZoomRatio",86);
            break;

        //zoomRatio: x20x6
        case 6:
            lDeviceParams->SetEnumValue("ZoomRatio",87);
            break;

        //zoomRatio: x20x7
        case 7:
            lDeviceParams->SetEnumValue("ZoomRatio",88);
            break;

        //zoomRatio: x20x8
        case 8:
            lDeviceParams->SetEnumValue("ZoomRatio",89);
            break;

        //zoomRatio: x20x9
        case 9:
            lDeviceParams->SetEnumValue("ZoomRatio",90);
            break;

        //zoomRatio: x20x10
        case 10:
            lDeviceParams->SetEnumValue("ZoomRatio",91);
            break;

        //zoomRatio: x20x11
        case 11:
            lDeviceParams->SetEnumValue("ZoomRatio",92);
            break;

        //zoomRatio: x20x12
        case 12:
            lDeviceParams->SetEnumValue("ZoomRatio",93);
            break;
        default:
            ROS_WARN("Zoom ratio should be given according to the camera standard. Default is set to 1.");
            lDeviceParams->SetEnumValue("ZoomRatio",0);
            break;
    }

    lDeviceParams->ExecuteCommand("CAM_Zoom");
}

//Service: Auto Focus functionalities
bool Sony_Camera_Node::focusControlService(seneka_srv::focusAuto::Request &req,
                                           seneka_srv::focusAuto::Response &res)
{
    autoFocusControl(req.focusAuto);
    res.success = true;

    return true;
}

void Sony_Camera_Node::autoFocusControl(int autoFocus)
{
    switch(autoFocus)
    {

        //Auto Focus: Off
        case 0:
            lDeviceParams->SetEnumValue("FocusAuto",0);
            break;

        //Auto Focus: Once
        case 1:
            lDeviceParams->SetEnumValue("FocusAuto",1);
            break;

        //Auto Focus: Infinity
        case 2:
            lDeviceParams->SetEnumValue("FocusAuto",2);
            break;

        //Auto Focus: Continuous
        case 3:
            lDeviceParams->SetEnumValue("FocusAuto",3);
            break;

        default:
            ROS_WARN("Please specify the correct focus formate. Default AutoFocus is set to Continuous mode.");
            lDeviceParams->SetEnumValue("FocusAuto",3);
            break;
    }

    lDeviceParams->ExecuteCommand("CAM_Focus");
}

//Service: videoModeNext- set on next power cycle
bool Sony_Camera_Node::videoModeNextService(seneka_srv::videoMode::Request &req,
                                            seneka_srv::videoMode::Response &res)
{
    videoModeNext(req.video_mode_next);
    res.success = true;

    return true;
}

void Sony_Camera_Node::videoModeNext(int videoMode_)
{
    switch(videoMode_)
    {
        //Sony Block Video Mode Next: HD_1080i_60Hz
        case 0:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",8);
            break;

        //Sony Block Video Mode Next: HD_1080i_59p94_Hz
        case 1:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",9);
            break;

        //Sony Block Video Mode Next: HD_1080i_50_Hz
        case 2:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",10);
            break;

        //Sony Block Video Mode Next: HD_1080p_30_Hz
        case 3:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",11);
            break;

        //Sony Block Video Mode Next: HD_1080p_29p97_Hz
        case 4:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",12);
            break;

        //Sony Block Video Mode Next: HD_1080p_25_Hz
        case 5:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",13);
            break;

        //Sony Block Video Mode Next: HD_720p_60_Hz
        case 6:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",14);
            break;

        //Sony Block Video Mode Next: HD_720p_59p94_Hz
        case 7:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",15);
            break;

        //Sony Block Video Mode Next: HD_720p_50_Hz
        case 8:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",16);
            break;

        //Sony Block Video Mode Next: HD_720p_30_Hz
        case 9:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",17);
            break;

        //Sony Block Video Mode Next: HD_720p_29p97_Hz
        case 10:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",18);
            break;

        //Sony Block Video Mode Next: HD_720p_25_Hz
        case 11:
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",19);
            break;

        default:
            ROS_WARN("Please specify the correct video mode. Default AutoFocus is set to HD_720p_25_Hz mode.");
            lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",19);
            break;
    }
}

//Service: set Focus Position
bool Sony_Camera_Node::focusPositionService(seneka_srv::focus::Request &req,
                                            seneka_srv::focus::Response &res)
{
    lDeviceParams->GetEnumValue("FocusAuto",focus_auto);

    if (focus_auto == 0){
        //min:4096 Max: 61440
        if(req.focus_position >= 4096 && req.focus_position <= 61440){

            lDeviceParams->SetIntegerValue("Focus",req.focus_position);
            //lDeviceParams->ExecuteCommand("CAM_Focus");
             lDeviceParams->ExecuteCommand("CAM_FocusPosInq");
            res.success = true;
        }
        else{
            ROS_WARN("Focus position must be in-between 4096 and 61440.");
            res.success = false;
        }
    }else{
        ROS_WARN("Focus position cannot be change, while the camera's FocusAuto is set to continuous.");
        res.success = false;
    }
    return true;
}

void Sony_Camera_Node::focusPosition(int val)
{
    lDeviceParams->GetEnumValue("FocusAuto",focus_auto);

    if (focus_auto == 0)
    {
        //min:4096 Max: 61440
        if(  4096 <= val && val <= 61440)
        {
            lDeviceParams->SetIntegerValue("Focus",val);
            lDeviceParams->ExecuteCommand("CAM_Focus");
            lDeviceParams->ExecuteCommand("CAM_FocusPosInq");
        }

        else
        {
            ROS_WARN("Focus position must be in-between 4096 and 61440.");
        }
    }

    else
    {
        ROS_WARN("Focus position cannot be change, while the camera's FocusAuto is set to Continuous.");
    }
}

//Service: set FocusNearLimit while FocusAuto is other than Off
bool Sony_Camera_Node::focusNearLimitService(seneka_srv::focusNearLimit::Request &req,
                                             seneka_srv::focusNearLimit::Response &res)
{
    if(focusNearLimit(req.focus_near_limit))
    {
        res.success = true;
    }

    else
    {
        res.success = false;
    }

    return true;
}

bool Sony_Camera_Node::focusNearLimit(int focusLimit)
{
    lDeviceParams->GetEnumValue("FocusAuto",focus_auto);

    if (focus_auto != 0)
    {
        switch(focusLimit)
        {
            //FocusNearLimit: Infinity
            case 0:
                lDeviceParams->SetEnumValue("FocusNearLimit",0);
                break;

            //FocusNearLimit: Fd25m
            case 1:
                lDeviceParams->SetEnumValue("FocusNearLimit",1);
                break;

            //FocusNearLimit: Fd11m
            case 2:
                lDeviceParams->SetEnumValue("FocusNearLimit",4);
                break;

            //FocusNearLimit: Fd7m
            case 3:
                lDeviceParams->SetEnumValue("FocusNearLimit",7);
                break;

            //FocusNearLimit: Fd4p9m
            case 4:
                lDeviceParams->SetEnumValue("FocusNearLimit",10);
                break;

            //FocusNearLimit: Fd3p7m
            case 5:
                lDeviceParams->SetEnumValue("FocusNearLimit",13);
                break;

            //FocusNearLimit: Fd2p9m
            case 6:
                lDeviceParams->SetEnumValue("FocusNearLimit",16);
                break;

            //FocusNearLimit: Fd2p3m
            case 7:
                lDeviceParams->SetEnumValue("FocusNearLimit",18);
                break;

            //FocusNearLimit: Fd1p85m
            case 10:
                lDeviceParams->SetEnumValue("FocusNearLimit",91);
                break;

            //FocusNearLimit: Fd1p85m
            case 11:
                lDeviceParams->SetEnumValue("FocusNearLimit",20);
                break;

            //FocusNearLimit: Fd1p5m
            case 12:
                lDeviceParams->SetEnumValue("FocusNearLimit",21);
                break;

            //FocusNearLimit: Fd1p23m
            case 13:
                lDeviceParams->SetEnumValue("FocusNearLimit",23);
                break;

            //FocusNearLimit: Fd1m
            case 14:
                lDeviceParams->SetEnumValue("FocusNearLimit",25);
                break;

            //FocusNearLimit: Fd30cm
            case 15:
                lDeviceParams->SetEnumValue("FocusNearLimit",30);
                break;

            //FocusNearLimit: Fd8cm
            case 16:
                lDeviceParams->SetEnumValue("FocusNearLimit",37);
                break;

            //FocusNearLimit: Fd1cm
            case 17:
                lDeviceParams->SetEnumValue("FocusNearLimit",45);
                break;

            default:
                ROS_WARN( "FocusNearLimit should be given according to the camera standard. Default is set to Infinity(0).");
                lDeviceParams->SetEnumValue("FocusNearLimit",0);
                break;
        }

        lDeviceParams->ExecuteCommand("CAM_FocusNearLimitInq");
        return true;
    }

    else
    {
        ROS_WARN("FocusNearLimit is only valid while the camera's  FocusAuto is set to Continuous.");
        return false;
    }
}

//Service: Enable or Disable Infrared CutFilterAuto
bool  Sony_Camera_Node::infraredCutFilterAutoService(seneka_srv::infraredCutFilterAuto::Request &req,
                                                     seneka_srv::infraredCutFilterAuto::Response &res)
{
    infraredCutFilterAuto(req.cutfilter_auto);
    res.success = true;
    return true;
}


void Sony_Camera_Node::infraredCutFilterAuto(int val)
{
    switch(val)
    {

        //InfraredCutFilterAuto: false
        case 0:
            lDeviceParams->SetBooleanValue("InfraredCutFilterAuto",false);
            break;

        //InfraredCutFilterAuto: true
        case 1:
            lDeviceParams->SetBooleanValue("InfraredCutFilterAuto",true);
            break;

        default:
            ROS_WARN("Default InfraredCutFilterAuto is set to false.");
            lDeviceParams->SetBooleanValue("InfraredCutFilterAuto",false);
            break;
    }

    lDeviceParams->ExecuteCommand("CAM_AutoICRModeInq");
}

//Service: Enable or Disable Infrared CutFilter
bool  Sony_Camera_Node::infraredCutFilterService(seneka_srv::infraredCutFilter::Request &req,
                                                 seneka_srv::infraredCutFilter::Response &res)
{
    infraredCutFilter(req.cutfilter);
    res.success = true;
    return true;
}

void Sony_Camera_Node::infraredCutFilter(int val)
{
    switch(val)
    {

    //InfraredCutFilter: false
    case 0:
        lDeviceParams->SetBooleanValue("InfraredCutFilter",false);
        break;

        //InfraredCutFilter: true
    case 1:
        lDeviceParams->SetBooleanValue("InfraredCutFilter",true);
        break;

    default:
        ROS_WARN("Default InfraredCutFilterAuto is set to false.");
        lDeviceParams->SetBooleanValue("InfraredCutFilter",false);
        break;

    }

    lDeviceParams->ExecuteCommand("CAM_ICRModeInq");
}

//Service: set Picture Effect: Off, NegReversal, BlackAndWhite
bool Sony_Camera_Node::pictureEffectService(seneka_srv::pictureEffect::Request &req,
                                            seneka_srv::pictureEffect::Response &res)
{
    pictureEffect(req.picture_effect);
    res.success = true;
    return true;
}

void Sony_Camera_Node::pictureEffect(int val)
{
    switch(val)
    {

        //PictureEffect: Off
        case 0:
            lDeviceParams->SetEnumValue("PictureEffect",0);
            break;

        //PictureEffect: NegReversal
        case 1:
            lDeviceParams->SetEnumValue("PictureEffect",1);
            break;

        //PictureEffect:BlackAndWhite
        case 2:
            lDeviceParams->SetEnumValue("PictureEffect",2);
            break;

        default:
            ROS_WARN("PictureEffect should be according to the camera standard. Default PictureEffect is set to Off(0).");
            lDeviceParams->SetEnumValue("PictureEffect",0);
            break;
    }

    lDeviceParams->ExecuteCommand("CAM_PictureEffect");
}

//Service: Set noise reduction maximum of 5
bool Sony_Camera_Node::noiseReductionService(seneka_srv::noiseReduction::Request &req,
                                             seneka_srv::noiseReduction::Response &res)
{
    if(noiseReduction(req.noise_reduction))
    {
        res.success = true;
    }

    else
    {
        res.success = false;
    }
    return true;
}

bool Sony_Camera_Node::noiseReduction(int val)
{
    if(0 <= val && val <= 5)
    {
        lDeviceParams->SetIntegerValue("NoiseReduction",val);
        lDeviceParams->ExecuteCommand("CAM_NR");
        return true;

    }

    else
    {
        ROS_WARN("Noise Reduction should be in between 0 to 5.");
        return false;
    }
}


//Service: Enable or Disable Back Light Compensation
bool Sony_Camera_Node::backLightCompensationService(seneka_srv::backLightCompensation::Request &req,
                                                    seneka_srv::backLightCompensation::Response &res)
{
    backLightCompensation(req.back_light);
    res.success = true;
    return true;
}

void Sony_Camera_Node::backLightCompensation(int val)
{
    switch(val)
    {
        //BackLightCompensation: false
        case 0:
            lDeviceParams->SetBooleanValue("BackLightCompensation",false);
            break;

        //BackLightCompensation: true
        case 1:
            lDeviceParams->SetBooleanValue("BackLightCompensation",true);
            break;

        default:
            ROS_WARN("Back Light Compensation can be either 0(false) or 1(true). Default BackLightCompensation is set to false.");
            lDeviceParams->SetBooleanValue("BackLightCompensation",false);
            break;
    }

    lDeviceParams->ExecuteCommand("CAM_BackLight");
}

//Service: Enable or disable status display
bool Sony_Camera_Node::statusDisplayService(seneka_srv::statusDisplay::Request &req,
                                            seneka_srv::statusDisplay::Response &res)
{
    statusDisplay(req.display_status);
    res.success = true;
    return true;
}

void Sony_Camera_Node::statusDisplay(int val)
{
    switch(val)
    {
        //Status Display: false
        case 0:
            lDeviceParams->SetBooleanValue("StatusDisplay",false);
            break;

        //StatusDisplay: true
        case 1:
            lDeviceParams->SetBooleanValue("StatusDisplay",true);
            break;

        default:
            ROS_WARN("Status Display can be either 0(false) or 1 (true). Default Status Display is set to false.");
            lDeviceParams->SetBooleanValue("StatusDisplay",false);
            break;
    }

    lDeviceParams->ExecuteCommand("CAM_Display");
}

//Service: Enable or disable to display title
bool Sony_Camera_Node::titleDisplayService(seneka_srv::titleDisplay::Request &req,
                                           seneka_srv::titleDisplay::Response &res)
{
    titleDisplay(req.title_display);
    res.success = true;
    return true;
}

void Sony_Camera_Node::titleDisplay(int val)
{
    switch(val)
    {
        //TitleDisplay: false
        case 0:
            lDeviceParams->SetBooleanValue("TitleDisplay",false);
            break;

        //TitleDisplay: true
        case 1:
            lDeviceParams->SetBooleanValue("TitleDisplay",true);
            break;

        default:
            ROS_WARN("Title Display can be either 0(false) or 1 (true). Default Title Display is set to false.");
            lDeviceParams->SetBooleanValue("TitleDisplay",false);
            break;
    }

    lDeviceParams->ExecuteCommand("CAM_Title");
}

//Service: set title text of images
bool Sony_Camera_Node::titleTextService(seneka_srv::titleText::Request &req,
                                        seneka_srv::titleText::Response &res)
{
    titleText(req.title_text);
    res.success = true;
    return true;
}

void Sony_Camera_Node::titleText(std::string str1)
{
    PvString str = str1.c_str();
    lDeviceParams->SetString("TitleText",str);
    lDeviceParams->ExecuteCommand("CAM_Title");
}

int main(int argc, char** argv)
{
    // initialize ROS, specify name of node
    ros::init(argc,argv,"seneka_sony_camera");

    Sony_Camera_Node SonyCameraNode;

    // run image_publisher periodically until node has been shut down
    ros::Rate loop_rate(25); // Hz
    while(SonyCameraNode.nh.ok())
    {
        if (SonyCameraNode.getStreamingParam())
            SonyCameraNode.publishImage();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



