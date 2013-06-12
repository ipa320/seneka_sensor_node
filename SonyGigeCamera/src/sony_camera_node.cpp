/*!
*****************************************************************
*
* sony_camera_node.cpp
*
*
* Copyright (c) 2012
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
*****************************************************************
*
*
* Project name: SeNeKa
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


#include "sony_camera_node.h"


PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT ( 1 )


using namespace std;

#define clip(x) (unsigned char)( (x) < 0 ? 0 : ( (x) > 255 ? 255 : (x) ) )


Sony_Camera_Node::Sony_Camera_Node():it(nh){

    lSize = 0;
    width_, height_ = 0;
    pnh_ = ros::NodeHandle("~");
    // Get device parameters need to control streaming
    lDeviceParams = lDevice.GetGenParameters();

    //publish images
    publish_rgb_image = it.advertise("SonyGigCam_rgb_image", 10);

    // advertise zooming service
    zoom_service_= nh.advertiseService("set_zoomin",&Sony_Camera_Node::zoom_in_out, this);

    // advertise autofocus service
    focus_service_ = nh.advertiseService("set_autofocus",&Sony_Camera_Node::focusControl, this);

    // advertise videomodenext service
    videoModeNext_service_= nh.advertiseService("set_videomodenext",&Sony_Camera_Node::videoModeNext, this);

    // advertise focus position service
    focusPosition_servie_= nh.advertiseService("set_focusPosition",&Sony_Camera_Node::focusPosition, this);

    // advertise focus infraredCutFilterAuto service
    focusNearLimit_service_= nh.advertiseService("set_focusNearLimit",&Sony_Camera_Node::focusNearLimit, this);

    // advertise focus focusNearLimit service
    infraredCutFilterAuto_service = nh.advertiseService("set_infraredCutFilterAuto",&Sony_Camera_Node::infraredCutFilterAuto, this);

    // advertise focus infraredCutFilter service
    infraredCutFilter_service = nh.advertiseService("set_infraredCutFilter",&Sony_Camera_Node::infraredCutFilter, this);

    // advertise pictureEffect service
    pictureEffect_service = nh.advertiseService("set_pictureEffect",&Sony_Camera_Node::pictureEffect, this);

    // advertise noiseReduction service
    noiseReduction_service = nh.advertiseService("set_noiseReduction",&Sony_Camera_Node::noiseReduction, this);

    // advertise backLightCompensation service
    backLightCompensation_service = nh.advertiseService("set_backLightCompensation",&Sony_Camera_Node::backLightCompensation, this);

    // advertise  statusDisplay service
    statusDisplay_service = nh.advertiseService("set_statusDisplay",&Sony_Camera_Node::statusDisplay, this);

    // advertise  titleDisplay service
    titleDisplay_service = nh.advertiseService("set_titleDisplay",&Sony_Camera_Node::titleDisplay, this);

    // advertise  titleText service
    titleText_service =  nh.advertiseService("set_titleText",&Sony_Camera_Node::titleText, this);


   // read parameter: camera_ip_address

    if(!pnh_.hasParam("camera_ip_address"))
    ROS_WARN("Checking default location (/SonyGigeCamera/launch) for initial ip_address parameter.");
    pnh_.param("camera_ip_address", camera_ip_address, std::string("192.168.0.1"));


    //connect with the camera device
    connectCamera();

    //start streaming of the camera device
    startStreaming();
}


Sony_Camera_Node::~Sony_Camera_Node(){

    //stop streaming of the camera device
    stopStreaming();

    //disconnect the camera device
    disconnectCamera();
}


void Sony_Camera_Node::connectCamera(){

    //PvString address("192.168.0.1");
    PvString address = camera_ip_address.c_str();


    printf( "\n1. Connecting to the device SonyHD Camera..." );
    PvResult lResult1 = lDevice.Connect( address, PvAccessControl );
    if(lResult1 != 0)
    {
        printf( "\n  Failed to connect to device \n" );
        //return 0;
    }else {
        printf( "\n2. Successfully connected to %s\n", address.GetAscii() );
    }
    printf( "\n" );
}


void Sony_Camera_Node::startStreaming(){
    PvString address = camera_ip_address.c_str();
   // PvString address("192.168.0.1");
    // Negotiate streaming packet size
    lDevice.NegotiatePacketSize();

    // Open stream - have the PvDevice do it for us
    printf( "\n3.Opening stream to device\n" );
    lStream.Open( address );

    // Reading payload size from device
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


    printf( "Resetting timestamp counter...\n" );
    PvGenCommand *lResetTimestamp = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "GevTimestampControlReset" ) );
    lResetTimestamp->Execute();

    // The buffers are queued in the stream, we just have to tell the device
    // to start sending us images
    printf( "\n4. Sending StartAcquisition command to device\n" );
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
            cv::namedWindow("Sony",cv::WINDOW_AUTOSIZE);

            cv_bridge::CvImage out_msg;
            out_msg.header.stamp   = ros::Time::now();
            out_msg.encoding = sensor_msgs::image_encodings::RGB8;
            out_msg.image    = image;

            publish_rgb_image.publish(out_msg.toImageMsg());

            cv::imshow("Current Image",image);
            cv::waitKey(30);

        }
        // re-queue the buffer in the stream object

        lStream.QueueBuffer( lBuffer );

    }
    else
    {
        // Timeout
        printf( "Timeout");
    }


}

void Sony_Camera_Node::stopStreaming(){
    // Clean-up
    printf( "\n\n" );
    // Tell the device to stop sending images
    printf( "Sending AcquisitionStop command to the device\n" );
    PvGenCommand* lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );
    lStop->Execute();

    // Abort all buffers from the stream, unqueue
    printf( "Aborting buffers still in stream\n" );
    lStream.AbortQueuedBuffers();
    while ( lStream.GetQueuedBufferCount() > 0 )
    {
        PvBuffer *lBuffer = NULL;
        PvResult lOperationResult;

        lStream.RetrieveBuffer( &lBuffer, &lOperationResult );

        printf( "  Post-abort retrieved buffer: %s\n", lOperationResult.GetCodeString().GetAscii() );
    }

    // Release buffers
    printf( "Releasing buffers\n" );
    delete []lBuffers;

    // Now close the stream. Also optionnal but nice to have
    printf( "Closing stream\n" );
    lStream.Close();
}

void Sony_Camera_Node::disconnectCamera(){
    // Finally disconnect the device. Optional, still nice to have
    printf( "Disconnecting device\n" );
    lDevice.Disconnect();
}


//Service: zooming functionalities- Optical Zoom/Digital Zoom
bool Sony_Camera_Node::zoom_in_out(seneka_srv::zoom::Request &req,
                                   seneka_srv::zoom::Response &res){

    // Optical zoom: x_x_= x{1-20}x{1}.
    // x in first position = req.zoom_ratio_optical
    // x in 2nd position = req.zoom_ratio_digital

    if(req.zoom_ratio_digital==1){

        switch(req.zoom_ratio_optical)
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
            ROS_WARN("\n Warning!: Zoom ratio should be given according to the camera standard. Default is set to 1 \n");
            lDeviceParams->SetEnumValue("ZoomRatio",0);
            break;
        }
        lDeviceParams->ExecuteCommand("CAM_Zoom");
        res.success = true;

    }

    //Digital zoom: x_x_= x{20}x{2-12}.
    //x in frist position = req.zoom_ratio_optical and
    //x in 2nd position = req.zoom_ratio_digital

    else if (req.zoom_ratio_optical==20 && req.zoom_ratio_digital>=2) {

        switch(req.zoom_ratio_digital)
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
            ROS_WARN( "\n Zoom ratio should be given according to the camera standard. Default is set to 1 \n");
            lDeviceParams->SetEnumValue("ZoomRatio",0);
            break;
        }

        lDeviceParams->ExecuteCommand("CAM_Zoom");
        res.success = true;
    }
    else{
        res.success = false;
    }
    return true;

}


//Service: Auto Focus functionalities
bool Sony_Camera_Node::focusControl(seneka_srv::focusAuto::Request &req,
                                    seneka_srv::focusAuto::Response &res){

    switch(req.focusAuto){

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
        ROS_WARN( "\nPlease specify the correct focus formate. Default AutoFocus is set to Continuous mode. \n");
        lDeviceParams->SetEnumValue("FocusAuto",3);
        break;
    }

    lDeviceParams->ExecuteCommand("CAM_Focus");
    res.success = true;

    return true;
}

//Service: videoModeNext- set on next power cycle
bool Sony_Camera_Node::videoModeNext(seneka_srv::videoMode::Request &req,
                                     seneka_srv::videoMode::Response &res){

    switch(req.video_mode_next){

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
        ROS_WARN("\nPlease specify the correct video mode. Default AutoFocus is set to HD_720p_60_Hz mode. \n");
        lDeviceParams->SetEnumValue("SonyBlockVideoModeNext",14);
        break;
    }

    res.success = true;

    return true;

}


//Service: set Focus Position
bool Sony_Camera_Node::focusPosition(seneka_srv::focus::Request &req,
                                     seneka_srv::focus::Response &res){

    lDeviceParams->GetEnumValue("FocusAuto",focus_auto);

    if (focus_auto == 0){
        //min:4096 Max: 61440
        if(4096 <= req.focus_position && req.focus_position <=61440){

            lDeviceParams->SetEnumValue("Focus",req.focus_position);
            lDeviceParams->ExecuteCommand("CAM_FocusPosInq");
            //lDeviceParams->ExecuteCommand("CAM_Focus");
            //lDeviceParams->GetEnumValue("Focus",focus_pos);
            //ROS_INFO("\nFocus position %d",focus_pos);
            res.success = true;
        }
        else{
            ROS_WARN("\nFocus position must be in-between 4096 and 61440.\n");
            res.success = false;
        }
    }else{
        ROS_WARN("\nFocus position cannot be change, while the camera is in continuous AutoFocus mode.\n");
        res.success = false;
    }
    return true;

}

//Service: set FocusNearLimit while FocusAuto is other than Off
bool Sony_Camera_Node::focusNearLimit(seneka_srv::focusNearLimit::Request &req,
                                      seneka_srv::focusNearLimit::Response &res){

    lDeviceParams->GetEnumValue("FocusAuto",focus_auto);

    if (focus_auto != 0){
        switch(req.focus_near_limit)
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

            //FocusNearLimit: Fd1p85m
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
            ROS_WARN( "\n FocusNearLimit should be given according to the camera standard. Default is set to Infinity(0) \n");
            lDeviceParams->SetEnumValue("FocusNearLimit",0);
            break;
        }
        lDeviceParams->ExecuteCommand("CAM_FocusNearLimitInq");
        //lDeviceParams->ExecuteCommand("CAM_Focus Near Limit");
        res.success = true;

    }
    else{
        ROS_WARN("\nFocusNearLimit is only valid while the camera is in continuous AutoFocus mode.\n");
        res.success = false;
    }

    return true;
}


//Service: Enable or Disable Infrared CutFilterAuto
bool  Sony_Camera_Node::infraredCutFilterAuto(seneka_srv::infraredCutFilterAuto::Request &req,
                                              seneka_srv::infraredCutFilterAuto::Response &res){

    switch(req.cutfilter_auto)
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
        ROS_WARN("\n Default InfraredCutFilterAuto is set to false. \n");
        lDeviceParams->SetBooleanValue("InfraredCutFilterAuto",false);
        break;
    }

    lDeviceParams->ExecuteCommand("CAM_AutoICRModeInq");
    res.success = true;
    return true;
}

//Service: Enable or Disable Infrared CutFilter
bool  Sony_Camera_Node::infraredCutFilter(seneka_srv::infraredCutFilter::Request &req,
                                          seneka_srv::infraredCutFilter::Response &res){
    switch(req.cutfilter)
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
        ROS_WARN("\n Default InfraredCutFilterAuto is set to false. \n");
        lDeviceParams->SetBooleanValue("InfraredCutFilter",false);
        break;

    }

    lDeviceParams->ExecuteCommand("CAM_ICRModeInq");
    res.success = true;
    return true;
}

//Service: set Picture Effect: Off, NegReversal, BlackAndWhite
bool Sony_Camera_Node::pictureEffect(seneka_srv::pictureEffect::Request &req,
                                     seneka_srv::pictureEffect::Response &res){
    switch(req.picture_effect)
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
        ROS_WARN("\n PictureEffect should be according to the camera standard. Default PictureEffect is set to Off(0). \n");
        lDeviceParams->SetEnumValue("PictureEffect",0);
        break;
    }
    lDeviceParams->ExecuteCommand("CAM_PictureEffect");
    res.success = true;
    return true;
}

//Service: Set noise reduction maximum of 5
bool Sony_Camera_Node::noiseReduction(seneka_srv::noiseReduction::Request &req,
                                      seneka_srv::noiseReduction::Response &res){

    if(0 <= req.noise_reduction && req.noise_reduction <= 5)
    {
        lDeviceParams->SetIntegerValue("NoiseReduction",req.noise_reduction);
        lDeviceParams->ExecuteCommand("CAM_NR");
        res.success = true;

    }else{
        ROS_WARN("\nNoise Reduction should be in between 0 to 5. \n");
        res.success = false;
    }
    return true;

}

//Service: Enable or Disable Back Light Compensation
bool  Sony_Camera_Node::backLightCompensation(seneka_srv::backLightCompensation::Request &req,
                                              seneka_srv::backLightCompensation::Response &res){
    switch(req.back_light)
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
        ROS_WARN("\nBack Light Compensation can be either 0(false) or 1(true). Default BackLightCompensation is set to false. \n");
        lDeviceParams->SetBooleanValue("BackLightCompensation",false);
        break;

    }

    lDeviceParams->ExecuteCommand("CAM_BackLight");
    res.success = true;
    return true;
}

//Service: Enable or disable status display
bool  Sony_Camera_Node::statusDisplay(seneka_srv::statusDisplay::Request &req,
                                      seneka_srv::statusDisplay::Response &res){

    switch(req.display_status)
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
        ROS_WARN("\nStatus Display can be either 0(false) or 1 (true). Default Status Display is set to false. \n");
        lDeviceParams->SetBooleanValue("StatusDisplay",false);
        break;

    }

    lDeviceParams->ExecuteCommand("CAM_Display");
    res.success = true;
    return true;
}

//Service: Enable or disable to display title
bool Sony_Camera_Node::titleDisplay(seneka_srv::titleDisplay::Request &req,
                                    seneka_srv::titleDisplay::Response &res){

    switch(req.title_display)
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
        ROS_WARN("Title Display can be either 0(false) or 1 (true). Default Title Display is set to false. \n");
        lDeviceParams->SetBooleanValue("TitleDisplay",false);
        break;

    }

    lDeviceParams->ExecuteCommand("CAM_Title");
    res.success = true;
    return true;
}

//Service: set title text of images
bool Sony_Camera_Node::titleText(seneka_srv::titleText::Request &req,
                                 seneka_srv::titleText::Response &res){
    PvString str = req.title_text.c_str();
    lDeviceParams->SetString("TitleText",str);
    lDeviceParams->ExecuteCommand("CAM_Title");
    res.success = true;
    return true;
}



int main(int argc, char** argv){

    // initialize ROS, specify name of node
    ros::init(argc,argv,"SonyGigeCamera");

    Sony_Camera_Node SonyCameraNode;

    // run image_publisher periodically until node has been shut down
    ros::Rate loop_rate(30); // Hz
    while(SonyCameraNode.nh.ok()){
        SonyCameraNode.publishImage();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



