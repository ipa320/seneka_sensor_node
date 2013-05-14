
#include "sony_camera_node.h"


PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT ( 1 )


using namespace std;

#define clip(x) (unsigned char)( (x) < 0 ? 0 : ( (x) > 255 ? 255 : (x) ) )


Sony_Camera_Node::Sony_Camera_Node():it(nh){

    lSize = 0;
    width_, height_ = 0;

    // Get device parameters need to control streaming
    lDeviceParams = lDevice.GetGenParameters();

    publish_rgb_image = it.advertise("SonyGigCam_rgb_image", 10);

    connectCamera();
    startStreaming();
}

Sony_Camera_Node::~Sony_Camera_Node(){
    stopStreaming();
    disconnectCamera();
}


void Sony_Camera_Node::connectCamera(){

    PvString address("192.168.0.1");

    printf( "\n1. Connecting to the device SOnyHD......" );
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

    PvString address("192.168.0.1");
    // Negotiate streaming packet size
    lDevice.NegotiatePacketSize();

    // Open stream - have the PvDevice do it for us
    printf( "Opening stream to device\n" );
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
    // printf( "Sending StartAcquisition command to device\n" );
    PvGenCommand *lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
    lResult = lStart->Execute();


}



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

            //if(cv::waitKey(30) >= 0) break;

        }
        // re-queue the buffer in the stream object

        lStream.QueueBuffer( lBuffer );

    }
    else
    {
        // Timeout
        // printf( "%c Timeout\r", lDoodle[ lDoodleIndex ] );
    }
    //++lDoodleIndex %= 6;

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




int main(int argc, char** argv){


    // initialize ROS, specify name of node
    ros::init(argc,argv,"SonyGigeCam_images");

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



