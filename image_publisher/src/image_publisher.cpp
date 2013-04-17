#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
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



PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT ( 16 )


using namespace std;



int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher rgb_image = it.advertise("GigCam_rgb_image", 10);

    //Connect to the GEV Device
    PvDevice lDevice;
    PvString address("192.168.0.1");
    printf( "\n1. Connecting to the device......" );
    PvResult lResult1 = lDevice.Connect( address, PvAccessControl );

    if(lResult1 != 0)
    {
        printf( "\n  Failed to connect to device \n" );
        return 0;
    }else {
        printf( "\n2. Successfully connected to %s\n", address.GetAscii() );
    }
    printf( "\n" );

    // Get device parameters need to control streaming
    PvGenParameterArray *lDeviceParams = lDevice.GetGenParameters();
    PvGenInteger *lPayloadSize = dynamic_cast<PvGenInteger *>( lDeviceParams->Get( "PayloadSize" ) );
    PvGenCommand *lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
    PvGenCommand *lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );

    // Negotiate streaming packet size
    lDevice.NegotiatePacketSize();

    // Create the PvStream object
    PvStream lStream;

    // Open stream - have the PvDevice do it for us
    printf( "Opening stream to device\n" );
    lStream.Open( address );

    // Reading payload size from device
    PvInt64 lSize = 0;
    lPayloadSize->GetValue( lSize );

    // Use min of BUFFER_COUNT and how many buffers can be queued in PvStream
    PvUInt32 lBufferCount = ( lStream.GetQueuedBufferMaximum() < BUFFER_COUNT ) ?
                lStream.GetQueuedBufferMaximum() :
                BUFFER_COUNT;

    // Create, alloc buffers
    PvBuffer *lBuffers = new PvBuffer[ lBufferCount ];
    for ( PvUInt32 i = 0; i < lBufferCount; i++ )
    {
        //lBuffers[ i ].GetImage()->Alloc( 1280, 720,  PvPixelRGBA8Packed);
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
    PvResult lResult = lStart->Execute();

    char lDoodle[] = "|\\-|-/";
    int lDoodleIndex = 0;
    int count = 0;



    PvInt64 width_, height_ = 0;
    lDeviceParams->GetIntegerValue( "Width", width_ );
    lDeviceParams->GetIntegerValue( "Height", height_ );
    cv::Mat raw_image_(cv::Size(width_,height_), CV_8UC3);
    cv::namedWindow("Sony",cv::WINDOW_AUTOSIZE);



    ros::Rate loop_rate(5);
    while (nh.ok()) {

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
                    width_ = lBuffer->GetImage()->GetWidth();
                    height_ = lBuffer->GetImage()->GetHeight();
                }

                //saving images (Test purpose)
                string mSavePath;
                mSavePath ="/home/mig-rb/Images/";
                count=count+1;
                mSavePath.append(static_cast<ostringstream*>( &(ostringstream() << count) )->str());
                PvBufferWriter lBufferWriter;
                lBufferWriter.Store( lBuffer, mSavePath.data(), PvBufferFormatRaw );



                //lBuffer->Attach(raw_image_.data,width_,height_,PvPixelBGR12Packed);
                Image->Attach(raw_image_.data,width_,height_,PvPixelRGB8Planar);


                //cv::cvtColor(raw_image_,raw_image_,CV_BGR2RGB);
                //cv::cvtColor(raw_image_,raw_image_,CV_GRAY2RGB);

                cv_bridge::CvImage out_msg;
                out_msg.header.stamp   = ros::Time::now();
                out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
                out_msg.image    = raw_image_;

                rgb_image.publish(out_msg.toImageMsg());

                cv::imshow("Current Image", raw_image_);

                if(cv::waitKey(30) >= 0) break;

            }
            // re-queue the buffer in the stream object

            lStream.QueueBuffer( lBuffer );


        }
        else
        {
            // Timeout
            printf( "%c Timeout\r", lDoodle[ lDoodleIndex ] );
        }

        ++lDoodleIndex %= 6;
        ros::spinOnce();
        loop_rate.sleep();
    }


    // Clean-up


    printf( "\n\n" );

    // Tell the device to stop sending images
    printf( "Sending AcquisitionStop command to the device\n" );
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

    // Finally disconnect the device. Optional, still nice to have
    printf( "Disconnecting device\n" );
    lDevice.Disconnect();

    return EXIT_SUCCESS;


}
