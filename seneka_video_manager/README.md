Seneka - seneka_video_manager (generic)
======

## Description
The seneka_video_manager works with any camera as input device, which provides a BGR8 coded sensor_msgs::image as output. 
The seneka_video_manager subscribes on configurable ROS topic, which provides the video input. 

At the moment the seneka_video_manager offers three ROS services which could be call from remote. But the output of this ROS node is just locally because the communication interfaces aren't defined yet. 

## ROS Services 
- (1) create VideoOnDemand (seneka_video_manager::getVideo) 
 - init mode 
- (2) start/stop SnapShot and optional an interval in seconds (e.g 5) (seneka_video_manager::getSnapShots)
 - manuel selection 
- (3) start/stop LiveStream (seneka_video_manager::getLiveStream)
 - manuel selection 

## Getting started
- roslaunch/rosrun XX, which provides input topic
 - publishes input stream (BGR8 coded sensor_msgs::image)
- roslaunch seneka_bringup video_manager.launch
- rosrun seneka_video_manager video_tester
 - ROS test node for seneka_video_manager

## Launch file configuration

#### Generic
- inputTopic
- showFrame

#### VideoOnDemand
- framesPerVideo
- framesPerCache
- framesPerBinary
- videoFrameRate
- binaryFilePath
- videoFilePath

