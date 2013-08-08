Seneka - videoManager (generic)
======

## Description
The videoManager works with any camera as input device, which provides a BGR8 coded sensor_msgs::image as output. 
The videoManager subscribes on configurable ROS topic, which provides the video input. 

At the moment the videoManager offers three ROS services which could be call from remote. But the output of this ROS node is just locally because the communication interfaces aren't defined yet. 

## ROS Services 
- (1) create VideoOnDemand (termoVideoManager::getVideo) 
 - init mode 
- (2) start/stop SnapShot and optional an interval in seconds (e.g 5) (termoVideoManager::getSnapShots)
 - manuel selection 
- (3) start/stop LiveStream (termoVideoManager::getLiveStream)
 - manuel selection 

## Getting started
- roslaunch/rosrun XX, which provides input topic
 - publishes input stream (BGR8 coded sensor_msgs::image)
- roslaunch videoManager videoManager.launch
- rosrun videoManager vTester
 - ROS test node for videoManager

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

