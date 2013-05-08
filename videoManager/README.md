Seneka - videoManager
======

## Description
The termoVideoManager works with Optris termo cameras as input device. 
The termoVideoManager subscribes on the /optris/thermal_image topic which is provided by the optirs ROS driver. 
This topics publishes a ROS sensor_msgs::image which contains the termo information. The mapping from termo information to RGB8 is done inside this ROS node.

At the moment the termoVideoManager offers three ROS services which could be call from remote. But the output of this ROS node is just locally because the communication interfaces aren't defined yet. 

## ROS Services 
- (1) create VideoOnDemand (termoVideoManager::getVideo) 
 - init mode 
- (2) start/stop SnapShot and optional an interval in seconds (e.g 5) (termoVideoManager::getSnapShots)
 - manuel selection 
- (3) start/stop LiveStream (termoVideoManager::getLiveStream)
 - manuel selection 

## Getting started
- roslaunch optris_drivers optris_drivers.launch 
 - publishes input stream (raw data)
- roslaunch termoVideoManager termoVideoManager.launch
- rosrun termoVideoManager vTester
 - ROS test node for termoVideoManager

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

#### Converting Optris image to RGB8
- minTemperature
- maxTemperature
- PaletteScalingMethod
- Palette

## Commands to start and restart the Optris kernel module:
- sudo rmmod uvcvideo
- sudo modprobe uvcvideo nodrop=1
