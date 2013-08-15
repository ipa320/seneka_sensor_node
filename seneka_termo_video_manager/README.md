Seneka - seneka_termo_video_manager
======

## Description
The seneka_termo_video_manager works with Optris termo cameras as input device. 
The seneka_termo_video_manager subscribes on the /optris/thermal_image topic which is provided by the optirs ROS driver. 
This topics publishes a ROS sensor_msgs::image which contains the termo information. The mapping from termo information to RGB8 is done inside this ROS node.

At the moment the seneka_termo_video_manager offers three ROS services which could be call from remote. But the output of this ROS node is just locally because the communication interfaces aren't defined yet. 

## ROS Services 
- (1) create VideoOnDemand (seneka_termo_video_manager::getVideo) 
 - init mode 
- (2) start/stop SnapShot and optional an interval in seconds (e.g 5) (seneka_termo_video_manager::getSnapShots)
 - manuel selection 
- (3) start/stop LiveStream (seneka_termo_video_manager::getLiveStream)
 - manuel selection 

## Getting started
- roslaunch optris_drivers optris_drivers.launch 
 - publishes input stream (raw data)
- roslaunch seneka_bringup termo_video_manager.launch
- rosrun seneka_termo_video_manager termo_video_tester
 - ROS test node for termo_video_manager

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
