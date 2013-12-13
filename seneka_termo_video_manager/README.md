Seneka - seneka_termo_video_manager
======

## Description
This ROS node manages the video data of an Optris IR-Camera. This node is able to import IR-frames from the ROS topic /optris/thermal_image and to cache the frames. This topic will be published by optris_imager_node. The topic data includes a ROS sensor_msgs::image which contains the IR-information. The mapping from IR-information to RGB8 is done inside this ROS node. So you don't have to run optris_colorconvert_node or you shouldn't run it because of performance issues. 

To manage this ROS node there exists three ROS services (see below). But the output of this ROS node is just locally because the communication interfaces aren't defined yet. 

## ROS Services 
- (1) create VideoOnDemand (seneka_termo_video_manager::getVideo) 
 - init mode 
- (2) start/stop SnapShot and optional an interval in seconds (e.g 5) (seneka_termo_video_manager::getSnapShots)
 - manuel selection 
- (3) start/stop LiveStream (seneka_termo_video_manager::getLiveStream)
 - manuel selection 

## Getting started
- connect the optis IR-Camera and initialize the camera:
 - sudo rmmod uvcvideo
 - sudo modprobe uvcvideo nodrop=1
- roscore
- roslaunch optris_drivers optris_drivers.launch 
 - publishes input stream (raw data), you have to remove the configuration for optris_colorconvert_node. 
- roslaunch seneka_node_bringup termo_video_manager.launch
- rosrun seneka_termo_video_manager termo_video_tester
 - ROS test node for termo_video_manager, simulates the remote comand center functionalities. 

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
