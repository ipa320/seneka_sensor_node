Seneka - seneka_video_manager (generic)
======

## Description
This ROS node manages the video data of any camera as input device, which provides a BGR8 encoded sensor_msgs::image. This node imports the sensor_msgs::images by a configurable ROS topic. The topic could be configurated in the launch-file. 

To manage this ROS node there exists three ROS services (see below). But the output of this ROS node is just locally because the communication interfaces aren't defined yet.


## ROS Services 
- (1) create VideoOnDemand (seneka_video_manager::getVideo) 
 - init mode 
- (2) start/stop SnapShot and optional an interval in seconds (e.g 5) (seneka_video_manager::getSnapShots)
 - manuel selection 
- (3) start/stop LiveStream (seneka_video_manager::getLiveStream)
 - manuel selection 

## Getting started
- roslaunch/rosrun <ROS node>, which provides the input topic
 - publishes input stream (BGR8 encoded sensor_msgs::image)
- roslaunch seneka_node_bringup video_manager.launch
- rosrun seneka_video_manager video_tester
 - ROS test node for seneka_video_manager, simulates the remote command center functionalities.

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
