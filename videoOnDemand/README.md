Seneka - VideoOnDemand
======
VideoOnDemand is a ROS node which provides a video of the last x seconds. The creation on a video could be started by calling a service, which is provided by this ROS node.

At the moment the output of this ROS node is a video, which is stored in an "avi" container. The input frame are applied by an Optris termo camera, which publishes its frames on "/optris/thermal_image_view" topic.

Commands to start and restart the Optris termo camera:
- sudo rmmod uvcvideo
- sudo modprobe uvcvideo nodrop=1
