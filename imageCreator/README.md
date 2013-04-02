Seneka - ImageCreator
======

This ros node creates evey x seconds a image file (.jpeg), while listening on topic "/optris/thermal_image_view".

It is required that a optris termo cam is running, which publishes "sensor_msgs::Image".


Start ThermoCam:
- sudo rmmod uvcvideo
- sudo modprobe uvcvideo nodrop=1

