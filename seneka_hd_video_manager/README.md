Seneka - seneka_hd_video_manager
======

## Description
Not all of the functionalities of hd_video_manager are completely implemented at the moment.

There had been problems with ffmpeg while developing the create video functionality. The problem could be located while initializing the AVCodecContext which has to be specific for h.264. The required parameters couldn't be figured out.

These functions are running:
 - reading Frames/Packets for an input-stream (could be a network session or file)
 - buffering the input-stream with boost serialization

These functions aren't running:
 - creating a videoOnDemand file from the buffered input-stream
 - as well as the snapshot functionality
 - the live stream is directly provided by the teracue hardware encoder

## Starting/Testing the seneka_hd_video_manager

- roscore
- rosrun seneka_hd_video_manager hd_video_manager_node <videoFile or url>

