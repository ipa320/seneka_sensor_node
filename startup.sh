#!/bin/bash

sleep 180

export IGNORE_INTERACTIVE=1
source /home/robot/.bashrc
roslaunch seneka_node_bringup sensor_node.launch >/tmp/log1 2>/tmp/log2
