#!/bin/bash

rosbag record -O cap_`date +%s` /laser_pc /gps /camera /tf
