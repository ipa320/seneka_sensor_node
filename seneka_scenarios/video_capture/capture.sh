#!/bin/bash

BAG="/home/robot/Capture/cap_"`date +%s`".bag"
trap killgroup SIGINT

killgroup(){
  echo killing...
  kill -INT 0
  echo compress...
  gzip -4 $BAG
  #rm $BAG
  echo DONE
}

#capture of sony
tcpdump -i br0:1 udp port 4444 -s 0 -w /home/robot/Capture/cap_`date +%s`.cap &
rosbag record -O $BAG /laser_pc /tf /wind /optris/thermal_image_view /scan /bridge_response /position &

wait
echo "done"
