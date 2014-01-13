#!/usr/bin/env python
#################################################################
##\file
#
# \note
#   Copyright (c) 2012 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: SENEKA
# \note
#   ROS stack name: SENEKA
# \note
#   ROS package name: seneka_termo_video_manager
#
# \author
#   Author: Johannes Goth (cmm-jg)
# \author
#   Supervised by: Christophe Maufroy (cmm)
#
# \date Date of creation: Sep 2013
#
# \brief
#   Test node for remote control of videoManager 
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################


import roslib, sys, rospy
from videoManager.srv import *

# loading the ros package manifest
roslib.load_manifest('videoManager')

def callback_getVideo():
    rospy.wait_for_service('getVideo')
    try:
        # get service handler object
        getLiveStreamHandler = rospy.ServiceProxy('getVideo', getVideo)
        # starts connection to server with request parameter
        # and gets response Object as return value  
        serviceResponse = getLiveStreamHandler(1)
        
        if serviceResponse.releasedVideo == 1:
            print "Creating videoOnDemand."
        elif serviceResponse.releasedVideo == -1:
            print "VideoRecorder is busy."
        elif serviceResponse.releasedVideo == -2:
            print "Not enough frames cached to create a videoOnDemand. You have to wait for some minutes."
        else:
            print "Unknown response."
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e 

def callback_getSnapShot(interval):
    rospy.wait_for_service('getSnapShots')
    try:
        try:
            # get service handler object
            getLiveStreamHandler = rospy.ServiceProxy('getSnapShots', getSnapShots)
            # starts connection to server with request parameter
            # and gets response Object as return value  
            serviceResponse = getLiveStreamHandler(True, interval)

            if serviceResponse.notifier == True:
                print "Creating SnapShots every", interval, "seconds."
            else:
                print "Stopped creating SnapShots"
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
             
    except sys.excepthook, e:
        print e
    
def callback_getLiveStream():
    rospy.wait_for_service('getLiveStream')
    try:
        try:
            # get service handler object
            getLiveStreamHandler = rospy.ServiceProxy('getLiveStream', getLiveStream)
            # starts connection to server with request parameter
            # and gets response Object as return value  
            serviceResponse = getLiveStreamHandler(True)

            if serviceResponse.notifier == True:
                print "Starting live stream."
            else:
                print "Stopped live stream."
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
             
    except sys.excepthook, e:
        print e
    

def usage():
    return "python_vTester: You have to choose a serviceID [1 or 2 or 3] and optional for serviceID 3 you are able to choose an interval in seconds [integer]"

if __name__ == "__main__":
    # init ros node
    rospy.init_node("python_vTester")
    interval = 0
    serviceID = 0
    
    # check system parameters 
    if len(sys.argv) == 3:
        serviceID = int(sys.argv[1])
        interval = int(sys.argv[2])
    elif len(sys.argv) == 2:
        serviceID = int(sys.argv[1])
        # default interval for snapShots
        interval = 5
    else:       
        print usage()
        sys.exit(1)
    # Choose method depending on the serviceID     
    if serviceID == 1:
        callback_getVideo()
    elif serviceID == 2:
        callback_getSnapShot(interval)
    elif serviceID == 3:
        callback_getLiveStream()
    else:
        usage()
