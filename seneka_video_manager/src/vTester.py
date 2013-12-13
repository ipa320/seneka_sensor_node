#!/usr/bin/env python

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
