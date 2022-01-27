#!/usr/bin/env python


##*********************************************************************************************************************
#   Service hand_operation: Do hand operation, close or open the hand
#   Pose1: Open the hand
#   Pose2. Close the hand
##*********************************************************************************************************************
import rospy
import time
import almath
import sys
import argparse
from naoqi import ALProxy
from speech.srv import *

def handcallback(req):
    
    if req.req==0:
        motionProxy.openHand('RHand')
    if req.req==1:
        motionProxy.closeHand('RHand')

    #set the stiffness to hold the object firmly, otherwise if there is external torque on hand the stiffness will be released. 
    names = "RHand"
    stiffnessLists = 0.6
    timeLists = 0.5
    motionProxy.stiffnessInterpolation(names, stiffnessLists, timeLists)
    
    return 1
      
    
def handoperation():   
    rospy.init_node('hand_operation')
    s = rospy.Service('hand_operation', prespeech, handcallback)
    rospy.spin()

if __name__ == "__main__":

    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    handoperation()