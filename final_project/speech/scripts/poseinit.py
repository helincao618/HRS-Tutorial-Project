#!/usr/bin/env python


##*************************************Chengjie Yuan*******************************************************************
#   Service poseinit: Execute different initial poses
#   Pose1: Stand
#   Pose2. Sit and the stiffness is disabled
#   Pose3. Crouch
##*********************************************************************************************************************


import sys
import rospy
from naoqi import ALProxy
from speech.srv import *
import sys

def callback(req):       
    
    isEnabled  = True
    motionProxy.wbEnable(isEnabled)
    if req.req==1:
        postureProxy.goToPosture("StandInit", 0.5)
    if req.req==2:
        postureProxy.goToPosture("Sit", 0.5)
        for i in range(10):
            motionProxy.setStiffnesses('Body',0)
    if req.req==3:            
        postureProxy.goToPosture("Crouch", 1.0)
    isEnabled  = False
    motionProxy.wbEnable(isEnabled)
    
    return 1

def Pose():
    rospy.init_node('pre_pose')
    s = rospy.Service('prepose', prespeech, callback)
    rospy.spin()

if __name__ == "__main__":
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)  
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    autoProxy= ALProxy("ALAutonomousMoves", robotIP, PORT)
    autoProxy.setExpressiveListeningEnabled(False)
    Pose()