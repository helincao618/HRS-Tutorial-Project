#!/usr/bin/env python


#*************************************Cong Wang**************************************************
# This file is a service server to let NAO walk according to its polar system.
# This server is used in detection and navigation part.
#************************************************************************************************
import rospy
import time
import sys
import motion
from naoqi import ALProxy
from std_srvs.srv import *
from speech.srv import *


def StiffnessOn(proxy):

    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation (pNames, pStiffnessLists, pTimeLists)

def walk_func(walkX, walkY, walkTheta):

    postureProxy = ALProxy("ALRobotPosture", "169.254.39.105", 9559)
    StiffnessOn(motionProxy)

    print("Begin walking!!!!")
    time.sleep(1.5)
    postureProxy.goToPosture("StandInit", 0.5)
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
    print("Now walking to: ", walkX, walkY, walkTheta)
    motionProxy.post.moveTo(walkX, walkY, walkTheta)





def walk_polar_Callback(req):

    walk_dist = req.dist
    walk_theta = req.theta
    if (walk_dist == 0):
        walk_func(0, 0, walk_theta)
    else:
        walk_func(0, 0, walk_theta)
        walk_func(walk_dist, 0, 0)
        
    return True


    #.service(name,servicetype(*.srv),callbackfunction)


if __name__ == '__main__':

    motionProxy = ALProxy("ALMotion", "169.254.39.105", 9559)   
    rospy.init_node('walk_polar')
    #.service(name,servicetype(*.srv),callbackfunction)
    rospy.Service('WalkPolar', walk_polar, walk_polar_Callback)
    rospy.spin()
