#!/usr/bin/env python


#*************************************Cong Wang**************************************************
# This file is a service server to let NAO move head to detect markers.
# This server is used in navigation part.
#************************************************************************************************
import rospy
import time
import almath
import sys
import motion
from naoqi import ALProxy
# '*' means add all files
from std_srvs.srv import *
motionProxy =0

def handle_Movejoints(req):
     #motionProxy.setAngles(["HeadYaw", "HeadPitch"],[1.0, -0.21],0.2)
    
    # Activate Whole Body Balancer.
    isEnabled  = True
    motionProxy.wbEnable(isEnabled)

    
    names      = "HeadYaw"
    angleLists = [-45.0]
    timeLists  = [12.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

    names      = "HeadPitch"
    angleLists = [10.0]
    timeLists  = [6.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

    names      = "HeadYaw"
    angleLists = [45.0]
    timeLists  = [25.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

    names      = "HeadPitch"
    angleLists = [-20.0]
    timeLists  = [6.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

    names      = "HeadYaw"
    angleLists = [0.0]
    timeLists  = [12.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)


    isEnabled  = False
    motionProxy.wbEnable(isEnabled)

    return EmptyResponse()


def Movejoints_server():

    #.service(name,servicetype(*.srv),callbackfunction)
    s = rospy.Service('Shakehead', Empty, handle_Movejoints)





if __name__ == '__main__':
    # robotIP=str(sys.argv[1])
    # PORT=int(sys.argv[2])
    # print sys.argv[2]
    # motionProxy = ALProxy("ALMotion", robotIP, PORT)
    motionProxy = ALProxy("ALMotion", "169.254.210.108", 9559)   
    rospy.init_node('shakehead')
    #TODO init service
    Movejoints_server()

    rospy.spin()
