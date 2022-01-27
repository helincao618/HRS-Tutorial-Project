#!/usr/bin/env python


#*************************************Cong Wang**************************************************
# This file is a service server to let NAO bow head.
# This server is used in detection part.
#************************************************************************************************
import rospy
import time
import almath
import sys
import motion
from naoqi import ALProxy
from std_srvs.srv import *
motionProxy =0



def handle_Movejoints(req):
     #motionProxy.setAngles(["HeadYaw", "HeadPitch"],[1.0, -0.21],0.2)
    
    # Activate Whole Body Balancer.
    isEnabled  = True
    motionProxy.wbEnable(isEnabled)
    
    names      = "HeadPitch"
    angleLists = [10.0]
    timeLists  = [3.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)


    isEnabled  = False
    motionProxy.wbEnable(isEnabled)

    return EmptyResponse()


def Movejoints_server():

    #.service(name,servicetype(*.srv),callbackfunction)
    s = rospy.Service('Downhead', Empty, handle_Movejoints)





if __name__ == '__main__':
    # robotIP=str(sys.argv[1])
    # PORT=int(sys.argv[2])
    # print sys.argv[2]
    # motionProxy = ALProxy("ALMotion", robotIP, PORT)
    motionProxy = ALProxy("ALMotion", "169.254.39.105", 9559)   
    rospy.init_node('down_head_server')
    Movejoints_server()

    rospy.spin()
