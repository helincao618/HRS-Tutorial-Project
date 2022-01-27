#!/usr/bin/env python


#*******************************Cong Wang******************************************
# This file is a server to let NAO stand up after picking up the cup.
# NAO will maintain holding the cup when standing up.
# This server is used in detection part.
#**********************************************************************************
import rospy
import time
import sys
import motion
from naoqi import ALProxy
# '*' means add all files
from std_srvs.srv import *
from speech.srv import *

# header!!
time_exe = 8.0
dist_walk = 0.15

def StiffnessOn(proxy):
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation (pNames, pStiffnessLists, pTimeLists)

def walk_func(req):

    postureProxy = ALProxy("ALRobotPosture", "169.254.39.105", 9559)


    StiffnessOn(motionProxy)

    #postureProxy.goToPosture("StandInit", 0.5)

    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

    JointNames = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
    joint_angleLists = [0.902033805847168, -0.22707390785217285, 1.3928301334381104, 0.9511218070983887, -1.190425872802734, 0.02]

    #pFractionMaxSpeed = 0.6
    time_lists = [time_exe, time_exe, time_exe, time_exe, time_exe, time_exe]
    isAbsolute = True
    motionProxy.angleInterpolation(JointNames, joint_angleLists, time_lists, isAbsolute)


    X = 0
    Y = 0
    Theta = 0.5
    Frequency = 1.0
    #motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    motionProxy.post.moveTo(X, Y, Theta)

    #time.sleep(2.0)
    return EmptyResponse()




def Walk_server():

    #.service(name,servicetype(*.srv),callbackfunction)
    s = rospy.Service('WalkBack', Empty, walk_func)


if __name__ == '__main__':

    motionProxy = ALProxy("ALMotion", "169.254.39.105", 9559)   
    rospy.init_node('walk_server')
    Walk_server()
    rospy.spin()

