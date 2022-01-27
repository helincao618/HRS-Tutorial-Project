#!/usr/bin/env python

#*******************************Helin Cao***************************************************************************************
#   Service localization: get the current 2D position(x,y,theta) of NAO in global map
#   
#   This service works as the localization of navigation part.
#   During navigation, NAO need to know current position after a step. 
#   If this service is called, NAO will shake its head to find the Arucomarkers.
#   Arucomarkers with id=0,1,2,3,4 work for localization. 
#   If NAO see those markers, it will compute the current position in global map and return it.
#*******************************************************************************************************************************
import rospy
import tf
import time
import motion
import cv2
import numpy as np
import almath
import sys
from naoqi import ALProxy
from speech.msg import *
import argparse
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from using_markers.srv import localization
#set the position information as a unreasonable value
naox = 10000
naoy = 10000
naotheta = 10000
flag = True
flagshake = True

#this function is to deal with the transformation according to information of one marker
#the input is translation vector(x,y,z) and rotation vector(R,P,Y) of a marker
#the output is the transform matrix form marker frame to base_link frame, the shape of matrix is 4*4
def localization_single(x,y,z,R,P,Y):
    translation_opt2marker = np.array([[x], [y], [z]])
    rotationRPY_opt2marker = np.array([[R], [P], [Y]])
    rotationMat_opt2marker = cv2.Rodrigues(rotationRPY_opt2marker)
    transform_opt2marker = np.identity(4)
    transform_opt2marker[0:3,0:3] = rotationMat_opt2marker[0]
    for i in range(3):
        transform_opt2marker[i,3] = translation_opt2marker[i]
    transform_marker2opt = np.linalg.inv(transform_opt2marker)
    transform_opt2cam = np.zeros((4,4))
    transform_opt2cam[0,1] = -1
    transform_opt2cam[1,2] = -1
    transform_opt2cam[2,0] = 1
    transform_opt2cam[3,3] = 1
    transform_marker2cam = transform_marker2opt.dot(transform_opt2cam)

    name  = 'CameraBottom'
    frame  = motion.FRAME_ROBOT
    useSensorValues  = True
    result = motionProxy.getTransform(name, frame, useSensorValues)
    result_np = np.arange(16.0)
    for i in range(0, 16):
        result_np[i] =  result[i]
    transform_base2cam = np.reshape(result, (4, 4))
    transform_cam2base = np.linalg.inv(transform_base2cam)
    tansform_marker2base = transform_marker2cam.dot(transform_cam2base)
    return tansform_marker2base

#compute the 2D position of NAO
def naolocalization(data):
    #If NAO stop shake its head, it will get the data of markers and then compute the 2D position
    if (flagshake == True):
        global naox,naoy,naotheta,flag
        local_num = 0
        #set a data matrix to store the position information
        for i in range(len(data.id)):
            if data.id[i]<5:
                local_num+=1
        if local_num>=1:
            world_x = np.zeros(local_num)
            world_y = np.zeros(local_num)
            world_theta = np.zeros(local_num)

        for i in range(len(data.id)):  
	    #the marker with id = 0,1,2,3,4 is used as localization marker
            if data.id[i]<5:
                x = data.trafo[i].linear.x 
                y = data.trafo[i].linear.y 
                z = data.trafo[i].linear.z 
                R = data.trafo[i].angular.x
                P = data.trafo[i].angular.y 
                Y = data.trafo[i].angular.z
		#this function is used for frame transform
                tansform_marker2base = localization_single(x,y,z,R,P,Y)
                rotation_marker2base = tansform_marker2base[0:3,0:3]
                translation_marker2base = np.zeros((3,1))
                for j in range(3):
                    translation_marker2base[j] = tansform_marker2base[j,3]
		#because different marker is located in different position in global map, we need to deal with it individually
                if data.id[i] == 0:
                    world_x[i] = translation_marker2base[1]
                    world_y[i] = translation_marker2base[0]
                    rotation_modify = np.array([[0.0,1.0,0.0],[1.0, 0.0, 0.0],[0.0, 0.0, -1.0]])
                elif data.id[i] == 1:
                    world_x[i] = translation_marker2base[1]
                    world_y[i] = translation_marker2base[0]+0.5
                    rotation_modify = np.array([[0.0,1.0,0.0],[1.0, 0.0, 0.0],[0.0, 0.0, -1.0]])
                elif data.id[i] == 2:
                    world_x[i] = translation_marker2base[1]
                    world_y[i] = translation_marker2base[0]+1
                    rotation_modify = np.array([[0.0,1.0,0.0],[1.0, 0.0, 0.0],[0.0, 0.0, -1.0]])
                elif data.id[i] == 3:
                    world_x[i] = translation_marker2base[1]+0.6
                    world_y[i] = translation_marker2base[0]
                    rotation_modify = np.array([[0.0,1.0,0.0],[1.0, 0.0, 0.0],[0.0, 0.0, -1.0]])
                else:
                    world_x[i] = translation_marker2base[1]+0.6
                    world_y[i] = translation_marker2base[0]+1
                    rotation_modify = np.array([[0.0,1.0,0.0],[1.0, 0.0, 0.0],[0.0, 0.0, -1.0]])
		#this equation is explained in report
                rotationMat_theta =  rotation_modify.dot(rotation_marker2base)
		#the sign of theta need to be considered, which is explained in report
                if rotationMat_theta[0,1]<=0:
                    world_theta[i] = np.arccos(rotationMat_theta[0,0])
                else:
                    world_theta[i] = -np.arccos(rotationMat_theta[0,0])     
	#if at least one localization marker is detected, we can know the current position of NAO. We add a constant bias here beacuse of inevitable errors
        if local_num>=1:
            naox = np.mean(world_x)-0.05
            naoy = np.mean(world_y)-0.03
            naotheta = world_theta[0]
            
    return 1


def localizationsub(req):
    global flagshake
    #subscribe the information of marker detection including rotation vector, translation vector and id
    rospy.Subscriber("detectedmarker",arucomsgs,naolocalization)

    #shake the head to search for localization and obstacle marker
    isEnabled  = True
    motionProxy.wbEnable(isEnabled)
    #set the flagshake as false to stop getting data and computing, because the motion of head will influence the accurancy of data
    flagshake = False
    names      = "HeadYaw"
    angleLists = [-30.0]
    timeLists  = [3.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1)
    #set the flagshake as true to start getting information and computing
    flagshake = True
    time.sleep(1)

    flagshake = False
    names      = "HeadPitch"
    angleLists = [10.0]
    timeLists  = [6.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1)
    flagshake = True
    time.sleep(1)

    flagshake = False
    names      = "HeadYaw"
    angleLists = [0.0]
    timeLists  = [3.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1)
    flagshake = True
    time.sleep(1)

    flagshake = False
    names      = "HeadYaw"
    angleLists = [30.0]
    timeLists  = [3.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1)
    flagshake = True
    time.sleep(1)

    flagshake = False
    names      = "HeadPitch"
    angleLists = [-20.0]
    timeLists  = [6.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1)
    flagshake = True
    time.sleep(1)

    flagshake = False
    names      = "HeadYaw"
    angleLists = [0.0]
    timeLists  = [3.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
    time.sleep(1)
    flagshake = True
    time.sleep(1)
    flagshake = False
    isEnabled  = False
    motionProxy.wbEnable(isEnabled)
    
    return [naox,naoy,naotheta]

def mappingSubscribe():
    rospy.Service("localization",localization,localizationsub) 

if __name__ == '__main__':
    #robotIP=str(sys.argv[1])
    #PORT=int(sys.argv[2])
    #print sys.argv[2]
    motionProxy = ALProxy("ALMotion", "169.254.39.105", 9559)
    rospy.init_node('localization')
    mappingSubscribe()
    rospy.spin()
			
		
