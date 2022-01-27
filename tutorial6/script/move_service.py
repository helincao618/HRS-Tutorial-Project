#!/usr/bin/env python
import rospy
import tf
import time
import motion
import cv2
import numpy as np
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_2.srv import *
motionProxy =0
import argparse
from geometry_msgs.msg import Twist
bonus=0
# bonus is used to switch the mode, the defalut value is 0, which refers to the catching the marker without taking the orientation into consideration
# to switch to the bonus mode (catching with position and orientation), just make bouns=1
def joint_cartesian(req):
    # Example showing how to get the position of the top camera
    name            = req.name
    frame           = motion.FRAME_TORSO
    useSensorValues = True
    result          = motionProxy.getPosition(name, frame, useSensorValues)
    pose6D=Twist()
    pose6D.linear.x=result[0]
    pose6D.linear.y=result[1]
    pose6D.linear.z=result[2]
    pose6D.angular.x=result[3]
    pose6D.angular.y=result[4]
    pose6D.angular.z=result[5]
    return pose6D

def movejoints(req):
    for i in range(10):
        motionProxy.setStiffnesses('Body',1)
    chainName            = req.name
    frame     = motion.FRAME_TORSO
    fractionMaxSpeed = req.velocity
    axisMask         = 63 # control position and oriention
    position         = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    position[0] = req.desired.linear.x
    position[1] = req.desired.linear.y
    position[2] = req.desired.linear.z
    position[3] = req.desired.angular.x
    position[4] = req.desired.angular.y
    position[5] = req.desired.angular.z
    motionProxy.setPositions(chainName, frame, position, fractionMaxSpeed, axisMask)
    return 1
    
def movejoints_aruco(req):
    for i in range(10):
        motionProxy.setStiffnesses('Body',1)
    positionL         = [0.130037, 0.03674, 0.0322098, -2.24893, 0.169607, -1.03185]
    positionR         = [0.129841, -0.0339201, 0.0344021, 2.18587, 0.048325, 0.971679]
    if req.desired.linear.z==0:
        motionProxy.setPositions("LArm", motion.FRAME_TORSO, positionL, 1.0, 63)
        motionProxy.setPositions("RArm", motion.FRAME_TORSO, positionR, 1.0, 63)

    else:
        frame     = motion.FRAME_TORSO
        fractionMaxSpeed = 1.0
        axisMask         = 7 # only control position
        position0         = np.array([[req.desired.linear.x], [req.desired.linear.y], [req.desired.linear.z]]) # marker pose in optical frame
        position1         = np.array([[0.0], [0.0], [0.0]]) #marker pose in top carmera frame
        hom_position2     = np.array([[0.0], [0.0], [0.0] ,[0.0]])
        position2_R         = [0.0, 0.0, 0.0, 1.23374, -0.904337, 1.69707]
        position2_L         = [0.0, 0.0, 0.0, -1.69423, -0.487957,-1.37036]

        # topoptical frame to topcamera frame
        rotation = np.array([[0.0,-1.0,0.0],[0.0,0.0,-1.0],[1.0,0.0,0.0]])
        position1 = (rotation.T).dot(position0)
        # topcamera frame to torso frame
        name  = 'CameraTop'
        useSensorValues  = True
        result = motionProxy.getTransform(name, frame, useSensorValues)
        result_np = np.arange(16.0)
        for i in range(0, 16):
            result_np[i] =  result[i]
        trans_tor2cam = np.reshape(result, (4, 4))
        hom_position1 = np.array([[0.0],[0.0],[0.0],[1.0]])
        hom_position1[0:3] = position1
        hom_position2 = trans_tor2cam.dot(hom_position1)
        hom_position2 = np.reshape(hom_position2, 4)
        rotation_cam2optical = np.array([[0.19703548,0.56251667,0.80296452],
        [-0.56251667, -0.60592903,  0.56251667],
        [ 0.80296452, -0.56251667,  0.19703548]])
        rotation_tor2optical = trans_tor2cam[0:3,0:3].dot(rotation_cam2optical)
        rotation_opt2marker_vec= np.array([[req.desired.angular.x], [req.desired.angular.y], [req.desired.angular.z]])
        rotation_tuple = cv2.Rodrigues(rotation_opt2marker_vec)
        rotation_opt2marker_mat = rotation_tuple[0]
        rotation_tor2marker_mat = rotation_tor2optical.dot(rotation_opt2marker_mat)
        rotation_tuple1 = cv2.Rodrigues(rotation_tor2marker_mat)
        rotation_tor2marker_vec = rotation_tuple1[0]
        rotation_tor2marker_vec = np.reshape(rotation_tor2marker_vec, 3)

        # move with setPositions
        if req.desired.linear.x>=0:
            position2_R[0] = hom_position2[0]
            position2_R[1] = hom_position2[1]
            position2_R[2] = hom_position2[2]
            if bonus==1:
                position2_R[3] = rotation_tor2marker_vec[0]
                position2_R[4] = rotation_tor2marker_vec[1]
                position2_R[5] = rotation_tor2marker_vec[2]
                axisMask=63
            position2_R[3] = rotation_tor2marker_vec[0]
            position2_R[4] = rotation_tor2marker_vec[1]
            position2_R[5] = rotation_tor2marker_vec[2]
            motionProxy.setPositions("RArm", frame, position2_R, fractionMaxSpeed, axisMask)
            motionProxy.setPositions("LArm", motion.FRAME_TORSO, positionL, 1.0, 63)

        else:
            position2_L[0] = hom_position2[0]
            position2_L[1] = hom_position2[1]
            position2_L[2] = hom_position2[2]
            if bonus==1:
                position2_L[3] = rotation_tor2marker_vec[0]
                position2_L[4] = rotation_tor2marker_vec[1]
                position2_L[5] = rotation_tor2marker_vec[2]
                axisMask=63
            motionProxy.setPositions("LArm", frame, position2_L, fractionMaxSpeed, axisMask)
            motionProxy.setPositions("RArm", motion.FRAME_TORSO, positionR, 1.0, 63)
    return 1

def cartesian_coordinate_server():

    #.service(name,servicetype(*.srv),callbackfunction)
    s1 = rospy.Service('coordinate', coordinate, joint_cartesian)
    s2 = rospy.Service('MoveJoint', MoveJoints, movejoints)
    s3 = rospy.Service('MoveJoint_aruco', MoveJoints, movejoints_aruco)


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')


    cartesian_coordinate_server()

    rospy.spin()
			
		
