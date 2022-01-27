#!/usr/bin/env python
import rospy
import time
import almath
import sys
from naoqi import ALProxy
# '*' means add all files
from nao_control_tutorial_1.srv import *
motionProxy =0

#TODO: create service handler


def handle_Movejoints(req):
     #motionProxy.setAngles(["HeadYaw", "HeadPitch"],[1.0, -0.21],0.2)

    if req.id == 1:
        for i in range(10):
            motionProxy.setStiffnesses('Body',1)
        motionProxy.setAngles(req.names,req.angles,req.fractionMaxSpeed)
        print "Ready to move joints in mode setAngle."

    if req.id == 2:
        for i in range(30):
            motionProxy.setStiffnesses('Body',1)
        names = req.names
        angleLists = req.angles
        timeLists  = req.time_second
        isAbsolute = True
        #motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        motionProxy.post.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        taskList = motionProxy.getTaskList()

        # Kill task angleInterpolation to make the movement smooth
        if len(taskList) ==1:
            motionProxy.killTask(taskList[0][1])     # sometimes the setStiffness doesn't exist
        if len(taskList) ==2:
            motionProxy.killTask(taskList[1][1])
        print "Ready to move joints in mode angleInterpolation."
    return MoveJointsResponse(1)




def Movejoints_server():

    #.service(name,servicetype(*.srv),callbackfunction)
    s = rospy.Service('Movejoints', MoveJoints, handle_Movejoints)





if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')



    #TODO init service
    Movejoints_server()

    rospy.spin()
