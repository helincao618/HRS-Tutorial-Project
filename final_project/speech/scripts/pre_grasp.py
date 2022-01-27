#!/usr/bin/env python


#*******************************Cong Wang & Yansong Wu*******************************************
# This file is a service server to let NAO grasp the object.
# The motions begin with pre-grasp, end with pre-stand.
# This server is used in detection part.
#************************************************************************************************
import rospy
import time
import almath
import sys
import motion
from naoqi import ALProxy
# '*' means add all files
from std_srvs.srv import *
from speech.srv import *
motionProxy =0

def handle_Movejoints(req):
     #motionProxy.setAngles(["HeadYaw", "HeadPitch"],[1.0, -0.21],0.2)
    
    # Activate Whole Body Balancer.
    isEnabled  = True
    motionProxy.wbEnable(isEnabled)

    motionProxy.wbFootState("Fixed", "RLeg")
    motionProxy.wbFootState("Plane", "LLeg")


    # pre_grasp position
    name_joint= ["HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
          "LHand", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipYawPitch",
          "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "RShoulderPitch", "RShoulderRoll",
          "RElbowYaw", "RElbowRoll", "RWristYaw","RHand"]


    # down -1
    joint_angleLists = [-0.0169, 0.00311, 2.07853, -0.154976, -1.10912, -0.0966, 
                        0.34051, 0.648, -0.14722, 0.34059, -0.371186, 1.263974, 
                        -0.78392, -0.04598, -0.147222, -0.23926, -0.270026, 1.47575, 
                        -1.12591, 0.073651, 1.669034, -0.165714, 1.655144, 0.917374, -0.007711, 0.2616]

    fractionMaxSpeed  = 0.03
    motionProxy.setAngles(name_joint, joint_angleLists, fractionMaxSpeed)
    time.sleep(10.0)


    # down -2
    joint_angleLists = [-0.01845, 0.03064, 2.06779, -0.135034, -1.116794, -0.05518198, 
                        1.2486341, 0.7304, -0.28988, 0.750168, -0.812978, 1.382092, 
                        -0.57989, -0.397264, -0.28988, -0.099668, -0.928112, 2.106224, 
                        -1.1863, 0.0015759, 0.89283, 0.102736, 0.67645216, 0.6029041, 0.944902, 0.26239997]

    fractionMaxSpeed  = 0.03
    motionProxy.setAngles(name_joint, joint_angleLists, fractionMaxSpeed)
    time.sleep(10.0)

    # pre-grasp
    joint_angleLists = [-0.007712, 0.05058, 1.994158, 0.17790198, -1.1214, -0.062852144, 
                        1.21642, 0.7312, -0.654976, 0.220938, -0.98018, 1.759456, 
                        -0.544612, -0.39776, -0.654976, -0.18097, -1.1474738, 2.10469, 
                        -0.8636, 0.113558, 0.681138, 0.251534, -0.159578, 0.49399, -0.886694, 0.888]

    fractionMaxSpeed  = 0.03
    motionProxy.setAngles(name_joint, joint_angleLists, fractionMaxSpeed)
    time.sleep(10.0)
                    


    ######################################################################################
    rospy. wait_for_service('hand_operation')
    hand_operation = rospy.ServiceProxy('hand_operation', prespeech)
    #resp1 = hand_operation(0)

    time.sleep(2.0)

    # rospy. wait_for_service('hand_operation')
    # hand_operation = rospy.ServiceProxy('hand_operation', prespeech)
    resp1 = hand_operation(1)

    


    ######################################################################################

    name_joint= ["HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
          "LHand", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipYawPitch",
          "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "RShoulderPitch", "RShoulderRoll",
          "RElbowYaw", "RElbowRoll", "RWristYaw"]

    # after grasp -1
    joint_angleLists = [-0.00464, 0.04905, 2.02637, 0.16256, -1.11986, -0.062852, 1.219488, 
                         0.7348, -0.6565, 0.211734, -0.94337, 1.72724, -0.556884, -0.39776, -0.65651,
                          -0.18864, -1.149, 2.1016221, -0.89888, 0.08748, 0.13964, -0.012314, 
                          -0.14884, 0.228608, 0.04291]


    fractionMaxSpeed  = 0.03
    motionProxy.setAngles(name_joint, joint_angleLists, fractionMaxSpeed)
    time.sleep(10.0)

    # after grasp - 2
    joint_angleLists = [-0.00464, 0.03984, 2.01563, 0.16563, -1.12293, -0.05825, 
                        1.22102, 0.7368, -0.65498, 0.21634, -0.95564, 1.73798, -0.55995, 
                        -0.3973, -0.65498, -0.1871, -1.152076, 2.103156, -0.8958139, 0.096684, 
                        -0.00609, -0.303773, -0.191792, 0.23321, 0.068988]

    fractionMaxSpeed  = 0.03
    motionProxy.setAngles(name_joint, joint_angleLists, fractionMaxSpeed)
    time.sleep(10.0)

    # pre-stand-1
    joint_angleLists = [-0.010779857635498047, 0.09046411514282227, 2.0677900314331055, 0.13034796714782715, -1.733461856842041, 
                        -0.17943596839904785, 1.501744031906128, 0.8592000007629395, -0.24846601486206055, -0.06131792068481445, 
                        -0.6411700248718262, 2.0938682556152344, -1.1796879768371582, 0.07520794868469238, -0.24846601486206055, 
                        0.06907200813293457, -0.6366519927978516, 2.1062240600585938, -1.1842060089111328, -0.07643899321556091, 
                        0.902033805847168, -0.22707390785217285, 1.3928301334381104, 0.9511218070983887, -1.190425872802734]

    fractionMaxSpeed  = 0.03
    motionProxy.setAngles(name_joint, joint_angleLists, fractionMaxSpeed)
    time.sleep(10.0)


    # # Deactivate Whole Body Balancer.
    isEnabled  = False
    motionProxy.wbEnable(isEnabled)

    return EmptyResponse()


def Movejoints_server():

    #.service(name,servicetype(*.srv),callbackfunction)
    s = rospy.Service('Movejoints', Empty, handle_Movejoints)





if __name__ == '__main__':
    # robotIP=str(sys.argv[1])
    # PORT=int(sys.argv[2])
    # print sys.argv[2]
    # motionProxy = ALProxy("ALMotion", robotIP, PORT)
    motionProxy = ALProxy("ALMotion", "169.254.39.105", 9559)   
    rospy.init_node('move_joints_server')
    Movejoints_server()

    rospy.spin()
