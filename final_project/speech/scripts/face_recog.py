#!/usr/bin/env python


##*************************************Chengjie Yuan*******************************************************************
#   Subscriber: Get the target name from speech recognition
#   Service facerecognition: Doing face recognition and looking for the person by self turnning around
#   
#   This service will be called at last when finishing grasping the goal object.
#   The robot will rotate 4 times to search the person mentioned in command.
#   If the person is found. The robot will stop and lift its arm to give the object to the person
#   After rotating 4 times, if the person can not be found, the robot will stop and say "I cannot find XX "
#   

#   How to test the code only with single speech recognition part:
#   1. $ roslaunch nao_bringup nao_full_py.launch
#   2. $ roslaunch nao_apps speech.launch
#   3. $ roslaunch nao_apps tactile.launch
#   4. $ roslaunch speech yuan.launch 
#   5. $ rosrun speech test_node
#   when finishing giving the command to the robot (robot understand the command)
#   6. $ rosservice call /facerecognition "req={}"
##*********************************************************************************************************************

import dlib
import face_recognition
import numpy as np
import sys
import rospy
import cv2
import almath
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import *
from speech.msg import *
from speech.srv import *
import time
from geometry_msgs.msg import *
from naoqi import ALProxy


# callback for image_sub subscriber
def callback(data):
        
        global a
        global b
        global num
        global c
        
        br = CvBridge()
        cv_image = br.imgmsg_to_cv2(data, "bgr8")
        global frame           
        frame = cv_image      
        #every 5 frames do the face recognition in oder to make it faster
        if (num%5==0):
            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            rgb_frame = frame[:, :, ::-1]
            # Find all the faces and face enqcodings in the frame of video
            face_locations = face_recognition.face_locations(rgb_frame)
            face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)               
            
            # judge if there is a person in view
            a=len(face_encodings)
            print(a)
            print('****************************')              
            
            # Loop through each face in this frame of video
            for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                name = "Unknown"
                face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = known_face_names[best_match_index]
                    print(name)
                    # if  name==recog_name:
                    if  ((name=="jack")&c):
                        b=False                    
                        tts.say("i find Jack")
                        c=False
                    if ((recog_name=="Tom")&c):
                        b=False
                        tts.say("i find Tom")
                        c=False                       
                # Draw a box around the face
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                # Draw a label with a name below the face
                cv2.rectangle(frame, (left, bottom - 20), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(frame, name, (left + 3, bottom - 3), font, 0.5, (255, 255, 255), 1)                
       
        cv2.imshow('Face_dectVideo', frame)
        cv2.waitKey(1)
        num=num+1
       

# callback for face name subscriber
def facename_callback(data):     
    #get the name from speech 
    # name = data.data
    global recog_name
    recog_name = data.data
    print(recog_name)  

# callback for service
def face_callback(req):
    
    #At first, rise head for searching
    names      = "HeadPitch"
    angleLists = [-20.0]
    timeLists  = [3.0]
    isAbsolute = True
    angleLists = [angle*almath.TO_RAD for angle in angleLists]
    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)    
   
    #set global flag
    global a
    global b
    global num
    global c
    num=0 
    b=True
    c=True
    a=0
      
    #begin to read frame from the camera
    image_sub = rospy.Subscriber(
            "/nao_robot/camera/top/camera/image_raw", Image, callback)
    
    #judge who needs to be found
    time.sleep(2.0)
    if (recog_name=="Tom"):
         tts.say("I'm searching for Tom")
    if (recog_name=="jack"):
         tts.say("I'm searching for Jack")   
    
    #rotate to find person until finding one 
    time_exe = 8.0
    if ((a==0)&b):
        time.sleep(10)
        motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

        JointNames = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
        joint_angleLists = [0.902033805847168, -0.22707390785217285, 1.3928301334381104, 0.9511218070983887, -1.190425872802734, 0.02]

        time_lists = [time_exe, time_exe, time_exe, time_exe, time_exe, time_exe]
        isAbsolute = True
        motionProxy.angleInterpolation(JointNames, joint_angleLists, time_lists, isAbsolute)
        X = 0
        Y = 0
        Theta = 1.57
        motionProxy.post.moveTo(X, Y, Theta)
        rate.sleep()
    
    time.sleep(5)
    if ((a==0)&b):
        time.sleep(10)
        motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

        JointNames = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
        joint_angleLists = [0.902033805847168, -0.22707390785217285, 1.3928301334381104, 0.9511218070983887, -1.190425872802734, 0.02]

        time_lists = [time_exe, time_exe, time_exe, time_exe, time_exe, time_exe]
        isAbsolute = True
        motionProxy.angleInterpolation(JointNames, joint_angleLists, time_lists, isAbsolute)
        X = 0
        Y = 0
        Theta = 1.57
        motionProxy.post.moveTo(X, Y, Theta)
        rate.sleep()
    
    time.sleep(5)
    if ((a==0)&b):
        time.sleep(10)
        motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

        JointNames = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
        joint_angleLists = [0.902033805847168, -0.22707390785217285, 1.3928301334381104, 0.9511218070983887, -1.190425872802734, 0.02]

        time_lists = [time_exe, time_exe, time_exe, time_exe, time_exe, time_exe]
        isAbsolute = True
        motionProxy.angleInterpolation(JointNames, joint_angleLists, time_lists, isAbsolute)
        X = 0
        Y = 0
        Theta = 1.57
        motionProxy.post.moveTo(X, Y, Theta)
        rate.sleep()
    
    # Fail to find the goal person
    time.sleep(5)
    if (recog_name=="Tom")&c:
         tts.say("i cannot find Tom")
    if (recog_name=="jack")&c:
         tts.say("i cannot find Jack")
    
    # Give the object to the person by lifting the shoulder
    time.sleep(10)
    if (c!=True):
        tts.say("Here you are")
        motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
        JointNames = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
        joint_angleLists = [-0.902033805847168, -0.22707390785217285, 1.3928301334381104, 0.9511218070983887, -1.190425872802734, 0.02]

        time_lists = [time_exe, time_exe, time_exe, time_exe, time_exe, time_exe]
        isAbsolute = True
        motionProxy.angleInterpolation(JointNames, joint_angleLists, time_lists, isAbsolute)

    
    print('end') 
    return EmptyResponse()

def fac_recog():
    
    facename_sub = rospy.Subscriber("/facename", String, facename_callback) 
    s = rospy.Service('/facerecognition', Empty, face_callback)
                 

if __name__ == '__main__':

    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    tts = ALProxy("ALTextToSpeech", robotIP, PORT)  
    motionProxy = ALProxy("ALMotion", robotIP, PORT)

    # Initialization for face recognition, here you can put the goal person's photo in the directory and give it a name
    rospy.init_node('face_recog')
    ycj_image = face_recognition.load_image_file("/home/hrs/group_A/src/speech/face_data/ycj.png")
    ycj_face_encoding = face_recognition.face_encodings(ycj_image)[0]
    wys_image = face_recognition.load_image_file("/home/hrs/group_A/src/speech/face_data/wys.png")
    wys_face_encoding = face_recognition.face_encodings(wys_image)[0]
    chl_image = face_recognition.load_image_file("/home/hrs/group_A/src/speech/face_data/chl.jpg")
    chl_face_encoding = face_recognition.face_encodings(chl_image)[0]
    wc_image = face_recognition.load_image_file("/home/hrs/group_A/src/speech/face_data/wc.jpg")
    wc_face_encoding = face_recognition.face_encodings(wc_image)[0]
    known_face_encodings = [
            ycj_face_encoding,
            wys_face_encoding,
            chl_face_encoding,
            wc_face_encoding,
        ]
    known_face_names = [
            "Yuan",
            "Wu",
            "Tom",
            "jack",
        ]   
    # tts = ALProxy("ALTextToSpeech", "169.254.39.105", 9559)
    #  motionProxy = ALProxy("ALMotion", "169.254.39.105" , 9559)
    # face_walkPublisher = rospy.Publisher('cmd_pose',Pose2D,queue_size =1000)
    rate = rospy.Rate (20)  
    
    fac_recog()
       
    rospy.spin()
