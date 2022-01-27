#!/usr/bin/env python


##*************************************Chengjie Yuan*******************************************************************
#   Service prespeech: Predefined sentences for different situations
#   1.Start : "Can I help you"
#   2.Situation1: understand the command:"Unterstand"
#   3.Situation2: do not understand the command:"I don't unterstand what you mean"
#                 if the person's name is not in the vocabulary
#                 "i don't know that guy"
#                 "Please say it again"
##*********************************************************************************************************************

import rospy
import tf
import time
import motion
import cv2
import numpy as np
import almath
import sys
from speech.srv import *
from naoqi import ALProxy

def callback(req):
    if req.req==1:       
        tts.say("Can i help you")
    if req.req==2:
        tts.say("Unterstand")
    if req.req==3:
        tts.say("I don't unterstand what you mean")          
    if req.req==4:
        tts.say("i don't know that guy")
        tts.say("Please say it again")
    
    return True

def speech():
    rospy.init_node('pre_speech')
    s = rospy.Service('prespeech', prespeech, callback)
    rospy.spin()

if __name__ == "__main__":
   
   robotIP=str(sys.argv[1])
   PORT=int(sys.argv[2])
   print sys.argv[2]
   tts = ALProxy("ALTextToSpeech", robotIP, PORT) 
   speech()
