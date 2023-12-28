#!/usr/bin/env python
# -*- coding: utf8 -*-
import rospy
import numpy as np
import imutils
import tf
import std_msgs.msg
import sys
from std_msgs.msg import String
from playsound import playsound

import speech_recognition as sr
import time

r = sr.Recognizer()

def Robotcallback(message):
    global robot_ready

    robot_ready = message.data

if __name__ == '__main__':
    object_name = "No object"
    rospy.init_node('voice_detect_node')
    br = tf.TransformBroadcaster()
    pub = rospy.Publisher('voice_object', String, queue_size=10)
    rospy.Subscriber("robot_ready", std_msgs.msg.Bool, Robotcallback)
    flag=0
    object_num=0
    robot_ready = False

    while True:
        if robot_ready == True:
#==============================================================================
            with sr.Microphone(sample_rate = 44100) as source:
                try:
                    print("降噪中----------------")
                    r.adjust_for_ambient_noise(source,duration = 1)
                    r.energy_threshold = 3500
                    print("錄音開始--------------")
                    audio = r.listen(source)
                    print("辨識中----------------")
                    talk = r.recognize_google(audio, language = "zh-TW")
#==============================================================================
                    if flag == 0 and talk == u"球":
                        playsound('ball.mp3')
                        voice_object = "ball"
                        flag+=1
                        print "catch ball"
#==============================================================================
                    elif flag == 0 and talk == u"杯子" or talk == u"保溫杯":
                        playsound('cup.mp3')
                        voice_object = "cup"
                        flag+=1
                        print "catch cup"
#==============================================================================
                    elif flag == 0 and talk == u"瓶子" or talk == u"塑膠瓶":
                        playsound('bottle.mp3')
                        voice_object = "bottle"
                        flag+=1
                        print "catch bottle"
#==============================================================================
                    elif flag == 0 and talk == u"打開" or talk == u"放開":
                        playsound('ok.mp3')
                        voice_object = "open"
                        pub.publish(voice_object)
                        print "open"
#==============================================================================
                    elif flag == 0 and talk == u"抓住":
                        playsound('ok.mp3')
                        voice_object = "close"
                        pub.publish(voice_object)
                        print "close"
#==============================================================================
                    elif flag == 0 and object_num!=0 and talk == u"放回去":
                        playsound('ok.mp3')
                        voice_object = "return"
                        object_num-=1
                        pub.publish(voice_object)
                        print "return"
#==============================================================================
                    elif talk == u"離開":
                        playsound('ok.mp3')
                        voice_object = "exit"
                        pub.publish(voice_object)
                        print "voice is leave"
                        break
#==============================================================================
                    if flag > 0 and talk == u"是" or talk == u"對":
                        playsound('yes.mp3')
                        object_num+=1
                        pub.publish(voice_object)
                        flag=0
                    elif flag > 0 and talk == u"不是" or talk == u"不對":
                        playsound('no.mp3')
                        flag=0
                    print "you talk:",talk
                    time.sleep(1)
#==============================================================================
                except sr.UnknownValueError:
                    print("請繼續talk")
                except sr.RequestError as e:
                    print("無法從Google語音辨識服務取得回覆: {0}".format(e))
