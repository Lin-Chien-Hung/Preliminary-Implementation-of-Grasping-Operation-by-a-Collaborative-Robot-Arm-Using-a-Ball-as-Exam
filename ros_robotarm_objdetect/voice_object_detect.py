#!/usr/bin/env python  
import rospy
import numpy as np
import time
import cv2
import imutils
import tf
import sys
import math
import std_msgs.msg
from std_msgs.msg import String

#from tensorflow.keras.preprocessing.image import img_to_array
#from tensorflow.keras.preprocessing.image import load_img
#from tensorflow.keras.models import load_model

from collections import deque
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

def Voicecallback(voice):
    global object_name

    object_name = voice.data

def Yolocallback(data):
    global object_name,x,y,x_upper,x_lower,y_upper,y_lower,center,model

    for box in data.bounding_boxes:
        if object_name == box.Class:
            x_upper = box.xmax
            x_lower = box.xmin
            y_upper = box.ymax
            y_lower = box.ymin
            x = ((box.xmax+box.xmin)/2)
            y = ((box.ymax+box.ymin)/2)
            center = (x,y)

def Depthcameracallback(image):
    global center,real_z

    bridge=CvBridge()
    depthimage = bridge.imgmsg_to_cv2(image, "32FC1")
    if center is not None:
        z = []
        for i in [-11,0,11]:
            for j in [-11,0,11]:
                if depthimage[center[1]+i,center[0]+j]!=0:
                    z.append( depthimage[center[1]+i, center[0]+j] )
        real_z = min(z)
    else:
        real_z = 0

def Camera_calculate(camera_info):
    global object_name,center,real_x,real_y,real_z,robot_ready

    time.sleep(0.5)

    if object_name=="open" or object_name=="close" or object_name=="return":

        pub.publish(object_name)
    
    elif object_name == "exit":
        object_name = "exit"
        pub.publish(object_name)
        sys.exit()

    elif center is None:
    
        object_name = "No object"
        
    else:

        real_x = (center[0] - camera_info.K[2]) / camera_info.K[0] * real_z
        real_y = (center[1] - camera_info.K[5]) / camera_info.K[4] * real_z
    
        if real_z and real_x and real_y is not 0.0:
            br.sendTransform((real_x/1000, real_y/1000, real_z/1000), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "object_frame", "camera_color_optical_frame")
            if object_name=="ball":
                br.sendTransform((0.0, 0.0, 0.035), tf.transformations.quaternion_from_euler(-0.0/180*math.pi, 0.0, 0.0), rospy.Time.now(), "pickup_frame","object_frame")
            elif object_name=="cup":
                br.sendTransform((0.0, 0.0, 0.035), tf.transformations.quaternion_from_euler(-90.0/180*math.pi, 0.0, -90.0/180*math.pi), rospy.Time.now(), "pickup_frame","object_frame")
            elif object_name=="bottle":
                br.sendTransform((0.0, 0.0, 0.035), tf.transformations.quaternion_from_euler(-90.0/180*math.pi, 0.0, -90.0/180*math.pi), rospy.Time.now(), "pickup_frame","object_frame")
            pub.publish(object_name)

        else:
            print ("xyz is 0.0, please check")

def Robotcallback(message):
    global robot_ready,center,object_name

    robot_ready = message.data
    if robot_ready is False:
        center = None
        object_name = "No object"

if __name__ == '__main__':
    object_name = "No object"
    x,y,x_upper,x_lower,y_upper,y_lower = 0,0,0,0,0,0
    center = None
    real_x,real_y,real_z = 0,0,0
    robot_ready = False

    rospy.init_node('object_detect_node')
    br = tf.TransformBroadcaster()
    pub = rospy.Publisher('object_name', String, queue_size=10)
    rospy.Subscriber('voice_object', String, Voicecallback)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, Yolocallback)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, Depthcameracallback)
    rospy.Subscriber("/camera/color/camera_info", CameraInfo, Camera_calculate)
    rospy.Subscriber("robot_ready", std_msgs.msg.Bool, Robotcallback)
    rospy.spin()
