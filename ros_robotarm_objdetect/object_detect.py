#!/usr/bin/env python  
import rospy
import numpy as np
import time
import cv2
import imutils
import tf
import math
import std_msgs.msg
from std_msgs.msg import String

from collections import deque
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
        global x,y,x_upper,x_lower,y_upper,y_lower,center
        global condition,object_name,click_x,click_y
	object_name="No object"
	
        for box in data.bounding_boxes:
		if click_x <= box.xmax and click_x >= box.xmin and click_y <= box.ymax and click_y >= box.ymin:
                	object_name=box.Class
                	x_upper=box.xmax
                	x_lower=box.xmin
                	y_upper=box.ymax
                	y_lower=box.ymin
                	x=((box.xmax+box.xmin)/2)
                	y=((box.ymax+box.ymin)/2)
                	center = (x,y)
			condition=True

def draw_image(image):
        global x,y,object_name,image_raw
        bridge = CvBridge()
        image_raw = bridge.imgmsg_to_cv2(image, "bgr8")
        cv2.line(image_raw,(320,0),(320,480),(0,0,255),2)
        cv2.line(image_raw,(0,240),(640,240),(0,0,255),2)
        cv2.rectangle(image_raw,(520,420),(640,480),(255,255,255),-1)
        cv2.putText(image_raw,'return',(525,465),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),1,cv2.LINE_AA)
	if condition is not False:
      		cv2.circle(image_raw, (x, y), 5, (0,0,255), -1)
        	cv2.rectangle(image_raw, (x_lower,y_lower), (x_upper,y_upper), (0,0,255), 2)
        	cv2.putText(image_raw, str(object_name), (x_lower,y_lower-10), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
	
	cv2.namedWindow('Image_raw')
	cv2.setMouseCallback('Image_raw', Click_bject)
        cv2.imshow('Image_raw',image_raw)
        cv2.waitKey(1)

def callcamera(image):
        global center,real_z

        bridge=CvBridge()
        depthimage = bridge.imgmsg_to_cv2(image, "32FC1")

        if center is not None:
                z = []
                for i in [-3,0,3]:
                        for j in [-3,0,3]:
                            if depthimage[center[1]+i,center[0]+j]!=0:
                                z.append( depthimage[center[1]+i, center[0]+j] )
                real_z = min(z)
        else:
                real_z = 0

def Camera_coordinates(camera_info):
        global x,y,x_upper,x_lower,y_upper,y_lower,center,real_z
        global real_x,real_y,robot_ready,object_name,click_x,click_y

        if object_name == "return":
                print("ready to return object!")
                pub.publish(object_name)

        if object_name is not "No object" and center is not None and robot_ready is not False and condition is not False and object_name is not "return":
                real_x = (center[0] - camera_info.K[2]) / camera_info.K[0] * real_z;
                real_y = (center[1] - camera_info.K[5]) / camera_info.K[4] * real_z;
                #print object_name,":Image_x=", x-camera_info.K[2], "Image_y=", (y-camera_info.K[5])*(-1)
                #print object_name,":Camera_x=", real_x, "Camera_y=", real_y, "Camera_z=", real_z

		if real_z and real_x and real_y is not 0.0:
                        br.sendTransform((real_x/1000, real_y/1000, real_z/1000), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "object_frame", "camera_color_optical_frame")
			# is ball grab
			if object_name=="ball" and click_x <= x_upper and click_x >= x_lower and click_y <= y_upper and click_y >= y_lower:
                        	br.sendTransform((0.0, 0.0, 0.035), tf.transformations.quaternion_from_euler(-0.0/180*math.pi, 0.0, 0.0), rospy.Time.now(), "pickup_frame","object_frame")
			# is cup
			elif object_name=="cup" and click_x <= x_upper and click_x >= x_lower and click_y <= y_upper and click_y >= y_lower:
			        br.sendTransform((0.0, 0.0, 0.035), tf.transformations.quaternion_from_euler(-90.0/180*math.pi, 0.0, -90.0/180*math.pi), rospy.Time.now(), "pickup_frame","object_frame")
                        elif object_name=="bottle" and click_x <= x_upper and click_x >= x_lower and click_y <= y_upper and click_y >= y_lower:
				br.sendTransform((0.0, 0.0, 0.035), tf.transformations.quaternion_from_euler(-0.0/180*math.pi, 0.0, 0.0), rospy.Time.now(), "pickup_frame","object_frame")
			pub.publish(object_name)
                else:
			print "xyz is 0.0, please check"


def robotCallback(message):
        global robot_ready,condition,click_x,click_y,real_z,object_name
        robot_ready = message.data
	if robot_ready == True and condition == True and real_z!=0:
		click_x,click_y=-1,-1
        if object_name == "return":
                click_x,click_y=-1,-1
                object_name = "No object"
	condition=False

def Click_bject(event, x, y, flags, param):
	global click_x,click_y,object_name
	if event == cv2.EVENT_LBUTTONDBLCLK:
		click_x,click_y=x,y
        if click_x >= 520 and click_y >=420:
                object_name="return"
                click_x,click_y=-1,-1
		
	
if __name__ == '__main__':
        x,y=-1,-1
	click_x,click_y=0,0
        x_upper,x_lower=0,0
        y_upper,y_lower=0,0
        object_name = "No object"
        condition=False
        robot_ready = False  # False
        center = None
        real_x = 0
        real_y = 0
        real_z = 0
	
        rospy.init_node('object_detect_node')
        br = tf.TransformBroadcaster()
        pub = rospy.Publisher('object_name', String, queue_size=10)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
        rospy.Subscriber('/camera/color/image_raw', Image, draw_image)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, callcamera)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, Camera_coordinates)
        rospy.Subscriber("robot_ready", std_msgs.msg.Bool, robotCallback)
        rospy.spin()
