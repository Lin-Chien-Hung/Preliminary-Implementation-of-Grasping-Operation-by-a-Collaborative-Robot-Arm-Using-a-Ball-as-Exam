#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy
import time
import numpy as np


import math
import argparse
import tf
import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

import math
import argparse

""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq
currentFingerPosition = [0.0, 0.0, 0.0]
homeposition = [-0.2279,-0.3880,0.4781,1.6271,-0.0115,0.2026]
dstPosition = [0.2109,-0.2705,0.5025,1.6715,1.1042,0.1156]
finger_value = [[4000,4000,4000],
		[1000,1000,1000]]
return_num = 0
return_position = np.zeros((150,3),dtype=np.float)
return_object = np.zeros(150,dtype=String)
return_rot = np.zeros((150,4),dtype=np.float)

def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None


def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_


def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_



def getcurrentCartesianCommand(prefix_):
    # wait to get current position
    topic_address = '/' + prefix_ + 'driver/out/cartesian_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, setcurrentCartesianCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
    print 'position listener obtained message for Cartesian pose. '


def setcurrentCartesianCommand(feedback):
    global currentCartesianCommand

    currentCartesianCommand_str_list = str(feedback).split("\n")

    for index in range(0,len(currentCartesianCommand_str_list)):
        temp_str=currentCartesianCommand_str_list[index].split(": ")
        currentCartesianCommand[index] = float(temp_str[1])
    # the following directly reading only read once and didn't update the value.
    # currentCartesianCommand = [feedback.X, feedback.Y, feedback.Z, feedback.ThetaX, feedback.ThetaY, feedback.Z] 
    # print 'currentCartesianCommand in setcurrentCartesianCommand is: ', currentCartesianCommand


def kinova_robotTypeParser(kinova_robotType_):
    """ Argument kinova_robotType """
    global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
    robot_category = kinova_robotType_[0]
    robot_category_version = int(kinova_robotType_[1])
    wrist_type = kinova_robotType_[2]
    arm_joint_number = int(kinova_robotType_[3])
    robot_mode = kinova_robotType_[4]
    finger_number = int(kinova_robotType_[5])
    prefix = kinova_robotType_ + "_"
    finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
    finger_maxTurn = 6800  # max thread turn for one finger


def unitParser(unit_, pose_value_, relative_):
    """ Argument unit """
    global currentCartesianCommand

    position_ = pose_value_[:3]
    orientation_ = pose_value_[3:]

    for i in range(0,3):
        if relative_:
            position_[i] = pose_value_[i] + currentCartesianCommand[i]
        else:
            position_[i] = pose_value_[i]

    # print('pose_value_ in unitParser 1: {}'.format(pose_value_))  # debug

    if unit_ == 'mq':
        if relative_:
            orientation_XYZ = Quaternion2EulerXYZ(orientation_)
            orientation_xyz_list = [orientation_XYZ[i] + currentCartesianCommand[3+i] for i in range(0,3)]
            orientation_q = EulerXYZ2Quaternion(orientation_xyz_list)
        else:
            orientation_q = orientation_

        orientation_rad = Quaternion2EulerXYZ(orientation_q)
        orientation_deg = list(map(math.degrees, orientation_rad))

    elif unit_ == 'mdeg':
        if relative_:
            orientation_deg_list = list(map(math.degrees, currentCartesianCommand[3:]))
            orientation_deg = [orientation_[i] + orientation_deg_list[i] for i in range(0,3)]
        else:
            orientation_deg = orientation_

        orientation_rad = list(map(math.radians, orientation_deg))
        orientation_q = EulerXYZ2Quaternion(orientation_rad)

    elif unit_ == 'mrad':
        if relative_:
            orientation_rad_list =  currentCartesianCommand[3:]
            orientation_rad = [orientation_[i] + orientation_rad_list[i] for i in range(0,3)]
        else:
            orientation_rad = orientation_

        orientation_deg = list(map(math.degrees, orientation_rad))
        orientation_q = EulerXYZ2Quaternion(orientation_rad)

    else:
        raise Exception("Cartesian value have to be in unit: mq, mdeg or mrad")

    pose_mq_ = position_ + orientation_q
    pose_mdeg_ = position_ + orientation_deg
    pose_mrad_ = position_ + orientation_rad

    # print('pose_mq in unitParser 1: {}'.format(pose_mq_))  # debug

    return pose_mq_, pose_mdeg_, pose_mrad_

def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + prefix + 'driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.logwarn('        the gripper action timed-out')
        return None

def getCurrentFingerPosition(prefix_):
    # wait to get current position
    topic_address = '/' + prefix_ + 'driver/out/finger_position'
    rospy.Subscriber(topic_address, kinova_msgs.msg.FingerPosition, setCurrentFingerPosition)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.FingerPosition)
    print 'obtained current finger position '

def setCurrentFingerPosition(feedback):
    global currentFingerPosition
    currentFingerPosition[0] = feedback.finger1
    currentFingerPosition[1] = feedback.finger2
    currentFingerPosition[2] = feedback.finger3

def fingers_unitParser(unit_, finger_value_, relative_):
    """ Argument unit """
    global currentFingerPosition

    # transform between units
    if unit_ == 'turn':
        # get absolute value
        if relative_:
            finger_turn_absolute_ = [finger_value_[i] + currentFingerPosition[i] for i in range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_value_

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]

    elif unit_ == 'mm':
        # get absolute value
        finger_turn_command = [x/1000 * finger_maxTurn / finger_maxDist for x in finger_value_]
        if relative_:
            finger_turn_absolute_ = [finger_turn_command[i] + currentFingerPosition[i] for i in range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_turn_command

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
    elif unit_ == 'percent':
        # get absolute value
        finger_turn_command = [x/100.0 * finger_maxTurn for x in finger_value_]
        if relative_:
            finger_turn_absolute_ = [finger_turn_command[i] + currentFingerPosition[i] for i in
                                     range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_turn_command

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
    else:
        raise Exception("Finger value have to be in turn, mm or percent")

    return finger_turn_, finger_meter_, finger_percent_


# me def
def objectCallback(message):
	global object_name
	object_name = message.data



robot_ready = True
object_name="No object"

if __name__ == '__main__':

    #args = argumentParser(None)
    parser = argparse.ArgumentParser(description='Drive robot end-effector to command Cartesian pose')
    args = parser.parse_args()
    kinova_robotTypeParser('j2s6s300')
    rospy.init_node(prefix + 'pose_action_client')
    pub = rospy.Publisher('robot_ready', std_msgs.msg.Bool, queue_size=1)
    rospy.Subscriber("object_name", String, objectCallback)
    getCurrentFingerPosition(prefix)
    getcurrentCartesianCommand(prefix)
    listener = tf.TransformListener()
    #rate = rospy.Rate(10.0)
    #while not rospy.is_shutdown():
    while True:
        if object_name != "No object":
            if object_name == "open":
                
                robot_ready = False
                object_name = "No object"
                pub.publish(robot_ready)

                finger_turn, finger_meter, finger_percent = fingers_unitParser('turn', [1000,1000,1000], '')
                positions_temp1 = [max(0.0, n) for n in finger_turn]
                positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
                positions = [float(n) for n in positions_temp2]
                print('Sending finger position ...')
                result = gripper_client(positions)
                
                robot_ready = True
                object_name = "No object"
                pub.publish(robot_ready)
                time.sleep(2)

            if object_name == "close":

                robot_ready = False
                object_name = "No object"
                pub.publish(robot_ready)

                finger_turn, finger_meter, finger_percent = fingers_unitParser('turn', [4500,4500,4500], '')
                positions_temp1 = [max(0.0, n) for n in finger_turn]
                positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
                positions = [float(n) for n in positions_temp2]
                print('Sending finger position ...')
                result = gripper_client(positions)

                robot_ready = True
                object_name = "No object"
                pub.publish(robot_ready)
                time.sleep(2)
            
            if object_name == "return" and return_num != 0:
                
                print("return")
                robot_ready = False
                object_name = "No object"
                pub.publish(robot_ready)
                time.sleep(2)
                return_num-=1
                rot[0] = return_rot[return_num][0]
                rot[1] = return_rot[return_num][1]
                rot[2] = return_rot[return_num][2]
                rot[3] = return_rot[return_num][3]
                trans[0]=return_position[return_num][0]
                trans[1]=return_position[return_num][1]
                trans[2]=return_position[return_num][2]
                trans[2]+=0.1
                result = cartesian_pose_client(trans, rot)
                trans[2]-=0.1
                result = cartesian_pose_client(trans, rot)
		print('Cartesian pose sent!')
                finger_turn, finger_meter, finger_percent = fingers_unitParser('turn', [1000,1000,1000], '')
                positions_temp1 = [max(0.0, n) for n in finger_turn]
		positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
		positions = [float(n) for n in positions_temp2]

		print('Sending finger position ...')
		result = gripper_client(positions)
		print('Finger position sent!')
		trans[2] +=0.16
                result = cartesian_pose_client(trans, rot)

		#cartesian step 2
		pose_mq, pose_mdeg, pose_mrad = unitParser('mrad',homeposition, '')
		poses = [float(n) for n in pose_mq]

		result = cartesian_pose_client(poses[:3], poses[3:])
                time.sleep(2)
                print('Cartesian pose sent!')
		robot_ready = True
                object_name = "No object"
                pub.publish(robot_ready)
		time.sleep(2)

            if object_name == "exit":
                
                sys.exit()

            else:
                try:
                    (trans,rot) = listener.lookupTransform('world', 'pickup_frame', rospy.Time(0))
	        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
		
	        try:
                    robot_ready = False
                    pub.publish(robot_ready)

		    finger_turn, finger_meter, finger_percent = fingers_unitParser('turn', [1000,1000,1000], '')
		    positions_temp1 = [max(0.0, n) for n in finger_turn]
		    positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
		    positions = [float(n) for n in positions_temp2]
		    print('Sending finger position ...')
		    result = gripper_client(positions)
		    print('Finger position sent!')
				
		    # move under object
                    if object_name=="ball":
                        trans[2] += 0.12
                        result = cartesian_pose_client(trans, rot)
                        time.sleep(2)
		        trans[2] -= 0.1
		        # finger step 2
                        finger_turn, finger_meter, finger_percent = fingers_unitParser('turn', [3500,3500,3500], '')

		    if object_name=="cup":
                        trans[2]-=0.05
                        trans[0]-=0.1
                        trans[1]-=0.01
                        result = cartesian_pose_client(trans, rot)
                        time.sleep(2)
                        trans[0]+=0.12
                        # finger step 2
                        finger_turn, finger_meter, finger_percent = fingers_unitParser('turn', [4500,4500,4500], '')

		    if object_name=="bottle":
                        trans[2]-=0.05
                        trans[0]-=0.09
                        trans[1]-=0.03
                        result = cartesian_pose_client(trans, rot)
                        time.sleep(2)
                        trans[0]+=0.1
                        # finger step 2
                        finger_turn, finger_meter, finger_percent = fingers_unitParser('turn', [4500,4500,4500], '')
                                        
		    return_position[return_num][0]=trans[0]
                    return_position[return_num][1]=trans[1]
                    return_position[return_num][2]=trans[2]
                    return_rot[return_num][0]=rot[0]
                    return_rot[return_num][1]=rot[1]
                    return_rot[return_num][2]=rot[2]
                    return_rot[return_num][3]=rot[3]
                    return_object[return_num]=object_name
                    return_num+=1
                    print "return record"

                    # cartesian step 1
		    result = cartesian_pose_client(trans, rot)
		    print('Cartesian pose sent!')
                    positions_temp1 = [max(0.0, n) for n in finger_turn]
		    positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
		    positions = [float(n) for n in positions_temp2]

		    print('Sending finger position ...')
		    result = gripper_client(positions)
		    print('Finger position sent!')
		    trans[2] +=0.15
                    result = cartesian_pose_client(trans, rot)

		    #cartesian step 2
		    pose_mq, pose_mdeg, pose_mrad = unitParser('mrad',homeposition, '')	
		    poses = [float(n) for n in pose_mq]

		    result = cartesian_pose_client(poses[:3], poses[3:])
                    time.sleep(2)

		    print('Cartesian pose sent!')
		    robot_ready = True
                    object_name = "No object" 
                    pub.publish(robot_ready)
		    time.sleep(2)
	        except rospy.ROSInterruptException:
		    print "program interrupted before completion"
	else:
	    robot_ready = True
	    pub.publish(robot_ready)
