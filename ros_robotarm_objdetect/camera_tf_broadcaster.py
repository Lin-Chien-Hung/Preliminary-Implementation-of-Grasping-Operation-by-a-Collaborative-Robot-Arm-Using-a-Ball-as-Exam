#!/usr/bin/env python  

import rospy
import tf
import math


if __name__ == '__main__':
    object_name = "No object"
    rospy.init_node('camera_tf_broadcaster_node')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
# 0, -0.38, 0.97
    while not rospy.is_shutdown():
        br.sendTransform((0.25, -0.38, 0.955),
                tf.transformations.quaternion_from_euler(0.0, math.pi/2.0, math.pi/2.0),
                         rospy.Time.now(),
                         "camera_link",
                         "world")
# br.sendTransform((x, y, z)   math.pi/4.0

        rate.sleep()
