#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from std_msgs.msg import Int32MultiArray

pan_msg = Int32MultiArray()
tilt_msg = Int32MultiArray()

pan_angle = 0

marker_x  = marker_y  =  0
marker_id1 = "not_in_frame"
def visual_callback(msg):
    global marker_x, marker_y, marker_yaw, marker_id1, pan_angle
    try:
        marker_x = msg.pose.position.x
        marker_y = msg.pose.position.y
        marker_id1 = "in_frame"
        #print(marker_y)
        if marker_y > 0.20:
            pan_angle = max(-100, min(100, pan_angle +2))
            pan_msg.data = [pan_angle]
            pan_pub.publish(pan_msg)
        if marker_y < -0.20:
            pan_angle = max(-100, min(100, pan_angle -2))
            pan_msg.data = [pan_angle]
            pan_pub.publish(pan_msg)
    except:
        marker_id1 = "not_in_frame"
        print("marker 1 not detected")



if __name__ == '__main__':
    # Initialize the ROS node with the name 'sound_direction_pan'
    rospy.init_node('person_track')
    rospy.Subscriber("/aruco_person_track/pose", PoseStamped, visual_callback)
    pan_pub = rospy.Publisher('pan_angles', Int32MultiArray, queue_size=10)
    tilt_pub = rospy.Publisher('tilt_angles', Int32MultiArray, queue_size=10)
    rospy.spin()