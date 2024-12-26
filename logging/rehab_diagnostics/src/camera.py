#!/usr/bin/env python

#Adding libraries
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import cv2, cv_bridge

image = [0]
depth_width = 0

def Image_callback(msg):
    global image
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

def depth_callback(msg):
    global depth_width
    depth_width = msg.width

rospy.init_node('Camera_status', anonymous=True)
rospy.Subscriber("/camera/color/image_raw", Image, Image_callback)
rospy.Subscriber("/camera/depth_registered/points", PointCloud2, depth_callback)


def camera_status():
    global image
    if len(image)>1:
        print("Rgb Camera is Okay")
        image = [0]
    else:
        print("Rgb Camera is not okay")

def depth_status():
    global depth_width
    if depth_width>0:
        print("depth Camera is Okay")
        depth_width = 0
    else:
        print("depth Camera is not okay")

rospy.sleep(3)

while not rospy.is_shutdown():
    camera_status()
    depth_status()
    rospy.sleep(2)
