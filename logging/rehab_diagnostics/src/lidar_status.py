#!/usr/bin/env python

#Adding libraries
import rospy
from sensor_msgs.msg import LaserScan
l = [0]

def scan_callback(msg):
    global l
    l = msg.ranges

rospy.init_node('Lidar_status', anonymous=True)
rospy.Subscriber("/scan", LaserScan, scan_callback)

def status():
    global l
    if len(l)>1:
        print("Lidar is Okay")
        l = [0]
    else:
        print("Lidar is not okay")

rospy.sleep(2)
while not rospy.is_shutdown():
    status()
    rospy.sleep(2)
