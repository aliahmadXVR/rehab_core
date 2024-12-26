#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Image, PointCloud2, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import cv2, cv_bridge
from rehab_msgs.msg import bumper
from std_msgs.msg import String
import time

# Global variables for sensor states and last received times
last_scan_time = 0
last_image_time = 0
last_depth_time = 0
last_bumper_time = 0
last_joint_time = 0
last_odom_time = 0

# Timeout threshold in seconds
timeout_threshold = 5

rotation = 0.0
twist = Twist()

# Sensor Callbacks to update last received time
def odom_callback(msg):
    global rotation, last_odom_time
    rotation = msg.pose.pose.orientation.z
    last_odom_time = time.time()

def jointState_callback(msg):
    global j, last_joint_time
    j = msg.position
    last_joint_time = time.time()

image = [0]
def Image_callback(msg):
    global image, last_image_time
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    last_image_time = time.time()

def depth_callback(msg):
    global depth_width, last_depth_time
    depth_width = msg.width
    last_depth_time = time.time()

l = [0]
def scan_callback(msg):
    global l, last_scan_time
    l = msg.ranges
    last_scan_time = time.time()

def bumper_callback(msg):
    global last_bumper_time
    last_bumper_time = time.time()

rospy.init_node('Rehab_Diagnostic_Node', anonymous=True)
rospy.Subscriber("/scan", LaserScan, scan_callback)
rospy.Subscriber("/camera/color/image_raw", Image, Image_callback)
rospy.Subscriber("/camera/depth_registered/points", PointCloud2, depth_callback)
rospy.Subscriber("/bumper", bumper, bumper_callback)
rospy.Subscriber("/joint_states", JointState, jointState_callback)
rospy.Subscriber("/mobile_base_controller/odom", Odometry, odom_callback)

cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
status_pub = rospy.Publisher('/sensor_status', String, queue_size=10)

# Sensor Status Functions
def camera_status():
    current_time = time.time()
    if current_time - last_image_time < timeout_threshold:
        return "RGB Camera: Okay"
    else:
        return "RGB Camera: Not Okay"

def depth_status():
    current_time = time.time()
    if current_time - last_depth_time < timeout_threshold:
        return "Depth Camera: Okay"
    else:
        return "Depth Camera: Not Okay"

def lidar_status():
    current_time = time.time()
    if current_time - last_scan_time < timeout_threshold:
        return "Lidar: Okay"
    else:
        return "Lidar: Not Okay"

# def bumper_status():
#     current_time = time.time()
#     if current_time - last_bumper_time < timeout_threshold:
#         return "Bumper: Okay"
#     else:
#         return "Bumper: Not Okay"

def bumper_status():
    global id_, state
    return "Bumper: Okay"


def jointState_status():
    current_time = time.time()
    if current_time - last_joint_time < timeout_threshold:
        return "JointState: Okay"
    else:
        return "JointState: Not Okay"

previous_z = round(rotation, 2)
def odom_status():
    current_time = time.time()
    if current_time - last_odom_time < timeout_threshold:
        return "Odom: Okay"
    else:
        return "Odom: Not Okay"

# Main loop
while not rospy.is_shutdown():
    status_list = []
    
    # Check each sensor status periodically
    status_list.append(lidar_status())
    status_list.append(camera_status())
    status_list.append(depth_status())
    status_list.append(jointState_status())
    status_list.append(bumper_status())
    status_list.append(odom_status())

    # Combine all statuses into a single string message
    combined_status = "\n".join(status_list)
    
    # Publish the combined status message
    status_pub.publish(combined_status)
    
    # Optional: Print the combined status to the terminal
    print(combined_status)
    print("---------------------")
    
    rospy.sleep(3)
