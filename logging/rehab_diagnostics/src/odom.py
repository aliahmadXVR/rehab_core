#!/usr/bin/env python

#Adding libraries
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

twist =Twist()
def odom_callback(msg):
    global o
    o = msg.pose.pose.orientation.z

rospy.init_node('Odom_status', anonymous=True)
rospy.Subscriber("/mobile_base_controller/odom", Odometry, odom_callback)
cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

rospy.sleep(2)
previous_z = round(o,2)
def status():
    global o
    for i in range(20):
        twist.angular.z = 0.2
        cmd_pub.publish(twist)
        rospy.sleep(0.1)
        #print("moving")
    if round(o,2)!=previous_z:
        print(round(o-previous_z,2))
        print("odom is working")
    else:
        print("not")
    for i in range(20):
        twist.angular.z = -0.2
        cmd_pub.publish(twist)
        rospy.sleep(0.1)
        #print("moving")

def live_status():
    global o
    


status()


while not rospy.is_shutdown():
    live_status()
    rospy.sleep(2)
