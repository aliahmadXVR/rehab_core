#!/usr/bin/env python

#Adding libraries
import rospy
from sensor_msgs.msg import JointState

j = [0]
def jointState_callback(msg):
    global j
    j = msg.position
    #print(j)

rospy.init_node('joint_status', anonymous=True)
rospy.Subscriber("/joint_states", JointState, jointState_callback)

def status():
    global j
    if len(j)>1:
        print("jointState: is Okay")
        j = [0]
    else:
        print("jointState: not okay")

rospy.sleep(2)
while not rospy.is_shutdown():
    status()
    rospy.sleep(2)
