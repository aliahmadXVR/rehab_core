#!/usr/bin/env python

#Adding libraries
import rospy
from rehab_msgs.msg import bumper


def bumper_callback(msg):
    global id_, state
    id_ = msg.sensor_id
    state = msg.sensor_state
    print(id_,state)
    status()

print("press any of the bumper")
print("")
rospy.init_node('bumper_status', anonymous=True)
rospy.Subscriber("/bumper", bumper, bumper_callback)


def status():
    global id_, state
    if id_ > 0:
        print("Bumper is working")
        print("------")

rospy.sleep(2)


while not rospy.is_shutdown():
    rospy.sleep(2)