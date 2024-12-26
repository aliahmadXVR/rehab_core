#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, String

pan_msg = Int32MultiArray()
tilt_msg = Int32MultiArray()

def move_pan_tilt(pan_angle, tilt_angle):
    """
    Move the pan and tilt to the specified angles and publish them.
    """
    pan_msg.data = [pan_angle]
    tilt_msg.data = [tilt_angle]
    pan_pub.publish(pan_msg)
    tilt_pub.publish(tilt_msg)

def angles_callback(data):
    """
    Callback function for the /angles_by_person_loc subscriber.
    """
    try:
        pan_angle, tilt_angle = data.data.split(',')
        move_pan_tilt(int(pan_angle), int(tilt_angle))
    except ValueError:
        rospy.logerr("Invalid angle format received: {}".format(data.data))

def main():
    global pan_pub, tilt_pub
    pan_pub = rospy.Publisher('pan_angles', Int32MultiArray, queue_size=1000)
    tilt_pub = rospy.Publisher('tilt_angles', Int32MultiArray, queue_size=1000)
    rospy.init_node('pan_tilt_int', anonymous=True)
    rospy.Subscriber('angles_by_person_loc', String, angles_callback)
    print("Listening for pan and tilt angles...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
