#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

pan_msg = Int32MultiArray()
tilt_msg = Int32MultiArray()

# Initial pan and tilt angles
pan_angle = 0
tilt_angle = 0

def move_pan_tilt(pan_increment, tilt_increment):
    """
    Move the pan and tilt by the specified increments, ensuring they stay within the limits, and publish them.
    """
    global pan_angle, tilt_angle
    pan_angle = max(-100, min(100, pan_angle + pan_increment))
    tilt_angle = max(0, min(35, tilt_angle + tilt_increment))
    pan_msg.data = [pan_angle]
    tilt_msg.data = [tilt_angle]
    pan_pub.publish(pan_msg)
    tilt_pub.publish(tilt_msg)


def joy_callback(data):
    """
    Callback function for the /joy subscriber.
    """
    # Check if the joystick message contains button data
    if data.buttons[0] == 1:  # Button 0 to control tilt down
        move_pan_tilt(0, -2)  # Adjust tilt down by -1 degree
    elif data.buttons[3] == 1:  # Button 3 to control tilt up
        move_pan_tilt(0, 1)  # Adjust tilt up by 1 degree
    elif data.buttons[1] == 1:  # Button 1 to control pan right
        move_pan_tilt(-2, 0)  # Adjust pan right by 1 degree
    elif data.buttons[2] == 1:  # Button 2 to control pan left
        move_pan_tilt(2, 0)  # Adjust pan left by -1 degree

def main():
    global pan_pub, tilt_pub
    pan_pub = rospy.Publisher('pan_angles', Int32MultiArray, queue_size=10)
    tilt_pub = rospy.Publisher('tilt_angles', Int32MultiArray, queue_size=10)
    rospy.init_node('pan_tilt_int', anonymous=True)
    rospy.Subscriber('/joy', Joy, joy_callback)
    print("Listening for joystick commands...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
