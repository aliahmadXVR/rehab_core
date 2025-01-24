#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32MultiArray

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

def cmd_callback(data):
    """
    Callback function for the /cmd_frm_tablet subscriber.
    """
    command = data.data.split(',')
    if len(command) > 1:
        action = command[1].strip()
        if action == "moveheadup":
            move_pan_tilt(0, 12)  # Adjust tilt up by 1 degree
        elif action == "moveheaddown":
            move_pan_tilt(0, -12)  # Adjust tilt down by -1 degree
        elif action == "moveheadright":
            move_pan_tilt(15, 0)  # Adjust pan right by -1 degree
        elif action == "moveheadleft":
            move_pan_tilt(-15, 0)  # Adjust pan left by 1 degree

def main():
    global pan_pub, tilt_pub
    pan_pub = rospy.Publisher('pan_angles', Int32MultiArray, queue_size=10)
    tilt_pub = rospy.Publisher('tilt_angles', Int32MultiArray, queue_size=10)
    rospy.init_node('screen_head_control', anonymous=True)
    rospy.Subscriber('/cmd_frm_tablet', String, cmd_callback)
    print("screen_head_control node is Listening for tablet commands...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
